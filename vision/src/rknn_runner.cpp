#include "omniseer/vision/rknn_runner.hpp"

#include <cassert>
#include <cerrno>
#include <cstring>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <sys/mman.h>
#include <utility>

#include "omniseer/vision/image_buffer_pool.hpp"

namespace omniseer::vision
{
  namespace
  {
    std::vector<uint8_t> read_model_file(const std::string& path)
    {
      std::ifstream ifs(path, std::ios::binary | std::ios::ate);
      if (!ifs)
        throw std::runtime_error("RknnRunner::preflight: failed to open model file: " + path);

      const std::ifstream::pos_type end = ifs.tellg();
      if (end <= 0)
        throw std::runtime_error("RknnRunner::preflight: model file is empty: " + path);

      std::vector<uint8_t> data(static_cast<size_t>(end));
      ifs.seekg(0, std::ios::beg);
      if (!ifs.read(reinterpret_cast<char*>(data.data()),
                    static_cast<std::streamsize>(data.size())))
      {
        throw std::runtime_error("RknnRunner::preflight: failed to read model file: " + path);
      }
      return data;
    }

    std::runtime_error make_rknn_error(const char* where, int code)
    {
      return std::runtime_error(std::string(where) + " failed, code=" + std::to_string(code));
    }

    uint32_t tensor_bytes_for_attr(const rknn_tensor_attr& attr)
    {
      return (attr.size_with_stride != 0) ? attr.size_with_stride : attr.size;
    }

    size_t map_size_for_input(const ImageBuffer& input)
    {
      if (input.total_alloc_size != 0)
        return input.total_alloc_size;

      if (input.num_planes < 1)
        return 0;

      const auto&  p          = input.planes[0];
      const size_t offset     = static_cast<size_t>(p.offset);
      const size_t plane_size = p.alloc_size;
      if (offset > (std::numeric_limits<size_t>::max() - plane_size))
        return 0;
      return offset + plane_size;
    }
  } // namespace

  RknnRunner::RknnRunner(RknnRunnerConfig cfg) : _cfg(std::move(cfg)) {}

  RknnRunner::~RknnRunner() noexcept
  {
    _shutdown();
  }

  bool RknnRunner::is_armed() const noexcept
  {
    return _armed;
  }

  const std::vector<RknnOutputView>& RknnRunner::outputs() const noexcept
  {
    return _output_views;
  }

  const std::vector<RknnOutputDesc>& RknnRunner::output_descs() const noexcept
  {
    return _output_descs;
  }

  void RknnRunner::preflight(const ImageBufferPool& pool, const int8_t* text_i8, size_t text_bytes)
  {
    _shutdown();

    try
    {
      _pool = &pool;
      _init_context_from_model();
      _query_model_io();
      _resolve_input_roles();
      _prebind_all_image_slots(pool);
      _bind_static_text_input(text_i8, text_bytes);
      _prepare_outputs();
      _run_warmup();

      _armed = true;
    }
    catch (...)
    {
      _shutdown();
      throw;
    }
  }

  InferResult RknnRunner::infer(int32_t pool_index) noexcept
  {
    InferResult out{};
    auto        fail = [&](InferStatus status, int rknn_code, int sys_errno) noexcept
    {
      out.status    = status;
      out.rknn_code = rknn_code;
      out.sys_errno = sys_errno;
      return out;
    };

    if (!_armed || _ctx == 0 || _pool == nullptr)
      return fail(InferStatus::NotArmed, RKNN_SUCC, ENODEV);

    if (pool_index < 0 || static_cast<size_t>(pool_index) >= _image_bindings.size())
      return fail(InferStatus::InvalidInputDescriptor, RKNN_SUCC, EINVAL);

    ImageInputBinding& binding = _image_bindings[static_cast<size_t>(pool_index)];
    assert(binding.pool_index == pool_index);
    assert(binding.mem != nullptr);
    assert(binding.fd >= 0);

    if (binding.mem == nullptr || binding.fd < 0)
      return fail(InferStatus::InvalidInputDescriptor, RKNN_SUCC, EINVAL);

    if (_active_image_slot != pool_index)
    {
      assert(_image_input_index >= 0);
      assert(static_cast<size_t>(_image_input_index) < _input_attrs.size());

      errno                       = 0;
      rknn_tensor_attr image_attr = _input_attrs[static_cast<size_t>(_image_input_index)];
      image_attr.pass_through     = 1;
      const int rc_set            = rknn_set_io_mem(_ctx, binding.mem, &image_attr);
      if (rc_set != RKNN_SUCC)
        return fail(InferStatus::RknnError, rc_set, (errno != 0) ? errno : EIO);
      _active_image_slot = pool_index;
    }

    errno            = 0;
    const int rc_run = rknn_run(_ctx, nullptr);
    if (rc_run != RKNN_SUCC)
      return fail(InferStatus::RknnError, rc_run, (errno != 0) ? errno : EIO);

    assert(_output_io.size() == static_cast<size_t>(_io_num.n_output));

    errno            = 0;
    const int rc_get = rknn_outputs_get(_ctx, _io_num.n_output, _output_io.data(), nullptr);
    if (rc_get != RKNN_SUCC)
      return fail(InferStatus::RknnError, rc_get, (errno != 0) ? errno : EIO);

    errno                = 0;
    const int rc_release = rknn_outputs_release(_ctx, _io_num.n_output, _output_io.data());
    if (rc_release != RKNN_SUCC)
      return fail(InferStatus::RknnError, rc_release, (errno != 0) ? errno : EIO);

    return fail(InferStatus::Ok, RKNN_SUCC, 0);
  }

  void RknnRunner::_init_context_from_model()
  {
    if (_cfg.model_path.empty())
      throw std::invalid_argument("RknnRunner::preflight: model_path is empty");

    std::vector<uint8_t> model_data = read_model_file(_cfg.model_path);

    const int rc =
        rknn_init(&_ctx, model_data.data(), static_cast<uint32_t>(model_data.size()), 0, nullptr);
    if (rc != RKNN_SUCC)
      throw make_rknn_error("rknn_init", rc);
  }

  void RknnRunner::_query_model_io()
  {
    int rc = rknn_query(_ctx, RKNN_QUERY_IN_OUT_NUM, &_io_num, sizeof(_io_num));
    if (rc != RKNN_SUCC)
      throw make_rknn_error("rknn_query(RKNN_QUERY_IN_OUT_NUM)", rc);

    _input_attrs.clear();
    _input_attrs.resize(static_cast<size_t>(_io_num.n_input));
    for (uint32_t i = 0; i < _io_num.n_input; ++i)
    {
      rknn_tensor_attr attr{};
      attr.index = i;
      rc         = rknn_query(_ctx, RKNN_QUERY_INPUT_ATTR, &attr, sizeof(attr));
      if (rc != RKNN_SUCC)
        throw make_rknn_error("rknn_query(RKNN_QUERY_INPUT_ATTR)", rc);
      _input_attrs[static_cast<size_t>(i)] = attr;
    }
  }

  void RknnRunner::_resolve_input_roles()
  {
    if (_io_num.n_input != 2)
      throw std::runtime_error("RknnRunner::preflight: model must have exactly two inputs");

    _image_input_index = -1;
    _text_input_index  = -1;
    for (uint32_t i = 0; i < _io_num.n_input; ++i)
    {
      const rknn_tensor_attr& attr = _input_attrs[static_cast<size_t>(i)];
      if (std::strcmp(attr.name, "images") == 0)
        _image_input_index = static_cast<int32_t>(i);
      else if (std::strcmp(attr.name, "texts") == 0)
        _text_input_index = static_cast<int32_t>(i);
    }

    if (_image_input_index < 0 || _text_input_index < 0)
      throw std::runtime_error("RknnRunner::preflight: failed to resolve image/text input tensors");

    if (_image_input_index == _text_input_index)
      throw std::runtime_error("RknnRunner::preflight: image/text input indices overlap");
  }

  void RknnRunner::_prebind_all_image_slots(const ImageBufferPool& pool)
  {
    _image_bindings.clear();
    _image_bindings.resize(static_cast<size_t>(ImageBufferPool::capacity()));
    for (int32_t slot = 0; slot < ImageBufferPool::capacity(); ++slot)
    {
      _bind_image_pool_slot(slot, pool.buffer_at(slot));
    }
  }

  void RknnRunner::_bind_static_text_input(const int8_t* text_i8, size_t text_bytes)
  {
    if (text_i8 == nullptr)
      throw std::invalid_argument("RknnRunner::preflight: text_i8 is null");
    if (_text_input_index < 0 || static_cast<size_t>(_text_input_index) >= _input_attrs.size())
      throw std::runtime_error("RknnRunner::preflight: invalid text input index");

    const rknn_tensor_attr& text_attr = _input_attrs[static_cast<size_t>(_text_input_index)];
    const uint32_t          required  = tensor_bytes_for_attr(text_attr);
    if (required == 0)
      throw std::runtime_error("RknnRunner::preflight: text input tensor has zero size");
    if (text_bytes != static_cast<size_t>(required))
      throw std::runtime_error("RknnRunner::preflight: text embedding byte size mismatch");

    _text_mem = rknn_create_mem(_ctx, required);
    if (_text_mem == nullptr)
      throw std::runtime_error("RknnRunner::preflight: rknn_create_mem(text) returned nullptr");
    std::memcpy(_text_mem->virt_addr, text_i8, static_cast<size_t>(required));

    rknn_tensor_attr bind_attr = text_attr;
    bind_attr.pass_through     = 1;
    const int rc_set           = rknn_set_io_mem(_ctx, _text_mem, &bind_attr);
    if (rc_set != RKNN_SUCC)
      throw make_rknn_error("rknn_set_io_mem(text preflight)", rc_set);
  }

  void RknnRunner::_prepare_outputs()
  {
    _output_bindings.clear();
    _output_io.clear();
    _output_descs.clear();
    _output_views.clear();
    _output_bindings.resize(_io_num.n_output);
    _output_io.resize(_io_num.n_output);
    _output_descs.resize(_io_num.n_output);
    _output_views.resize(_io_num.n_output);

    for (uint32_t i = 0; i < _io_num.n_output; ++i)
    {
      OutputBinding& out = _output_bindings[static_cast<size_t>(i)];
      out.attr           = rknn_tensor_attr{};
      out.attr.index     = i;

      const int rc = rknn_query(_ctx, RKNN_QUERY_OUTPUT_ATTR, &out.attr, sizeof(out.attr));
      if (rc != RKNN_SUCC)
        throw make_rknn_error("rknn_query(RKNN_QUERY_OUTPUT_ATTR)", rc);

      const uint32_t out_bytes = tensor_bytes_for_attr(out.attr);
      if (out_bytes == 0)
        throw std::runtime_error("RknnRunner::preflight: output tensor has zero size");

      RknnOutputDesc& desc = _output_descs[static_cast<size_t>(i)];
      desc.index  = i;
      desc.name   = out.attr.name;
      desc.n_dims = (out.attr.n_dims <= RKNN_MAX_DIMS) ? out.attr.n_dims : RKNN_MAX_DIMS;
      for (uint32_t dim = 0; dim < desc.n_dims; ++dim)
      {
        desc.dims[static_cast<size_t>(dim)] = out.attr.dims[dim];
      }
      desc.type       = out.attr.type;
      desc.zero_point = out.attr.zp;
      desc.scale      = out.attr.scale;

      out.storage.assign(static_cast<size_t>(out_bytes), 0u);

      out.output                         = rknn_output{};
      out.output.want_float              = 0;
      out.output.is_prealloc             = 1;
      out.output.index                   = i;
      out.output.buf                     = out.storage.data();
      out.output.size                    = out_bytes;
      _output_io[static_cast<size_t>(i)] = out.output;

      RknnOutputView& view = _output_views[static_cast<size_t>(i)];
      view.index           = i;
      view.data            = out.storage.data();
      view.bytes           = out.storage.size();
    }
  }

  void RknnRunner::_run_warmup()
  {
    if (_cfg.warmup_runs == 0)
      return;
    if (_image_bindings.empty())
      throw std::runtime_error("RknnRunner::preflight: no image bindings available for warmup");
    if (_image_input_index < 0 || static_cast<size_t>(_image_input_index) >= _input_attrs.size())
      throw std::runtime_error("RknnRunner::preflight: invalid image input index");

    ImageInputBinding& warm_binding = _image_bindings[0];

    rknn_tensor_attr image_attr = _input_attrs[static_cast<size_t>(_image_input_index)];
    image_attr.pass_through     = 1;
    int rc                      = rknn_set_io_mem(_ctx, warm_binding.mem, &image_attr);
    if (rc != RKNN_SUCC)
      throw make_rknn_error("rknn_set_io_mem(image warmup)", rc);
    _active_image_slot = warm_binding.pool_index;

    for (uint32_t i = 0; i < _cfg.warmup_runs; ++i)
    {
      rc = rknn_run(_ctx, nullptr);
      if (rc != RKNN_SUCC)
        throw make_rknn_error("rknn_run(warmup)", rc);

      rc = rknn_outputs_get(_ctx, _io_num.n_output, _output_io.data(), nullptr);
      if (rc != RKNN_SUCC)
        throw make_rknn_error("rknn_outputs_get(warmup)", rc);

      rc = rknn_outputs_release(_ctx, _io_num.n_output, _output_io.data());
      if (rc != RKNN_SUCC)
        throw make_rknn_error("rknn_outputs_release(warmup)", rc);
    }
  }

  bool RknnRunner::_validate_image_descriptor(const ImageBuffer& input) const noexcept
  {
    if (_image_input_index < 0 || static_cast<size_t>(_image_input_index) >= _input_attrs.size())
      return false;
    if (input.num_planes < 1)
      return false;
    if (input.size.w <= 0 || input.size.h <= 0)
      return false;

    const auto& p = input.planes[0];
    if (p.fd < 0)
      return false;
    if (p.stride == 0)
      return false;

    const rknn_tensor_attr& image_attr     = _input_attrs[static_cast<size_t>(_image_input_index)];
    const uint32_t          required_bytes = tensor_bytes_for_attr(image_attr);
    if (required_bytes == 0)
      return false;

    const size_t plane_available =
        (p.alloc_size != 0) ? p.alloc_size : static_cast<size_t>(input.total_alloc_size);
    if (plane_available < static_cast<size_t>(required_bytes))
      return false;

    const size_t map_size = map_size_for_input(input);
    if (map_size == 0)
      return false;
    const size_t offset = static_cast<size_t>(p.offset);
    if (offset >= map_size)
      return false;
    if (static_cast<size_t>(required_bytes) > (map_size - offset))
      return false;

    return true;
  }

  void RknnRunner::_bind_image_pool_slot(int32_t pool_index, const ImageBuffer& input)
  {
    if (pool_index < 0 || static_cast<size_t>(pool_index) >= _image_bindings.size())
      throw std::runtime_error("RknnRunner::preflight: pool index out of range");

    if (!_validate_image_descriptor(input))
      throw std::runtime_error("RknnRunner::preflight: invalid input buffer descriptor");

    const auto& p = input.planes[0];
    if (p.offset > static_cast<uint32_t>(std::numeric_limits<int32_t>::max()))
      throw std::runtime_error("RknnRunner::preflight: input offset exceeds int32 range");

    const rknn_tensor_attr& image_attr  = _input_attrs[static_cast<size_t>(_image_input_index)];
    const uint32_t          input_bytes = tensor_bytes_for_attr(image_attr);
    const size_t            map_size    = map_size_for_input(input);
    void* mapped = ::mmap(nullptr, map_size, PROT_READ | PROT_WRITE, MAP_SHARED, p.fd, 0);
    if (mapped == MAP_FAILED)
    {
      throw std::runtime_error("RknnRunner::preflight: mmap input failed: " +
                               std::string(std::strerror(errno)));
    }

    rknn_tensor_mem* mem =
        rknn_create_mem_from_fd(_ctx, p.fd, mapped, input_bytes, static_cast<int32_t>(p.offset));
    if (mem == nullptr)
    {
      (void) ::munmap(mapped, map_size);
      throw std::runtime_error("RknnRunner::preflight: rknn_create_mem_from_fd returned nullptr");
    }

    ImageInputBinding& binding = _image_bindings[static_cast<size_t>(pool_index)];
    binding.pool_index         = pool_index;
    binding.fd                 = p.fd;
    binding.mapped_base        = mapped;
    binding.map_size           = map_size;
    binding.offset             = static_cast<int32_t>(p.offset);
    binding.mem                = mem;
  }

  void RknnRunner::_release_image_bindings() noexcept
  {
    for (ImageInputBinding& binding : _image_bindings)
    {
      if (binding.mem != nullptr && _ctx != 0)
      {
        (void) rknn_destroy_mem(_ctx, binding.mem);
      }
      binding.mem = nullptr;

      if (binding.mapped_base != nullptr && binding.map_size > 0)
      {
        (void) ::munmap(binding.mapped_base, binding.map_size);
      }
      binding.mapped_base = nullptr;
      binding.map_size    = 0;
      binding.pool_index  = -1;
      binding.fd          = -1;
      binding.offset      = 0;
    }

    _image_bindings.clear();
    _active_image_slot = -1;
  }

  void RknnRunner::_release_text_binding() noexcept
  {
    if (_text_mem != nullptr && _ctx != 0)
      (void) rknn_destroy_mem(_ctx, _text_mem);
    _text_mem = nullptr;
  }

  void RknnRunner::_release_outputs() noexcept
  {
    _output_views.clear();
    _output_descs.clear();
    _output_io.clear();
    _output_bindings.clear();
  }

  void RknnRunner::_shutdown() noexcept
  {
    _release_outputs();
    _release_text_binding();
    _release_image_bindings();

    if (_ctx != 0)
    {
      (void) rknn_destroy(_ctx);
      _ctx = 0;
    }

    _armed             = false;
    _pool              = nullptr;
    _active_image_slot = -1;
    _io_num            = rknn_input_output_num{};
    _input_attrs.clear();
    _image_input_index = -1;
    _text_input_index  = -1;
  }
} // namespace omniseer::vision
