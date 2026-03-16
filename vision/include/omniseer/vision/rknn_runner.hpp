#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <rknn_api.h>
#include <string>
#include <vector>

#include "omniseer/vision/types.hpp"

namespace omniseer::vision
{
  class ImageBufferPool;

  /**
   * @brief Configuration for RKNN runtime initialization and warmup.
   */
  struct RknnRunnerConfig
  {
    /// @brief File path to a compiled RKNN model file.
    std::string model_path{};
    /// @brief Number of warmup inferences to run during preflight.
    uint32_t warmup_runs{2};
  };

  /**
   * @brief Result category for one RKNN inference operation.
   */
  enum class InferStatus : uint8_t
  {
    Ok,
    NotArmed,
    InvalidInputDescriptor,
    RknnError,
  };

  /**
   * @brief Status payload for RKNN inference operations.
   */
  struct InferResult
  {
    /// @brief High-level inference status.
    InferStatus status{InferStatus::Ok};
    /// @brief RKNN return code associated with the failure stage, if any.
    int rknn_code{RKNN_SUCC};
    /// @brief errno captured from OS APIs in the failure path, if any.
    int sys_errno{0};

    bool ok() const noexcept
    {
      return status == InferStatus::Ok;
    }
  };

  /**
   * @brief One immutable output tensor view over preallocated storage.
   */
  struct RknnOutputView
  {
    /// @brief Output tensor index.
    uint32_t index{0};
    /// @brief Pointer to output bytes owned by RknnRunner.
    const void* data{nullptr};
    /// @brief Valid byte count available at @ref data.
    size_t bytes{0};
  };

  /**
   * @brief Immutable metadata for one RKNN output tensor captured during preflight.
   */
  struct RknnOutputDesc
  {
    /// @brief Output tensor index.
    uint32_t index{0};
    /// @brief Stable tensor name reported by RKNN, when available.
    std::string name{};
    /// @brief Number of valid dimensions in @ref dims.
    uint32_t n_dims{0};
    /// @brief Tensor dimensions in RKNN-reported order.
    std::array<uint32_t, RKNN_MAX_DIMS> dims{};
    /// @brief RKNN tensor element type.
    rknn_tensor_type type{RKNN_TENSOR_FLOAT32};
    /// @brief Quantization zero point for asymmetric int tensors.
    int32_t zero_point{0};
    /// @brief Quantization scale for asymmetric int tensors.
    float scale{1.0F};
  };

  /**
   * @brief RKNN consumer-stage runtime with FD-backed input binding.
   *
   * Typical flow:
   * - Construct with RknnRunnerConfig.
   * - preflight(pool, text_i8, text_bytes) once at startup.
   * - infer(pool_index) per frame on the hot path.
   * - Read outputs() and output_descs() after successful preflight.
   *
   * Implementation contract:
   * - Model contract is two inputs: image + text embeddings.
   * - Image binding uses rknn_create_mem_from_fd + rknn_set_io_mem.
   * - Text binding uses one prequantized int8 tensor provided at preflight().
   * - preflight() pre-binds every ImageBufferPool slot and static text input.
   * - infer() performs no mmap()/rknn_create_mem_from_fd/quantization work.
   * - Output storage is preallocated during preflight.
   */
  class RknnRunner
  {
  public:
    /**
     * @brief Construct runner with immutable startup configuration.
     */
    explicit RknnRunner(RknnRunnerConfig cfg = {});
    ~RknnRunner() noexcept;

    RknnRunner(const RknnRunner&)            = delete;
    RknnRunner& operator=(const RknnRunner&) = delete;
    RknnRunner(RknnRunner&&)                 = delete;
    RknnRunner& operator=(RknnRunner&&)      = delete;

    /**
     * @brief Perform all heavy startup work required before hot-path inference:
     *
     * 1) resets prior runner state,
     * 2) loads model bytes and initializes RKNN context,
     * 3) queries model input/output tensor attributes,
     * 4) resolves image/text input roles and validates the model contract,
     * 5) pre-binds every ImageBufferPool slot as RKNN image input memory,
     * 6) binds one static prequantized int8 text-embedding tensor,
     * 7) preallocates output storage/descriptors,
     * 8) runs warmup inference passes.
     *
     * On success, the runner is armed and infer() can execute without per-frame
     * mapping/allocation work.
     *
     * Notes:
     * - @p text_i8 must point to contiguous prequantized int8 embeddings matching
     *   the model text-input byte size.
     *
     * @throws std::runtime_error on model/runtime/IO setup failures.
     */
    void preflight(const ImageBufferPool& pool, const int8_t* text_i8, size_t text_bytes);

    /**
     * @brief Run one inference using one pre-bound pool slot index.
     *
     * Hot-path responsibilities:
     * 1) Validate armed state and slot index.
     * 2) Select the pre-bound image-input binding for @p pool_index.
     * 3) Rebind RKNN image input only when active slot changes.
     * 4) Execute rknn_run.
     * 5) Retrieve outputs into preallocated buffers via rknn_outputs_get/release.
     *
     * Invariants:
     * - noexcept and non-throwing on all paths.
     * - no per-frame mmap()/rknn_create_mem_from_fd work.
     * - no per-frame text quantization/binding work.
     * - no per-frame dynamic allocation in steady state.
     * - assumes pool slot bindings established by preflight() are still valid.
     *
     * @return InferResult with status and diagnostics.
     */
    InferResult infer(int32_t pool_index) noexcept;

    /// @brief True after successful preflight.
    bool is_armed() const noexcept;

    /// @brief Immutable output views for the latest successful inference.
    const std::vector<RknnOutputView>& outputs() const noexcept;
    /// @brief Immutable output metadata captured during preflight.
    const std::vector<RknnOutputDesc>& output_descs() const noexcept;

  private:
    struct ImageInputBinding
    {
      int32_t          pool_index{-1};
      int              fd{-1};
      void*            mapped_base{nullptr};
      size_t           map_size{0};
      int32_t          offset{0};
      rknn_tensor_mem* mem{nullptr};
    };

    struct OutputBinding
    {
      rknn_tensor_attr     attr{};
      std::vector<uint8_t> storage{};
      rknn_output          output{};
    };

    /// @brief Load model bytes and initialize RKNN context.
    void _init_context_from_model();
    /// @brief Query model input/output metadata required for bindings.
    void _query_model_io();
    /// @brief Resolve image/text input roles from queried model metadata.
    void _resolve_input_roles();
    /// @brief Pre-bind every pool slot as RKNN image-input memory.
    void _prebind_all_image_slots(const ImageBufferPool& pool);
    /// @brief Bind one static prequantized int8 text-input tensor.
    void _bind_static_text_input(const int8_t* text_i8, size_t text_bytes);
    /// @brief Preallocate output buffers and rknn_output descriptors.
    void _prepare_outputs();
    /// @brief Run startup warmup passes on one pre-bound image slot.
    void _run_warmup();

    /// @brief Validate one image descriptor against preflight constraints.
    bool _validate_image_descriptor(const ImageBuffer& input) const noexcept;
    /// @brief Pre-bind one pool slot as RKNN image-input memory.
    void _bind_image_pool_slot(int32_t pool_index, const ImageBuffer& input);
    /// @brief Destroy all cached image-input memory bindings and mappings.
    void _release_image_bindings() noexcept;
    /// @brief Destroy static text-input tensor memory binding.
    void _release_text_binding() noexcept;
    /// @brief Destroy output memory wrappers and clear storage metadata.
    void _release_outputs() noexcept;
    /// @brief Tear down context-owned resources.
    void _shutdown() noexcept;

    RknnRunnerConfig       _cfg{};
    rknn_context           _ctx{0};
    bool                   _armed{false};
    const ImageBufferPool* _pool{nullptr};

    rknn_input_output_num         _io_num{};
    std::vector<rknn_tensor_attr> _input_attrs{};
    int32_t                       _image_input_index{-1};
    int32_t                       _text_input_index{-1};

    int32_t                        _active_image_slot{-1};
    std::vector<ImageInputBinding> _image_bindings{};
    rknn_tensor_mem*               _text_mem{nullptr};
    std::vector<OutputBinding>     _output_bindings{};
    std::vector<rknn_output>       _output_io{};
    std::vector<RknnOutputDesc>    _output_descs{};
    std::vector<RknnOutputView>    _output_views{};
  };
} // namespace omniseer::vision
