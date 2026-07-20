#include "omniseer_vision_bridge/evidence_frame_sink.hpp"

#include <algorithm>
#include <cerrno>
#include <csetjmp>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <deque>
#include <filesystem>
#include <fstream>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <thread>
#include <utility>
#include <vector>

#include <jpeglib.h>
#include <linux/dma-buf.h>

#include "omniseer/vision/config.hpp"
#include "omniseer/vision/detections.hpp"
#include "omniseer/vision/types.hpp"

namespace omniseer_vision_bridge
{
namespace
{
constexpr unsigned kEvidenceSchemaVersion = 1;
constexpr uint64_t kNsPerSec = 1000000000ULL;

struct JpegErrorManager
{
  jpeg_error_mgr pub{};
  jmp_buf setjmp_buffer{};
};

void jpeg_error_exit(j_common_ptr cinfo)
{
  auto * err = reinterpret_cast<JpegErrorManager *>(cinfo->err);
  longjmp(err->setjmp_buffer, 1);
}

uint64_t mb_to_bytes(int64_t value) noexcept
{
  if (value <= 0) {
    return 0;
  }
  const auto mb = static_cast<uint64_t>(value);
  if (mb > (std::numeric_limits<uint64_t>::max() / (1024ULL * 1024ULL))) {
    return std::numeric_limits<uint64_t>::max();
  }
  return mb * 1024ULL * 1024ULL;
}

uint64_t interval_ns(double interval_sec)
{
  if (interval_sec <= 0.0) {
    throw std::invalid_argument("evidence.interval_sec must be > 0");
  }
  const double ns = interval_sec * static_cast<double>(kNsPerSec);
  if (ns > static_cast<double>(std::numeric_limits<uint64_t>::max())) {
    throw std::invalid_argument("evidence.interval_sec is too large");
  }
  return std::max<uint64_t>(1, static_cast<uint64_t>(ns));
}

int checked_quality(int64_t quality)
{
  if (quality < 1 || quality > 100) {
    throw std::invalid_argument("evidence.jpeg_quality must be in [1, 100]");
  }
  return static_cast<int>(quality);
}

void dmabuf_sync(int fd, uint64_t flags) noexcept
{
  dma_buf_sync sync{};
  sync.flags = flags;
  (void) ::ioctl(fd, DMA_BUF_IOCTL_SYNC, &sync);
}

std::string json_escape(const std::string & value)
{
  std::string out;
  out.reserve(value.size() + 8);
  for (const char ch : value) {
    switch (ch) {
      case '\\':
        out += "\\\\";
        break;
      case '"':
        out += "\\\"";
        break;
      case '\n':
        out += "\\n";
        break;
      case '\r':
        out += "\\r";
        break;
      case '\t':
        out += "\\t";
        break;
      default:
        out += ch;
        break;
    }
  }
  return out;
}

bool write_jpeg(
  const std::filesystem::path & path,
  const std::vector<uint8_t> & rgb,
  int width,
  int height,
  int quality) noexcept
{
  FILE * file = std::fopen(path.c_str(), "wb");
  if (file == nullptr) {
    return false;
  }

  jpeg_compress_struct cinfo{};
  JpegErrorManager jerr{};
  cinfo.err = jpeg_std_error(&jerr.pub);
  jerr.pub.error_exit = jpeg_error_exit;
  if (setjmp(jerr.setjmp_buffer)) {
    jpeg_destroy_compress(&cinfo);
    std::fclose(file);
    return false;
  }
  jpeg_create_compress(&cinfo);
  jpeg_stdio_dest(&cinfo, file);

  cinfo.image_width = static_cast<JDIMENSION>(width);
  cinfo.image_height = static_cast<JDIMENSION>(height);
  cinfo.input_components = 3;
  cinfo.in_color_space = JCS_RGB;

  jpeg_set_defaults(&cinfo);
  jpeg_set_quality(&cinfo, quality, TRUE);
  jpeg_start_compress(&cinfo, TRUE);

  const auto row_stride = static_cast<std::size_t>(width) * 3u;
  while (cinfo.next_scanline < cinfo.image_height) {
    JSAMPROW row_pointer[1];
    row_pointer[0] = const_cast<JSAMPROW>(
      rgb.data() + static_cast<std::size_t>(cinfo.next_scanline) * row_stride);
    (void) jpeg_write_scanlines(&cinfo, row_pointer, 1);
  }

  jpeg_finish_compress(&cinfo);
  jpeg_destroy_compress(&cinfo);
  std::fclose(file);
  return true;
}
} // namespace

struct EvidenceFrameSink::Impl
{
  struct DetectionRecord
  {
    uint16_t class_id{0};
    std::string class_name{};
    float score{0.0F};
    float x1{0.0F};
    float y1{0.0F};
    float x2{0.0F};
    float y2{0.0F};
  };

  struct Item
  {
    uint64_t frame_id{0};
    uint64_t sequence{0};
    uint64_t capture_ts_real_ns{0};
    int model_width{0};
    int model_height{0};
    int source_width{0};
    int source_height{0};
    float remap_scale{0.0F};
    int remap_pad_x{0};
    int remap_pad_y{0};
    int remap_resized_w{0};
    int remap_resized_h{0};
    std::vector<uint8_t> rgb{};
    std::vector<DetectionRecord> detections{};
  };

  explicit Impl(EvidenceFrameSinkConfig config_in)
  : config(std::move(config_in)),
    sample_interval_ns(interval_ns(config.interval_sec)),
    jpeg_quality(checked_quality(config.jpeg_quality)),
    storage_budget_bytes(mb_to_bytes(config.storage_budget_mb)),
    min_free_bytes(mb_to_bytes(config.min_free_mb))
  {
    if (config.evidence_dir.empty()) {
      throw std::invalid_argument("evidence.dir must not be empty");
    }
    if (config.queue_capacity == 0) {
      throw std::invalid_argument("evidence queue capacity must be > 0");
    }

    evidence_dir = std::filesystem::path(config.evidence_dir);
    frames_dir = evidence_dir / "frames";
    std::filesystem::create_directories(frames_dir);

    const auto relative_base = evidence_dir.filename().empty() ? std::string("evidence") :
      evidence_dir.filename().string();
    relative_frames_dir = relative_base + "/frames";

    metadata.open(evidence_dir / "evidence.jsonl", std::ios::out | std::ios::trunc);
    if (!metadata.is_open()) {
      throw std::runtime_error("EvidenceFrameSink: failed to open evidence.jsonl");
    }

    worker = std::thread([this]() {run();});
  }

  ~Impl()
  {
    stop_requested.store(true, std::memory_order_release);
    queue_cv.notify_one();
    if (worker.joinable()) {
      worker.join();
    }
    metadata.flush();
    metadata.close();
  }

  void publish(
    const omniseer::vision::ImageBuffer & image,
    const omniseer::vision::DetectionsFrame & detections,
    const omniseer::vision::PipelineRemapConfig & remap) noexcept
  {
    if (stop_requested.load(std::memory_order_acquire) || stopped.load(std::memory_order_acquire)) {
      return;
    }
    if (!should_sample(detections.capture_ts_real_ns)) {
      return;
    }

    try {
      auto item = copy_item(image, detections, remap);
      if (!item.has_value) {
        dropped_count.fetch_add(1, std::memory_order_relaxed);
        return;
      }

      std::unique_lock<std::mutex> lock(queue_mutex, std::try_to_lock);
      if (!lock.owns_lock()) {
        dropped_count.fetch_add(1, std::memory_order_relaxed);
        return;
      }
      if (queue.size() >= config.queue_capacity) {
        dropped_count.fetch_add(1, std::memory_order_relaxed);
        return;
      }
      queue.emplace_back(std::move(item.value));
      enqueued_count.fetch_add(1, std::memory_order_relaxed);
      lock.unlock();
      queue_cv.notify_one();
    } catch (...) {
      dropped_count.fetch_add(1, std::memory_order_relaxed);
    }
  }

  EvidenceFrameSinkSnapshot snapshot() const
  {
    EvidenceFrameSinkSnapshot out{};
    out.enqueued_count = enqueued_count.load(std::memory_order_relaxed);
    out.written_count = written_count.load(std::memory_order_relaxed);
    out.dropped_count = dropped_count.load(std::memory_order_relaxed);
    out.stopped = stopped.load(std::memory_order_acquire);
    std::lock_guard<std::mutex> lock(stopped_reason_mutex);
    out.stopped_reason = stopped_reason;
    return out;
  }

  struct CopyResult
  {
    bool has_value{false};
    Item value{};
  };

  bool should_sample(uint64_t capture_ts_real_ns) noexcept
  {
    const uint64_t ts = capture_ts_real_ns;
    if (ts == 0) {
      return false;
    }
    if (last_sample_ts_real_ns == 0 || ts >= last_sample_ts_real_ns + sample_interval_ns) {
      last_sample_ts_real_ns = ts;
      return true;
    }
    return false;
  }

  CopyResult copy_item(
    const omniseer::vision::ImageBuffer & image,
    const omniseer::vision::DetectionsFrame & frame,
    const omniseer::vision::PipelineRemapConfig & remap)
  {
    if (image.fmt != omniseer::vision::PixelFormat::RGB888 || image.num_planes == 0 ||
      image.size.w <= 0 || image.size.h <= 0)
    {
      return {};
    }
    const auto & plane = image.planes[0];
    if (plane.fd < 0 || plane.stride < static_cast<uint32_t>(image.size.w * 3) ||
      plane.alloc_size == 0)
    {
      return {};
    }

    const size_t map_length = (image.total_alloc_size != 0) ? image.total_alloc_size :
      (static_cast<size_t>(plane.offset) + plane.alloc_size);
    if (map_length == 0 || static_cast<size_t>(plane.offset) >= map_length) {
      return {};
    }

    const size_t row_bytes = static_cast<size_t>(image.size.w) * 3u;
    const size_t needed = static_cast<size_t>(plane.offset) +
      static_cast<size_t>(plane.stride) * static_cast<size_t>(image.size.h);
    if (needed > map_length) {
      return {};
    }

    dmabuf_sync(plane.fd, DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ);
    void * mapped = ::mmap(nullptr, map_length, PROT_READ, MAP_SHARED, plane.fd, 0);
    if (mapped == MAP_FAILED) {
      dmabuf_sync(plane.fd, DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ);
      return {};
    }

    Item item{};
    item.frame_id = frame.frame_id;
    item.sequence = frame.sequence;
    item.capture_ts_real_ns = frame.capture_ts_real_ns;
    item.model_width = image.size.w;
    item.model_height = image.size.h;
    item.source_width = frame.source_size.w;
    item.source_height = frame.source_size.h;
    item.remap_scale = remap.scale;
    item.remap_pad_x = remap.pad_x;
    item.remap_pad_y = remap.pad_y;
    item.remap_resized_w = remap.resized_w;
    item.remap_resized_h = remap.resized_h;
    item.rgb.resize(row_bytes * static_cast<size_t>(image.size.h));

    const auto * base = static_cast<const uint8_t *>(mapped) + plane.offset;
    for (int y = 0; y < image.size.h; ++y) {
      const auto * src = base + static_cast<size_t>(y) * static_cast<size_t>(plane.stride);
      auto * dst = item.rgb.data() + static_cast<size_t>(y) * row_bytes;
      std::memcpy(dst, src, row_bytes);
    }

    ::munmap(mapped, map_length);
    dmabuf_sync(plane.fd, DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ);

    item.detections.reserve(frame.count);
    for (uint32_t i = 0; i < frame.count; ++i) {
      const auto & det = frame.detections[i];
      DetectionRecord out{};
      out.class_id = det.class_id;
      if (det.class_id < config.class_names.size()) {
        out.class_name = config.class_names[det.class_id];
      }
      out.score = det.score;
      out.x1 = det.x1;
      out.y1 = det.y1;
      out.x2 = det.x2;
      out.y2 = det.y2;
      item.detections.emplace_back(std::move(out));
    }

    return CopyResult{true, std::move(item)};
  }

  void run()
  {
    for (;; ) {
      auto item = wait_for_item();
      if (!item.has_value) {
        return;
      }
      write_item(item.value);
    }
  }

  CopyResult wait_for_item()
  {
    std::unique_lock<std::mutex> lock(queue_mutex);
    queue_cv.wait(lock, [this]() {
        return stop_requested.load(std::memory_order_acquire) || !queue.empty();
    });
    if (queue.empty()) {
      return {};
    }
    auto item = std::move(queue.front());
    queue.pop_front();
    return CopyResult{true, std::move(item)};
  }

  void write_item(const Item & item) noexcept
  {
    if (!storage_available()) {
      dropped_count.fetch_add(1, std::memory_order_relaxed);
      return;
    }

    const std::string filename = "frame_" + std::to_string(item.frame_id) + ".jpg";
    const std::filesystem::path image_path = frames_dir / filename;
    if (!write_jpeg(image_path, item.rgb, item.model_width, item.model_height, jpeg_quality)) {
      dropped_count.fetch_add(1, std::memory_order_relaxed);
      return;
    }

    std::error_code ec;
    const auto size = std::filesystem::file_size(image_path, ec);
    if (!ec) {
      written_bytes.fetch_add(size, std::memory_order_relaxed);
    }
    write_metadata(item, relative_frames_dir + "/" + filename);
    written_count.fetch_add(1, std::memory_order_relaxed);
  }

  bool storage_available() noexcept
  {
    const uint64_t bytes = written_bytes.load(std::memory_order_relaxed);
    if (bytes >= storage_budget_bytes) {
      stop_with_reason("storage_budget_exceeded");
      return false;
    }

    std::error_code ec;
    const auto info = std::filesystem::space(evidence_dir, ec);
    if (!ec && min_free_bytes != 0 && info.available < min_free_bytes) {
      stop_with_reason("min_free_disk_exceeded");
      return false;
    }
    return true;
  }

  void stop_with_reason(const std::string & reason) noexcept
  {
    bool expected = false;
    if (!stopped.compare_exchange_strong(expected, true, std::memory_order_acq_rel)) {
      return;
    }
    {
      std::lock_guard<std::mutex> lock(stopped_reason_mutex);
      stopped_reason = reason;
    }
    metadata << "{\"schema_version\":" << kEvidenceSchemaVersion
             << ",\"artifact_type\":\"evidence_status\",\"stopped_reason\":\""
             << json_escape(reason) << "\",\"dropped_count\":"
             << dropped_count.load(std::memory_order_relaxed) << "}\n";
    metadata.flush();
  }

  void write_metadata(const Item & item, const std::string & image_path)
  {
    metadata << "{\"schema_version\":" << kEvidenceSchemaVersion
             << ",\"artifact_type\":\"sampled_frame\""
             << ",\"image_path\":\"" << json_escape(image_path) << "\""
             << ",\"jpeg_quality\":" << jpeg_quality
             << ",\"capture_reason\":\"periodic\""
             << ",\"frame_id\":" << item.frame_id
             << ",\"sequence\":" << item.sequence
             << ",\"capture_ts_real_ns\":" << item.capture_ts_real_ns
             << ",\"model_input\":{\"width\":" << item.model_width
             << ",\"height\":" << item.model_height << "}"
             << ",\"source_image\":{\"width\":" << item.source_width
             << ",\"height\":" << item.source_height << "}"
             << ",\"remap\":{\"scale\":" << item.remap_scale
             << ",\"pad_x\":" << item.remap_pad_x
             << ",\"pad_y\":" << item.remap_pad_y
             << ",\"resized_w\":" << item.remap_resized_w
             << ",\"resized_h\":" << item.remap_resized_h << "}"
             << ",\"detections\":[";

    for (std::size_t i = 0; i < item.detections.size(); ++i) {
      const auto & det = item.detections[i];
      if (i != 0) {
        metadata << ",";
      }
      metadata << "{\"class_id\":" << det.class_id
               << ",\"class_name\":\"" << json_escape(det.class_name) << "\""
               << ",\"score\":" << det.score
               << ",\"bbox\":{\"x1\":" << det.x1
               << ",\"y1\":" << det.y1
               << ",\"x2\":" << det.x2
               << ",\"y2\":" << det.y2 << "}}";
    }
    metadata << "]}\n";
    metadata.flush();
  }

  EvidenceFrameSinkConfig config{};
  std::filesystem::path evidence_dir{};
  std::filesystem::path frames_dir{};
  std::string relative_frames_dir{};
  std::ofstream metadata{};
  uint64_t sample_interval_ns{0};
  int jpeg_quality{85};
  uint64_t storage_budget_bytes{0};
  uint64_t min_free_bytes{0};
  uint64_t last_sample_ts_real_ns{0};

  mutable std::mutex queue_mutex{};
  std::condition_variable queue_cv{};
  std::deque<Item> queue{};
  std::thread worker{};
  std::atomic<bool> stop_requested{false};
  std::atomic<bool> stopped{false};
  mutable std::mutex stopped_reason_mutex{};
  std::string stopped_reason{};
  std::atomic<uint64_t> enqueued_count{0};
  std::atomic<uint64_t> written_count{0};
  std::atomic<uint64_t> dropped_count{0};
  std::atomic<uint64_t> written_bytes{0};
};

EvidenceFrameSink::EvidenceFrameSink(EvidenceFrameSinkConfig config)
: _impl(std::make_unique<Impl>(std::move(config)))
{
}

EvidenceFrameSink::~EvidenceFrameSink() = default;

void EvidenceFrameSink::publish(
  const omniseer::vision::ImageBuffer & image,
  const omniseer::vision::DetectionsFrame & detections,
  const omniseer::vision::PipelineRemapConfig & remap) noexcept
{
  _impl->publish(image, detections, remap);
}

EvidenceFrameSinkSnapshot EvidenceFrameSink::snapshot() const
{
  return _impl->snapshot();
}
} // namespace omniseer_vision_bridge
