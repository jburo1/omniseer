#include <atomic>
#include <cerrno>
#include <csignal>
#include <cstdio>
#include <fcntl.h>
#include <gst/gst.h>
#include <string>
#include <unistd.h>
#include <utility>

namespace
{
  std::atomic_bool stop_requested{false};

  void request_stop(int)
  {
    stop_requested = true;
  }

  bool write_timing(const std::string& path, GstClockTime video_pts)
  {
    timespec realtime{};
    if (clock_gettime(CLOCK_REALTIME, &realtime) != 0)
    {
      return false;
    }
    const auto realtime_ns = static_cast<long long>(realtime.tv_sec) * 1'000'000'000LL +
                             static_cast<long long>(realtime.tv_nsec);
    const std::string temporary_path = path + ".tmp";
    const std::string content        = "{\n  \"anchor_video_time_ns\": " +
                                std::to_string(static_cast<unsigned long long>(video_pts)) +
                                ",\n  \"anchor_robot_time_ns\": " + std::to_string(realtime_ns) +
                                "\n}\n";
    const int fd = open(temporary_path.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (fd < 0)
    {
      return false;
    }
    const auto written      = write(fd, content.data(), content.size());
    const bool complete     = written == static_cast<ssize_t>(content.size()) && fsync(fd) == 0;
    const int  close_result = close(fd);
    return complete && close_result == 0 && rename(temporary_path.c_str(), path.c_str()) == 0;
  }

  GstPadProbeReturn first_recorded_buffer(GstPad*, GstPadProbeInfo* info, gpointer user_data)
  {
    auto*      timing = static_cast<std::pair<std::string, std::atomic_bool*>*>(user_data);
    GstBuffer* buffer = GST_PAD_PROBE_INFO_BUFFER(info);
    if (buffer == nullptr || !GST_BUFFER_PTS_IS_VALID(buffer) || timing->second->exchange(true))
    {
      return GST_PAD_PROBE_OK;
    }
    if (!write_timing(timing->first, GST_BUFFER_PTS(buffer)))
    {
      g_printerr("failed to write first-buffer video timing\n");
    }
    return GST_PAD_PROBE_OK;
  }
} // namespace

int main(int argc, char** argv)
{
  std::string timing_path;
  std::string pipeline_description;
  for (int index = 1; index + 1 < argc; index += 2)
  {
    const std::string option = argv[index];
    if (option == "--timing-path")
    {
      timing_path = argv[index + 1];
    }
    else if (option == "--pipeline")
    {
      pipeline_description = argv[index + 1];
    }
    else
    {
      g_printerr("unknown option: %s\n", option.c_str());
      return 64;
    }
  }
  if (timing_path.empty() || pipeline_description.empty())
  {
    g_printerr("--timing-path and --pipeline are required\n");
    return 64;
  }

  gst_init(&argc, &argv);
  GError*     error    = nullptr;
  GstElement* pipeline = gst_parse_launch(pipeline_description.c_str(), &error);
  if (pipeline == nullptr)
  {
    g_printerr("failed to parse preview pipeline: %s\n",
               error == nullptr ? "unknown error" : error->message);
    g_clear_error(&error);
    return 1;
  }
  GstElement* timing_probe = gst_bin_get_by_name(GST_BIN(pipeline), "timing_probe");
  if (timing_probe == nullptr)
  {
    g_printerr("preview pipeline has no timing probe\n");
    gst_object_unref(pipeline);
    return 1;
  }
  GstPad*          timing_pad = gst_element_get_static_pad(timing_probe, "src");
  std::atomic_bool timing_written{false};
  auto             timing = std::make_pair(timing_path, &timing_written);
  gst_pad_add_probe(timing_pad, GST_PAD_PROBE_TYPE_BUFFER, first_recorded_buffer, &timing, nullptr);
  gst_object_unref(timing_pad);
  gst_object_unref(timing_probe);

  std::signal(SIGINT, request_stop);
  std::signal(SIGTERM, request_stop);
  gst_element_set_state(pipeline, GST_STATE_PLAYING);
  GstBus* bus    = gst_element_get_bus(pipeline);
  int     result = 0;
  while (!stop_requested)
  {
    GstMessage* message = gst_bus_timed_pop_filtered(bus, 100 * GST_MSECOND,
                                                     static_cast<GstMessageType>(GST_MESSAGE_ERROR |
                                                                                 GST_MESSAGE_EOS));
    if (message == nullptr)
    {
      continue;
    }
    if (GST_MESSAGE_TYPE(message) == GST_MESSAGE_ERROR)
    {
      GError* pipeline_error = nullptr;
      gchar*  debug          = nullptr;
      gst_message_parse_error(message, &pipeline_error, &debug);
      g_printerr("preview pipeline error: %s\n", pipeline_error->message);
      g_clear_error(&pipeline_error);
      g_free(debug);
      result = 1;
    }
    gst_message_unref(message);
    break;
  }
  if (stop_requested)
  {
    gst_element_send_event(pipeline, gst_event_new_eos());
    GstMessage* message = gst_bus_timed_pop_filtered(bus, 5 * GST_SECOND,
                                                     static_cast<GstMessageType>(GST_MESSAGE_ERROR |
                                                                                 GST_MESSAGE_EOS));
    if (message != nullptr)
    {
      if (GST_MESSAGE_TYPE(message) == GST_MESSAGE_ERROR)
      {
        result = 1;
      }
      gst_message_unref(message);
    }
  }
  gst_element_set_state(pipeline, GST_STATE_NULL);
  gst_object_unref(bus);
  gst_object_unref(pipeline);
  return result;
}
