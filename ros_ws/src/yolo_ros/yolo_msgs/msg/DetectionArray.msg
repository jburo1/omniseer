# represents all YOLO detections in one frame

std_msgs/Header header
# Native producer metadata. Zero means unknown for non-native YOLO publishers.
uint64 native_frame_id
uint64 native_sequence
yolo_msgs/Detection[] detections
