# Scan final recalibration: detector comparison results

This study compares six YOLO-World v2 RKNN configurations: v2-S FP, v2-S
INT8, v2-M FP, v2-M INT8, v2-L FP, and v2-L Hybrid. Immutable RunBundle IDs
are recorded in [runs.yaml](runs.yaml).

## Method

The completed controlled replay uses the reference bundle's immutable 1,222
frame 360-degree source scene. It applies the same corrected source frame,
class vocabulary, score threshold (0.25), NMS IoU threshold (0.45), and
maximum detections (100) to each detector; only detector configuration
changes. The six physical trial bundles provide the end-to-end trial context,
while the replay makes the visual comparison frame-aligned. Its completed
`latest_task` comparison video and report remain in the reference RunBundle;
they were not regenerated for this record.

## Visibility metrics

The current `visibility.txt` supplies 5,679 inclusive visible-frame
annotations, exactly matching the 1,222-frame replay's 0–1221 index domain.
The aggregate detection rates were 37.4% (v2-S FP), 32.6% (v2-S INT8), 41.7%
(v2-M FP), 36.2% (v2-M INT8), 41.6% (v2-L FP), and 25.4% (v2-L Hybrid). All
configurations detected the annotated person on all 293 visible frames. The
report also records explicit-absence false detections:
the absent `dog` was emitted on 34, 6, 45, 23, 33, and 0 frames respectively;
the absent `cat` was never emitted.

## Observations

- v2-M FP had the highest aggregate visible-frame rate (41.7%), narrowly
  ahead of v2-L FP (41.6%); both exceeded their INT8 counterparts.
- v2-L FP was notably stronger for bottle (80.5%), book (31.9%), scissors
  (94.3%), and tripod (100.0%), but nearly missed shoe (1.1%) and was weaker
  on subwoofer (32.8%).
- The v2-S configurations were strongest on shoe (100.0% each); v2-S INT8
  also led beer mug (73.1%), while v2-S FP led subwoofer (98.7%).
- v2-L Hybrid had the lowest aggregate rate (25.4%) and missed several
  annotated classes that other configurations detected, though it produced no
  absent-dog detections.

These are scene-presence/visibility results from one controlled scene, not
mAP, bounding-box recall, or a measure of general detector accuracy. They do
not establish performance on other scenes, object instances, viewpoints,
lighting conditions, or vocabularies. Labels absent from the selected
comparison vocabulary were excluded rather than inferred.
