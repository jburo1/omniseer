# Scan recalibration detector comparison

This study evaluates six YOLO-World v2 detector configurations against the
same controlled 360° source scene. The scene is recorded once in the source
RunBundle; it is a bounded environment scan, not six independently captured
scenes. Each physical trial RunBundle supplies one model configuration, while
the reference RunBundle supplies the named replay that aligns source frames
across the comparison.

The six-model comparison is intended to make model-family and precision
differences reviewable under a fixed scene and vocabulary. It is a
scene-presence/visibility study, not mAP, bounding-box recall, or a claim of
general detector accuracy.

`visibility.txt` contains manual annotations, one per line:

```text
<class> start <inclusive-source-frame>
<class> end <inclusive-source-frame>
<class> absent
```

Classes may have multiple visible intervals. The comparison report records
malformed or vocabulary-mismatched lines rather than inferring a range.

To reproduce the review surface, retain the source RunBundle and the six trial
RunBundles under ignored `runs/`, generate the named comparison replay from
the source RunBundle, then run `scripts/omni runs comparison-report` with the
trial bundles and `--objects studies/detector_comparison/scan_recal/visibility.txt`.
The report is derived evidence under the reference RunBundle and does not alter
the raw source or trial evidence.

## Scene 1 presentation grid

`evidence/scene_1_overlay_grid_2x3.mp4` is a presentation derivative assembled
from the already-rendered corrected overlay videos in six independently
recorded `scene_1` RunBundles. It is distinct from the fixed-source replay
comparison above: all panels begin at their respective run starts and the grid
ends at the shortest input rather than freezing or looping a panel.

Panel order is `v2-S FP | v2-M FP | v2-L FP` on the top row and `v2-S INT8 |
v2-M INT8 | v2-L Hybrid` on the bottom row. The corresponding manifest-verified
RunBundle IDs are `v2s_fp_scene_1`, `v2m_fp_scene_1`, `v2l_fp_scene_1`,
`v2s_int8_scene_1`, `v2m_int8_scene_1`, and `v2l_hybrid_scene_1`.

`evidence/scene_1_v2l_int8_vs_hybrid_1x2.mp4` is the corresponding two-panel
derivative for the v2-L INT8 (collapsed) RunBundle `v2l_int8_scene_1` and the
v2-L Hybrid RunBundle `v2l_hybrid_scene_1`. The panels start together and the
video ends at the 32.5-second INT8 input, the shorter of the two overlays.
