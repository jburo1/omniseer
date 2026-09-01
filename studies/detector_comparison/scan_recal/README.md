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
