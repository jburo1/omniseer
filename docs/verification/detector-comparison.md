# Detector comparison: recalibrated 360° scene

The canonical [six-model YOLO-World comparison](../../studies/detector_comparison/scan_final_recal/README.md)
publishes a controlled 2×3 replay, six replay JSONLs, visibility annotations,
repaired provenance, a comparison report, and a compact summary of six
independent physical ROCK 5B+ runs.

The controlled replay holds the repaired source frames, vocabulary, and
post-processing fixed while changing only the detector configuration. Its leading
visible-frame result is v2-M FP at 2,367 / 5,679 (41.7%), narrowly ahead of v2-L
FP at 41.6%. This is one-scene presence/visibility evidence, not mAP,
bounding-box recall, latency evidence, or general detector accuracy.

The tracked comparison video is target-hardware-derived public evidence. The
complete RunBundle, source transport stream, and raw physical manifests remain
ignored local evidence at `runs/imported/scan_final_recal`. JSONLs plus visibility
annotations are sufficient to recompute the published metrics; rerendering the
video also requires the retained source stream. The six physical manifests support
only the recorded facts published in the compact summary; they are not a claim
that the controlled panels are simultaneous physical trials.
