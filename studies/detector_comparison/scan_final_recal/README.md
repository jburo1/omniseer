# Six-model YOLO-World comparison on a recalibrated 360° scene

**Engineering question:** with one repaired, frame-aligned 360° source scene,
how do the six final YOLO-World RKNN detector configurations differ in
presence/visibility behavior under the same vocabulary and post-processing?

[![Six-panel controlled replay poster](evidence/controlled_replay_poster.jpg)](evidence/controlled_replay_2x3.mp4)

The linked [controlled 2×3 comparison video](evidence/controlled_replay_2x3.mp4)
replays exactly the same 1,222 repaired source frames through each detector. Its
six panels are, in order: v2-S FP, v2-S INT8, v2-M FP, v2-M INT8, v2-L FP, and
v2-L Hybrid. The poster is a representative frame from that video; it shows no
people or identifying information.

The principal result is a 2,367 / 5,679 (41.7%) visible-frame detection rate for
v2-M FP, narrowly ahead of v2-L FP at 2,363 / 5,679 (41.6%). v2-L Hybrid is the
lowest at 1,440 / 5,679 (25.4%), but is the only configuration with no
absent-dog detections. Full metrics and limits are in [results.md](results.md).

## What is being compared

The controlled replay holds source, repair, class vocabulary, score threshold
(0.25), NMS IoU threshold (0.45), and maximum detections (100) fixed. Only the
detector configuration changes. The preserved
[replay provenance](evidence/replay_provenance.json), six
[replay JSONLs](evidence/replay_jsonl/), [visibility annotations](visibility.txt),
and [comparison report](evidence/comparison_report.html) make the published
metrics independently recomputable.

The [six-run physical presentation grid](evidence/physical_trials_grid_2x3.mp4)
is supporting evidence from six independent physical ROCK 5B+ runs. It is not a
frame-aligned substitute for the controlled replay: panels start at their own
run starts and the grid ends at the shortest overlay. Their public-safe,
hash-bound identities are in [physical_runs.yaml](physical_runs.yaml).

## Evidence boundary and identity

`scan_final_recal` is the public study ID. The authoritative local RunBundle is
retained, ignored, at `runs/imported/scan_final_recal` (also recorded in
[runs.yaml](runs.yaml)); its manifest deliberately retains the legacy recorded
`run_id: scan_final`. That legacy ID identifies the original recording and is
not the public study name.

The original provenance fields are preserved verbatim. Consequently, its
RunBundle-relative paths resolve only from the locally retained bundle, not from
this tracked study directory. The source transport stream is intentionally not
published; its SHA-256 is in [checksums.sha256](checksums.sha256). The tracked
JSONLs and annotations permit metric recomputation; rerendering the comparison
video additionally requires that locally retained `video/source.ts`.

This is one-scene presence/visibility evidence, not mAP, bounding-box recall,
or evidence of general detector accuracy. No inference or video rendering was
rerun to publish this study.
