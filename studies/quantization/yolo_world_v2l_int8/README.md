# YOLO-World v2-L INT8 quantization: RK3588 investigation

## The question

Could a recalibrated INT8 YOLO-World v2-L model preserve useful
open-vocabulary detection behavior on RK3588, and, if not, could a targeted
mixed-precision layout identify a practical recovery path?

The answer for this model and conversion contract was no. Recalibrated INT8
collapsed before postprocessing: it produced no detections in the final
hardware replay. TD01, a targeted mixed-precision hybrid, was useful for
localizing and partially recovering the failure, but it was not
FP-equivalent and is **not a validated production replacement**.

## Final 300-frame RK3588 result

The canonical evaluation replays 300 frozen representative frames through the
existing detector path on RK3588. FP is a behavioral reference, not ground
truth; retention means same-class, one-to-one matches at IoU >= 0.50.

| Model | Active frames | Detections | FP detections retained |
| --- | ---: | ---: | ---: |
| v2-L FP | 300 | 1,301 | — |
| v2-L recalibrated INT8 | 0 | 0 | 0 / 1,301 (0.0%) |
| v2-L TD01 hybrid | 295 | 1,030 | 789 / 1,301 (60.6%) |

TD01 recovered substantial behavior, including strong FP-relative retention
for person, potted plant, tripod, cup, garbage can, and bottle. It also left
severe degradation for classes including handbag, bag, cellphone, book,
subwoofer, and desk. Its 60.6% retention is therefore diagnostic evidence of
partial recovery, not a deployment-quality result.

## What the investigation localized

The failure is in the classification path before YOLO postprocessing. The
narrow exMatMul micrograph stayed healthy, but the expanded lowered
projection/exMatMul context reproduced the RK3588 failure for the failed
hybrid arrangement: its output became exactly constant. FP16 and
healthy-hybrid controls remained non-constant. This localizes the issue to
the required projection context and precision arrangement, rather than an
isolated matrix multiply alone. The exact Toolkit2/backend mechanism remains
unresolved.

## Engineering decision

- v2-L FP remains the quality reference.
- Recalibrated v2-L INT8 is unsuitable.
- TD01 is an experimental, diagnostic mixed-precision mitigation only—not a
  validated production replacement.
- The root-cause investigation is closed unless a vendor bug report or a
  production-quality v2-L mixed-precision deployment becomes necessary.

## Evidence and reproducibility

- Watch the retained side-by-side [INT8 versus TD01 RK3588 replay](evidence/scan_final_v2l_int8_vs_hybrid.mp4), with its [provenance](evidence/scan_final_v2l_int8_vs_hybrid.provenance.md). This is the strongest retained visual comparison.
- Inspect the canonical [300-frame summary](evidence/final_multiframe_rk3588/results/summary.md), machine-readable [result/provenance bundle](evidence/final_multiframe_rk3588/results/summary.json), and frozen [frame manifest](evidence/final_multiframe_rk3588/manifest.json).
- Review the retained [projection-context RK3588 metrics](evidence/projection_context_rk3588/metrics.json) and complete [evidence inventory](evidence/EVIDENCE_INDEX.md).
- For methodology, layer-level diagnostics, conversion inputs and hashes, and the full limitations discussion, read the [detailed investigation](../../../docs/perception/int8-quantization-investigation.md).

## Evidence boundaries

These results are a fixed 300-frame RK3588 comparison with the recorded
runtime, models, thresholds, and replay path. They are not ground-truth
precision or recall, broad deployment validation, isolated inference-latency
measurements, or proof of a proprietary Toolkit2/backend root cause. Reported
replay times include initialization, decode, CPU preprocessing, inference,
postprocessing, and serialization. The frozen source stream and extracted
PNG frames are identified, non-retained external inputs recorded by hashes and
the manifest; an independent rerun requires access to the original source
artifacts.
