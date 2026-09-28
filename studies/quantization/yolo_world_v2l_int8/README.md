# YOLO-World v2-L INT8 quantization: retained evidence

**Canonical engineering write-up:** [INT8 Quantization Failure Analysis](https://jburo1.github.io/omniseer/verification/int8-quantization-failure/).
That GitHub Pages page is the authoritative engineering narrative, results,
interpretation, decision, and limitations. This repository directory is the
retained evidence and reproducibility index for that investigation.

## Directory contents

This study preserves the RK3588 evaluation artifacts for recalibrated v2-L
INT8 and the TD01 mixed-precision diagnostic probe, including the final frozen
300-frame replay and projection-context diagnostics.

[![Side-by-side RK3588 replay: no detections from recalibrated INT8 at left; TD01 hybrid detections at right](../../../docs/assets/evidence/v2l-int8-quantization-comparison.webp)](evidence/scan_final_v2l_int8_vs_hybrid.mp4)

- Retained [INT8 versus TD01 RK3588 replay](evidence/scan_final_v2l_int8_vs_hybrid.mp4)
  and [provenance](evidence/scan_final_v2l_int8_vs_hybrid.provenance.md).
- Final evaluation [300-frame summary](evidence/final_multiframe_rk3588/results/summary.md),
  machine-readable [result/provenance bundle](evidence/final_multiframe_rk3588/results/summary.json),
  and frozen [frame manifest](evidence/final_multiframe_rk3588/manifest.json).
- [Projection-context RK3588 metrics](evidence/projection_context_rk3588/metrics.json)
  and the complete [evidence inventory](evidence/EVIDENCE_INDEX.md).
- [Detailed technical appendix](../../../docs/perception/int8-quantization-investigation.md)
  for conversion inputs and hashes, host diagnostics, and artifact-retention
  details.

## Reproducibility boundary

The frozen source stream and extracted PNG inputs are intentionally
non-retained external inputs, identified by hashes and the manifest; an
independent rerun requires access to them. Consult the
[canonical write-up](https://jburo1.github.io/omniseer/verification/int8-quantization-failure/)
for the experimental conclusions, decision, and limitations.
