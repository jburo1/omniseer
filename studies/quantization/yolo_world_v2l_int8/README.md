# YOLO-World v2-L INT8 quantization study

This study preserves the compact canonical evidence for the 2026-08-29--31
RKNN investigation of the v2-L recalibrated INT8 failure and the TD01
mixed-precision diagnostic mitigation. It does not make TD01 a validated
production replacement.

Canonical retained evidence is in [`evidence/`](evidence/), including the
one-frame RK3588 collection, the final 300-frame RK3588 evaluation, and the
projection-context hardware metrics. The evidence inventory is
[`evidence/EVIDENCE_INDEX.md`](evidence/EVIDENCE_INDEX.md).

Reproduction programs remain in `tools/model/`, notably
`reproduce_cls80_exmatmul.py` and `reproduce_cls80_projection_context.py`.
Host Toolkit output and model builds remain generated local material under
ignored `artifacts/quant_analysis/` and `artifacts/models/`.

For conclusions, conversion inputs, metrics, and limitations, see
[`docs/perception/int8-quantization-investigation.md`](../../../docs/perception/int8-quantization-investigation.md).
