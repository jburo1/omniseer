# Studies

`studies/` contains version-controlled study definitions and canonical
evidence. It is the repository location for material needed to identify a
study, interpret its evidence, and reproduce its documented analysis.

Full RunBundles normally live in ignored `runs/`; generated and local working
RunBundles remain there. Selected canonical public evidence RunBundles may be
tracked under their corresponding `studies/` directory. Generated model and
build outputs belong in ignored `artifacts/`. Conclusions and explanatory
material belong in `docs/`. Experiment and reproducer programs belong in
`tools/` or the owning source subsystem.

## Canonical autonomy evidence

[v2m_int8_target_acquisition](autonomy/v2m_int8_target_acquisition/README.md)
is one complete, publicly inspectable ROCK 5B+ bounded-autonomy RunBundle.

## Canonical detector comparison

[scan_final_recal](detector_comparison/scan_final_recal/README.md) is the
canonical six-model YOLO-World comparison on a recalibrated 360° scene.
