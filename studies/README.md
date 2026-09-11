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

## Canonical studies

- [v2m_int8_target_acquisition](autonomy/v2m_int8_target_acquisition/README.md):
  one complete, publicly inspectable ROCK 5B+ bounded target-acquisition
  RunBundle.

- [scan_final_recal](detector_comparison/scan_final_recal/README.md): canonical
  six-detector controlled comparison with six independent physical runs.

- [yolo_world_v2l_int8](quantization/yolo_world_v2l_int8/README.md): v2-L INT8
  quantization failure investigation and TD01 mixed-precision mitigation.
