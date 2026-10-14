   # Custom indoor detector: COCO-pretrained YOLO11 fine-tuning

   **Status:** experimental design; no dataset release, training, evaluation, or
   YOLO11 RK3588 deployment has been completed by this study.

   ## Objective and hypotheses

   Determine whether a compact COCO-pretrained YOLO11 detector, fine-tuned on
   Omniseer camera imagery, can preserve `person` detection, improve unreliable
   `book` detection, and learn `power drill` for indoor target search. The initial
   application boundary is the existing bounded visual target-acquisition and
   framing behavior, not navigation-based exploration or learned control; see the
   [architecture](../../docs/architecture/overview.md) and
   [target-acquisition evidence](../../docs/verification/target-acquisition.md).

   The available targets are one regular person (the operator), many distinct
   physical books, and one specific physical power drill. Person and drill results
   must establish robustness across unseen recording sessions, viewpoints,
   distances, lighting, backgrounds, placements, and occlusions; generalization to
   other people or drills is outside this study. Books additionally require
   evaluation on physical books excluded from training. Environmental robustness
   and unseen-book generalization are separate claims. The outputs remain class
   labels and boxes: `person` does not identify the operator, `power drill` does
   not identify the particular drill, and `book` does not identify a title or copy.

   The proposed custom label order is `0: person`, `1: book`, `2: power drill`.
   These are study IDs, not COCO IDs or YOLO-World prompt indices. The
   [repository COCO vocabulary](../../config/classes/coco80.txt) contains `person`
   and `book`, but no power-drill category. Ultralytics documents YOLO11 detection
   checkpoints as COCO-pretrained, with 80 output classes; its
   [YOLO11 documentation](https://docs.ultralytics.com/models/yolo11/) and
   [COCO definition](https://github.com/ultralytics/ultralytics/blob/main/ultralytics/cfg/datasets/coco.yaml)
   confirm this distinction. Pretrained YOLO11 cannot acquire a drill output merely
   by renaming a COCO class, filtering predictions, or supplying a text prompt.
   Fine-tuning must configure a three-class detection head and learn from labeled
   drill examples. Reusing pretrained features does not establish pretrained drill
   detection capability; verify which weights transfer when the head changes.

   Hypotheses to test, not established results:

   - **H1 — person robustness:** examples of the regular subject across collection
   conditions preserve person precision, recall, and localization on unseen
   sessions and conditions after adaptation, without requiring unseen people.
   - **H2 — book adaptation:** robot-view examples spanning book size, pose,
   occlusion, and lighting improve book localization and recall over unmodified
   YOLO11 and the selected YOLO-World references at useful precision, both for
   training-seen books in new conditions and for books withheld from training.
   - **H3 — drill robustness:** labeled examples of the specific drill support
   detection in unseen sessions and conditions rather than memorized backgrounds,
   where COCO-pretrained YOLO11 has no supported category. Other drill instances
   are not required to test this application hypothesis.
   - **H4 — deployment:** the selected compact model retains these gains after
   ONNX/RKNN conversion while meeting measured RK3588 freshness and resource
   budgets. Smaller closed-vocabulary inference may reduce work, but neither
   speed nor conversion fidelity is established here.

   ## Existing evidence and reusable infrastructure

   | Repository evidence | Established observation or implementation | Reuse and limitation |
   | --- | --- | --- |
   | [Six-detector study](../detector_comparison/scan_final_recal/README.md), [results](../detector_comparison/scan_final_recal/results.md), [visibility intervals](../detector_comparison/scan_final_recal/visibility.txt), and [replay provenance](../detector_comparison/scan_final_recal/evidence/replay_provenance.json) | Six YOLO-World RKNN configurations evaluated the same 1,222 repaired frames, with 29 prompts, score threshold 0.25, NMS IoU 0.45, and maximum 100 detections. Aggregate visible class-frame coverage ranged from 32.6% to 41.7%. | Reusable regression outputs and presence annotations; one scene, no ground-truth boxes, and no drill evaluation. The source stream is withheld and identified by hash, so a public clone alone cannot rerun its inference. |
   | [Public v2-L FP RunBundle](../autonomy/v2l_fp_target_acquisition/README.md) | One physical ROCK 5B+ run successfully acquired and framed `person`; manifest, raw streams, captured frames, video, telemetry, and provenance are retained. | Useful camera-domain examples and a recording template. Selected captures and detector overlays are not an exhaustively annotated dataset or independent accuracy evidence. This is a separate workload from the controlled scan. |
   | [Quantization study](../quantization/yolo_world_v2l_int8/README.md) and [300-frame summary](../quantization/yolo_world_v2l_int8/evidence/final_multiframe_rk3588/results/summary.md) | Recalibrated v2-L INT8 produced zero detections on that fixed RK3588 replay; TD01 mixed precision retained 789/1,301 FP reference detections (60.6%). | Require conversion parity checks and ground-truth evaluation. FP detections are a behavioral reference, not ground truth; this does not predict YOLO11 quantization behavior or prove a general compiler cause. |
   | [RunBundle contract](../../docs/verification/evidence.md), [bundle implementation](../../ros_ws/src/omniseer_experiments/omniseer_experiments/bundle.py), and [bundle tests](../../ros_ws/src/omniseer_experiments/test/test_bundle.py) | Recorder supports model family/variant/precision/backend, model hashes, class lists, launch/config provenance, git/container identity, and comparison/trial/workload metadata. | Reuse for future collection and device trials. Existing summaries and reports count predictions; they do not compute ground-truth AP. Optional telemetry must be explicitly enabled and checked for availability. |
   | [Operator front door](../../docs/operations/scripts-frontdoor.md), [native replay](../../vision/apps/vision_replay.cpp), and [comparison tool](../../vision/apps/runbundle_compare.cpp) | `scripts/omni runs inspect`, `annotate`, and `report` review local bundles; existing RKNN replay/comparison executes YOLO-World. | Reuse evidence conventions and decoded inputs. `runs compare` and `comparison-report` are specialized to the six YOLO-World configurations, not a ready-made arbitrary YOLO11 comparison or AP evaluator. |
   | [YOLO11 ROS launch](../../ros_ws/src/bringup/launch/yolov11.launch.py) and [Ultralytics ROS node](../../ros_ws/src/yolo_ros/yolo_ros/yolo_ros/yolo_node.py) | A launch defaults to `yolo11n.pt`; the node implements Ultralytics inference and named detections. | A possible host/simulation integration reference, not evidence of native RKNN support, target performance, or YOLO11 accuracy on robot imagery. The launch threshold defaults to 0.5, unlike the retained replay's 0.25. |
   | [Model deployment workflow](../../docs/perception/yolo-world-model-deployment.md), [ONNX validator](../../tools/model/validate_yolo_world_onnx.py), [RKNN compiler](../../tools/model/compile_yolo_world_rknn.py), and [toolchain tests](../../tests/test_model_toolchain.py) | Pinned YOLO-World v2-S/M/L export and conversion: image input `[1,3,640,640]`, text input `[1,80,512]`, six fixed box/class outputs. Calibration pairs images with CLIP embeddings. | Reuse provenance, pinning, fail-fast validation, and artifact discipline. These commands and tensor contracts do not support a three-class YOLO11 graph as a drop-in replacement; YOLO11 needs its own verified export/decode contract. |

   Recomputing class-presence hits from the tracked JSONLs using the study's
   [metric implementation](../detector_comparison/scan_final_recal/evidence/render_docs_assets.py)
   gives the following motivation:

   | YOLO-World configuration | Person visible-frame hits / opportunities | Book visible-frame hits / opportunities |
   | --- | ---: | ---: |
   | v2-S FP | 293 / 293 | 39 / 840 |
   | v2-S INT8 | 293 / 293 | 0 / 840 |
   | v2-M FP | 293 / 293 | 32 / 840 |
   | v2-M INT8 | 293 / 293 | 7 / 840 |
   | v2-L FP | 293 / 293 | 268 / 840 |
   | v2-L TD01 mixed precision | 293 / 293 | 91 / 840 |

   These are inclusive class-frame presence counts, not object recall, precision,
   AP, or localization accuracy. They support preserving person capability and
   investigating book reliability in this scene; they do not establish YOLO11
   results. The existing task vocabulary has no `power drill`, so absence of drill
   detections in those streams is not evidence that prompted YOLO-World would fail.

   ## Data and annotation requirements

   Build a versioned inventory before training. Each sample must link to an
   immutable source hash, RunBundle/run and session IDs, room, collection date,
   subject/drill identity and physical book IDs, source frame/time, camera settings,
   any correction/remap, annotation version, and assigned split. Record external
   dataset origin, license, and original partition if external imagery is used.

   | Data source or gap | Required work |
   | --- | --- |
   | Retained clean RunBundle frames/video | Audit quality and sample distinct views from clean inputs. Record extraction and any validated preview-wrap repair as derived transformations. Avoid training on overlays or annotated review JPEGs. |
   | Existing scan visibility labels | Preserve them for historical presence regression. New human-reviewed boxes are required for AP; predictions cannot substitute for ground truth. Source access is a prerequisite for new extraction. |
   | Existing calibration imagery | Deployment docs describe 100 local robot images under ignored `models/source/calibration_images/`. Audit actual availability, source/session overlap, and representativeness. This is calibration input, not evidence of a labeled training set or an independent test set. |
   | Person retention | Record the regular subject across independent sessions, viewpoints, distances, seated/standing poses, lighting, backgrounds, placements, and occlusions, including mixed scenes with books/tools. Other people and a separate broad person dataset are not required. |
   | Book adaptation | Record many physical books: covers, spines, upright/flat/open poses, shelves/piles, small/distant views, clutter, glare, and occlusion. Include historically missed books as development cases; reserve separate physical books for held-out evaluation. Define individual-book versus inseparable-pile labeling consistently. |
   | Drill learning | Record the specific drill across independent sessions, viewpoints, distances, lighting, backgrounds, placements, and occlusions, including held/resting views. Vary attachments if used operationally. Additional drill models/instances are not required. |
   | Hard negatives and empty scenes | Include rooms without each target and tool-like distractors such as screwdrivers, hammers, hair dryers, and loose batteries. Include notebooks/boxes/screens that resemble books. Label all visible targets in mixed scenes so missing labels do not become false negatives during training. |
   | Search-domain conditions | Capture robot-height views and representative blur. Reserve unseen sessions and combinations of viewpoints, distances, lighting, backgrounds, placements, and occlusions; record coverage explicitly. New rooms can supply background variation but are not independently required. New collection involving physical motion requires explicit operator authorization. |

   Use axis-aligned source-image boxes with canonical class names, image dimensions,
   and explicit policies for truncation, occlusion, ambiguous objects, and ignored
   regions. Document the available drill and book-labeling policy with examples;
   exclude unrelated tools and standalone components. Inspect all three classes in
   every sampled frame and include genuinely empty negative images. If other people
   or drills appear, label them by class rather than treating them as negatives
   because of their identity; report them separately from the scoped target cohort.
   Instance metadata supports split audits and cohort reporting, not detector identity.
   Use a second review pass on ambiguous labels and a sample of every split.

   Retain a canonical box annotation set from which COCO-style evaluation and
   YOLO training labels can be derived. The proposed YOLO representation uses
   normalized center/width/height plus study class ID. Check bounds, class mapping,
   duplicate frames, and empty/ignore handling before conversion. Track counts by
   class, instance, independent session/room, and difficulty stratum; raw frame
   counts alone do not establish dataset adequacy. Set collection quotas after a
   pilot inventory, before confirmatory training; no labeled drill dataset or
   sufficient held-out coverage is established by the inspected studies.

   ## Separation and leakage prevention

   1. Assign entire source sessions/runs and related scene groups to train,
      validation, or test **before** extracting frames. Start with an approximate
      70/15/15 allocation of independent groups, subject to all-class coverage;
      do not randomly split adjacent video frames. Keep repeated takes of the same
      scene/object arrangement together. The same regular person and specific drill
      intentionally appear across splits; their identity overlap is not leakage.
      Reserve genuinely new sessions and conditions, including backgrounds and
      placements, rather than near-duplicate takes of training scenes.
   2. Report two test cohorts: **environmental robustness**, with the regular
      subject, specific drill, and training-seen books in unseen sessions/conditions;
      and **unseen-book generalization**, with physical books excluded from both
      training and validation, also in unseen sessions/conditions. Reserve books
      excluded from training for validation-only tuning of generalization choices. Keep
      every frame containing a test-reserved book out of training, validation,
      and calibration, including background appearances. If a session contains
      reserved books, assign the whole group to the corresponding split. This
      prevents instance reservations from weakening the session-level separation.
   3. Deduplicate by source/frame identity, hashes, and visual near-duplicates
      across splits. Raw video, corrected video, crops, remuxes, captured JPEGs,
      and augmentations of one source remain in its assigned group. Apply training
      augmentations only after splitting. Audit external data for overlap too.
   4. Train and sample hard negatives only from training groups. Use validation
      for checkpoint/variant selection, early stopping, hyperparameters, class
      thresholds, and YOLO-World prompt choices. Calibration uses representative
      **training-only** images, never validation or test imagery; record its own
      immutable subset and hashes.
   5. Freeze a test manifest and annotations before model selection. Keep test
      groups out of all fitting, calibration, prompt tuning, and error-driven
      collection decisions. Run the final locked comparison once; further tuning
      requires a new independent confirmatory test cohort.
   6. Treat already inspected canonical studies and historically missed books as
      development/regression data, not blind test evidence. If their frames enter
      training, retain that fact
      and never present evaluation on those runs as held-out generalization.
      New recordings of those books can test environmental robustness, but cannot
      substitute for the reserved unseen-book cohort. Books excluded from this
      study's training are not claimed to be absent from model pretraining.

   Publish split membership, grouping rules, deduplication decisions, counts, and
   dataset/annotation hashes with results. If too few independent groups remain,
   collect more data rather than claiming precision from correlated frames.

   ## Evaluation protocol and baselines

   Use the same frozen, annotated indoor workload for all candidates. Start with
   YOLO11n at 640 pixels as an edge-oriented hypothesis; record and freeze the exact
   COCO checkpoint hash and Ultralytics version. A YOLO11s capacity experiment is
   optional and must have its own matched pretrained baseline and validation-only
   selection. Do not attribute architecture/size differences to fine-tuning.

   | Baseline | Purpose and reporting |
   | --- | --- |
   | Unmodified COCO-pretrained YOLO11n | Establish person/book quality before adaptation. Map COCO IDs by name to study IDs, retaining only target predictions for primary comparison. `power drill` is **unsupported/N/A** for category AP, with structurally zero supported drill recall; report drill opportunities separately. Never reinterpret another COCO label as a drill. |
   | Frozen YOLO-World v2-M FP and v2-L FP | v2-M FP is the historical aggregate-coverage reference; v2-L FP is the strongest historical book reference and has a public person-acquisition bundle. Evaluate both anew with fixed prompts `person`, `book`, `power drill`, exact prompt order/embedding hashes, and required padding for the native 80-row contract. Drill capability is a hypothesis. Historical 29-prompt counts are a separate regression, not results for this new vocabulary. |
   | Fine-tuned YOLO11n, three-class head | Use the same starting checkpoint, image size, evaluator, and source frames as pretrained YOLO11n. Report all three classes, including person regressions. Repeat training with three recorded seeds if practical; disclose any single-seed limitation. |

   Primary quality evaluation is offline, all selected frames processed, without
   tracking or autonomy feedback. Use a common RGB/letterbox specification and
   source-space boxes; document any model-specific normalization or decode
   requirements. Fix NMS IoU 0.45, maximum 100 detections, and suppression policy
   for the primary comparison; expose unavoidable differences. Report a common
   score-threshold 0.25 operating point to connect to prior studies, plus
   validation-selected per-model/class operating points at a common precision
   target. AP uses a sufficiently low, predeclared score floor to retain the PR
   curve, not just the 0.25 operating point. Confidence scores are not assumed
   calibrated across families.

   Required outputs:

   - Per-class AP50 and AP50–95, precision/recall curves, and precision/recall/F1
   at locked operating points. Use class-aware, one-to-one IoU matching; state
   evaluator version and crowd/ignore policy. Report target counts and false
   positives on negative/distractor scenes, not only positive-frame coverage.
   - Person/book macro AP for the fair pretrained comparison; three-class macro
   AP only for models supporting all three labels. If an all-target score with
   zero drill output is also shown for pretrained YOLO11, identify that convention
   explicitly instead of averaging away the category limitation.
   - Separate results for environmental robustness and unseen-book generalization,
   with book metrics on each cohort rather than only a pooled score. Break down
   session, viewpoint, distance/apparent size, lighting, background, placement,
   occlusion, and blur; show historically missed books as development regression
   cases. No unseen-person/drill or instance-identification claim is required.
   Report paired differences and confidence intervals bootstrapped by
   independent collection group, not individual adjacent frames. Report training
   seed variation separately from dataset uncertainty.
   - For annotated sequences, visible-frame coverage, time to first correct
   detection, correctly localized detection persistence, longest miss interval,
   and false-positive events/minute on negative sequences. Define minimum visible
   size and event-merging rules before evaluation. These complement box AP;
   they do not convert presence annotations into localization ground truth.

   Separate quality evaluation from performance: host PyTorch/ONNX tests, Toolkit2
   host analysis, offline ROCK 5B+ RKNN replay, live camera perception, and eventual
   physical acquisition are distinct evidence boundaries. Historical physical FPS
   and success times are context only; independent runs cannot supply a controlled
   new-model accuracy or speed comparison. On-device trials must use a matched
   workload, repeated runs, warmup, fixed camera/config/core mask, thermal/power
   context, and explicit model/runtime/git/container hashes. Live latest-wins
   processing can skip frames, unlike the all-frame offline accuracy protocol.

   ## Proposed success gates

   These are initial research acceptance targets, **not achieved results or existing
   robot requirements**. Freeze them after the data/validation pilot and before
   opening the confirmatory test. Changes require a versioned rationale.

   | Gate | Proposed acceptance criterion |
   | --- | --- |
   | Person retention | For the regular subject in unseen sessions/conditions, person AP50–95 and recall at precision ≥0.90 lose no more than 2 percentage points versus matched pretrained YOLO11. Check both YOLO-World references and disclose regressions; passing YOLO11 retention alone does not establish retention relative to the deployed detector. |
   | Book improvement and generalization | In each test cohort separately, book AP50–95 improves by at least 5 percentage points and recall by at least 10 points at precision ≥0.90 versus pretrained YOLO11 and the stronger YOLO-World book reference selected on validation. Report uncertainty, per-book coverage, and difficult-condition failures; pooled gains cannot hide failure on unseen books. |
   | Drill learning | For the specific drill in unseen sessions/conditions, AP50 ≥0.70, precision ≥0.90, and recall ≥0.70 at the validation-locked operating point. Compare with prompted YOLO-World, without claiming improvement over a nonexistent pretrained YOLO11 drill category or robustness to other drills. |
   | False detections and temporal robustness | At the locked operating point, at most one false-positive event/minute per target class on class-absent test sequences; book/drill correct-detection coverage improves over supported baselines without a material person persistence regression. Predeclare event rules and report miss durations. A correctly labeled other person/drill is not a false class detection, and these metrics do not test identity selection. |
   | Conversion fidelity | PyTorch → ONNX → FP RKNN → candidate INT8/mixed-precision RKNN preserves names, source-space geometry, normalization, score meaning, and output decoding. On identical labeled frames, no class loses more than 2 AP50–95 points or 3 recall points versus the selected PyTorch model; check both book cohorts separately. Report class-aware IoU matching to the FP reference as a diagnostic. Compilation alone cannot pass this gate. |
   | RK3588 performance and stability | Provisional live perception budget: mean consumer rate ≥10 FPS, inference p95 ≤100 ms, source-age p95 ≤200 ms, with no fatal pipeline errors or unbounded memory growth during a ≥30-minute stationary camera soak. Record startup time, CPU/NPU, memory, temperature, dropped/replaced frames, and optional-signal availability. Compare repeated matched target runs; host timings cannot satisfy this gate. |
   | Eventual physical usefulness | After perception gates and explicit authorization, repeat bounded acquisition/framing trials for the regular subject, specific drill, and seen/unseen books in new conditions, plus class-absent trials. Retain outcomes, false acquisitions, timeouts, target loss, freshness, and terminal evidence. Freeze trial counts/outcome criteria before motion; class detection alone does not establish physical search success or identity selection among multiple same-class objects. |

   If uncertainty spans a success margin, report the result as inconclusive and
   collect independent evidence. No stage authorizes changing arbitration,
   watchdogs, timeouts, proximity stops, or other control semantics.

   ## Implementation roadmap and artifact conventions

   1. **Next implementation step: dataset audit and split manifest.** Inventory
      available clean RunBundle sources, verify hashes/repairs, identify session and
      book identities, write the three-class annotation guide, and allocate new
      independent collection groups for the regular subject and specific drill.
      Produce a reviewed pilot set and split manifest separating environmental
      robustness from unseen-book evaluation, with explicit collection gaps.
   2. **Offline baseline harness.** Add focused host-side experiment tooling under
      `tools/`, with dataset/class/geometry validation and ground-truth AP evaluation.
      Pin YOLO11 and YOLO-World assets/dependencies; run the baselines on validation
      data, then freeze prompts, thresholds, metrics, and acceptance targets.
   3. **Fine-tuning experiment.** Add a reproducible three-class training recipe,
      balanced retention examples, recorded seeds/hyperparameters/augmentations,
      and validation-only checkpoint selection. Test the learning hypotheses on
      the frozen test, preserving failures and uncertainty.
   4. **Separate deployment experiment.** Define and test YOLO11 ONNX/RKNN IO and
      decoding before considering native integration. Compare FP and quantized
      models with training-only calibration; verify on ROCK 5B+ before production
      integration. Do not force YOLO11 through the existing YOLO-World validator
      or assume the YOLO-World TD01 layer choices apply to YOLO11.
   5. **Application evidence.** After quality and runtime gates, plan authorized
      physical trials and a deployment decision with reviewable RunBundles. Runtime
      packaging/promotion follows the existing release workflow as a separate task.

   Follow [study conventions](../README.md): this directory holds the definition,
   small manifests/configuration, provenance, and selected canonical evidence;
   ordinary working RunBundles remain under ignored `runs/`. Bulk local datasets
   and checkpoints can live under ignored `models/source/custom_detector/`, with
   generated models under ignored `artifacts/models/`; record durable hashes and
   retrieval instructions rather than committing bulk intermediates. Experiment
   programs belong in `tools/` or their owning subsystem, and conclusions in
   `docs/` once evidence exists. Preserve raw RunBundles and write extractions,
   corrections, labels, reports, and comparisons to explicit derived locations.

   This design adds no training code, changes no existing experiment or production
   interface, and makes no hardware-readiness claim. Future implementation uses the
   [repository workflow](../../AGENTS.md), the
   [native vision rules](../../vision/AGENTS.md), and the
   [ROS/RunBundle rules](../../ros_ws/src/AGENTS.md) at the applicable boundary.
