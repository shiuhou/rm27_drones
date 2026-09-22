# Vision perception unit

This directory owns the camera-based part of RM27 perception. It is the home
for both localization and recognition; `vision` is not an independent layer
beside `perception`.

- `localization/` contains the versioned ego-pose contract and offline video,
  dataset-view and calibration-observation tools; it contains no SLAM backend.
- `docs/` retains VSL evidence and the current
  [preparation update](docs/VSL-0B1_CALIBRATION_PREP.md).
- `M3C_OS04A10_SLAM_实验起步包_v1/` defines the M3C + OS04A10 monocular
  camera calibration, localization and SLAM experiment route.
- `maixcam2_dart_vision/` contains the device-side MaixCDK target/green-light
  and red/blue armor recognition application, replay tools, configuration and
  reports. Its output is the versioned `TargetEstimate` consumed by the rest of
  `rm27/perception/`.

The two tracks answer different perception questions and must keep their
evidence separate: camera recognition answers *what target is visible and where
is it in the image*, while SLAM/localization answers *how the camera is moving
and where it is in the scene*. Neither result is automatically metric or
control-safe. Both must pass calibration, timing, frame-convention and
failure-state checks before feeding ExternalNav or flight control.

Future offline infrastructure is documented in
[the VSL-3 harness](docs/VSL-3_EXPERIMENT_HARNESS.md),
[the VSL-4 host benchmark plan](docs/VSL-4_M3C_BENCHMARK_PLAN.md), and
[the VSL-6 evidence contract](docs/VSL-6_VIO_DATA_REQUIREMENTS.md).
These are preparation tools; stage execution and physical qualification remain blocked.

Public backend execution evidence: [VSL-3P qualification](docs/VSL-3P_PUBLIC_BACKEND_QUALIFICATION.md). Stella passes on pinned public TUM data; ORB-SLAM3 requires the recorded shutdown patch. Real RM27 VSL-3 remains NOT_STARTED.

Runtime state observation: [VSL-3P.1 report](docs/VSL-3P1_ONLINE_STATE_REPORT.md), with separate online and final artifacts.
