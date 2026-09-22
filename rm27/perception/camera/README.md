# Camera producer boundary

The device-side MaixCAM2/OS04A10 application is under
`rm27/perception/vision/maixcam2_dart_vision/`. It remains a separate
C++/MaixCDK build with board-specific toolchains. The root provides a `TargetEstimate` JSON v2 subset adapter in
`rm27.perception.guidance.schema.TargetEstimate`. Synthetic and truth-based
producers exercise it; a live camera transport is not connected here.

This directory describes the camera boundary, not a second vision package.
Lens calibration, exposure/frame timing, image transport and camera-frame
conventions are perception inputs; detector and tracker implementations live in `vision/`. The localization
contract and offline tools are separate; a SLAM implementation is not present.

Compilation or replay rate alone does not establish flight readiness.
Calibration, timestamp meaning, frame age, failure states and
`safe_for_control` must be measured before camera output feeds ExternalNav or
closed-loop flight.
