# Coordinate Frames - RM27 Visual Guidance (V0-V2)

This document is the single source of truth for every frame and sign
convention used by the visual-guidance chain:

    TargetEstimate -> UAVGuidance -> GuidanceCommand -> MAVLink -> ArduPilot -> Gazebo

No module may invent its own sign conversion. If a conversion is needed,
it is defined here and implemented exactly once, at a layer boundary.

## 1. Camera optical frame (maixcam2_dart, canonical)

Defined by the vision repo (`maixcam2_dart`, `schema_version = 2`):

- `+z` : camera forward (out of the lens)
- `+x` : camera right (image right)
- `+y` : camera down  (image down)

`TargetEstimate` angular fields (see `target_fusion.cpp`):

- `yaw_rad   = atan(right / forward)`  -> target right of boresight => yaw_rad > 0
- `pitch_rad = -atan(down / forward)`  -> target above boresight => pitch_rad > 0
- `line_of_sight_camera = normalize([ tan(yaw_rad), -tan(pitch_rad), 1 ])`

## 2. Body frame (vehicle, FRD)

- `+x` : forward
- `+y` : right
- `+z` : down

`GuidanceCommand` velocities are body-frame FRD:

- `vx_body_mps` : forward
- `vy_body_mps` : right
- `vz_body_mps` : down
- `yaw_rate_rad_s` : positive = yaw right (nose turns right, clockwise seen
  from above)
  from above)

## 3. Camera -> body mounting (V0-V2 assumption)

The camera is assumed rigidly mounted looking straight ahead, upright:

| camera axis | body axis |
|-------------|-----------|
| +z (fwd)    | +x (fwd)  |
| +x (right)  | +y (right)|
| +y (down)   | +z (down) |

Consequences:

- camera `yaw_rad` (target right positive) maps directly to a body-frame
  yaw error with the same sign: `yaw_error_body = target.yaw_rad`.
- camera `pitch_rad` maps to body pitch error with the same sign (unused in
  V0-V2; vy/vz held at 0).

This identity mounting is an explicit simplification for V0-V2. A camera
extrinsic transform hook exists in the V2 truth adapter; any real mount
offset must be added there, not inside guidance.

## 4. ArduPilot / MAVLink frames

- `LOCAL_NED` telemetry: x = North, y = East, z = Down.
- ArduPilot yaw angle: positive = clockwise seen from above = nose right
  (same sense as our body `yaw_rate_rad_s` convention).
- Body-frame velocity commands use `SET_POSITION_TARGET_LOCAL_NED` with
  `coordinate_frame = MAV_FRAME_BODY_NED (8)` and type mask `0x05C7`:

  | bits set (ignore) | meaning                   |
  |-------------------|---------------------------|
  | 0x0007            | ignore position x/y/z     |
  | 0x01C0            | ignore accel afx/y/z      |
  | 0x0400            | ignore yaw angle          |
  | (bit 11 clear)    | yaw_rate field is honored |

  So the adapter sends `vx, vy, vz` (body FRD) and `yaw_rate`
  (rad/s, positive right). Command rate: 10 Hz minimum so the guided-mode
  3-second setpoint timeout never triggers during healthy operation.

## 5. Gazebo world frame

- Gazebo world is ENU: x = East, y = North, z = Up; yaw positive =
  counter-clockwise seen from above = nose left.
- The ardupilot_gazebo plugin converts ENU <-> NED internally; ArduPilot
  SITL only ever sees NED.
- The V2 truth adapter reads ENU poses from Gazebo and converts:

  1. `bearing_enu = atan2(ty - uy, tx - ux)` (ENU, CCW-positive)
  2. `heading_err_enu = wrap_pi(bearing_enu - uav_yaw_enu)` - positive
     means the target is to the vehicle's LEFT.
  3. Vision convention needs target RIGHT positive, so:
     `TargetEstimate.yaw_rad = -heading_err_enu`.

  This single negation is the only ENU/vision sign flip in the system and
  is covered by unit tests (target left/right cases in both conventions).

## 6. Sign summary

| Situation                 | vision yaw_rad | body yaw error | yaw_rate cmd  | vehicle response |
|---------------------------|----------------|----------------|---------------|------------------|
| Target right of boresight | +              | +              | + (clamped)   | yaws right       |
| Target left of boresight  | -              | -              | - (clamped)   | yaws left        |

Verified three times: V0 unit tests (sign cases), V1 SITL telemetry
(observed yaw direction), V2 closed-loop convergence.
