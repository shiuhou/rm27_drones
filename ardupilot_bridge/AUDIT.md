# Pre-implementation audit and design

Baseline: nested mujoco_uav repository, verified M3 `2be0029`; isolated branch
`ardupilot-m3`. Original master `b5afe68` and its README edit are preserved.
ArduPilot source: `96f98857f7fc4bd564fbbdc43c5f26114246369f`, clean checkout.

Local sources (relative to sibling ardupilot): SIM_JSON.h/.cpp in libraries/SITL,
examples/JSON/readme.md and examples/JSON/pybullet/robot.py, and
libraries/AP_Motors/AP_MotorsMatrix.cpp (Quad X setup, lines 592–600).
These override online documentation. PyBullet supplies the reference exchange,
frame conversion and linear PWM model; its rotor-4 position typo is not reused.

Protocol: little-endian host packets <HHI16H (40 bytes, magic 18458) or
<HHI32H (72 bytes, magic 29569). Default UDP 9002, reply to sender endpoint.
Instance N uses 9002+10*N. frame_rate is requested physics Hz, counter increments
on processed sensor input. Local code retries unchanged frame after ~1.1 seconds
(despite README's 10 seconds). Cache duplicate replies without stepping. Lower
counter resets plant; uint32 wrap is continuous. UDP reordering cannot in general
be distinguished from restart solely by counter: follow official lower-counter
convention and document this localhost assumption.
JSON needs leading and trailing newline; timestamp, imu.gyro, imu.accel_body,
velocity, position, quaternion are supplied. Quaternion WXYZ rotates body FRD to
world NED. Keep default time sync and lockstep; never set no_time_sync (which can
select truth AHRS). Test observed frame rates, use one .002 s step per new frame.

Plant: models/quadrotor_m1.xml, RK4 .002 s, world Z-up, body forward/left/up,
1 kg, principal inertia (.018,.018,.030) kg m², gravity (0,0,-9.81).
We define world +X north, +Y west: NWU, not ENU. Both basis transforms to AP are
C=diag(1,-1,-1); R_AP=C R_MJ C.T. Position/velocity C v, body IMU C v.
Positive MJ roll stays positive, pitch/yaw reverse. Scalar-first quaternion
conversion is performed through matrices, not scattered sign adjustments.

Actuators XML order FL FR RR RL, offsets ±.163 m, thrust along +Z,
reaction coefficients +.018,-.018,+.018,-.018 Nm/N. AP outputs 1 FR CCW,
2 RL CCW, 3 FL CW, 4 RR CW: plant indices [1,3,0,2].
PWM 1000..2000 maps linearly to 0..8 N (clamped). Set MOT_THST_EXPO=0,
MOT_PWM_MIN/MAX=1000/2000; no battery thrust compensation. Retain only existing
filterexact .040 s lag. No model/controller rewrites or arbitrary gain tuning.

IMU: reuse M3 semantics R.T@(qacc[:3]-gravity). Call mj_forward after stepping
so qacc and attitude correspond to returned state. Free joint rotational qvel is
body angular rate. No M3 noise/dropout. Reset mj_resetData, forward dynamics,
actuator state, time, sequence/reply cache and sensor bookkeeping.

Architecture: add only ardupilot_bridge/{protocol,frames,actuators,plant_adapter,
bridge,launch}.py, __init__.py, hover.parm, tests/, AUDIT.md, REPORT.md and
requirements.txt. Protocol/frame/mapping tests first; then plant/reset, live
exchange and MAVLink Ready→GUIDED→ARM→1.5 m takeoff→10 s hover benchmark.
Use separate SITL instance 20 to preserve existing instance 0. Logs include every
physics frame and MAVLink observations. All baseline tests and four benchmark
CLIs must pass. No navigation/perception work.

## Post-audit findings (implementation/flight verification)

The pre-implementation plan above is preserved. Actual initial packet advertises
1200 Hz from SIM_Aircraft.h's default even with rate arguments. Frame 0 now
negotiates 500 Hz with the first .002 timestamp; final logs show exactly one
1200 Hz frame followed only by 500 Hz frames. Local INS requires gyro rate
>=1.8*loop rate, so configure SCHED_LOOP_RATE=250 with the 500 Hz plant.
No arming checks bypassed. Full evidence and final file inventory: REPORT.md.
