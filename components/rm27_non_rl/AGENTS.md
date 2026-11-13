# RM27 Non-RL

Current research prose uses 接觸試驗台 (contact-test structure), 接觸判定模組
(contact-evaluation module), and 目標接觸板 (target contact plate) for the older
"base", "hit module", and "armor plate" names. Describe the actual physical
contact, separation and recovery behavior. Preserve asset/API identifiers,
source quotations and historical evidence; naming does not change qualification.

Traditional model-based planning, guidance, state estimation and recovery live
in `rm27_non_rl/`; tests live in `tests/`. Read `.agents/docs/SPEC.md`,
`DESIGN.md` and `ROADMAP.md` before changing behavior. Follow the parent
engineering standard. The active research plan is
`.agents/plans/plan_asm2_autonomous_handover_01.md`; its §0 is the sole progress,
todo and handoff record.

The shared simulator is `../rm27_sim`, including native physics, scene and
sensor generation, collision scoring and replay. Consume its public interfaces;
do not copy its implementation or introduce simulator-core dependencies on
Non-RL or RL. `../rm27_RL` owns learning and evaluation and is not modified.

Preserve recorded trajectories, seeds, original contact/safety criteria and
runtime truth isolation. Ground truth is limited to sensor generation and
offline scoring. The ongoing R3.1–R3.2 budget remains 2 baseline, 10 development,
12 held-out trials and 8 bounded forecasts; directory migration grants no
additional trials. No hardware, RL training, commits, branches or pushes.

Run focused tests before full component tests and the simulator/M3 regression.
Report PASS, FAIL, NOT_RUN and BLOCKED separately. Historical ASM2 is an aircraft
and experiment identifier; the research line is formally **RM27 Non-RL**.
