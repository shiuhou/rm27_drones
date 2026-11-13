# RM27 Non-RL — ROADMAP

| Stage | Deliverable | Gate |
|---|---|---|
| R3.1 Witness characterization | Matched success/failure divergence, frozen engineering witnesses | Preserve native physics/scorer and separate estimator/timing/control limitations |
| R3.2 Autonomous handover | State-dependent candidate adoption, bounded adjustment and real safe exits | Different physical starts, continuous commands, independent contact/recovery evaluation |
| R3.3 Recovery-aware planning | Native short forecasts and recoverable terminal selection | Match executed separation branch and measure forecast cost; no assumed 30 ms qualification |
| R3.4 Continuous mission validation | Broader starts, delay/model stress, later visual input and native takeoff | Independent evidence for each added capability |
| Deployment | Separate real sensing, VIO/PX4/RK3576 integration | Explicit authorization and separate flight qualification |

Existing ASM2 contact witnesses establish feasibility at particular development
conditions, not general robustness or detected-impact recovery. Migration of
ownership establishes neither new contact success nor additional budget.

Advancement from R3.2 requires estimator/finite-boundary assumptions to cover
observed error and a physically executed safe rejection path. Candidate-map
acceptance alone cannot unlock broader starts or recovery-aware acceptance.
Forecasts must be checked against both successful and failed witnesses before
they can become a qualification gate. New independent evaluation requires fresh
registered cases after development is frozen; exposed failures remain regression
data. Current results and remaining actions belong only in the active plan §0.

Straight-flight contact development must demonstrate the complete recovery
envelope before broadening starts. Coordinate terminal height, attitude and
braking together; treat fixed-COM rotation checks, moving-body clearance and
finite target boundaries as distinct claims. Earlier rotation or larger normal
thrust alone does not establish recoverability. Retain both actual motion and
qualification outcomes when a trajectory ends at a safety limit.
