# Target guidance frame conventions

The existing TargetEstimate subset uses target bearing: positive `yaw_rad`
means right of camera boresight; positive `pitch_rad` means above boresight.
These are target-relative angles, not camera world orientation. The truth
adapter converts its scene geometry to these signs before producing a target.
The command adapter consumes `GuidanceCommand`, including body-frame velocity;
see `sitl/mavlink_adapter.py` and root `docs/INTEGRATION.md` for integration frames.

Localization is a separate contract: `T_parent_child` maps child coordinates
into parent coordinates. No world-frame convention for a future localization
estimator is established by these target bearing signs. This page documents
existing behavior; it does not change TargetEstimate or guidance.
