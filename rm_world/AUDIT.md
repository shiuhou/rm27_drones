# G1' pre-implementation audit

Source ../rmuc2026_battlefield_mjcf at dcf1202f04e1ba31ab5dffac6b2c1c26f1129453,
clean. No LICENSE/COPYING found in tracked files. Keep source/assets external.
Read model.xml, tools/README.md, validation inventory and checker sources.

Native model loads under installed MuJoCo 3.12.0 in ~0.42 s; upstream performance
claims used 3.4.1 and another robot (not transferable measurements).
4 bodies including world; 555 geoms, 428 collidable (group 3), 127 visual-only
(group 2, contype=conaffinity=0); nq=0. Field ground full dimensions
29.05 x 16.05 x .2 m, top Z=0. World field body offset
(-.02,-1.624344,1.641344) is significant and must stay intact. Replaced decks
at (-9.55,3.75,0) and (8.35,-3.71,0), deck top .5 m.
Source frame defines metres and Z-up, +X right/+Y up in plan view, no geographic
north assignment. Integrate at identity; assign +X north/+Y west consistently
with existing bridge. No physical field rotation or translation.

Field collision defaults: boxes/convex mesh proxies, contype=conaffinity=1,
friction (.8,.01,.001), solref (.01,1), solimp (.95,.99,.001).
Detailed support/barrier decomposition and residual errors are documented in
upstream tools/README.md. Important: much of the original contact envelope is
limited to Z<=1 m; entirely high features are omitted. Later decks/roads have
explicit collision. This is not a complete aerial collision envelope.

Field compiler radians, autolimits=true; dt .002, gravity -9.81, implicitfast.
UAV compiler degrees, autolimits=true; dt .002, gravity -9.81, RK4,
density1.2 viscosity1.8e-5. Preserve UAV options exactly. Compose generated XML
from parsed trees: normalize UAV angles to radians, remove only baseline ground,
merge named field defaults/assets/static world bodies/lights/camera; rewrite
asset paths absolute. Reject naming conflicts and unsupported root sections.
Keep every field visual and collision geom and original transform. Verify
compiled field transforms and masks against standalone field, and UAV mass,
inertia, actuator names/gears/ranges/dynamics against standalone UAV.

Safe spawn: centre of northwest deck (-9.55,3.75,.5) plus original .12 m UAV
clearance. Named local navigation origin is deck surface; C=diag(1,-1,-1)
converts p_world-origin to local NED. Test ray height and zero initial contacts.
After hover, translate .3 m across clear deck airspace, then gently descend onto
this known static deck as a controlled contact experiment, no mission autonomy.

Integration in rm_world only except minimal optional bridge/runner injection:
existing Bridge hardcodes Plant; existing runner tests absolute qpos against
origin-centred guards. Accept injected plant/bridge and use already transmitted
local position for guards, preserving defaults. Add optional post-hover test
callback; default launcher still stops after ten-second hover. No controller,
parameter, UAV XML or upstream edits.

Reproduce upstream C++ commanding-height, undulating-road and ramp-crossing
checks using installed headers/shared library. Outputs/builds outside upstream.
Run Python surface/barrier audits with -B and output maps in integration artifacts.
Then integration physics tests, timings, actual field SITL flight/contact and
standalone hover regression. Retain exact commands and failures before claims.
