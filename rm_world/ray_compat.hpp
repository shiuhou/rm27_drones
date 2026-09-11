// Upstream checker sources use the MuJoCo 3.4 ray API. MuJoCo 3.12 adds
// an optional normal output. Preserve their calls without modifying upstream.
#include <mujoco/mujoco.h>
#define mj_ray(...) mj_ray(__VA_ARGS__, nullptr)
#define mj_rayMesh(...) mj_rayMesh(__VA_ARGS__, nullptr)
