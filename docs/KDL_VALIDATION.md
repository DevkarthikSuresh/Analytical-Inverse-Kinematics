# Matched-task KDL validation protocol

1. Obtain an authorized URDF and verify chain frames, axes, link dimensions, units, joint signs and order. The simplified model may not match the physical model.
2. Choose reachable targets; use the analytical radial-pitch policy to construct an equivalent full pose in the actual URDF frame. Verify the analytical joint vector with independent URDF FK, including both position and rotation. If the orientation cannot be mapped reliably, do not make a full-pose comparison.
3. Pass the same target pose to KDL's LMA solver. Record solver return codes, convergence and the same FK error metrics for both methods. Declare the seed policy and tolerances. Do not include failed solves as successes.
4. Warm up both solvers, time many representative targets with a monotonic high-resolution timer, and separate solver time from FK checks, logging, setup and allocation. Report distributions and success rates, not only a single speedup.
5. Preserve the raw outputs, software versions and permitted URDF revision identifier. Report the analytical solver's restricted task scope and lack of joint-limit/collision handling.

The old script constructed a KDL frame from position alone (identity orientation), but the analytical solver chose a radial pitch. It also did not check the KDL return code. Consequently the earlier timing comparison should not be treated as matched-task validation based on the snippet alone.
