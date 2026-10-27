# Joint racing-line and speed-profile development

**Latest measured eligible clean full lap: 5.8218 s. Five seconds remains unachieved.**

Further acceleration/feedforward/steering trials did **not** beat this result. The proven default line/profile was retained, with planner fallback, startup and median-tuning fixes. See `FASTER_FOLLOWUP_RESULTS.md` for all nine actual candidates, failures and the newer continuous observation; no new speed record is claimed.

This continues the single-file, simulation-only benchmark in `ONEFILE_RACER.py`. There is **no imposed command-speed ceiling** by default (`--max-speed 0`). The controller still uses finite steering/throttle commands, actuator saturation, curvature, modelled grip/acceleration/braking, STOP, sensor freshness and host-liveness supervision. No simulator clock, circuit, lap counters or vehicle physics were changed.

## Actual tests, including failures

| Run | Change | Actual eligible clean laps | Session collisions |
|---|---|---|---:|
| `onefile-20261027T125354Z` | Joint time/line planning with the previous tracker | 7.9196, **7.2717**, 7.6546 s | 0 |
| `onefile-20261027T125824Z` | Curvature-dependent pursuit, revised braking preview and speed feedforward | 6.4747, **5.9258**, 6.0827 s | 1, during subsequent shutdown |
| `onefile-20261027T130241Z` | Longer adaptive validation with the coasting-steering STOP fix | 13 eligible clean full laps, best **6.3607 s** | 0 |
| `onefile-20261027T130751Z` | Independent fixed-0.95 repeat with the STOP fix | 14 eligible clean full laps, best **6.0397 s** | 0 |

The second run's lap 3 is the 5.92579984664917 s result. Measured wall time was **5.917035645 s**, travel **28.2108 m**, track coverage **100%**, and recorded collisions **zero at that lap**. No collision, teleport or transition flag was set. A later collision occurred while coasting after the bounded test had disarmed; the final session count of one is retained. A subsequent STOP fix keeps zero propulsion but follows the curve while coasting, rather than straightening the wheels into the wall. Its verification is recorded separately.

These are **individual-clean-lap results**, not a claim that an entire session containing another collision was collision-free. Historical 6.9607 s evidence and source remain unchanged. Restarts must establish their own lap times.

The two longer validation runs both ended with verified stationary telemetry and zero collisions, including shutdown. They did **not** repeat the earlier under-six-second lap; 5.9258 s remains the best single observation, not a guaranteed continuous pace. The Windows Start shortcut launches continuous adaptive pacing with the selected joint line/profile and no lap/duration limit; STOP and fault handling remain active.

The final Windows shortcut check is `onefile-20261027T132154Z`: the preserved early observation has four simulator laps, two eligible clean full laps, zero collisions and best 6.4637 s, still running with no imposed speed cap. A new session's display does not inherit the historical 5.9258 s. Two intervening preparation failures are preserved: a blank loading frame and a hover-dimmed menu icon. Both were resolved in the UI-readiness check without bypassing telemetry or injecting lap events; no driving was armed in those failures.

**Later continuous improvement:** that same run genuinely recorded **5.821800231933594 s on lap 16**, wall time **5.802970847 s**, travel **28.1909 m**, full coverage and zero recorded collisions at that lap. The immutable follow-up `report_figures/data/joint-line-speed-20261027/live-followup-20261027T132654Z/` observed 41 simulator laps, 24 eligible clean full laps, one later collision and continued driving. Only one eligible lap in that snapshot was below six seconds; do not claim a guaranteed sub-six-second pace. The impacted later lap was rejected and the controller backed off without crash-stopping. Exact evidence is also in `BEST_CLEAN_LAP.json`.

## What was actually improved

- Joint optimisation evaluates complete cyclic profile time, curvature-rate and edge penalties, rather than only local curvature smoothing. Smooth multi-scale moves use the actual recorded free-space corridor, with up to 0.65 m displacement from the prior reference.
- Clearance uses a conservative interpolated distance field with a half-cell-diagonal allowance. The final dense line must satisfy centre clearance and a sampled **0.62 x 0.42 m inflated footprint** check. This is saved-map/model validation, not a guarantee of collision avoidance.
- The selected reference is **28.1808 m**, compared with the previous **30.9054 m** reference on the same unchanged circuit. It uses more available track width; it does not change the track or inject lap events. Minimum checked centre clearance is **0.3522 m**, and minimum sampled inflated-footprint clearance is **0.1421 m**.
- Speed planning shares a combined grip budget between lateral and longitudinal acceleration, includes motor roll-off and steering-rate limits, and propagates acceleration/braking constraints around the lap boundary. A final projection keeps the planned segment accelerations inside the modelled grip ellipse.
- Curvature-dependent pursuit reduces excessive corner lookahead. The new best lap's maximum recorded path error was **0.0618 m**, versus **0.6577 m** in the slower first joint-line test. That first failure is preserved, not hidden.
- Revised motor acceleration feedforward, PI tracking and earlier braking preview improve speed tracking. Actual peak speed on the 5.9258 s lap was **8.2542 m/s**; no 6 m/s cap is being claimed.

Several planner/control settings changed together; this is engineering development, not an isolated causal test of one parameter.

## Selected model/control settings

- Command-speed ceiling: disabled; physical/model constraints still apply.
- Lateral planning: 10 m/s²; combined lateral grip model: 12 m/s².
- Longitudinal acceleration model: 9 m/s² with roll-off knee 7 m/s; braking model: 12 m/s².
- Steering rate: 8 rad/s; footprint margin: 0.06 m beyond the inflated footprint.
- Lookahead: 0.55 + 0.14 × speed, bounded by geometry and curvature; braking preview: 0.10 s.
- Motor feedforward: 0.04; acceleration feedforward: 0.035; speed gain: 0.18; integral gain: 0.04.
- Initial speed scale: 0.95. Continuous tuning tries nearby scales, backs off from collisions/slower results and retains the fastest observed eligible lap. This is not proof of a global optimum.

The selected profile's offline estimate was **5.4559 s**. That is explicitly a model estimate, not a measured lap or proof of five seconds.

## Evidence and scope

- Original run logs, profiles, actual paths, source hashes and lap events: `logs/onefile-20261027T125354Z/` and `logs/onefile-20261027T125824Z/`.
- Development/baseline archive: `logs/line-speed-development/20261027T124638Z/`.
- Canonical and Windows-mirrored report data: `report_figures/data/joint-line-speed-20261027/`.
- Actual-data figures: `report_figures/10_joint_racing_line.png`, `07_joint_speed_profile.png`, `07_joint_speed_profile_followup.png` and `08_joint_line_development.png`. Convenient copies: `TRACK_MAP_LATEST.png`, `SPEED_PROFILE_LATEST.png` and `SPEED_PROFILE_LATEST.csv`. The first speed figure preserves the earlier 5.9258 s lap; the follow-up/root speed figure shows the 5.8218 s lap.
- Tests cover finite/cyclic profiles, optional cap enforcement, uncapped operation, grip-ellipse bounds, conservative footprint/outside-map rejection, cancellation and invalid-lap/lease checks. Synthetic tests are not driving evidence.

All measurements here use the **Windows official simulator + Linux ROS Docker**, privileged simulator odometry and genuine prior memory. Native Ubuntu GPU/UI driving remains unvalidated. This controller is not LiDAR-only autonomy and must not control a physical car. Crash-stop and proximity/predictive governors remain disabled; collisions are logged, impacted laps rejected and recovery/tuning backs off.
