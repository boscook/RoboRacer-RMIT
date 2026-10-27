# Faster-follow-up tests — honest result

**The nine new bounded candidates did not beat the historical 5.8218-second eligible clean lap.** Their best was 6.5537 s. Five collisions across the candidate sessions remain recorded; impacted laps were not accepted. No faster result is claimed from these changes.

The earlier best remains lap 16 of `onefile-20261027T132154Z`, with exact evidence in `BEST_CLEAN_LAP.json` and `JOINT_LINE_SPEED_RESULTS.md`. That original continuous run was verified stationary before development handover. Its completed results and later collisions remain preserved.

## Actual trials

| Run | Candidate | Best eligible clean lap | Session collisions |
|---|---|---:|---:|
| `onefile-20261027T142653Z` | Acceleration 14 / braking 16 model | 6.6207 s | 0 |
| `onefile-20261027T143244Z` | Spatial acceleration feedforward prototype | 6.7717 s after backoff | 1 |
| `onefile-20261027T143840Z` | Guarded spatial feedforward / earlier braking | 6.5537 s | 0 |
| `onefile-20261027T144531Z` | Longer motion preview, predicted pursuit origin | 7.0477 s | 0 |
| `onefile-20261027T145143Z` | Steering response gain / delay calibration | 7.4796 s | 1 |
| `onefile-20261027T145552Z` | Acceleration 18 / earlier braking | 6.9117 s after backoff | 1 |
| `onefile-20261027T145957Z` | Exact archived controller, stronger joint profile | 7.3027 s after backoff | 1 |
| `onefile-20261027T150637Z` | Strict dense-feasible line retention | 6.7467 s | 0 |
| `onefile-20261027T151102Z` | Larger curvature-rate penalty | 6.8337 s after backoff | 1 |

Every completed trial verified stationary shutdown. One additional preparation failure, `onefile-20261027T144754Z`, occurred before arming because transient window dimensions caused a pixel read outside the image. It has no invented lap time. Rectangle/geometry and pixel-bounds guards now reject that transient state; the real error and synthetic regression tests are retained.

## Changes retained

- **Proven performance settings remain selected:** no imposed command-speed ceiling, 10 m/s² lateral model, 12 m/s² combined grip model, 9 m/s² acceleration model, 12 m/s² braking model, command-derived acceleration feedforward, 0.04 s preview, measured-origin pursuit, linear motion prediction, steering-response gain 1.0 and curvature-rate penalty 0.003. Experimental options remain explicit, not silently promoted as faster defaults.
- **No changed default racing line/profile:** `logs/faster-development/20261027T142513Z/compatibility-regression.json` verifies that every default exported path/speed-profile CSV value exactly matches the measured 5.8218 s setup.
- **Planner fallback fix:** the original search could discard all improvements if its last coarse candidate failed dense validation, reverting to a roughly 31 m line. The new compatibility policy preserves an already valid original final result; if it fails, it retains a previous densely feasible improvement instead. Selected lines must still pass all dense clearance/footprint checks before arming. Strict per-move dense rejection is available as an experimental alternative, not selected by default.
- **More robust continuous tuning:** a pace is held for at least three eligible clean laps; recent lap medians, not one unusually fast lap, guide the next pace. A stale failure bracket cannot permanently trap the search below an already observed viable pace. Fastest individual clean laps remain separately recorded. This is a robustness change, not proof of faster laps.
- **Feedforward recovery guard:** the optional spatial-feedforward mode cannot demand positive acceleration against explicit off-path/heading recovery or a sharply falling target. It remains experimental and unselected.
- **Less unused load:** only the inactive, ownership-verified older `rr_twin` project container was stopped after confirming its campaign was no longer running and no physical devices were passed through. Its container, images and logs were retained. It had consumed approximately 1.2 CPU cores; this observation is not a claimed lap-speed improvement.

## Current continuous validation

The repaired Windows Start shortcut launched `onefile-20261027T151922Z`. Its saved observation had **18 simulator laps, 13 eligible clean full laps, zero collisions and best 6.0287 s**, still running with fresh telemetry and no imposed speed cap. Median-based tuning had actually executed. This does not inherit the older 5.8218 s into the new session or guarantee that later driving stays collision-free.

Keep using `RoboRacer One-File Start.lnk` / `RoboRacer One-File STOP.lnk` and the local dashboard. The selected controller is still `ONEFILE_RACER.py`; the executable one-file Linux archive is refreshed separately. Native Ubuntu GPU/UI driving remains unvalidated.

## Evidence

- Original source/configuration/result/telemetry per run: `logs/onefile-*/`.
- Development snapshots, explicitly offline-only estimates and regression: `logs/faster-development/20261027T142513Z/`.
- Preserved report dataset: `report_figures/data/faster-20261027/`.
- Comparison figure: `report_figures/08_faster_followup_attempts.png`.

All timings are actual simulator full-lap events with wall-time/distance/coverage checks. No clock, circuit, counters or vehicle physics were modified. This remains a privileged simulator-pose-assisted prior-memory benchmark, not LiDAR-only autonomy, physical-car safety validation or a globally optimal racing solution. STOP, stale-data, finite-command, host-liveness and watchdog checks remain; crash-stop and proximity/predictive speed governors remain disabled.
