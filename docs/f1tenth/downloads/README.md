# Separate F1TENTH simulation sandbox

This setup runs the genuine F1TENTH Gym physics and its ROS 2 bridge in an isolated
ROS Humble Docker image. It does **not** replace the AutoDRIVE simulator, controller,
maps, results or Docker image.

## Windows — no ROS commands to type

From the Capstone folder:

- **Start:** double-click `RUN_F1TENTH.bat`.
- **Stop:** double-click `STOP_F1TENTH.bat`.
- **Export an offline Ubuntu/Windows transfer kit:** `BUILD_F1TENTH_KIT.bat`.

Equivalent launchers live inside this folder as `START_F1TENTH.bat`,
`STOP_F1TENTH.bat` and `EXPORT_F1TENTH_KIT.bat`.

The launcher opens **http://localhost:15080**. The browser shows actual simulated
vehicle state, a provided reference, the recent actual trail and optionally sampled
actual LiDAR. Scroll to zoom, drag to pan, or enable **Follow vehicle**.
This is a labelled 2D browser rendering, **not Unity, RViz footage or a prerecorded
animation**. The physics is running in the container, not in the web page.

Keep the launcher window running/minimised. Prefer STOP over closing it. Closing
the launcher also expires the host lease and stops the owned F1TENTH simulation.
The existing Ubuntu WSL distribution and Docker Desktop must be available; the
launcher starts Docker Desktop if necessary. Nothing installs ROS on Windows.

## Track selection

There are **23 supplied maps** from `f1tenth/f1tenth_racetracks`, pinned in
`SOURCE_LOCK.json`. Select a track in the browser and press **Start / load selected
track**. The old session is stopped first; logs are retained in a new session
directory. STOP cancels queued/preparing starts. A later explicit Start is required
to resume.

Map pixels, resolution and origin remain unchanged. Supplied racelines are checked
against a sampled inflated footprint and a tracking allowance. If a supplied
raceline does not pass, the loader explicitly uses the provided centreline instead
and records why. Montreal and Shanghai have no upstream raceline CSV. No synthetic
track is substituted.

Melbourne currently uses the **supplied centreline**, approximately **474.20 m**,
because its supplied raceline hugs the wall too closely for the selected tracking
allowance. Its centreline's minimum sampled inflated-body clearance is 0.855 m.

## What is and is not validated

See `VALIDATION_REPORT.json` for the final machine-readable validation status.
Actual source checks, setup failures, sensor samples, requests and run results are
retained under `logs/` rather than replaced with invented successful results.

- All 23 maps passed actual map/line/clearance data checks. This is **not** a claim
  that every map has completed a driven lap.
- Melbourne completed a real standing-start full lap: **79.790 s simulation time**,
  **84.579 s wall time**, 474.840 m measured travel, 100% coverage, one native lap,
  zero collisions, and real-time factor **0.943**. It then verified stationary,
  disarmed stopping. Source run: `f1tenth-20261027T180359950770Z`.
- An earlier collision-free full lap took 62.110 simulated / 103.185 wall seconds
  (factor 0.602) and was **not** accepted as real-time-qualified. Its later collision
  and recovery-timeout failure remain recorded. Earlier host-lease/command-watchdog
  stops are also retained.
- This test host is **Windows + Ubuntu WSL + Linux Docker**. Native Ubuntu host
  driving on the other PC remains unvalidated. This 2D setup does not require
  hardware Vulkan or a GPU, unlike the separate official Linux AutoDRIVE app.
- These times are **not comparable to the 5.8218-second AutoDRIVE record**: track,
  length, physics, controller and timing context are different.
- The controlled recovery check deliberately applied steering/speed inputs into
  a real simulated wall after 65 seconds (beyond initial preflight). It recorded
  one actual collision, verified standstill, performed one upstream reset,
  resumed at pace 0.595 and travelled 8.060 m before verified stopping. It claims
  **no completed lap** and is never enabled by normal launchers.
- A queued Monza start followed immediately by STOP produced no later armed
  event. The independent ROS observer received real scans/odometry/map/native
  state without publishing motor commands. The actual Windows STOP launcher was
  tested while moving and verified stationary disarming.

## Controller and timing scope

The controller follows a **supplied track prior using real Gym odometry**. It is
not fresh SLAM, an unknown-track exploration test, or LiDAR-only autonomy. Actual
1,080-beam LiDAR, odometry, map and TF are available through ROS for further work.

The reference speed profile is recomputed using the stock vehicle parameters;
supplied speed columns are not assumed validated. `--max-speed 0` means **no extra
flat software ceiling**, not unlimited vehicle physics. The unmodified stock model
has a 20 m/s maximum and an initial controller pace of 70% of its derived profile.
Steering, grip, acceleration and braking constraints remain. The stock timestep is
0.01 s with RK4. No timestep, circuit, original map pixels or physics parameters are
changed to manufacture lap times.

Full laps require a real gate crossing, native-counter progress, sufficient actual
travel, at least 95% track coverage, no collision/teleport and simulation/wall-time
agreement. Standing-start and flying laps are labelled separately. Individual clean
laps can remain valid if another lap later collides. Collision data are actual Gym
observations. Collision recovery requires a verified stationary state, a genuine
upstream reset and a lower pace; it does not forge native lap counters or continue
through a latched collision. STOP/fault handling takes priority over recovery.

The upstream ROS bridge continues stepping after its two-lap `done` condition;
this wrapper follows that existing behaviour and records the raw flag. Later full
lap durations use actual Gym simulation-time differences, not the upstream
`lap_times` array that freezes after the initial episode.

## Safety and isolation

**Simulation only. Never use this controller to command a physical car.**

- Dedicated owned Docker container/network, ROS domain **77**, no physical
  VESC/CAN/serial devices and no host-network mode.
- Loopback-only web interface; nonce-protected control requests and origin checks.
- Host lease, stale ROS/native-step checks, finite commands and command watchdog.
- A heartbeat cache tolerates observed Docker Desktop/NTFS atomic-replace gaps
  using only the original timestamp of the last real matching heartbeat. It never
  extends the 1.5-second lease. Polling is bounded to avoid file I/O in every physics
  callback. STOP is sticky for the session and cancels older pending starts.
- Zero speed request while stopping, with curve-following steering during braking.
  If fresh stationary telemetry cannot be verified, only the owned F1TENTH physics
  container is closed. No unrelated containers or AutoDRIVE windows are stopped.
- Previous owned containers are renamed rather than deleted; all session evidence
  is retained. These measures do not certify collision avoidance or real-car safety.
- A local, non-secret kit identity keeps ownership stable across Windows/WSL
  Docker Desktop path aliases. It is not exported to another PC; each extracted
  kit gets its own local identity. Session paths are scoped relative to that kit.

## ROS interface

- `/drive`: `ackermann_msgs/AckermannDriveStamped`, speed and tire steering angle.
- `/scan`: actual 1,080-beam simulated `sensor_msgs/LaserScan`.
- `/ego_racecar/odom`: actual simulated `nav_msgs/Odometry`.
- `/map`: latched occupancy grid matching the native Gym threshold.
- `/f1tenth/reference_path`: latched supplied/validated reference.
- `/f1tenth/native_state`, `/f1tenth/status`: actual step/collision/lap diagnostics.
- TF: `map -> ego_racecar/base_link -> ego_racecar/laser`.

Physics steps are nominally 100 Hz; ROS observation publication and controller are
50 Hz. Both nodes remain in one persistent executor to avoid starving command
callbacks. The legacy Gym API is adapted to Humble without editing the upstream
physics/bridge source. The TF2 package family is aligned to avoid mixed-version
bindings. Actual source and image fingerprints are checked before arming.

## Native Ubuntu / another PC

1. Export with `BUILD_F1TENTH_KIT.bat`; the archive/checksum appear in
   `F1TENTH_Exports/` beside this folder.
2. Transfer/extract the complete archive into a writable Linux home directory.
3. Ensure native **x86_64 Linux**, Docker access for the current user, Python 3.10+
   and a desktop browser are installed. No host ROS or GPU driver is required.
4. Run `INSTALL_LINUX_SHORTCUTS.sh` **as a program** (allow executable permission if
   the file manager asks). Press Super and find **RoboRacer F1TENTH Start / STOP**.
5. The first launch loads the included offline image; no ROS terminal sessions or
   image rebuild are required. Keep the entire extracted folder together.

Windows on the second PC needs Ubuntu WSL and Docker Desktop first, then the
inside-folder Start/Stop `.bat` launchers. ARM/Jetson is not the bundled offline
image target. The actual new PC still needs its own driving/STOP rehearsal.

The export contains sources, maps, upstream licenses and the real dependency image,
but excludes active leases, web nonces and private session logs. Existing builds
can also be recreated from the pinned base/vendor/dependency sources. Image IDs may
differ between Docker stores; ordered filesystem/runtime fingerprints are checked
instead. Explicit local builds are recorded separately, not silently relabelled as
the original validated image.

## Evidence and provenance

Each `logs/sessions/f1tenth-*/` contains the runtime source snapshot, launch and run
manifests, actual first scan, reference/profile CSV, telemetry, lap/event journals,
state and verified-stop result where obtained. Do not treat setup/data checks as
lap-driving evidence. Failures are preserved. `SOURCE_LOCK.json`, `IMAGE_LOCK.json`
and `THIRD_PARTY_NOTICES.md` identify upstream commits, exact packaged input hashes,
dependencies and attribution. Preserve the upstream licenses when redistributing
the maps or source.
