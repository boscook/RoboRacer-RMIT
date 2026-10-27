#!/usr/bin/env python3
"""Separate F1TENTH Gym/ROS 2 sandbox. Not the AutoDRIVE benchmark.

All custom host, controller, audit and browser-view logic lives in this file.
Upstream simulator/ROS bridge sources and third-party maps remain external and
unmodified. Provided-map/line + simulator-odometry following is NOT fresh SLAM or
LiDAR-only autonomy. Default physics, its 0.01 s timestep and stock limits remain.
"""
import argparse
import base64
import csv
from datetime import datetime, timezone
import hashlib
import http.server
import io
import json
import math
import os
from pathlib import Path
import queue
import secrets
import signal
import socket
import subprocess
import sys
import threading
import time
from urllib.error import URLError
from urllib.parse import urlsplit
from urllib.request import Request, urlopen

OWNER = 'capstone-f1tenth-sandbox'
IMAGE = 'capstone-f1tenth:humble-v1'
PORT = 15080
DT = .01
np = None
LEASE_CACHE = {}
LEASE_READ_TIMES = {}
CANCEL_CACHE = {}


def utc():
    return datetime.now(timezone.utc).isoformat()


def read_json(path):
    try:
        return json.loads(Path(path).read_text(encoding='utf-8-sig'))
    except (OSError, ValueError):
        return None


def atomic_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_name(path.name + '.tmp-' + str(os.getpid()) + '-' + str(threading.get_ident()))
    temp.write_text(json.dumps(value, indent=2, allow_nan=False))
    temp.replace(path)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def clip(value, low, high):
    return max(low, min(high, value))


def wrap(value):
    return math.atan2(math.sin(value), math.cos(value))


def lease_fresh(session, token, force_read=False):
    key = (str(session), token)
    now = time.monotonic()
    if force_read or now-LEASE_READ_TIMES.get(key, -math.inf) >= .05:
        value = read_json(Path(session) / 'host-lease.json')
        LEASE_READ_TIMES[key] = now
    else:
        value = LEASE_CACHE.get(key)
    # Docker Desktop/NTFS can briefly report ENOENT during an atomic replace.
    # Retain only the last real matching heartbeat, with its ORIGINAL timestamp;
    # missing reads never extend the lease or hide a genuinely stale sender.
    if value is None:
        value = LEASE_CACHE.get(key)
    try:
        age = time.time() - float(value['updated_unix_s'])
        valid = value['run_id'] == token and math.isfinite(age) and -.5 <= age < 1.5
        if valid:
            LEASE_CACHE[key] = value
        else:
            LEASE_CACHE.pop(key, None)
        return valid
    except (TypeError, ValueError, KeyError):
        return False


def cancelled(root, session, started, force_read=False):
    key = (str(root), str(session), started)
    previous = CANCEL_CACHE.get(key)
    if previous and previous[1]:
        return True  # cancellation stays sticky for this run even if a file moves
    now = time.monotonic()
    if previous and not force_read and now-previous[0] < .02:
        return False
    stop = (Path(session) / 'STOP').exists()
    value = read_json(Path(root) / 'logs/stop-request.json') or {}
    stop = stop or value.get('requested_unix_s', 0) >= started
    CANCEL_CACHE[key] = (now, stop)
    return stop


def run(argv, check=True, **kwargs):
    result = subprocess.run([str(a) for a in argv], check=False, **kwargs)
    if check and result.returncode:
        detail = getattr(result, 'stderr', '') or getattr(result, 'stdout', '') or ''
        if isinstance(detail, bytes):
            detail = detail.decode(errors='replace')
        raise RuntimeError(str(argv[0]) + ' failed: ' + str(detail)[-2500:])
    return result


def resample(points, spacing=.20):
    points = np.asarray(points, dtype=float)
    good = np.r_[True, np.linalg.norm(np.diff(points, axis=0), axis=1) > 1e-5]
    points = points[good]
    if np.linalg.norm(points[0] - points[-1]) < .03:
        points = points[:-1]
    closed = np.vstack([points, points[0]])
    distance = np.r_[0., np.cumsum(np.linalg.norm(np.diff(closed, axis=0), axis=1))]
    assert distance[-1] > 10 and np.isfinite(distance).all(), 'Invalid supplied trajectory'
    samples = np.arange(max(100, int(math.ceil(distance[-1] / spacing))))
    samples = samples * distance[-1] / len(samples)
    return np.column_stack([np.interp(samples, distance, closed[:, i]) for i in (0, 1)])


def geometry(points):
    ds = np.linalg.norm(np.roll(points, -1, axis=0) - points, axis=1)
    window = max(2, int(round(.50 / max(float(ds.mean()), .01))))
    before, after = np.roll(points, window, axis=0), np.roll(points, -window, axis=0)
    a, b = points - before, after - points
    curvature = 2 * (a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]) / np.maximum(
        np.linalg.norm(a, axis=1) * np.linalg.norm(b, axis=1) * np.linalg.norm(a+b, axis=1), 1e-8)
    tangent = after - before
    heading = np.arctan2(tangent[:, 1], tangent[:, 0])
    return curvature, heading, ds


def clearances(points, field, origin, resolution):
    delta = points - np.asarray(origin[:2])
    c, s = math.cos(origin[2]), math.sin(origin[2])
    x = np.floor((c*delta[:, 0] + s*delta[:, 1]) / resolution).astype(int)
    y = np.floor((-s*delta[:, 0] + c*delta[:, 1]) / resolution).astype(int)
    valid = (x >= 0) & (y >= 0) & (x < field.shape[1]) & (y < field.shape[0])
    values = np.zeros(len(points))
    values[valid] = field[y[valid], x[valid]]
    return np.maximum(0., values - resolution/math.sqrt(2))


def load_track(lock, maps, track, output=None):
    """Read actual upstream data without changing pixels, scale or coordinates."""
    import yaml
    from PIL import Image
    from scipy.ndimage import distance_transform_edt
    entry = lock['tracks'][track]
    maps = Path(maps)
    map_yaml = maps / entry['map_yaml']
    map_image = maps / entry['map_image']
    for name, expected in entry['sha256'].items():
        assert sha(map_yaml.parent / name) == expected, 'Upstream track checksum mismatch: ' + name
    meta = yaml.safe_load(map_yaml.read_text())
    assert Path(meta['image']).name == map_image.name and map_yaml.with_suffix('.png') == map_image
    raw_image = Image.open(map_image)
    pixels = np.asarray(raw_image)
    assert pixels.ndim == 2, 'Upstream Gym requires a 2D grayscale/palette map; refusing a silently converted map'
    resolution = float(meta['resolution'])
    origin = [float(v) for v in meta['origin']]
    assert resolution > 0 and all(math.isfinite(v) for v in origin)
    # Exactly the binary threshold used by the unmodified upstream scan model.
    free = np.flipud(pixels > 128)
    field = distance_transform_edt(free) * resolution
    alternatives = []
    if entry.get('raceline'):
        data = np.loadtxt(maps / entry['raceline'], delimiter=';', comments='#')
        assert data.ndim == 2 and data.shape[1] >= 7
        alternatives.append(('supplied raceline', entry['raceline'], data[:, 1:3], float(data[:, 5].max())))
    center = np.loadtxt(maps / entry['centerline'], delimiter=',', comments='#')
    alternatives.append(('supplied centreline', entry['centerline'], center[:, :2], None))
    rejected = []
    selected = None
    for source, relative, coordinates, supplied_speed in alternatives:
        assert len(coordinates) > 50 and np.isfinite(coordinates).all()
        points = resample(coordinates)
        curvature, heading, ds = geometry(points)
        forward = np.column_stack([np.cos(heading), np.sin(heading)])
        normal = np.column_stack([-np.sin(heading), np.cos(heading)])
        probes = np.vstack([points + longitudinal*forward + lateral*normal
                            for longitudinal in (-.34, -.17, 0., .17, .34)
                            for lateral in (-.205, 0., .205)])
        body = clearances(probes, field, origin, resolution)
        centre = clearances(points, field, origin, resolution)
        if body.min() < .10 or centre.min() < .25:
            rejected.append(dict(source=source, reason='sampled inflated footprint/centre clearance',
                                 minimum_body_clearance_m=float(body.min()), minimum_centre_clearance_m=float(centre.min())))
            continue
        selected = (points, curvature, heading, ds, source, relative, supplied_speed, float(body.min()), float(centre.min()))
        break
    assert selected is not None, 'Neither supplied line is clearance-valid; no synthetic circuit fallback'
    points, curvature, heading, ds, source, relative, supplied_speed, body, centre = selected
    proof = dict(track=track, source='genuine third-party F1TENTH racetrack data',
                 prior=source, prior_file=relative, prior_sha256=sha(maps/relative), rejected_priors=rejected,
                 source_is_not_new_slam=True, not_lidar_only=True, points=len(points), reference_length_m=float(ds.sum()),
                 map_resolution_m=resolution, map_origin=origin, map_width=int(pixels.shape[1]), map_height=int(pixels.shape[0]),
                 map_image=entry['map_image'], map_yaml=entry['map_yaml'], input_sha256=entry['sha256'],
        minimum_sampled_inflated_body_clearance_m=body, minimum_centre_clearance_m=centre,
        required_sampled_body_clearance_m=.10, inflated_footprint_length_m=.68, inflated_footprint_width_m=.41,
                 supplied_speed_peak_mps=supplied_speed,
                 note='Provided line + actual Gym odometry benchmark. Supplied velocity column is not treated as validated for this vehicle.')
    if output:
        atomic_json(Path(output)/'track-provenance.json', proof)
    return dict(points=points, curvature=curvature, heading=heading, ds=ds, proof=proof,
                pixels=pixels, free=free, field=field, resolution=resolution, origin=origin,
                map_base=str(map_yaml.with_suffix('')), map_image=map_image)


def speed_profile(track, params, ceiling=0.):
    curvature, ds = track['curvature'], track['ds']
    lateral = params['mu'] * 9.81 * .70
    acceleration = params['a_max'] * .65
    braking = params['a_max'] * .65
    cap = min(params['v_max'], ceiling if ceiling > 0 else math.inf)
    velocity = np.minimum(cap, np.sqrt(lateral/np.maximum(abs(curvature), 1e-5)))
    steer = np.arctan((params['lf']+params['lr'])*curvature)
    assert abs(steer).max() < params['s_max'], 'Provided line exceeds stock steering geometry'
    velocity = np.minimum(velocity, params['sv_max']/np.maximum(abs(np.roll(steer, -1)-steer)/ds, 1e-5))
    for _ in range(6):
        for i in range(len(velocity)-1, -1, -1):
            j = (i+1) % len(velocity)
            velocity[i] = min(velocity[i], math.sqrt(velocity[j]**2 + 2*braking*ds[i]))
        for i in range(len(velocity)):
            j = (i+1) % len(velocity)
            power = acceleration * min(1., params['v_switch']/max(velocity[i], .1))
            velocity[j] = min(velocity[j], math.sqrt(velocity[i]**2 + 2*power*ds[i]))
    assert np.isfinite(velocity).all() and velocity.min() > 0
    return velocity


def verify_physics(lock):
    for name, expected in lock['repositories']['f1tenth_gym']['physics_sha256'].items():
        assert sha(Path('/opt/f1tenth_gym')/name) == expected, 'Physics source changed: ' + name
    assert sha('/opt/f1tenth_gym_ros/f1tenth_gym_ros/gym_bridge.py') == lock['repositories']['f1tenth_gym_ros']['bridge_sha256']


class LapAudit:
    """Full standing-start/flying laps from actual pose travel + native counters."""
    def __init__(self, points, heading, length):
        self.points, self.heading, self.length = points, heading, length
        self.completed = 0
        self.best = None
        self.records = []
        self.reset(0., 0., 0)

    def reset(self, sim_time, wall_time, native_laps):
        self.start_sim, self.start_wall = sim_time, wall_time
        self.previous = self.points[0].copy()
        self.signed = 0.
        self.distance = 0.
        self.coverage = set()
        self.collision = False
        self.teleport = False
        self.native_start = native_laps
        self.standing_start = True

    def update(self, position, index, sim_time, wall_time, native_laps, collision):
        delta = float(np.linalg.norm(position-self.previous))
        if delta > 2.:
            self.teleport = True
        else:
            self.distance += delta
        self.previous = position.copy()
        self.coverage.add(min(199, int(index*200/len(self.points))))
        self.collision |= bool(collision)
        tangent = np.array([math.cos(self.heading[0]), math.sin(self.heading[0])])
        signed = float(np.dot(position-self.points[0], tangent))
        lateral = abs(float(np.dot(position-self.points[0], [-tangent[1], tangent[0]])))
        crossed = self.signed < 0 <= signed and lateral < 1.5 and np.linalg.norm(position-self.points[0]) < 2.
        self.signed = signed
        if not crossed or self.distance < .70*self.length:
            return None
        elapsed, wall = sim_time-self.start_sim, wall_time-self.start_wall
        event = dict(completed=True, lap=self.completed+1, simulator_seconds=elapsed, wall_seconds=wall,
                     distance_m=self.distance, coverage_fraction=len(self.coverage)/200,
                     collision=self.collision, teleport=self.teleport, native_laps_start=self.native_start,
                     native_laps_finish=native_laps, standing_start=self.standing_start,
                     real_time_factor=elapsed/max(wall, .001))
        event['valid_clean_full_lap'] = (not self.collision and not self.teleport and
            self.distance >= .90*self.length and len(self.coverage) >= 190 and
            native_laps > self.native_start and elapsed > 0 and wall > 0 and
            abs(elapsed-wall) <= max(.5, .10*elapsed))
        self.completed += 1
        self.records.append(event)
        if event['valid_clean_full_lap'] and (self.best is None or elapsed < self.best):
            self.best = elapsed
        self.start_sim, self.start_wall = sim_time, wall_time
        self.distance = 0.
        self.coverage = set()
        self.collision = False
        self.teleport = False
        self.native_start = native_laps
        self.standing_start = False
        return event


def service(args):
    global np
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from ackermann_msgs.msg import AckermannDriveStamped
    from nav_msgs.msg import Odometry, OccupancyGrid, Path as RosPath
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    import f1tenth_gym_ros.gym_bridge as upstream
    import yaml
    assert os.environ.get('CAPSTONE_F1TENTH_SIM') == '1', 'Simulation-only service environment required'
    root, session = args.root, args.session
    assert session and session.is_dir()
    initial_lease_deadline = time.monotonic()+3
    while not lease_fresh(session, args.token):
        assert not cancelled(root, session, args.started), 'Cancelled before simulator preparation'
        assert time.monotonic() < initial_lease_deadline, 'No fresh initial host lease'
        time.sleep(.01)
    lock = read_json('/opt/SOURCE_LOCK.json')
    assert sha('/opt/SOURCE_LOCK.json') == sha(root/'SOURCE_LOCK.json'), 'Image/kit source locks differ; rebuild the F1TENTH image'
    verify_physics(lock)
    selected = load_track(lock, '/opt/racetracks', args.track, session)
    points, heading, ds = selected['points'], selected['heading'], selected['ds']
    start = [float(points[0, 0]), float(points[0, 1]), float(heading[0])]
    parameters = dict(ego_namespace='ego_racecar', ego_odom_topic='odom', ego_opp_odom_topic='opp_odom',
        ego_scan_topic='/scan', ego_drive_topic='/drive', opp_namespace='opp_racecar', opp_odom_topic='odom',
        opp_ego_odom_topic='opp_odom', opp_scan_topic='/opp_scan', opp_drive_topic='/opp_drive',
        scan_distance_to_base_link=.275, scan_fov=4.7, scan_beams=1080,
        map_path=selected['map_base'], map_img_ext='.png', num_agent=1,
        sx=start[0], sy=start[1], stheta=start[2], sx1=0., sy1=0., stheta1=0., kb_teleop=False)
    config = session/'bridge-parameters.yaml'
    config.write_text(yaml.safe_dump({'gym_bridge': {'ros__parameters': parameters}}))
    rclpy.init(args=['--ros-args', '--params-file', str(config)])

    class Bridge(upstream.GymBridge):
        def declare_parameter(self, name, *values, **options):
            # The upstream Foxy bridge omits parameter defaults. Supply the
            # actual generated values as typed Humble defaults, not new physics.
            if not values and 'value' not in options and name in parameters:
                values = (parameters[name],)
            return super().declare_parameter(name, *values, **options)

        def __init__(self):
            import f110_gym  # register the legacy environment explicitly in Gym 0.23
            original = upstream.gym.make
            upstream.gym.make = lambda name, **kw: original(name.split(':')[-1], **kw).unwrapped
            try:
                super().__init__()
            finally:
                upstream.gym.make = original
            self.timer.cancel()
            self.timer = self.create_timer(.02, self.timer_callback)
            self.angle_inc = self.env.sim.agents[0].scan_simulator.get_increment()
            self.native_pub = self.create_publisher(String, '/f1tenth/native_state', 10)
            self.last_command = 0.
            self.halted = False
            self.halt_reason = ''
            self.steps = 0
            self.reset_count = 0
            self.collision_count = 0
            self.was_collision = False

        def drive_callback(self, message):
            values = (message.drive.speed, message.drive.steering_angle)
            if not all(math.isfinite(v) for v in values) or values[0] < 0:
                self.halted = True
                self.halt_reason = 'invalid_command'
                return
            self.last_command = time.monotonic()
            if self.halted:
                message.drive.speed = 0.
            super().drive_callback(message)

        def ego_reset_callback(self, message):
            if self.halted or cancelled(root, session, args.started) or not lease_fresh(session, args.token):
                return
            if abs(self.ego_speed[0]) > .03:
                return
            super().ego_reset_callback(message)
            self.reset_count += 1
            self.was_collision = False

        def drive_timer_callback(self):
            if cancelled(root, session, args.started):
                self.halted = True
                self.halt_reason = 'explicit_stop'
            elif not lease_fresh(session, args.token):
                self.halted = True
                self.halt_reason = 'host_lease_lost'
            elif self.last_command and time.monotonic()-self.last_command > .35 and (
                    abs(self.ego_requested_speed) > .05 or abs(self.ego_speed[0]) > .05):
                self.halted = True
                self.halt_reason = 'command_watchdog'
            if self.halted:
                self.ego_requested_speed = 0.
                if abs(self.ego_speed[0]) < .03:
                    self.ego_steer = 0.
            previous = self.env.current_time
            super().drive_timer_callback()
            if self.env.current_time > previous:
                self.steps += 1
            collision = bool(self.obs['collisions'][0])
            if collision and not self.was_collision:
                self.collision_count += 1
            self.was_collision = collision
            native = dict(step=self.steps, simulator_time_s=float(self.env.current_time),
                native_laps=int(self.obs['lap_counts'][0]), native_lap_time_s=float(self.obs['lap_times'][0]),
                native_done=bool(self.done), collision=collision, collisions=self.collision_count,
                reset_count=self.reset_count, halted=self.halted, halt_reason=self.halt_reason)
            self.native_pub.publish(String(data=json.dumps(native)))

    bridge = Bridge()
    assert bridge.env.timestep == DT and bridge.env.num_agents == 1
    stock = dict(bridge.env.params)
    velocity = speed_profile(selected, stock, args.max_speed)
    length = float(ds.sum())
    with (session/'reference-profile.csv').open('w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['x', 'y', 'heading', 'curvature', 'planned_speed_mps', 'arc_length_m'])
        writer.writerows(zip(points[:, 0], points[:, 1], heading, selected['curvature'], velocity, np.r_[0., np.cumsum(ds[:-1])]))
    manifest = dict(created_utc=utc(), simulator='F1TENTH Gym, NOT AutoDRIVE',
        physics_source_unmodified=True, repositories=lock['repositories'], vehicle_parameters=stock,
        timestep_s=DT, integrator=bridge.env.integrator.name,
        compatibility='Gym wrapper removed for upstream positional-reset API; ROS publication is 50 Hz, stock physics steps are 100 Hz; scan angle_increment matches actual native scanner.',
        controller_mode=selected['proof']['prior']+' + actual simulator odometry (not LiDAR-only)',
        imposed_command_speed_cap_mps=args.max_speed if args.max_speed else None,
        physics_speed_limit_mps=stock['v_max'], initial_pace=args.pace,
        controller_source_sha256=sha(__file__), track=selected['proof'],
        intentional_collision_validation=bool(args.validation_recovery),
        loop_policy='Native Gym done after two laps is recorded; continuous stepping follows upstream ROS bridge behaviour without changing counters/physics. Full laps independently require native-counter progress, gate crossing, travel, coverage and simulation/wall-time agreement.')
    atomic_json(session/'run-manifest.json', manifest)
    stride = max(1, len(points)//1500)
    atomic_json(session/'track-view.json', dict(track=args.track, proof=selected['proof'],
        line=points[::stride].tolist(), display_line_stride=stride, start_pose=start,
        vehicle_length_m=stock['length'], vehicle_width_m=stock['width']))
    shutil_copy = __import__('shutil').copy2
    shutil_copy(__file__, session/'controller_source.py')

    class Controller(Node):
        def __init__(self):
            super().__init__('f1tenth_prior_controller', use_global_arguments=False)
            self.pose = None
            self.speed = 0.
            self.pose_time = self.scan_time = self.native_time = 0.
            self.native = {}
            self.scan = []
            self.scan_saved = False
            self.index = 0
            self.armed = False
            self.arm_attempted = False
            self.stopped = False
            self.phase = 'waiting_for_ros'
            self.reason = ''
            self.started_wall = None
            self.attempt = 1
            self.pace = args.pace
            self.clean_at_pace = 0
            self.last_collision_count = 0
            self.recovery_since = None
            self.expected_reset = None
            self.collision_test_started = None
            self.stationary_since = None
            self.stopped_verified = False
            self.last_status = 0.
            self.last_log = 0.
            self.last_request = (0., 0.)
            self.trail = []
            self.audit = LapAudit(points, heading, length)
            self.events = (session/'events.jsonl').open('w')
            self.laps = (session/'laps.jsonl').open('w')
            self.telemetry = (session/'telemetry.csv').open('w', newline='')
            self.csv = None
            self.drive = self.create_publisher(AckermannDriveStamped, '/drive', 1)
            self.reset_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
            self.status_pub = self.create_publisher(String, '/f1tenth/status', 10)
            self.create_subscription(Odometry, '/ego_racecar/odom', self.on_pose, qos_profile_sensor_data)
            self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
            self.create_subscription(String, '/f1tenth/native_state', self.on_native, 10)
            self.create_timer(.02, self.tick)
            self.publish_map()

        def event(self, kind, **data):
            value = dict(kind=kind, utc=utc(), attempt=self.attempt)
            value.update(data)
            self.events.write(json.dumps(value, allow_nan=False)+'\n')
            self.events.flush()

        def publish_map(self):
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
            self.map_pub = self.create_publisher(OccupancyGrid, '/map', qos)
            self.path_pub = self.create_publisher(RosPath, '/f1tenth/reference_path', qos)
            message = OccupancyGrid()
            message.header.frame_id = 'map'
            message.header.stamp = self.get_clock().now().to_msg()
            message.info.resolution = selected['resolution']
            message.info.width = selected['proof']['map_width']
            message.info.height = selected['proof']['map_height']
            message.info.origin.position.x, message.info.origin.position.y = selected['origin'][:2]
            message.info.origin.orientation.z = math.sin(selected['origin'][2]/2)
            message.info.origin.orientation.w = math.cos(selected['origin'][2]/2)
            message.data = np.where(selected['free'], 0, 100).astype(np.int8).ravel().tolist()
            self.map_pub.publish(message)
            path = RosPath()
            path.header = message.header
            for point, angle in zip(points, heading):
                pose = PoseStamped()
                pose.header = message.header
                pose.pose.position.x, pose.pose.position.y = map(float, point)
                pose.pose.orientation.z, pose.pose.orientation.w = math.sin(angle/2), math.cos(angle/2)
                path.poses.append(pose)
            self.path_pub.publish(path)

        def on_pose(self, message):
            p, q = message.pose.pose.position, message.pose.pose.orientation
            theta = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            speed = float(message.twist.twist.linear.x)
            if all(math.isfinite(v) for v in (p.x, p.y, theta, speed)):
                self.pose = (float(p.x), float(p.y), theta)
                self.speed = speed
                self.pose_time = time.monotonic()

        def on_scan(self, message):
            values = np.asarray(message.ranges, dtype=float)
            if len(values) == 1080 and np.isfinite(values).all():
                self.scan = values.tolist()
                self.scan_time = time.monotonic()
                if not self.scan_saved:
                    np.savez_compressed(session/'first-actual-scan.npz', ranges=values,
                        angle_min=message.angle_min, angle_increment=message.angle_increment,
                        range_min=message.range_min, range_max=message.range_max)
                    self.scan_saved = True

        def on_native(self, message):
            value = json.loads(message.data)
            if value.get('step', 0) != self.native.get('step', -1):
                self.native_time = time.monotonic()
            self.native = value

        def request(self, speed, steer):
            if not all(math.isfinite(v) for v in (speed, steer)) or speed < 0:
                self.stop('invalid_controller_command')
                speed = steer = 0.
            message = AckermannDriveStamped()
            message.header.stamp = self.get_clock().now().to_msg()
            message.drive.speed = float(clip(speed, 0., stock['v_max']))
            message.drive.steering_angle = float(clip(steer, stock['s_min'], stock['s_max']))
            self.drive.publish(message)
            self.last_request = (message.drive.speed, message.drive.steering_angle)

        def stop(self, reason):
            if not self.stopped:
                self.event('stop_requested', reason=reason)
            self.stopped = True
            self.armed = False
            self.phase = 'stopping'
            self.reason = reason

        def steering(self):
            x, y, angle = self.pose
            candidates = (self.index+np.arange(-25, 101)) % len(points)
            distance = np.linalg.norm(points[candidates]-[x, y], axis=1)
            self.index = int(candidates[np.argmin(distance)])
            error = float(distance.min())
            look = clip(.8+.12*abs(self.speed), 1., 2.5)
            target = self.index
            travel = 0.
            while travel < look:
                travel += ds[target]
                target = (target+1) % len(points)
            dx, dy = points[target]-[x, y]
            steer = math.atan((stock['lf']+stock['lr'])*2*(-math.sin(angle)*dx+math.cos(angle)*dy)/max(dx*dx+dy*dy, .02))
            return clip(steer, stock['s_min'], stock['s_max']), error

        def arm(self):
            assert self.pose and abs(self.speed) < .03 and not self.native.get('collision')
            assert math.hypot(self.pose[0]-start[0], self.pose[1]-start[1]) < .10
            assert not cancelled(root, session, args.started, force_read=True) and lease_fresh(session, args.token)
            assert self.drive.get_subscription_count() > 0
            self.audit.reset(float(self.native['simulator_time_s']), time.monotonic(), int(self.native['native_laps']))
            self.index = 0
            self.arm_attempted = True
            self.armed = True
            self.phase = 'driving'
            if self.started_wall is None:
                self.started_wall = time.monotonic()
            self.event('armed', supplied_prior=selected['proof']['prior'], start_pose=list(self.pose),
                       native_step=self.native['step'], native_simulator_time_s=self.native['simulator_time_s'], pace=self.pace)

        def tick(self):
            now = time.monotonic()
            fresh = self.pose is not None and now-self.pose_time < .25 and now-self.scan_time < .25 and now-self.native_time < .25
            valid_lease = lease_fresh(session, args.token)
            if cancelled(root, session, args.started):
                self.stop('explicit_stop')
            elif not valid_lease:
                self.stop('host_lease_lost')
            elif self.native.get('halted'):
                self.stop(self.native.get('halt_reason', 'bridge_halted'))
            elif self.armed and not fresh:
                self.stop('stale_ros_or_native_step')
            requested, steer, error = 0., 0., 0.
            if self.pose is not None and fresh:
                steer, error = self.steering()
            if not self.stopped and not self.arm_attempted and fresh and self.native.get('step', 0) >= 20:
                try:
                    self.arm()
                except AssertionError as e:
                    self.stop('preflight_failed: '+str(e))
            if self.native.get('collisions', 0) > self.last_collision_count:
                self.last_collision_count = self.native['collisions']
                self.audit.collision = True
                self.event('collision', native=dict(self.native), pose=list(self.pose) if self.pose else None,
                           invalidated_current_lap=True, partial_distance_m=self.audit.distance,
                           partial_coverage_fraction=len(self.audit.coverage)/200, completed_lap=False)
                self.armed = False
                self.phase = 'collision_recovery'
                self.recovery_since = None
                self.expected_reset = None
                self.pace = max(.30, self.pace*.85)
            if self.phase == 'collision_recovery' and not self.stopped and fresh:
                if abs(self.speed) < .03:
                    if self.recovery_since is None:
                        self.recovery_since = now
                    if now-self.recovery_since > .5 and self.expected_reset is None:
                        message = PoseWithCovarianceStamped()
                        message.header.frame_id = 'map'
                        message.pose.pose.position.x, message.pose.pose.position.y = start[:2]
                        message.pose.pose.orientation.z, message.pose.pose.orientation.w = math.sin(start[2]/2), math.cos(start[2]/2)
                        self.expected_reset = self.native.get('reset_count', 0)+1
                        self.reset_pub.publish(message)
                        self.event('official_reset_requested', stationary_verified=True, lower_pace=self.pace)
                    if (self.expected_reset is not None and self.native.get('reset_count', 0) >= self.expected_reset and
                            math.hypot(self.pose[0]-start[0], self.pose[1]-start[1]) < .10):
                        self.attempt += 1
                        self.arm_attempted = False
                        self.phase = 'waiting_for_ros'
                        self.recovery_since = None
                        self.expected_reset = None
                        self.index = 0
                else:
                    self.recovery_since = None
            if self.armed and fresh and not self.stopped:
                travel, indices = 0., [self.index]
                while travel < max(.10, abs(self.speed)*.08):
                    travel += ds[indices[-1]]
                    indices.append((indices[-1]+1) % len(points))
                requested = float(np.min(velocity[indices]))*self.pace
                if error > .25:
                    requested *= max(.25, 1-1.2*(error-.25))
                if args.validation_recovery and self.attempt == 1 and now-self.started_wall >= 65.:
                    if self.collision_test_started is None:
                        self.collision_test_started = now
                        self.event('intentional_simulation_collision_test',
                            elapsed_wall_s=now-self.started_wall, beyond_initial_preflight_deadline=True,
                            note='Diagnostic steering/speed input into the real stock map wall. No collision flag, pose or physics is forged.')
                    if now-self.collision_test_started < 5.:
                        requested, steer = 2., stock['s_max']
                event = self.audit.update(np.array(self.pose[:2]), self.index, self.native['simulator_time_s'], now,
                                          self.native['native_laps'], self.native['collision'])
                if event:
                    event.update(track=args.track, attempt=self.attempt, total_collisions=self.native['collisions'],
                                 pace=self.pace, simulator='F1TENTH Gym (not AutoDRIVE)')
                    self.laps.write(json.dumps(event)+'\n')
                    self.laps.flush()
                    self.event('completed_lap', **event)
                    if event['valid_clean_full_lap']:
                        self.clean_at_pace += 1
                        print('GENUINE CLEAN F1TENTH LAP', json.dumps(event), flush=True)
                    if args.validation_laps and sum(e['valid_clean_full_lap'] for e in self.audit.records) >= args.validation_laps:
                        self.stop('validation_laps_complete')
                if args.validation_recovery and self.attempt >= 2 and self.audit.distance >= 8.:
                    self.event('collision_recovery_validation_passed',
                        measured_post_reset_distance_m=self.audit.distance, native=dict(self.native),
                        stationary_stop_before_reset_verified=True, no_complete_lap_claimed=True)
                    self.stop('collision_recovery_validation_complete')
            if self.stopped or not self.armed:
                requested = 0.
                if abs(self.speed) < .03:
                    steer = 0.
            self.request(requested, steer)
            if self.stopped and fresh and abs(self.speed) < .03 and requested == 0:
                if self.stationary_since is None:
                    self.stationary_since = now
                if now-self.stationary_since >= .4:
                    self.stopped_verified = True
                    self.phase = 'stopped'
            else:
                self.stationary_since = None
                self.stopped_verified = False
            if self.pose and fresh and (not self.trail or math.hypot(self.pose[0]-self.trail[-1][0], self.pose[1]-self.trail[-1][1]) > .25):
                self.trail.append(list(self.pose[:2]))
                self.trail = self.trail[-1000:]
            if now-self.last_log >= .05:
                row = dict(wall_unix_s=time.time(), simulator_time_s=self.native.get('simulator_time_s'), native_step=self.native.get('step'),
                    x=self.pose[0] if self.pose else None, y=self.pose[1] if self.pose else None, yaw=self.pose[2] if self.pose else None,
                    measured_speed_mps=self.speed, requested_speed_mps=requested, steering_rad=steer, path_error_m=error,
                    path_index=self.index, armed=self.armed, fresh=bool(fresh), collisions=self.native.get('collisions', 0),
                    native_laps=self.native.get('native_laps', 0), attempt=self.attempt, pace=self.pace)
                if self.csv is None:
                    self.csv = csv.DictWriter(self.telemetry, fieldnames=list(row))
                    self.csv.writeheader()
                self.csv.writerow(row)
                self.telemetry.flush()
                self.last_log = now
            if now-self.last_status >= .10:
                status = dict(kind=OWNER, run_id=args.token, updated_unix_s=time.time(), track=args.track,
                    phase=self.phase, reason=self.reason, ready=bool(fresh), armed=self.armed,
                    stopped_verified=self.stopped_verified, pose=list(self.pose) if self.pose else None,
                    measured_speed_mps=self.speed, requested_speed_mps=requested, steering_rad=steer, path_error_m=error,
                    native=dict(self.native), laps=self.audit.completed, eligible_clean_laps=sum(e['valid_clean_full_lap'] for e in self.audit.records),
                    best_clean_simulator_lap_s=self.audit.best, pace=self.pace, attempt=self.attempt,
                    reference_length_m=length, imposed_command_speed_cap_mps=args.max_speed if args.max_speed else None,
                    physics_speed_limit_mps=stock['v_max'], timestep_s=DT, controller_mode=selected['proof']['prior']+' + Gym pose',
                    scan_beams=len(self.scan), lidar_display_stride=12, lidar_display_ranges=self.scan[::12],
                    lidar_angle_min=-2.35, lidar_angle_increment=bridge.angle_inc, lidar_offset_m=.275, trail=self.trail,
                    no_autodrive_time_comparison=True)
                atomic_json(session/'status.json', status)
                self.status_pub.publish(String(data=json.dumps({k:v for k,v in status.items() if k not in ['trail','lidar_display_ranges']})))
                self.last_status = now
            if args.max_seconds and self.started_wall is not None and now-self.started_wall > args.max_seconds:
                self.stop('validation_duration_complete')

    controller = Controller()
    initial_lease_deadline = time.monotonic()+3
    while not lease_fresh(session, args.token):
        assert not cancelled(root, session, args.started), 'Cancelled before arming'
        assert time.monotonic() < initial_lease_deadline, 'No fresh host lease after ROS preparation'
        time.sleep(.01)
    # Keep both nodes in one executor. Alternating rclpy.spin_once(node) adds /
    # removes nodes and rebuilds its ready iterator, which can starve /drive
    # subscription callbacks behind continuously ready physics timers.
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(bridge)
    executor.add_node(controller)
    reported_stop = False
    try:
        deadline = time.monotonic()+60
        while rclpy.ok():
            executor.spin_once(timeout_sec=.005)
            if controller.started_wall is None and not controller.arm_attempted and not controller.stopped and time.monotonic() > deadline:
                controller.stop('ros_preflight_timeout')
            if controller.stopped_verified and not reported_stop:
                controller.last_status = 0.
                controller.tick()
                result = dict(reason=controller.reason, stationary_disarmed_verified=True,
                    best_clean_simulator_lap_s=controller.audit.best, completed_laps=controller.audit.completed,
                    eligible_clean_laps=sum(e['valid_clean_full_lap'] for e in controller.audit.records),
                    total_collisions=controller.native.get('collisions', 0), attempts=controller.attempt,
                    simulator='F1TENTH Gym (not AutoDRIVE)', native_linux_host_verified=False)
                atomic_json(session/'result.json', result)
                reported_stop = True
                if args.validation_laps or args.max_seconds:
                    print('VERIFIED F1TENTH RESULT', json.dumps(result), flush=True)
                    break
    except KeyboardInterrupt:
        controller.stop('process_signal')
    finally:
        controller.stop(controller.reason or 'service_exit')
        until = time.monotonic()+5
        while rclpy.ok() and not controller.stopped_verified and time.monotonic() < until:
            executor.spin_once(timeout_sec=.005)
        controller.events.close()
        controller.laps.close()
        controller.telemetry.close()
        controller.destroy_node()
        bridge.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


def docker_owned(name, key):
    result = run(['docker', 'inspect', name], check=False, capture_output=True, text=True)
    if result.returncode:
        return False
    try:
        labels = json.loads(result.stdout)[0]['Config']['Labels'] or {}
        return labels.get('project') == OWNER and labels.get('root_key') == key
    except (ValueError, KeyError, IndexError):
        return False


def kit_key(root):
    """Stable identity across /mnt/c and Docker Desktop's aliases of that folder."""
    root = Path(root)
    identity = read_json(root/'KIT_ID.json') or {}
    value = identity.get('root_key', '')
    valid = isinstance(value, str) and len(value) == 10 and all(c in '0123456789abcdef' for c in value)
    if not valid:
        active = read_json(root/'logs/active.json') or {}
        adopted = active.get('root_key', '') if active.get('kind') == OWNER else ''
        valid = isinstance(adopted, str) and len(adopted) == 10 and all(c in '0123456789abcdef' for c in adopted)
        value = adopted if valid else secrets.token_hex(5)
        atomic_json(root/'KIT_ID.json', dict(kind=OWNER, root_key=value, created_utc=utc(),
            note='Local instance identity, not a credential. Not included in portable exports.'))
    return value


def active_session(root, active):
    stored = active.get('session_relative') or active.get('session')
    if not stored:
        return None
    name = Path(stored).name
    if not name.startswith('f1tenth-') or '/' in name or '\\' in name:
        raise RuntimeError('Invalid owned session name')
    session = Path(root)/'logs/sessions'/name
    assert session.resolve().is_relative_to((Path(root)/'logs/sessions').resolve()), 'Unowned session path'
    return session


def image_fingerprint(info):
    configuration = info['Config']
    payload = dict(layers=info['RootFS']['Layers'], environment=configuration.get('Env'),
        entrypoint=configuration.get('Entrypoint'), command=configuration.get('Cmd'),
        working_directory=configuration.get('WorkingDir'), user=configuration.get('User'))
    return hashlib.sha256(json.dumps(payload, sort_keys=True, separators=(',', ':')).encode()).hexdigest()


def open_browser(url):
    if 'microsoft' in __import__('platform').release().lower():
        windows_open(url)
    else:
        import webbrowser
        webbrowser.open(url)


def windows_open(target):
    value = base64.b64encode(target.encode()).decode()
    script = "$t=[Text.Encoding]::UTF8.GetString([Convert]::FromBase64String('"+value+"'));Start-Process -FilePath $t"
    encoded = base64.b64encode(script.encode('utf-16le')).decode()
    run(['powershell.exe', '-NoProfile', '-EncodedCommand', encoded], check=False,
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def stop_from_cli(root, port):
    root = root.resolve()
    atomic_json(root/'logs/stop-request.json', dict(requested_unix_s=time.time(), reason='explicit_stop_shortcut'))
    active = read_json(root/'logs/active.json') or {}
    try:
        body = json.dumps({'nonce': active.get('nonce', '')}).encode()
        request = Request('http://127.0.0.1:'+str(port)+'/api/stop', data=body,
                          headers={'Content-Type': 'application/json'}, method='POST')
        with urlopen(request, timeout=2) as response:
            response.read()
    except (URLError, OSError):
        pass
    session = active_session(root, active)
    if session is None:
        print('F1TENTH STOP recorded; no active session. AutoDRIVE was not touched.')
        return
    atomic_json(session/'STOP', dict(requested_unix_s=time.time()))
    deadline = time.monotonic()+8
    while time.monotonic() < deadline:
        status = read_json(session/'status.json') or {}
        if status.get('stopped_verified') and time.time()-status.get('updated_unix_s', 0) < 1.:
            print('F1TENTH STOP VERIFIED: fresh stationary state, zero speed request, disarmed.')
            return
        time.sleep(.15)
    key = kit_key(root)
    name = active.get('container', '')
    if name and docker_owned(name, key):
        run(['docker', 'stop', '--timeout', '3', name], check=False, capture_output=True)
        atomic_json(session/'unverified-stop-container-closure.json', dict(
            reason='Fresh stationary state not verified within STOP deadline; only the owned F1TENTH physics container was closed.',
            stationary_was_not_verified=True, closure_requested=True))
        print('F1TENTH physics container closed as fallback; stationary telemetry was not verified. AutoDRIVE was not touched.')


def install_native_shortcuts(root):
    root = root.resolve()
    directory = Path(os.environ.get('XDG_DATA_HOME', str(Path.home()/'.local/share')))/'applications'
    directory.mkdir(parents=True, exist_ok=True)
    path = str(root).replace('\\', '\\\\').replace(' ', '\\s')
    for action, suffix in [([], 'Start'), (['--stop'], 'STOP')]:
        code = "import os,pathlib;os.execvp('python3',['python3',str(pathlib.Path.cwd()/'F1TENTH_RACER.py')]+"+repr(action)+")"
        content = ('[Desktop Entry]\nType=Application\nName=RoboRacer F1TENTH '+suffix+'\n'
                   'Path='+path+'\nTerminal=false\nIcon=applications-games\nExec=python3 -c "'+code+'"\n')
        target = directory/('capstone-f1tenth-'+suffix.lower()+'.desktop')
        if target.exists() and 'RoboRacer F1TENTH' not in target.read_text():
            raise RuntimeError('Unrelated application shortcut exists: '+str(target))
        target.write_text(content)
        target.chmod(0o755)
    print('Installed native application-menu F1TENTH Start/STOP shortcuts. Installation itself did not drive.')


def host_main(args):
    import fcntl
    root = args.root.resolve()
    root.joinpath('logs').mkdir(exist_ok=True)
    key = kit_key(root)
    lock = read_json(root/'SOURCE_LOCK.json')
    assert lock and args.track in lock['tracks'], 'Track/source manifest missing'
    if args.stop:
        return stop_from_cli(root, args.port)
    base_url = 'http://127.0.0.1:'+str(args.port)
    try:
        with urlopen(base_url+'/api/health', timeout=1) as response:
            existing = json.loads(response.read())
        if existing.get('kind') != OWNER or existing.get('root_key') != key:
            raise RuntimeError('Port belongs to another application; refusing to replace it')
        active = read_json(root/'logs/active.json') or {}
        with urlopen(base_url+'/api/state', timeout=1) as response:
            state = json.loads(response.read())
        if not state.get('simulation', {}).get('armed') and state.get('host_phase') not in ['building', 'preparing']:
            request = Request(base_url+'/api/start', data=json.dumps(dict(nonce=active.get('nonce'), track=args.track)).encode(),
                              headers={'Content-Type': 'application/json'}, method='POST')
            with urlopen(request, timeout=2) as response:
                response.read()
        if not args.no_browser:
            open_browser(base_url)
        print('Existing F1TENTH application opened; an active run was not reset.')
        return
    except (URLError, OSError):
        pass
    host_lock = (root/'logs/host.lock').open('a+')
    try:
        fcntl.flock(host_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        raise RuntimeError('F1TENTH launcher is already preparing; check its window/logs')
    ready = run(['docker', 'info'], check=False, capture_output=True)
    if ready.returncode:
        if 'microsoft' in __import__('platform').release().lower():
            windows_open(r'C:\Program Files\Docker\Docker\Docker Desktop.exe')
            deadline = time.monotonic()+120
            while time.monotonic() < deadline:
                if run(['docker', 'info'], check=False, capture_output=True).returncode == 0:
                    break
                time.sleep(2)
            else:
                raise RuntimeError('Docker Desktop did not become ready')
        else:
            raise RuntimeError('Docker is not available to this user. Check Docker installation/permissions.')

    class Host:
        def __init__(self):
            self.mutex = threading.RLock()
            self.nonce = secrets.token_urlsafe(24)
            self.phase = 'idle'
            self.error = ''
            self.generation = 0
            self.current = None
            self.shutdown = threading.Event()
            self.requests = queue.Queue()
            self.container = 'rr_f1tenth_'+key
            self.network = 'rr_f1tenth_net_'+key
            self.last_track = args.track
            self.host_log = root/'logs/host.log'
            self.status_cache = {}

        def info(self):
            with self.mutex:
                current = self.current
                phase, error, generation = self.phase, self.error, self.generation
            status = read_json(current/'status.json') if current else None
            if current and status is not None:
                self.status_cache[str(current)] = status
            elif current:
                status = self.status_cache.get(str(current))
            stale = status is not None and time.time()-status.get('updated_unix_s', 0) > 1.
            text = ''
            if self.host_log.exists() and phase in ['building', 'preparing', 'error']:
                with self.host_log.open(errors='replace') as stream:
                    text = stream.read()[-5000:]
            if current and (current/'service.log').exists() and phase in ['preparing', 'error']:
                text += '\n'+(current/'service.log').read_text(errors='replace')[-4000:]
            return dict(kind=OWNER, root_key=key, host_phase=phase, error=error, generation=generation,
                instance=hashlib.sha256(self.nonce.encode()).hexdigest()[:12],
                tracks=list(lock['tracks']), selected_track=self.last_track, simulation=status or {},
                telemetry_stale=bool(stale), preparation_log_tail=text,
                note='Actual F1TENTH Gym state; separate physics/track benchmark, not AutoDRIVE or fresh SLAM.')

        def start(self, track):
            if track not in lock['tracks']:
                raise ValueError('Unknown catalogued track')
            with self.mutex:
                self.generation += 1
                request = dict(track=track, generation=self.generation, started=time.time())
                self.last_track = track
                atomic_json(root/'settings.json', dict(track=track))
                self.phase = 'preparing'
                self.error = ''
                self.requests.put(request)
            return request

        def stop(self, reason='explicit_stop'):
            with self.mutex:
                self.generation += 1
                self.phase = 'stopping'
                current = self.current
            atomic_json(root/'logs/stop-request.json', dict(requested_unix_s=time.time(), reason=reason))
            if current:
                atomic_json(current/'STOP', dict(requested_unix_s=time.time(), reason=reason))
            else:
                self.phase = 'stopped'

        def is_cancelled(self, request):
            return self.shutdown.is_set() or request['generation'] != self.generation or (
                (read_json(root/'logs/stop-request.json') or {}).get('requested_unix_s', 0) >= request['started'])

        def stop_old(self):
            current = self.current
            if current:
                if not (current/'STOP').exists():
                    atomic_json(current/'STOP', dict(requested_unix_s=time.time(), reason='explicit_new_session_handover'))
                deadline = time.monotonic()+8
                while time.monotonic() < deadline:
                    status = read_json(current/'status.json') or {}
                    if status.get('stopped_verified') and time.time()-status.get('updated_unix_s', 0) < 1.:
                        break
                    if not docker_owned(self.container, key):
                        break
                    time.sleep(.15)
            inspection = run(['docker', 'inspect', self.container], check=False, capture_output=True, text=True)
            if inspection.returncode == 0:
                if not docker_owned(self.container, key):
                    raise RuntimeError('Container name is not owned by this kit')
                run(['docker', 'stop', '--timeout', '3', self.container], check=False, capture_output=True)
                stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
                run(['docker', 'rename', self.container, self.container+'_previous_'+stamp], capture_output=True)

        def worker(self):
            while not self.shutdown.is_set():
                try:
                    request = self.requests.get(timeout=.2)
                except queue.Empty:
                    continue
                if self.is_cancelled(request):
                    continue
                try:
                    self.stop_old()
                    if self.is_cancelled(request):
                        continue
                    built_here = False
                    image = run(['docker', 'image', 'inspect', IMAGE], check=False, capture_output=True)
                    if image.returncode:
                        offline = root/'images/f1tenth-image.tar.gz'
                        if offline.exists():
                            self.phase = 'loading_offline_image'
                            export_manifest = read_json(root/'EXPORT_MANIFEST.json') or {}
                            if export_manifest.get('image_archive_sha256') and sha(offline) != export_manifest['image_archive_sha256']:
                                raise RuntimeError('Offline image archive checksum mismatch')
                            with self.host_log.open('a') as log:
                                result = run(['docker', 'load', '--input', offline], check=False, stdout=log, stderr=subprocess.STDOUT)
                            if result.returncode:
                                raise RuntimeError('Offline image loading failed; see logs/host.log')
                        else:
                            self.phase = 'building'
                            with self.host_log.open('a') as log:
                                build = subprocess.Popen(['docker', 'build', '--progress', 'plain', '-t', IMAGE, str(root)],
                                                         stdout=log, stderr=subprocess.STDOUT)
                                code = build.wait()
                            if code:
                                raise RuntimeError('Dependency image build failed; see logs/host.log')
                            built_here = True
                    inspection_image = run(['docker', 'image', 'inspect', IMAGE], capture_output=True, text=True)
                    image_info = json.loads(inspection_image.stdout)[0]
                    image_lock = read_json(root/'IMAGE_LOCK.json')
                    actual_fingerprint = image_fingerprint(image_info)
                    built_record = read_json(root/'logs/locally-built-image.json') or {}
                    if built_here:
                        built_record = dict(created_utc=utc(), runtime_fingerprint=actual_fingerprint,
                            source_lock_sha256=sha(root/'SOURCE_LOCK.json'), dockerfile_sha256=sha(root/'Dockerfile'))
                        atomic_json(root/'logs/locally-built-image.json', built_record)
                    locally_verified = (built_record.get('runtime_fingerprint') == actual_fingerprint and
                        built_record.get('source_lock_sha256') == sha(root/'SOURCE_LOCK.json') and
                        built_record.get('dockerfile_sha256') == sha(root/'Dockerfile'))
                    if image_lock and actual_fingerprint != image_lock['runtime_fingerprint'] and not locally_verified:
                        raise RuntimeError('F1TENTH image differs from the packaged or locally built runtime fingerprint; refusing to arm')
                    if self.is_cancelled(request):
                        continue
                    inspection = run(['docker', 'network', 'inspect', self.network], check=False, capture_output=True, text=True)
                    if inspection.returncode:
                        run(['docker', 'network', 'create', '--label', 'project='+OWNER, '--label', 'root_key='+key, self.network], capture_output=True)
                    else:
                        labels = json.loads(inspection.stdout)[0].get('Labels') or {}
                        if labels.get('project') != OWNER or labels.get('root_key') != key:
                            raise RuntimeError('Existing Docker network is not owned by this kit')
                    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
                    session = root/'logs/sessions'/('f1tenth-'+stamp)
                    session.mkdir(parents=True, exist_ok=False)
                    token = 'f1tenth-'+secrets.token_hex(12)
                    __import__('shutil').copy2(__file__, session/'runtime_source.py')
                    self.current = session
                    self.phase = 'preparing'
                    atomic_json(session/'host-lease.json', dict(run_id=token, updated_unix_s=time.time()))
                    atomic_json(session/'launch.json', dict(track=request['track'], requested_utc=utc(), root_key=key,
                        run_id=token, controller_source_sha256=sha(__file__), started_unix_s=request['started'],
                        max_speed=args.max_speed, pace=args.pace, no_autodrive_changes=True,
                        image_id=image_info['Id'], runtime_fingerprint=actual_fingerprint))
                    atomic_json(root/'logs/active.json', dict(kind=OWNER, root_key=key, nonce=self.nonce, port=args.port,
                        container=self.container, session=str(session), session_relative='logs/sessions/'+session.name, run_id=token))
                    if self.is_cancelled(request):
                        atomic_json(session/'STOP', dict(reason='cancelled_before_container_start'))
                        continue
                    command = ['docker', 'run', '-d', '--init', '--name', self.container,
                        '--label', 'project='+OWNER, '--label', 'root_key='+key, '--network', self.network,
                        '--memory', '3g', '--pids-limit', '512', '-e', 'CAPSTONE_F1TENTH_SIM=1',
                        '-e', 'ROS_DOMAIN_ID=77', '-v', str(root)+':/kit:ro', '-v', str(session)+':/session',
                        IMAGE, 'xvfb-run', '-a', '-s', '-screen 0 1280x720x24',
                        'python3', '-u', '/session/runtime_source.py', '--service', '--root', '/kit', '--session', '/session',
                        '--token', token, '--started', str(request['started']), '--track', request['track'],
                        '--pace', str(args.pace), '--max-speed', str(args.max_speed),
                        '--validation-laps', str(args.validation_laps), '--max-seconds', str(args.max_seconds)]
                    if args.validation_recovery:
                        command.append('--validation-recovery')
                    run(command, capture_output=True, text=True)
                    stream = (session/'service.log').open('a')
                    subprocess.Popen(['docker', 'logs', '--follow', self.container], stdout=stream, stderr=subprocess.STDOUT)
                    stream.close()
                    if self.is_cancelled(request):
                        atomic_json(session/'STOP', dict(reason='cancelled_during_container_start'))
                    deadline = time.monotonic()+120
                    while time.monotonic() < deadline and not self.is_cancelled(request):
                        status = read_json(session/'status.json') or {}
                        if status.get('armed'):
                            self.phase = 'running'
                            break
                        if status.get('phase') == 'stopped':
                            self.phase = 'stopped'
                            break
                        detail = run(['docker', 'inspect', self.container], check=False, capture_output=True, text=True)
                        if detail.returncode == 0:
                            state = json.loads(detail.stdout)[0]['State']
                            if not state['Running']:
                                raise RuntimeError('Simulator service exited before ready. Inspect this session service.log.')
                        time.sleep(.2)
                    else:
                        if not self.is_cancelled(request):
                            raise RuntimeError('Simulator did not pass ready/arming checks before the deadline')
                except Exception as e:
                    self.error = str(e)
                    self.phase = 'error'
                    print('F1TENTH PREPARATION ERROR:', e, flush=True)
                    if self.current:
                        atomic_json(self.current/'STOP', dict(reason='preparation_error', error=str(e)))

        def heartbeat(self):
            while not self.shutdown.is_set():
                active = read_json(root/'logs/active.json') or {}
                current = self.current
                if current and active.get('nonce') == self.nonce and active.get('session') == str(current) and active.get('run_id'):
                    atomic_json(current/'host-lease.json', dict(run_id=active['run_id'], updated_unix_s=time.time()))
                    status = read_json(current/'status.json') or {}
                    if status.get('phase') == 'stopped' and self.phase not in ['preparing', 'building', 'error']:
                        self.phase = 'stopped'
                    if self.phase == 'stopping':
                        requested = (read_json(root/'logs/stop-request.json') or {}).get('requested_unix_s', 0)
                        if not status.get('stopped_verified') and requested and time.time()-requested > 8:
                            if docker_owned(self.container, key):
                                run(['docker', 'stop', '--timeout', '3', self.container], check=False, capture_output=True)
                                atomic_json(current/'unverified-stop-container-closure.json', dict(reason='STOP deadline', stationary_was_not_verified=True))
                            self.phase = 'stopped'
                self.shutdown.wait(.15)

    host = Host()

    class Handler(http.server.BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def reply(self, value, code=200, content_type='application/json'):
            data = value if isinstance(value, bytes) else json.dumps(value).encode()
            self.send_response(code)
            self.send_header('Content-Type', content_type)
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            path = urlsplit(self.path).path
            if path == '/api/health':
                return self.reply(dict(kind=OWNER, root_key=key))
            if path == '/api/state':
                return self.reply(host.info())
            if path == '/api/track':
                return self.reply(read_json(host.current/'track-view.json') if host.current else None)
            if path == '/map.png':
                info = read_json(host.current/'track-view.json') if host.current else None
                track = info.get('track') if info else host.last_track
                if track not in lock['tracks']:
                    return self.send_error(404)
                image = root/'vendor/f1tenth_racetracks'/lock['tracks'][track]['map_image']
                return self.reply(image.read_bytes(), content_type='image/png')
            if path == '/':
                return self.reply(browser_page(host.nonce).encode(), content_type='text/html; charset=utf-8')
            return self.send_error(404)

        def do_POST(self):
            if int(self.headers.get('Content-Length', '0')) > 4096:
                return self.send_error(413)
            origin = self.headers.get('Origin')
            if origin and origin not in [base_url, 'http://localhost:'+str(args.port)]:
                return self.send_error(403)
            try:
                data = json.loads(self.rfile.read(int(self.headers.get('Content-Length', '0'))))
                if data.get('nonce') != host.nonce:
                    return self.send_error(403)
                if self.path == '/api/stop':
                    host.stop()
                    return self.reply(dict(stop_requested=True))
                if self.path == '/api/close':
                    host.stop('explicit_application_close')
                    threading.Thread(target=server.shutdown, daemon=True).start()
                    return self.reply(dict(application_close_requested=True))
                if self.path == '/api/start':
                    request = host.start(data.get('track', args.track))
                    return self.reply(dict(start_requested=True, request=request))
                return self.send_error(404)
            except (ValueError, TypeError) as e:
                return self.reply(dict(error=str(e)), code=400)

    server = http.server.ThreadingHTTPServer(('127.0.0.1', args.port), Handler)
    threading.Thread(target=host.worker, daemon=True).start()
    threading.Thread(target=host.heartbeat, daemon=True).start()
    atomic_json(root/'logs/active.json', dict(kind=OWNER, root_key=key, nonce=host.nonce, port=args.port))
    host.start(args.track)
    if not args.no_browser:
        open_browser(base_url)
    print('Separate F1TENTH simulator: '+base_url+' ; STOP applies only to this sandbox.', flush=True)
    def signal_stop(*unused):
        host.stop('host_signal')
        threading.Thread(target=server.shutdown, daemon=True).start()
    for sig in [signal.SIGINT, signal.SIGTERM]+([signal.SIGHUP] if hasattr(signal, 'SIGHUP') else []):
        signal.signal(sig, signal_stop)
    try:
        server.serve_forever(poll_interval=.2)
    finally:
        host.stop('host_exit')
        deadline = time.monotonic()+8
        while host.current and time.monotonic() < deadline:
            status = read_json(host.current/'status.json') or {}
            if status.get('stopped_verified'):
                break
            time.sleep(.15)
        if docker_owned(host.container, key):
            run(['docker', 'stop', '--timeout', '3', host.container], check=False, capture_output=True)
        host.shutdown.set()
        server.server_close()
        host_lock.close()


def browser_page(nonce):
    return BROWSER_HTML.replace('__NONCE__', nonce).replace('__INSTANCE__', hashlib.sha256(nonce.encode()).hexdigest()[:12])


def checks(args):
    global np
    import numpy as np
    lock = read_json('/opt/SOURCE_LOCK.json')
    verify_physics(lock)
    results = {}
    names = list(lock['tracks']) if args.data_check else [args.track]
    for name in names:
        print('CHECKING ACTUAL UPSTREAM TRACK DATA:', name, flush=True)
        try:
            track = load_track(lock, '/opt/racetracks', name)
            results[name] = dict(data_validated=True, proof=track['proof'])
        except Exception as e:
            results[name] = dict(data_validated=False, error=str(e))
    if args.physics_probe:
        import gym
        import f110_gym
        track = load_track(lock, '/opt/racetracks', args.track)
        pose = np.array([[*track['points'][0], track['heading'][0]]])
        env = gym.make('f110-v0', map=track['map_base'], map_ext='.png', num_agents=1, lidar_dist=.275).unwrapped
        obs, _, _, _ = env.reset(pose)
        for _ in range(10):
            obs, _, _, _ = env.step(np.array([[0., 0.]]))
        assert not obs['collisions'][0] and abs(obs['linear_vels_x'][0]) < .03 and len(obs['scans'][0]) == 1080
        results[args.track]['physics_probe'] = dict(actual_stationary_steps=10, scan_beams=1080,
            actual_collision=bool(obs['collisions'][0]), timestep_s=env.timestep,
            integrator=env.integrator.name, stock_vehicle_parameters=env.params,
            note='Real simulator smoke check only, not a driven lap or synthetic track validation.')
    summary = dict(created_utc=utc(), checks=results, physics_source_hashes_verified=True,
                   source_commits=lock['repositories'], note='Data/zero-command checks do not certify driving on every track.')
    if args.output:
        atomic_json(args.output, summary)
    print(json.dumps(summary, indent=2))
    assert results.get('Melbourne', results[args.track])['data_validated'], 'Melbourne data validation failed'


def self_tests():
    import tempfile
    root = Path(__file__).resolve().parent/'logs/tests'
    root.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=root) as name:
        folder = Path(name)
        token = 'synthetic-unit-token'
        atomic_json(folder/'host-lease.json', dict(run_id=token, updated_unix_s=time.time()))
        assert lease_fresh(folder, token)
        (folder/'host-lease.json').unlink()
        assert lease_fresh(folder, token), 'Transient missing read discarded a still-current real heartbeat'
        atomic_json(folder/'host-lease.json', dict(run_id=token, updated_unix_s=time.time()))
        assert not lease_fresh(folder, 'wrong-owner')
        atomic_json(folder/'host-lease.json', dict(run_id=token, updated_unix_s=time.time()-10))
        assert not lease_fresh(folder, token, force_read=True)
        assert not cancelled(folder, folder/'session', time.time())
        atomic_json(folder/'logs/stop-request.json', dict(requested_unix_s=time.time()))
        assert cancelled(folder, folder/'session', time.time()-1)
        assert not cancelled(folder, folder/'session', time.time()+1)
        (folder/'session').mkdir()
        (folder/'session/STOP').write_text('synthetic fixture')
        assert cancelled(folder, folder/'session', time.time()+1)
    print('PASS: stale/wrong-owner leases, sticky STOP and new explicit-start epochs. Synthetic fixtures only; no Docker, simulator or driving started.')


def export_kit(root, destination=None):
    """Build a new offline transfer archive; never starts or arms a vehicle."""
    import gzip
    import shutil
    import tarfile
    root = root.resolve()
    destination = (destination or root.parent/'F1TENTH_Exports').resolve()
    destination.mkdir(parents=True, exist_ok=True)
    info = json.loads(run(['docker', 'image', 'inspect', IMAGE], capture_output=True, text=True).stdout)[0]
    assert info['Architecture'] == 'amd64', 'This offline kit targets x86_64 hosts'
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
    stage = destination/('staging-'+stamp)/'RoboRacer_F1TENTH'
    stage.mkdir(parents=True, exist_ok=False)
    for name in ['F1TENTH_RACER.py', 'Dockerfile', 'requirements.txt', '.dockerignore',
                 'SOURCE_LOCK.json', 'THIRD_PARTY_NOTICES.md', 'README.md', 'VALIDATION_REPORT.json',
                 'Launch-F1TENTH.ps1', 'START_F1TENTH.bat', 'STOP_F1TENTH.bat',
                 'INSTALL_LINUX_SHORTCUTS.sh', 'EXPORT_F1TENTH_KIT.bat']:
        if (root/name).is_file():
            shutil.copy2(root/name, stage/name)
    shutil.copytree(root/'vendor', stage/'vendor', ignore=shutil.ignore_patterns('.git', '__pycache__', '*.pyc', '.DS_Store'))
    image_lock = dict(created_utc=utc(), tag=IMAGE, id=info['Id'], architecture=info['Architecture'],
        runtime_fingerprint=image_fingerprint(info), source_lock_sha256=sha(root/'SOURCE_LOCK.json'),
        note='Ordered layer/runtime fingerprint, not solely a Docker manifest ID. All physics sources are checked separately.')
    atomic_json(stage/'IMAGE_LOCK.json', image_lock)
    image_dir = stage/'images'
    image_dir.mkdir()
    archive = image_dir/'f1tenth-image.tar.gz'
    error_log = destination/('docker-save-'+stamp+'.log')
    print('Exporting the real F1TENTH dependency image; no simulator reset or driving requested.', flush=True)
    with error_log.open('wb') as errors:
        process = subprocess.Popen(['docker', 'save', IMAGE], stdout=subprocess.PIPE, stderr=errors)
        with gzip.open(archive, 'wb', compresslevel=3) as target:
            shutil.copyfileobj(process.stdout, target, length=1024*1024)
        process.stdout.close()
        if process.wait() != 0:
            raise RuntimeError('Docker image export failed; retained diagnostics: '+str(error_log))
    validation = read_json(root/'VALIDATION_REPORT.json') or {}
    manifest = dict(created_utc=utc(), format_version=1, simulator='F1TENTH Gym / ROS 2, not AutoDRIVE',
        architecture='amd64', code_sha256=sha(stage/'F1TENTH_RACER.py'),
        source_lock_sha256=sha(stage/'SOURCE_LOCK.json'), image_archive_sha256=sha(archive),
        runtime_fingerprint=image_lock['runtime_fingerprint'], supplied_map_count=23,
        validation_report_included=bool(validation), native_ubuntu_host_driving_verified=False,
        requirements='Native x86_64 Ubuntu Docker access + Python 3 + browser; or Windows Ubuntu WSL + Docker Desktop. GPU not required for this labelled 2D browser view.',
        note='Contains real source/maps/licenses and a real dependency-image export, not fabricated evidence. Logs, host leases, web nonces and active sessions are excluded. Other-PC driving must be validated there.')
    atomic_json(stage/'EXPORT_MANIFEST.json', manifest)
    output = destination/('RoboRacer_F1TENTH_'+stamp+'.tar.gz')
    def permissions(member):
        member.uid = member.gid = 0
        member.uname = member.gname = ''
        member.mode = 0o755 if member.isdir() or member.name.endswith('.sh') or member.name.endswith('/F1TENTH_RACER.py') else 0o644
        return member
    with tarfile.open(output, 'w:gz', compresslevel=1) as bundle:
        bundle.add(stage, arcname='RoboRacer_F1TENTH', filter=permissions)
    digest = sha(output)
    output.with_name(output.name+'.sha256').write_text(digest+'  '+output.name+'\n')
    atomic_json(destination/'latest-export.json', dict(path=str(output), bytes=output.stat().st_size, sha256=digest,
        source_sha256=manifest['code_sha256'], staging_preserved=str(stage)))
    print(json.dumps(dict(archive=str(output), bytes=output.stat().st_size, sha256=digest), indent=2), flush=True)


BROWSER_HTML = r'''<!doctype html>
<html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>RoboRacer F1TENTH — separate simulation</title>
<style>
:root{color-scheme:dark}*{box-sizing:border-box}body{margin:0;background:#101827;color:#e9f2ff;font:16px system-ui,sans-serif}header{padding:22px 28px;border-bottom:1px solid #334155}h1{font-size:26px;margin:0 0 8px}.muted{color:#b1bed2}main{display:grid;grid-template-columns:minmax(0,1fr) 330px;gap:18px;padding:20px}aside,.view{background:#172338;border:1px solid #334155;border-radius:12px;padding:16px}.view{position:relative;min-height:480px}canvas{display:block;width:100%;height:64vh;min-height:420px;background:#d5d5d5;border-radius:8px;touch-action:none}button,select{font:inherit;padding:10px 14px;border-radius:7px;border:1px solid #4f6586;margin:4px 3px 10px 0}button{cursor:pointer;background:#146e52;color:white}button.stop{background:#ae303b;font-weight:800}select{max-width:100%;background:#101827;color:white}.metric{font-size:30px;font-weight:750}.row{display:flex;justify-content:space-between;border-bottom:1px solid #334155;padding:8px 0}.badge{display:inline-block;border:1px solid #587295;border-radius:6px;padding:4px 8px;margin:3px 0}.warn{background:#3d2e15;padding:10px;border-radius:7px}pre{white-space:pre-wrap;font-size:12px;max-height:240px;overflow:auto}small{font-size:12px}label{display:block;margin:8px 0}@media(max-width:850px){main{grid-template-columns:1fr}canvas{height:50vh}aside{order:-1}}
</style>
<header><h1>RoboRacer F1TENTH <span class="muted">· 2D simulation</span></h1><div class="muted">Actual Gym physics + ROS 2 · provided track prior and Gym odometry · not AutoDRIVE, not fresh SLAM</div></header>
<main><section class="view"><canvas id="view"></canvas><p><span style="color:#47a5ee">Blue: provided reference</span> · <span style="color:#f5ac40">Orange: actual vehicle/trail</span> · <span style="color:#d15454">Red: sampled actual LiDAR</span></p><label><input type="checkbox" id="laser"> Show LiDAR (display decimation only)</label><label><input type="checkbox" id="follow"> Follow vehicle</label><button id="fit">Fit map</button><small class="muted">Scroll to zoom; drag to pan. This is a labelled browser rendering of real simulator state, not Unity or RViz footage.</small></section>
<aside><span class="badge" id="phase">Preparing…</span><h2 id="trackname">Melbourne</h2><label for="track">Track</label><select id="track"></select><div><button id="start">Start / load selected track</button><button class="stop" id="stop">STOP</button></div><p class="warn"><strong>Simulation only.</strong> Track changes stop the previous session first. STOP cancels pending starts and requires a new explicit Start.</p><div class="metric" id="speed">0.00 m/s</div><div class="row"><span>Completed laps</span><b id="laps">0</b></div><div class="row"><span>Eligible clean laps</span><b id="clean">0</b></div><div class="row"><span>Best clean sim time</span><b id="best">—</b></div><div class="row"><span>Collisions (retained)</span><b id="collisions">0</b></div><div class="row"><span>Reference length</span><b id="length">—</b></div><div class="row"><span>Physics step</span><b>0.01 s (stock)</b></div><div class="row"><span>ROS scan</span><b id="beams">—</b></div><div class="row"><span>STOP verified</span><b id="verified">No</b></div><p id="mode" class="muted"></p><p id="reason"></p><small>New-track lap times are not comparable to the previous AutoDRIVE record. Standing-start/full flying laps are labelled in the raw log. Supplied speed columns are not physical-car validation.</small><details><summary>Preparation / diagnostics</summary><pre id="diagnostics"></pre></details></aside></main>
<script>
const nonce='__NONCE__',instance='__INSTANCE__';let state={},trackData=null,runId=null,loaded=false,scale=1,tx=0,ty=0,drag=null;
const canvas=document.getElementById('view'),ctx=canvas.getContext('2d'),image=new Image();
const el=id=>document.getElementById(id);
async function action(path,extra={}){let r=await fetch(path,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({nonce,...extra})});if(!r.ok)el('reason').textContent='Request rejected: '+r.status;}
el('stop').onclick=()=>action('/api/stop');el('start').onclick=()=>action('/api/start',{track:el('track').value});
function resize(){let d=devicePixelRatio||1;canvas.width=Math.round(canvas.clientWidth*d);canvas.height=Math.round(canvas.clientHeight*d);if(loaded)fit();}
function fit(){if(!loaded)return;scale=Math.min(canvas.width/image.width,canvas.height/image.height)*.94;tx=(canvas.width-image.width*scale)/2;ty=(canvas.height-image.height*scale)/2;}
el('fit').onclick=fit;window.addEventListener('resize',resize);resize();
function pixel(x,y){let p=trackData.proof,o=p.map_origin,c=Math.cos(o[2]),s=Math.sin(o[2]),dx=x-o[0],dy=y-o[1];return[(c*dx+s*dy)/p.map_resolution_m,p.map_height-(-s*dx+c*dy)/p.map_resolution_m];}
function path(points,color,width){if(!points.length)return;ctx.beginPath();points.forEach((p,i)=>{let a=pixel(p[0],p[1]);i?ctx.lineTo(...a):ctx.moveTo(...a)});ctx.strokeStyle=color;ctx.lineWidth=width/scale;ctx.stroke();}
function draw(){ctx.setTransform(1,0,0,1,0,0);ctx.fillStyle='#d5d5d5';ctx.fillRect(0,0,canvas.width,canvas.height);if(!loaded||!trackData){requestAnimationFrame(draw);return;}
let sim=state.simulation||{};if(el('follow').checked&&sim.pose){let p=pixel(sim.pose[0],sim.pose[1]);tx=canvas.width/2-p[0]*scale;ty=canvas.height/2-p[1]*scale;}
ctx.setTransform(scale,0,0,scale,tx,ty);ctx.drawImage(image,0,0);path(trackData.line,'#087cbd',2*(devicePixelRatio||1));path(sim.trail||[],'#e99420',2*(devicePixelRatio||1));
if(sim.pose){let[x,y,a]=sim.pose,p=pixel(x,y),yaw=-(a-trackData.proof.map_origin[2]),res=trackData.proof.map_resolution_m;
if(el('laser').checked&&(sim.lidar_display_ranges||[]).length){ctx.strokeStyle='rgba(210,65,65,.35)';ctx.lineWidth=1/scale;let lx=x+.275*Math.cos(a),ly=y+.275*Math.sin(a),start=pixel(lx,ly);sim.lidar_display_ranges.forEach((r,i)=>{let angle=a+sim.lidar_angle_min+i*sim.lidar_display_stride*sim.lidar_angle_increment,end=pixel(lx+r*Math.cos(angle),ly+r*Math.sin(angle));ctx.beginPath();ctx.moveTo(...start);ctx.lineTo(...end);ctx.stroke();});}
ctx.save();ctx.translate(...p);ctx.rotate(yaw);ctx.fillStyle='#ffb13b';ctx.strokeStyle='#553800';ctx.lineWidth=1/scale;let length=trackData.vehicle_length_m/res,width=trackData.vehicle_width_m/res;ctx.fillRect(-length/2,-width/2,length,width);ctx.strokeRect(-length/2,-width/2,length,width);ctx.beginPath();ctx.moveTo(length/2,0);ctx.lineTo(-length/5,-width/3);ctx.lineTo(-length/5,width/3);ctx.closePath();ctx.fillStyle='#563d12';ctx.fill();ctx.restore();}
requestAnimationFrame(draw);}
canvas.onwheel=e=>{e.preventDefault();const d=devicePixelRatio||1,r=canvas.getBoundingClientRect(),x=(e.clientX-r.left)*d,y=(e.clientY-r.top)*d,k=Math.exp(-e.deltaY*.001);tx=x-(x-tx)*k;ty=y-(y-ty)*k;scale*=k;};
canvas.onpointerdown=e=>{canvas.setPointerCapture(e.pointerId);drag=[e.clientX,e.clientY];el('follow').checked=false;};canvas.onpointermove=e=>{if(drag){let d=devicePixelRatio||1;tx+=(e.clientX-drag[0])*d;ty+=(e.clientY-drag[1])*d;drag=[e.clientX,e.clientY];}};canvas.onpointerup=()=>drag=null;
async function poll(){try{state=await(await fetch('/api/state')).json();if(state.instance&&state.instance!==instance){location.reload();return;}let sim=state.simulation||{};el('phase').textContent=state.telemetry_stale?'Telemetry stale — stopped/fault':(sim.phase||state.host_phase);el('trackname').textContent=sim.track||state.selected_track;
if(!el('track').options.length){state.tracks.forEach(name=>{let option=document.createElement('option');option.value=option.textContent=name;el('track').appendChild(option);});el('track').value=state.selected_track;}
el('speed').textContent=(sim.measured_speed_mps||0).toFixed(2)+' m/s';el('laps').textContent=sim.laps||0;el('clean').textContent=sim.eligible_clean_laps||0;el('best').textContent=sim.best_clean_simulator_lap_s==null?'—':sim.best_clean_simulator_lap_s.toFixed(3)+' s';el('collisions').textContent=(sim.native||{}).collisions||0;el('length').textContent=sim.reference_length_m?(sim.reference_length_m.toFixed(1)+' m'):'—';el('beams').textContent=sim.scan_beams||'—';el('verified').textContent=sim.stopped_verified?'Yes':'No';el('mode').textContent=sim.controller_mode||'Waiting for actual ROS observations';el('reason').textContent=state.error||sim.reason||'';el('diagnostics').textContent=state.preparation_log_tail||JSON.stringify({host:state.host_phase,native:sim.native,ready:sim.ready,armed:sim.armed,pace:sim.pace},null,2);
if(sim.run_id&&sim.run_id!==runId){runId=sim.run_id;trackData=await(await fetch('/api/track')).json();loaded=false;image.onload=()=>{loaded=true;fit();};image.src='/map.png?run='+encodeURIComponent(runId);}
}catch(e){el('reason').textContent='Connection unavailable. The host-liveness watchdog stops driving.';}setTimeout(poll,150);}
poll();draw();
</script></html>'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parent)
    parser.add_argument('--service', action='store_true')
    parser.add_argument('--session', type=Path)
    parser.add_argument('--token', default='')
    parser.add_argument('--started', type=float, default=0.)
    parser.add_argument('--track')
    parser.add_argument('--max-speed', type=float, default=0.)
    parser.add_argument('--pace', type=float, default=.70)
    parser.add_argument('--validation-laps', type=int, default=0)
    parser.add_argument('--max-seconds', type=float, default=0.)
    parser.add_argument('--validation-recovery', action='store_true',help='Internal simulation-only intentional collision/recovery test; never enabled by normal launchers')
    parser.add_argument('--stop', action='store_true')
    parser.add_argument('--no-browser', action='store_true')
    parser.add_argument('--port', type=int, default=PORT)
    parser.add_argument('--install', action='store_true')
    parser.add_argument('--data-check', action='store_true')
    parser.add_argument('--physics-probe', action='store_true')
    parser.add_argument('--self-test', action='store_true')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--export-kit', action='store_true')
    parser.add_argument('--export-directory', type=Path)
    args = parser.parse_args()
    if args.track is None:
        args.track = (read_json(args.root/'settings.json') or {}).get('track', 'Melbourne')
    if args.self_test:
        return self_tests()
    if args.install:
        return install_native_shortcuts(args.root)
    if args.export_kit:
        return export_kit(args.root, args.export_directory)
    assert math.isfinite(args.max_speed) and args.max_speed >= 0
    assert math.isfinite(args.pace) and .2 <= args.pace <= 1.
    assert args.validation_laps >= 0 and math.isfinite(args.max_seconds) and args.max_seconds >= 0
    if args.data_check or args.physics_probe:
        return checks(args)
    if args.service:
        return service(args)
    return host_main(args)


if __name__ == '__main__':
    main()
