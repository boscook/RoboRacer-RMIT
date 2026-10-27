#!/usr/bin/env python3
"""Single-file, SIMULATION-ONLY trajectory-replay racing benchmark.

All custom launch/control/planning/watchdog/timing code is in this file.
Dependencies remain external: Docker images containing ROS/NumPy/OpenCV,
the unmodified official simulator, and genuine previously recorded memory.

Crash-triggered stopping is OFF. Collision/teleport/transition laps are INVALID.
Earlier clean laps remain eligible if a later lap crashes; this is a per-lap
benchmark, not a claim that the entire session was collision-free.
STOP, lost host lease and stale sensors still command zero throttle.
Predictive/proximity speed governors are bypassed in this direct-actuator mode.
Uses privileged simulator pose feedback, NOT LiDAR-only/physical-car autonomy.
Does not alter the simulator clock, circuit, physics or lap counters.

Linux: put beside kit-manifest.json in the existing extracted native kit.
Allow execution and Run as a Program; dependencies/data are taken from that kit.
--install creates app-menu START/STOP shortcuts without driving.
Windows/WSL test: explicitly use --platform windows in the existing project.
Live STOP dashboard: http://localhost:14999 ; or run this same file with --stop.
Measured development here uses the Windows simulator + Linux ROS Docker.
Native Linux GPU/UI operation and these lap times must be revalidated there.
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
import platform
import secrets
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time
import urllib.parse

SIM = '/autodrive/roboracer_1/'
OWNER = 'capstone-onefile-simulator'
ROS_SETUP = 'source /opt/ros/humble/setup.bash\nsource /home/autodrive_devkit/install/setup.bash\nsource /root/autoware_msgs_ws/install/setup.bash\n'
np = None  # Imported only inside the dependency container, not on the host.


def atomic_json(path, value):
    path = Path(path); path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name+'.tmp-'+str(os.getpid()))
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False))
    temporary.replace(path)


def read_json(path):
    try: return json.loads(Path(path).read_text(encoding='utf-8-sig'))
    except (OSError, ValueError): return None


def fresh_lease(value, token, now=None):
    try:
        age=(time.time() if now is None else now)-float(value['updated_unix_s'])
        return value['run_id']==token and math.isfinite(age) and -.5<=age<1.2
    except (TypeError,KeyError,ValueError):return False


def clip(value, low, high): return max(low, min(high, value))
def wrap(value): return math.atan2(math.sin(value), math.cos(value))


def resample(points, spacing=.04):
    closed = np.vstack((points, points[0]))
    distances = np.r_[0., np.cumsum(np.linalg.norm(np.diff(closed, axis=0), axis=1))]
    samples = np.arange(max(100, int(math.ceil(distances[-1]/spacing))))
    samples = samples*distances[-1]/len(samples)
    return np.column_stack([np.interp(samples, distances, closed[:, axis]) for axis in (0, 1)])


def geometry(points, window=None):
    if window is None:
        spacing = float(np.linalg.norm(np.roll(points, -1, axis=0)-points, axis=1).mean())
        window = max(3, int(round(.30/max(spacing, .005))))
    before, after = np.roll(points, window, axis=0), np.roll(points, -window, axis=0)
    a, b = points-before, after-points
    denominator = np.linalg.norm(a, axis=1)*np.linalg.norm(b, axis=1)*np.linalg.norm(a+b, axis=1)
    curvature = 2*(a[:, 0]*b[:, 1]-a[:, 1]*b[:, 0])/np.maximum(denominator, 1e-8)
    tangent = after-before
    heading = np.arctan2(tangent[:, 1], tangent[:, 0])
    distance = np.linalg.norm(np.roll(points, -1, axis=0)-points, axis=1)
    return curvature, heading, distance


def clearance_at(points, clearance, origin, resolution):
    delta = points-np.asarray(origin[:2]); c, s = math.cos(origin[2]), math.sin(origin[2])
    x = np.rint((c*delta[:, 0]+s*delta[:, 1])/resolution-.5).astype(int)
    y = np.rint(clearance.shape[0]-(-s*delta[:, 0]+c*delta[:, 1])/resolution-.5).astype(int)
    valid = (x >= 0) & (y >= 0) & (x < clearance.shape[1]) & (y < clearance.shape[0])
    values = np.zeros(len(points)); values[valid] = clearance[y[valid], x[valid]]
    return values


def reference_speed(points, cap, lateral, acceleration, braking):
    curvature, heading, ds = geometry(points)
    speeds = np.minimum(cap, np.sqrt(lateral/np.maximum(abs(curvature), .015)))
    # Exact cyclic forward/backward energy constraints, not a short local horizon.
    for _ in range(5):
        for i in range(len(points)-1, -1, -1):
            j=(i+1)%len(points)
            lateral_used=min(lateral, speeds[j]**2*max(abs(curvature[i]),abs(curvature[j])))
            available=max(.35,math.sqrt(max(0.,lateral**2-lateral_used**2)))
            speeds[i] = min(speeds[i], math.sqrt(speeds[j]**2+2*min(braking,available)*ds[i]))
        for i in range(len(points)):
            j = (i+1) % len(points)
            lateral_used=min(lateral,speeds[i]**2*max(abs(curvature[i]),abs(curvature[j])))
            available=max(.35,math.sqrt(max(0.,lateral**2-lateral_used**2)))
            speeds[j] = min(speeds[j], math.sqrt(speeds[i]**2+2*min(acceleration,available)*ds[i]))
    return speeds, curvature, heading, ds


def refine_for_time(points, clearance, origin, resolution, args):
    """Map-constrained local search on genuine data, never a supplied fake circuit."""
    original = resample(points, .04); result = original.copy()
    tangent = np.roll(original, -5, axis=0)-np.roll(original, 5, axis=0)
    normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))
    normal /= np.maximum(np.linalg.norm(normal, axis=1)[:, None], 1e-8)
    def cost(path):
        k, _, ds = geometry(path)
        v = np.minimum(args.max_speed, np.sqrt(args.lateral/np.maximum(abs(k), .015)))
        return float(np.sum(ds/v)+.015*np.sum(k*k*ds))
    best = initial = cost(result)
    spacing=float(np.linalg.norm(np.roll(original,-1,axis=0)-original,axis=1).mean())
    control_step=max(1,int(round(.30/spacing)))
    for sweep in range(args.refine_sweeps):
        step = .04*.86**sweep
        for i in range(0,len(result),control_step):
            saved = result.copy(); chosen = saved; local_best = best
            separation=np.abs(np.arange(len(result))-i)
            separation=np.minimum(separation,len(result)-separation)*spacing
            influence=np.exp(-.5*(separation/.32)**2)
            influence[separation>1.0]=0
            for sign in (-1, 1):
                proposal = saved+sign*step*normal*influence[:,None]
                if np.linalg.norm(proposal-original,axis=1).max() > args.line_offset: continue
                if clearance_at(proposal, clearance, origin, resolution).min() < args.clearance: continue
                if abs(geometry(proposal)[0]).max()>math.tan(.5236)/.33:continue
                value = cost(proposal)
                if value < local_best: chosen, local_best = proposal, value
            result = chosen; best = local_best
    result = resample(result)
    minimum = float(clearance_at(result, clearance, origin, resolution).min())
    if minimum < args.clearance or cost(result) >= initial: result = resample(points)
    return result, dict(original_cost=initial, selected_cost=cost(result),
                        minimum_clearance_m=float(clearance_at(result, clearance, origin, resolution).min()))


def load_memory(directory, args):
    import cv2
    import yaml
    directory = Path(directory)
    active = directory/'active_track' if (directory/'active_track').is_dir() else directory
    reference = directory/'benchmark_reference.json'
    if not reference.exists(): reference = active.parent/'benchmark_reference.json'
    anchor = json.loads(reference.read_text())
    path = active/'benchmark_racing_line.csv'
    if not path.exists(): path = active/'learned_track.csv'
    with path.open() as stream:
        points = np.array([[float(row['x']), float(row['y'])] for row in csv.DictReader(stream)])
    assert len(points) >= 50 and np.isfinite(points).all(), 'Invalid genuine memory'
    metadata = yaml.safe_load((active/'map.yaml').read_text())
    image = cv2.imread(str(active/'map.pgm'), cv2.IMREAD_GRAYSCALE)
    assert image is not None
    clearance = cv2.distanceTransform((image >= 250).astype(np.uint8), cv2.DIST_L2, 5)*metadata['resolution']
    original_hash = hashlib.sha256(path.read_bytes()).hexdigest()
    points = resample(points)
    # Representation-invariant periodic smoothing of genuine measured data.
    sigma = .10/max(float(np.linalg.norm(np.roll(points,-1,axis=0)-points,axis=1).mean()),.01)
    offsets = np.arange(-int(math.ceil(3*sigma)), int(math.ceil(3*sigma))+1)
    weights = np.exp(-.5*(offsets/sigma)**2); weights /= weights.sum()
    smoothed = sum(weight*np.roll(points,int(offset),axis=0) for offset,weight in zip(offsets,weights))
    if clearance_at(smoothed,clearance,metadata['origin'],metadata['resolution']).min() >= args.clearance:
        points = smoothed
    if args.refine_sweeps:
        points, refinement = refine_for_time(points, clearance, metadata['origin'], metadata['resolution'], args)
    else:
        points = resample(points); refinement = dict(selected_cost=None)
    checked = clearance_at(points, clearance, metadata['origin'], metadata['resolution'])
    assert checked.min() >= .20, 'Path not clearance-valid in the actual saved map'
    speed, curvature, heading, ds = reference_speed(points, args.max_speed, args.lateral, args.acceleration, args.braking)
    proof = dict(source='genuine previously recorded simulator-assisted memory, NOT online LiDAR-only learning',
                 source_path=str(path), source_sha256=original_hash, points=len(points), length_m=float(ds.sum()),
                 minimum_saved_map_clearance_m=float(checked.min()), refinement=refinement,
                 ideal_profile_s=float(np.sum(2*ds/(speed+np.roll(speed, -1)))),
                 note='Ideal profile is a planning estimate, not a driven lap time.')
    return points, speed, curvature, heading, ds, anchor, proof


def valid_lap(event, length):
    return (not event['collision'] and not event['teleport'] and not event['transition']
            and event['seen_previous_finish'] and event['distance_m'] >= .85*length
            and event['coverage_fraction'] >= .80 and math.isfinite(event['simulator_seconds'])
            and event['simulator_seconds'] > 0
            and abs(event['simulator_seconds']-event['wall_seconds']) <= max(.4, .10*event['simulator_seconds']))


def ros_mode(args):
    global np
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Bool, Float32, Int32, String
    from autoware_control_msgs.msg import Control
    if os.environ.get('CAPSTONE_SIM_TEST') != '1':
        raise SystemExit('CAPSTONE_SIM_TEST=1 required; this program is simulation only')
    output = Path(args.output); output.mkdir(parents=True, exist_ok=False)
    points, planned, curvature, heading, ds, anchor, proof = load_memory(args.memory, args)
    proof['controller_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    proof['crash_stop_enabled']=args.stop_on_crash
    proof['safety_policy']='SIMULATOR_DIRECT_BENCHMARK_NO_PROXIMITY_GOVERNOR'
    atomic_json(output/'memory_provenance.json', proof)
    with (output/'actually_used_path.csv').open('w', newline='') as stream:
        writer = csv.writer(stream); writer.writerow(['x', 'y', 'planned_speed']); writer.writerows(zip(points[:, 0], points[:, 1], planned))
    rclpy.init()

    class Racer(Node):
        def __init__(self):
            super().__init__('onefile_simulator_racer')
            self.pose = None; self.speed = 0.; self.yaw_rate = 0.; self.pose_time = self.scan_time = 0.
            self.last_position = None; self.forward = False; self.last_motion = time.monotonic()
            self.collisions = 0; self.total_collisions = 0; self.count = 0; self.last_time = float('nan')
            self.armed = False; self.stopped = False; self.reason = 'running'; self.index = None
            self.racing_started = False
            self.steering = 0.; self.throttle = 0.; self.reference = 0.; self.integral = 0.
            self.scale = args.scale; self.best_scale = args.scale; self.best = None
            self.started = time.monotonic(); self.last_tick = self.started; self.lap_started = self.started
            self.lap_collision = False; self.lap_teleport = False; self.lap_transition = False
            self.seen_finish = False; self.distance = 0.; self.coverage = set(); self.pending = []
            self.records = []; self.last_log = 0.; self.last_status = 0.; self.clean_since_tune = 0
            self.tune_step = .04; self.failed_scale = None
            self.reset_until = 0.; self.resetting = False
            self.not_fresh_since = None
            q = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
            self.throttle_pub = self.create_publisher(Float32, SIM+'throttle_command', q)
            self.steer_pub = self.create_publisher(Float32, SIM+'steering_command', q)
            self.reset_pub = self.create_publisher(Bool, '/autodrive/reset_command', 10)
            self.heartbeat_pub = self.create_publisher(Bool, '/onefile/heartbeat', 10)
            self.control_pub = self.create_publisher(Control, '/control/command/control_cmd', 10)
            self.status_pub = self.create_publisher(String, '/onefile/status', 10)
            self.create_subscription(Odometry, SIM+'odom', self.on_pose, qos_profile_sensor_data)
            self.create_subscription(LaserScan, SIM+'lidar', self.on_scan, qos_profile_sensor_data)
            self.create_subscription(Int32, SIM+'collision_count', self.on_collision, qos_profile_sensor_data)
            self.create_subscription(Int32, SIM+'lap_count', self.on_lap, qos_profile_sensor_data)
            self.create_subscription(Float32, SIM+'last_lap_time', lambda m: setattr(self, 'last_time', float(m.data)), qos_profile_sensor_data)
            self.create_subscription(Bool, '/roboracer/drive_enable', self.on_enable, 10)
            self.create_timer(.01, self.tick)
            self.journal = (output/'laps.jsonl').open('w')
            self.telemetry = (output/'telemetry.csv').open('w', newline=''); self.writer = None

        def on_enable(self, message):
            if not message.data:
                self.armed = False; self.stopped = True; self.reason = 'explicit_stop'
                self.publish(0., 0., 0.)

        def on_scan(self, message):
            values = [v for v in message.ranges if math.isfinite(v) and v > max(.08, message.range_min)]
            if len(values) > 100: self.scan_time = time.monotonic()

        def on_pose(self, message):
            p, q = message.pose.pose.position, message.pose.pose.orientation
            angle = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            v = message.twist.twist.linear; speed = math.hypot(v.x, v.y)
            if not all(math.isfinite(z) for z in (p.x, p.y, angle, speed)): return
            now = time.monotonic(); dt = now-self.pose_time
            if self.last_position is not None:
                dx, dy = p.x-self.last_position[0], p.y-self.last_position[1]
                distance = math.hypot(dx, dy)
                if distance > max(.8, 2*speed*max(dt, .01)+.2):
                    if self.racing_started:self.lap_teleport = True
                    self.index = None; self.forward = False
                else:
                    if self.racing_started:self.distance += distance
                    if distance > .002:
                        self.forward = dx*math.cos(angle)+dy*math.sin(angle) > 0
                if distance > .005: self.last_motion = now
            self.last_position = (p.x, p.y)
            transform = anchor['world_to_map']; c, s = math.cos(transform['yaw']), math.sin(transform['yaw'])
            previous_angle=self.pose[2] if self.pose else None
            self.pose = (c*p.x-s*p.y+transform['x'], s*p.x+c*p.y+transform['y'], wrap(angle+transform['yaw']))
            self.speed = speed
            rate=wrap(self.pose[2]-previous_angle)/dt if previous_angle is not None and .008<dt<.15 else float(message.twist.twist.angular.z)
            if math.isfinite(rate): self.yaw_rate = .4*self.yaw_rate+.6*rate
            self.pose_time = now

        def on_collision(self, message):
            value = int(message.data)
            if not self.racing_started:
                self.collisions=value
                return
            if value > self.collisions:
                self.total_collisions += value-self.collisions
                self.lap_collision = True
                for pending in self.pending: pending['collision'] = True
                self.failed_scale = self.scale if self.failed_scale is None else min(self.failed_scale,self.scale)
                self.tune_step=max(.005,self.tune_step*.5)
                self.scale = max(.4, min(self.scale*.85, self.best_scale*.95))
                self.clean_since_tune = 0; self.index = None
                print('COLLISION RECORDED; crash-stop disabled; impacted lap INVALID; new scale', round(self.scale, 3), flush=True)
                if args.stop_on_crash:
                    self.stopped = True; self.armed = False; self.reason = 'requested_crash_stop'
            self.collisions = value

        def on_lap(self, message):
            value = int(message.data)
            if not self.racing_started:
                self.count=value
                return
            if value < self.count:
                self.lap_transition = True; self.seen_finish = False; self.index = None
            if value > self.count:
                now = time.monotonic()
                self.pending.append(dict(lap=value, collision=self.lap_collision, teleport=self.lap_teleport,
                    transition=self.lap_transition, seen_previous_finish=self.seen_finish,
                    distance_m=self.distance, coverage_fraction=len(self.coverage)/100,
                    wall_seconds=now-self.lap_started, scale=self.scale, due=now+.16))
                self.lap_started = now; self.seen_finish = True
                self.lap_collision = False; self.lap_teleport = False; self.lap_transition = False
                self.distance = 0.; self.coverage = set()
            self.count = value

        def finish_laps(self, now):
            for event in list(self.pending):
                if now < event['due']: continue
                self.pending.remove(event); event.pop('due')
                event['simulator_seconds'] = self.last_time if math.isfinite(self.last_time) else 0.
                event['valid_clean_full_lap'] = valid_lap(event, proof['length_m'])
                event['total_collisions'] = self.total_collisions
                self.records.append(event); self.journal.write(json.dumps(event)+'\n'); self.journal.flush()
                if event['valid_clean_full_lap']:
                    seconds = event['simulator_seconds']; self.clean_since_tune += 1
                    if self.best is None or seconds < self.best:
                        self.best = seconds; self.best_scale = event['scale']
                        print(f'CLEAN BEST {seconds:.4f} s; scale={event["scale"]:.3f}; collisions overall={self.total_collisions}', flush=True)
                    if not args.fixed_scale and self.clean_since_tune >= 1:
                        old_scale=self.scale
                        if self.best is not None and seconds > self.best*1.03 and event['scale']>self.best_scale+.005:
                            self.failed_scale=event['scale'] if self.failed_scale is None else min(self.failed_scale,event['scale'])
                            self.tune_step=max(.005,self.tune_step*.5);self.scale=self.best_scale
                        else:
                            self.scale=min(args.maximum_scale,self.scale+self.tune_step)
                            if self.failed_scale is not None:self.scale=min(self.scale,(self.best_scale+self.failed_scale)/2)
                        if abs(self.scale-old_scale)>.001:self.lap_transition=True
                        self.clean_since_tune = 0
                else:
                    print('INVALID/TRANSITION LAP', json.dumps(event), flush=True)
                if args.stop_after_laps and len([r for r in self.records if r['valid_clean_full_lap']]) >= args.stop_after_laps:
                    self.stopped = True; self.armed = False; self.reason = 'requested_clean_laps_complete'

        def publish(self, throttle, steering, reference):
            self.throttle_pub.publish(Float32(data=float(throttle)))
            self.steer_pub.publish(Float32(data=float(clip(steering/.5236, -1., 1.))))
            control = Control(); control.stamp = self.get_clock().now().to_msg()
            control.longitudinal.velocity = float(reference); control.lateral.steering_tire_angle = float(steering)
            self.control_pub.publish(control)

        def tick(self):
            now = time.monotonic(); dt = clip(now-self.last_tick, .001, .05); self.last_tick = now
            self.heartbeat_pub.publish(Bool(data=True)); self.finish_laps(now)
            if Path('/logs/STOP').exists(): self.stopped = True; self.armed = False; self.reason = 'explicit_stop_file'
            lease = read_json('/logs/host-heartbeat.json')
            lease_ok = fresh_lease(lease,args.token)
            fresh = self.pose is not None and now-self.pose_time < .20 and now-self.scan_time < .25 and lease_ok
            if self.armed and not fresh:
                if self.not_fresh_since is None:self.not_fresh_since=now
                if now-self.not_fresh_since > (5 if self.resetting else 2):
                    self.stopped=True;self.armed=False;self.reason='host_or_sensor_timeout'
            else:self.not_fresh_since=None
            throttle = steering = reference = error = 0.; position_index = 0
            if self.armed and fresh and not self.stopped and not self.resetting:
                x, y, angle = self.pose
                candidates = np.arange(len(points)) if self.index is None else (self.index+np.arange(-30, 121)) % len(points)
                distances = np.linalg.norm(points[candidates]-[x, y], axis=1)
                self.index = position_index = int(candidates[np.argmin(distances)])
                error = float(distances.min()); self.coverage.add(min(99, int(self.index*100/len(points))))
                # Account for the remaining bridge/actuator delay, without
                # replacing the real-pose error used by timing diagnostics.
                px = x+self.speed*args.preview*math.cos(angle)
                py = y+self.speed*args.preview*math.sin(angle)
                pa = wrap(angle+self.yaw_rate*args.preview)
                future = (self.index+np.arange(0, 51)) % len(points)
                target = int(future[np.argmin(np.linalg.norm(points[future]-[px, py], axis=1))])
                normal = np.array([-math.sin(heading[target]), math.cos(heading[target])])
                cross_track = float(np.dot(points[target]-[px, py], normal))
                feedback_curvature = (curvature[target]+args.heading_gain*wrap(heading[target]-pa)/(self.speed+1.)
                                      +args.cross_gain*cross_track/(self.speed+1.)**2)
                desired = math.atan(.33*feedback_curvature)
                if args.tracker == 'pursuit':
                    look = clip(.65+.18*self.speed, .85, 1.7); target=self.index; travel=0.
                    while travel < look:
                        travel += ds[target]; target=(target+1) % len(points)
                    dx, dy=points[target]-[px,py]
                    desired=math.atan(.33*2*(-math.sin(pa)*dx+math.cos(pa)*dy)/max(dx*dx+dy*dy,.02))
                desired = clip(desired, -.5236, .5236)
                steering = self.steering+clip(desired-self.steering, -args.steering_rate*dt, args.steering_rate*dt)
                preview_distance = max(.08, self.speed*args.brake_preview)
                indices=[self.index]; travel=0.
                while travel < preview_distance:
                    i=indices[-1]; travel+=ds[i]; indices.append((i+1) % len(points))
                reference = float(np.min(planned[indices]))*self.scale
                reference = min(reference, args.max_speed)
                if error > .15: reference *= max(.35, 1-1.6*(error-.15))
                requested_acceleration = clip((reference-self.reference)/dt, -args.braking, args.acceleration)
                self.reference += requested_acceleration*dt
                velocity_error = self.reference-self.speed
                self.integral = clip(self.integral+velocity_error*dt, -.3, .3)
                throttle = .04*self.reference+args.speed_gain*velocity_error+.008*self.integral+.013*requested_acceleration
                # Negative command is braking torque ONLY while confirmed
                # forward motion is well above a positive target, never STOP.
                braking_allowed = (not args.no_active_brake and self.forward and reference > 1.2 and
                                   self.speed > reference+.25 and self.speed > 1.8)
                throttle = clip(throttle, -args.brake_cap if braking_allowed else 0., 1.)
                if throttle >= 0: throttle = min(throttle, max(0.,self.throttle)+args.throttle_rate*dt)
                self.steering, self.throttle = steering, throttle
                if self.speed < .1 and now-self.last_motion > 5 and self.total_collisions:
                    # Genuine official reset if a collision leaves the car stuck.
                    # No counter injection; warmup/progress gates reject shortcuts.
                    self.reset_until=now+.12; self.resetting=True; self.lap_transition=True; self.lap_teleport=True; self.seen_finish=False
                    self.last_motion=now; self.index=None
            else:
                if self.armed: self.lap_transition=True
                self.integral=0.; self.reference=0.; self.throttle=0.
            if self.resetting:
                self.reset_pub.publish(Bool(data=now < self.reset_until))
                throttle=steering=reference=0.
                if now > self.reset_until+.4: self.resetting=False
            if self.stopped: throttle=steering=reference=0.
            self.publish(throttle,steering,reference)
            if now-self.last_log >= .05:
                row=dict(t=now-self.started,world_map_x=self.pose[0] if self.pose else None,
                    world_map_y=self.pose[1] if self.pose else None,speed=self.speed,reference=reference,
                    steering=steering,throttle=throttle,path_error=error,scale=self.scale,
                    collisions=self.total_collisions,simulator_laps=self.count,armed=self.armed,fresh=bool(fresh),
                    pose_age_s=now-self.pose_time,scan_age_s=now-self.scan_time,yaw_rate=self.yaw_rate,
                    forward_motion_confirmed=self.forward)
                if self.writer is None: self.writer=csv.DictWriter(self.telemetry,fieldnames=list(row));self.writer.writeheader()
                self.writer.writerow(row);self.telemetry.flush();self.last_log=now
            if now-self.last_status >= .5:
                status=dict(running=not self.stopped,armed=self.armed,fresh=bool(fresh),simulator_laps=self.count,
                    total_collisions=self.total_collisions,best_clean_lap_s=self.best,scale=self.scale,
                    valid_laps=sum(r['valid_clean_full_lap'] for r in self.records),reason=self.reason,
                    crash_stop_enabled=args.stop_on_crash,simulator_pose_assisted=True,
                    safety_policy='SIMULATOR_DIRECT_BENCHMARK_NO_PROXIMITY_GOVERNOR',
                    five_second_target_achieved=self.best is not None and self.best < 5)
                atomic_json(output/'status.json',status);self.status_pub.publish(String(data=json.dumps(status)));self.last_status=now
            if self.racing_started and args.run_seconds and now-self.started > args.run_seconds:
                self.stopped=True;self.armed=False;self.reason='requested_duration_complete'

    racer = Racer()
    guard = subprocess.Popen([sys.executable,__file__,'--ros-watchdog','--owner-pid',str(os.getpid())],
                             stdout=(output/'watchdog.log').open('w'),stderr=subprocess.STDOUT)
    try:
        deadline=time.monotonic()+45
        while time.monotonic()<deadline and not racer.stopped:
            rclpy.spin_once(racer,timeout_sec=.02)
            if racer.pose and time.monotonic()-racer.pose_time<.2 and time.monotonic()-racer.scan_time<.25 and racer.throttle_pub.get_subscription_count():break
        assert racer.pose is not None and racer.speed<.05 and not racer.stopped,'Missing telemetry or vehicle not stationary'
        racer.reset_pub.publish(Bool(data=True));end=time.monotonic()+.12
        while time.monotonic()<end:rclpy.spin_once(racer,timeout_sec=.01)
        racer.reset_pub.publish(Bool(data=False));end=time.monotonic()+1.5
        while time.monotonic()<end:rclpy.spin_once(racer,timeout_sec=.01)
        expected=anchor['map_anchor'];assert math.hypot(racer.pose[0]-expected['x'],racer.pose[1]-expected['y'])<.06,'Different track/start pose; refusing replay'
        assert racer.speed<.05 and racer.collisions==0 and racer.count==0,'Reset not verified'
        assert guard.poll() is None,'Independent watchdog did not remain alive through preflight'
        publishers=racer.get_publishers_info_by_topic(SIM+'throttle_command')
        assert {p.node_name for p in publishers} <= {'onefile_simulator_racer','onefile_independent_watchdog'},'Conflicting actuator publisher'
        racer.total_collisions=0;racer.lap_collision=False;racer.lap_teleport=False;racer.lap_transition=False
        racer.distance=0.;racer.coverage=set();racer.pending=[];racer.seen_finish=False
        racer.started=racer.lap_started=time.monotonic();racer.racing_started=True;racer.armed=True
        print('ARMED SINGLE-FILE SIMULATOR BENCHMARK. Crash stop:',args.stop_on_crash,'STOP/stale watchdog remain active.',flush=True)
        while rclpy.ok() and not racer.stopped:
            rclpy.spin_once(racer,timeout_sec=.01)
            if guard.poll() is not None:raise RuntimeError('Independent watchdog exited')
    except KeyboardInterrupt:
        racer.reason='keyboard_stop'
    except Exception as error:
        racer.reason='controller_error: '+str(error)
        raise
    finally:
        racer.armed=False;racer.stopped=True
        end=time.monotonic()+5
        while rclpy.ok() and time.monotonic()<end:
            racer.publish(0.,0.,0.);rclpy.spin_once(racer,timeout_sec=.02)
            if racer.speed<.03 and time.monotonic()-racer.pose_time<.3 and time.monotonic()>end-4:break
        result=dict(reason=racer.reason,best_clean_lap_s=racer.best,valid_clean_laps=[r for r in racer.records if r['valid_clean_full_lap']],
                    rejected_laps=[r for r in racer.records if not r['valid_clean_full_lap']],total_collisions=racer.total_collisions,
                    stationary_verified=racer.speed<.03 and time.monotonic()-racer.pose_time<.3,
                    crash_stop_enabled=args.stop_on_crash,simulator_pose_assisted=True,
                    safety_policy='SIMULATOR_DIRECT_BENCHMARK_NO_PROXIMITY_GOVERNOR',
                    five_second_target_achieved=racer.best is not None and racer.best<5)
        atomic_json(output/'result.json',result);print('RESULT',json.dumps(result),flush=True)
        guard.send_signal(signal.SIGINT) if guard.poll() is None else None
        try:guard.wait(timeout=4)
        except subprocess.TimeoutExpired:guard.kill();guard.wait()
        racer.journal.close();racer.telemetry.close();racer.destroy_node();rclpy.shutdown()


def watchdog(args):
    import rclpy
    from std_msgs.msg import Bool,Float32
    rclpy.init();node=rclpy.create_node('onefile_independent_watchdog')
    throttle=node.create_publisher(Float32,SIM+'throttle_command',1)
    steering=node.create_publisher(Float32,SIM+'steering_command',1)
    last=[time.monotonic()];seen=[False]
    node.create_subscription(Bool,'/onefile/heartbeat',lambda m:(last.__setitem__(0,time.monotonic()),seen.__setitem__(0,True)),10)
    # This publisher exists only as a fail-safe competitor. The preflight
    # identity check recognises it explicitly rather than allowing old stacks.
    try:
        while True:
            try:os.kill(args.owner_pid,0)
            except ProcessLookupError:break
            rclpy.spin_once(node,timeout_sec=.03)
            if seen[0] and time.monotonic()-last[0]>.35:
                throttle.publish(Float32(data=0.));steering.publish(Float32(data=0.))
    except KeyboardInterrupt:pass
    finally:
        for _ in range(10):throttle.publish(Float32(data=0.));steering.publish(Float32(data=0.));rclpy.spin_once(node,timeout_sec=.02)
        node.destroy_node();rclpy.shutdown()


def command(argv, check=True, **kwargs):
    result=subprocess.run([str(x) for x in argv],check=False,**kwargs)
    if check and result.returncode:
        detail=getattr(result,'stderr','') or getattr(result,'stdout','') or ''
        if isinstance(detail,bytes):detail=detail.decode(errors='replace')
        raise RuntimeError('Command failed: '+str(argv[0])+' (exit '+str(result.returncode)+') '+str(detail)[-2500:])
    return result


def ps(script, data=None):
    if data is not None:
        encoded=base64.b64encode(json.dumps(data).encode()).decode()
        script="$c=[Text.Encoding]::UTF8.GetString([Convert]::FromBase64String('"+encoded+"'))|ConvertFrom-Json\n"+script
    encoded=base64.b64encode(script.encode('utf-16le')).decode()
    return command(['powershell.exe','-NoProfile','-ExecutionPolicy','Bypass','-EncodedCommand',encoded],capture_output=True,text=True)


def windows_simulator(executable, port, log):
    result=ps(r'''
$ErrorActionPreference='Stop'
if (Get-Process -ErrorAction SilentlyContinue | Where-Object { $_.Path -eq $c.exe }) {
 throw 'A project simulator is already open. Stop/close that old simulator before starting the one-file mode.'
}
$arguments='-screen-fullscreen 0 -screen-width 1280 -screen-height 720 -ip 127.0.0.1 -port '+$c.port+' -logFile "'+$c.log+'"'
$p=Start-Process -FilePath $c.exe -WorkingDirectory (Split-Path $c.exe) -ArgumentList $arguments -PassThru
try {$p.WaitForInputIdle(30000)|Out-Null}catch{}
@{pid=$p.Id}|ConvertTo-Json -Compress
''',dict(exe=executable,port=port,log=log))
    return json.loads(result.stdout)['pid']


def windows_connect(pid, executable, screenshot):
    ps(r'''
$ErrorActionPreference='Stop'
Add-Type -AssemblyName System.Drawing
Add-Type @'
using System;using System.Runtime.InteropServices;
public static class OneFileWindow {
 [StructLayout(LayoutKind.Sequential)]public struct RECT{public int Left,Top,Right,Bottom;}
 [StructLayout(LayoutKind.Sequential)]public struct POINT{public int X,Y;}
 [DllImport("user32.dll")]public static extern bool GetWindowRect(IntPtr h,out RECT r);
 [DllImport("user32.dll")]public static extern bool GetClientRect(IntPtr h,out RECT r);
 [DllImport("user32.dll")]public static extern bool ClientToScreen(IntPtr h,ref POINT p);
 [DllImport("user32.dll")]public static extern bool ShowWindow(IntPtr h,int c);
 [DllImport("user32.dll")]public static extern bool SetWindowPos(IntPtr h,IntPtr a,int x,int y,int w,int z,uint f);
 [DllImport("user32.dll")]public static extern bool PrintWindow(IntPtr h,IntPtr d,uint f);
 [DllImport("user32.dll")]public static extern bool PostMessage(IntPtr h,uint m,UIntPtr w,IntPtr l);
 [DllImport("user32.dll")]public static extern IntPtr SetThreadDpiAwarenessContext(IntPtr c);
 [DllImport("user32.dll")]public static extern bool SetForegroundWindow(IntPtr h);
 [DllImport("user32.dll")]public static extern bool SetCursorPos(int x,int y);
 [DllImport("user32.dll")]public static extern IntPtr WindowFromPoint(POINT p);
 [DllImport("user32.dll")]public static extern uint GetWindowThreadProcessId(IntPtr h,out uint p);
 [DllImport("user32.dll")]public static extern void mouse_event(uint f,uint x,uint y,uint d,UIntPtr e);
}
'@
[OneFileWindow]::SetThreadDpiAwarenessContext([IntPtr](-4))|Out-Null
$deadline=(Get-Date).AddSeconds(60)
do{$p=Get-Process -Id $c.pid;$p.Refresh();if($p.MainWindowHandle){break};Start-Sleep -Milliseconds 200}while((Get-Date)-lt$deadline)
if($p.Path -ne $c.exe -or !$p.MainWindowHandle){throw 'Unverified simulator process/window'}
$script:h=$p.MainWindowHandle
[OneFileWindow]::ShowWindow($h,9)|Out-Null
[OneFileWindow]::SetWindowPos($h,[IntPtr](-1),20,20,0,0,0x41)|Out-Null
$outer=New-Object OneFileWindow+RECT;$inside=New-Object OneFileWindow+RECT
[OneFileWindow]::GetWindowRect($h,[ref]$outer)|Out-Null
[OneFileWindow]::GetClientRect($h,[ref]$inside)|Out-Null
$width=1280+($outer.Right-$outer.Left)-$inside.Right
$height=720+($outer.Bottom-$outer.Top)-$inside.Bottom
[OneFileWindow]::SetWindowPos($h,[IntPtr](-1),20,20,$width,$height,0x40)|Out-Null
function Capture {
 $r=New-Object OneFileWindow+RECT;$client=New-Object OneFileWindow+RECT;$origin=New-Object OneFileWindow+POINT
 do {
  $current=Get-Process -Id $c.pid;$current.Refresh();$script:h=$current.MainWindowHandle
  [OneFileWindow]::GetClientRect($h,[ref]$client)|Out-Null
  if($client.Right-eq1280-and$client.Bottom-eq720){break}
  Start-Sleep -Milliseconds 200
 } while ((Get-Date) -lt $deadline)
 if($client.Right-ne1280-or$client.Bottom-ne720){throw "Unverified client geometry: $($client.Right)x$($client.Bottom)"}
 [OneFileWindow]::GetWindowRect($h,[ref]$r)|Out-Null
 [OneFileWindow]::ClientToScreen($h,[ref]$origin)|Out-Null
 $script:ox=$origin.X-$r.Left;$script:oy=$origin.Y-$r.Top
 $im=New-Object Drawing.Bitmap ($r.Right-$r.Left),($r.Bottom-$r.Top)
 $g=[Drawing.Graphics]::FromImage($im);$dc=$g.GetHdc()
 try{if(![OneFileWindow]::PrintWindow($h,$dc,2)){throw 'Window capture failed'}}finally{$g.ReleaseHdc($dc);$g.Dispose()}
 return $im
}
function White($im,$x,$y){$v=$im.GetPixel($script:ox+$x,$script:oy+$y);return $v.R-gt220-and$v.G-gt220-and$v.B-gt220}
function Menu($im){return (White $im 219 113)-and(White $im 219 131)-and(White $im 32 113)}
function Click($x,$y){
 [OneFileWindow]::SetForegroundWindow($h)|Out-Null
 $point=New-Object OneFileWindow+POINT;$point.X=$x;$point.Y=$y
 [OneFileWindow]::ClientToScreen($h,[ref]$point)|Out-Null
 [OneFileWindow]::SetCursorPos($point.X,$point.Y)|Out-Null
 Start-Sleep -Milliseconds 100
 $under=[OneFileWindow]::WindowFromPoint($point);[uint32]$owner=0
 [OneFileWindow]::GetWindowThreadProcessId($under,[ref]$owner)|Out-Null
 if($owner-ne$c.pid){throw 'Simulator is occluded; no click into another application is allowed'}
 [OneFileWindow]::mouse_event(2,0,0,0,[UIntPtr]::Zero)
 Start-Sleep -Milliseconds 80
 [OneFileWindow]::mouse_event(4,0,0,0,[UIntPtr]::Zero)
}
do{$im=Capture;$ready=White $im 50 30;if($ready){break};$im.Dispose();Start-Sleep -Milliseconds 300}while((Get-Date)-lt$deadline)
if(!$ready){throw 'Scene not ready; no click performed'}
$im.Save($c.screenshot+'.before.png')
if(!(Menu $im)){$im.Dispose();Click 50 30;Start-Sleep -Milliseconds 300;$im=Capture}
$im.Save($c.screenshot+'.after-menu.png')
if(!(Menu $im)){throw 'Connection fields not verified; no blind click'}
$im.Save($c.screenshot);$im.Dispose()
Click 144 275
[OneFileWindow]::SetWindowPos($h,[IntPtr](-2),0,0,0,0,0x43)|Out-Null
''',dict(pid=pid,exe=executable,screenshot=screenshot))


def native_gui_connect(args):
    # Same-file GUI helper runs inside the dependency image, not a separate module.
    from PIL import Image,ImageOps
    from Xlib import X,display
    from Xlib.ext import xtest
    d=display.Display();pid=int(Path('/run/onefile-unity.pid').read_text());window=None
    deadline=time.monotonic()+60
    while time.monotonic()<deadline:
        found=command(['xdotool','search','--onlyvisible','--pid',str(pid)],check=False,capture_output=True,text=True)
        candidates=[]
        for value in found.stdout.split():
            w=d.create_resource_object('window',int(value));name=w.get_wm_name() or ''
            if isinstance(name,bytes):name=name.decode(errors='replace')
            prop=w.get_full_property(d.intern_atom('_NET_WM_PID'),X.AnyPropertyType)
            if 'AutoDRIVE' in name and prop is not None and int(prop.value[0])==pid:candidates.append(w)
        if len(candidates)==1:window=candidates[0];break
        time.sleep(.2)
    assert window is not None,'No unique project Unity window'
    command(['xdotool','windowsize','--sync',str(window.id),'1280','720'])
    window.configure(stack_mode=X.Above);window.set_input_focus(X.RevertToParent,X.CurrentTime);d.sync()
    def capture():
        r=window.get_geometry();assert(r.width,r.height)==(1280,720)
        raw=window.get_image(0,0,1280,720,X.ZPixmap,0xffffffff)
        assert len(raw.data)==1280*720*4
        return Image.frombytes('RGB',(1280,720),raw.data,'raw','BGRX')
    def white(im,x,y):return min(im.getpixel((x,y)))>220
    def menu(im):return all(white(im,x,y)for x,y in[(219,113),(219,131),(32,113)])
    def click(x,y):
        assert window.grab_pointer(False,X.ButtonPressMask|X.ButtonReleaseMask|X.PointerMotionMask,X.GrabModeAsync,X.GrabModeAsync,window,X.NONE,X.CurrentTime)==X.GrabSuccess
        try:
            window.warp_pointer(x,y);d.sync();xtest.fake_input(d,X.ButtonPress,1);xtest.fake_input(d,X.ButtonRelease,1);d.sync()
        finally:d.ungrab_pointer(X.CurrentTime);d.sync()
    def label(im,box):
        crop=ImageOps.grayscale(im.crop(box)).point(lambda v:0 if v>180 else 255)
        crop=crop.resize((crop.width*3,crop.height*3));data=io.BytesIO();crop.save(data,format='PNG')
        return command(['tesseract','stdin','stdout','--psm','7'],input=data.getvalue(),capture_output=True).stdout.decode(errors='replace').lower()
    image=capture()
    while not white(image,50,30)and time.monotonic()<deadline:time.sleep(.2);image=capture()
    assert white(image,50,30),'Scene not ready'
    if not menu(image):click(50,30);time.sleep(.3);image=capture()
    assert menu(image),'Unverified connection fields'
    connection=label(image,(75,245,305,305))
    if 'disconnected'in connection:click(144,275);time.sleep(.4)
    elif 'connected'not in connection:raise RuntimeError('Unknown connection label')
    image=capture();mode=label(image,(75,310,305,365))
    if 'manual'in mode:click(145,338);time.sleep(.3);mode=label(capture(),(75,310,305,365))
    assert 'autonomous'in mode,'Autonomous mode not verified'
    capture().save('/logs/onefile-connection.png')


def gpu_check():
    import re
    result=command(['vulkaninfo','--summary'],check=False,capture_output=True,text=True,timeout=30)
    devices=[]
    for block in re.split(r'(?m)^\s*GPU\d+:\s*$',result.stdout)[1:]:
        if re.search(r'deviceType\s*=\s*(?:PHYSICAL_DEVICE_TYPE_)?(?:DISCRETE_GPU|INTEGRATED_GPU)',block):devices.append(block)
    print(json.dumps(dict(hardware_vulkan=bool(devices),returncode=result.returncode,stderr=result.stderr[-1500:])),flush=True)
    if result.returncode or not devices:raise SystemExit('No hardware Vulkan GPU; refusing software rendering')


def install_shortcuts(args):
    root=args.root.resolve();name=Path(__file__).name
    applications=Path(os.environ.get('XDG_DATA_HOME',str(Path.home()/'.local/share')))/'applications'
    applications.mkdir(parents=True,exist_ok=True)
    path_value=str(root).replace('\\','\\\\').replace(' ','\\s')
    for action,title in [([], 'Start'),(['--stop'],'STOP')]:
        code="import os,pathlib;os.execvp('python3',['python3',str(pathlib.Path.cwd()/"+repr(name)+")]+"+repr(action)+")"
        content='[Desktop Entry]\nType=Application\nName=RoboRacer One-File '+title+'\nPath='+path_value+'\nTerminal=false\nIcon=applications-games\nExec=python3 -c "'+code+'"\n'
        destination=applications/('capstone-onefile-'+title.lower()+'.desktop')
        if destination.exists() and 'RoboRacer One-File'not in destination.read_text():raise RuntimeError('Unrelated shortcut exists; refusing overwrite')
        destination.write_text(content);destination.chmod(0o755)
    print('Installed RoboRacer One-File Start/STOP in the applications menu. No driving started by installation.',flush=True)


def launch_host(args):
    root=args.root.resolve();manifest=read_json(root/'kit-manifest.json') or {}
    key=hashlib.sha256(str(root).encode()).hexdigest()[:8]
    stack='rr_onefile_'+key+'_stack';sim='rr_onefile_'+key+'_sim'
    network='rr_onefile_'+key+'_network'
    state_path=root/'logs/onefile-active.json'
    def owned(name):
        r=command(['docker','inspect',name,'--format','{{index .Config.Labels "project"}}'],check=False,capture_output=True,text=True)
        return r.returncode==0 and r.stdout.strip()==OWNER
    if args.stop:
        state=read_json(state_path)
        if not state:print('No one-file session found.');return
        directory=Path(state['session']);atomic_json(directory/'STOP',dict(requested_utc=datetime.now(timezone.utc).isoformat()))
        if owned(state['stack']):
            command(['docker','exec',state['stack'],'bash','-lc',ROS_SETUP+'exec python3 /root/ONEFILE_RACER.py --ros-stop'],check=False)
        print('One-file STOP requested; crash-stop policy does not override STOP.');return
    memory=args.memory
    if memory is None:
        candidates=[root/'evidence/windows-best-benchmark/track_memory',root/'report_figures/data/lap5-benchmark-20261022/presentation-20261022T025027112/track_memory']
        memory=next((p for p in candidates if p.is_dir()),None)
    if memory is None or not Path(memory).is_dir():raise SystemExit('Genuine recorded memory not found. Put this file in the existing kit, or specify --memory.')
    memory=Path(memory).resolve();active=memory/'active_track'if(memory/'active_track').is_dir()else memory
    assert(active/'benchmark_racing_line.csv').exists()and(active/'map.pgm').exists(),'Real map/racing line missing'
    if args.platform=='native'and('microsoft'in platform.release().lower()or sys.platform!='linux'):
        raise SystemExit('Native mode requires native Linux. Windows/WSL tests must explicitly select --platform windows.')
    if args.platform=='native':install_shortcuts(args)
    ready=command(['docker','info'],check=False,capture_output=True)
    if ready.returncode and args.platform=='windows':
        ps("Start-Process 'C:\\Program Files\\Docker\\Docker\\Docker Desktop.exe'")
        end=time.monotonic()+120
        while time.monotonic()<end:
            time.sleep(2)
            if command(['docker','info'],check=False,capture_output=True).returncode==0:break
        else:raise RuntimeError('Docker did not start')
    elif ready.returncode:raise SystemExit('Docker unavailable; install/start Docker and configure user permission')
    old=read_json(state_path)
    if old and owned(stack):
        previous=Path(old['session'])
        status=read_json(previous/'drive/status.json')or{}
        if status.get('running')and status.get('armed')and not(previous/'STOP').exists():
            raise SystemExit('One-file racer already active. Use its STOP button or --stop first.')
        if args.platform=='windows'and old.get('windows_pid')and old.get('windows_exe'):
            ps(r'''
$ErrorActionPreference='Stop'
# A missing previous PID is normal after the simulator was closed. Get-Process
# -Id with SilentlyContinue still leaves PowerShell's exit status unsuccessful.
$p=Get-Process | Where-Object { $_.Id -eq $c.pid }
if($p -and $p.Path -eq $c.exe){$p.CloseMainWindow()|Out-Null;if(!$p.WaitForExit(2000)){Stop-Process -Id $p.Id -Force}}
exit 0
''',dict(pid=old['windows_pid'],exe=old['windows_exe']))
    for name in(sim,stack):
        if command(['docker','inspect',name],check=False,capture_output=True).returncode==0:
            if not owned(name):raise SystemExit('Refusing to touch unowned container '+name)
            command(['docker','stop','--time','5',name],check=False,capture_output=True)
            command(['docker','rename',name,name+'_previous_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S')])
    token='onefile-'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    session=root/'logs'/token;session.mkdir(parents=True,exist_ok=False)
    atomic_json(session/'configuration.json',vars_to_json(args))
    image=manifest.get('controller',{}).get('tag','roboracer-twin:benchmark-best-clean-20261022')
    simulator=manifest.get('simulator',{}).get('tag','roboracer-native-simulator:practice-20261022')
    if command(['docker','image','inspect',image],check=False,capture_output=True).returncode:
        archive=root/'images/docker-images.tar.gz'
        if not archive.exists():raise SystemExit('Required ROS dependency image is not available')
        command(['docker','load','--input',archive])
    inspected=command(['docker','network','inspect',network,'--format','{{index .Labels "project"}}'],check=False,capture_output=True,text=True)
    if inspected.returncode:command(['docker','network','create','--label','project='+OWNER,network])
    elif inspected.stdout.strip()!=OWNER:raise SystemExit('Refusing unowned network '+network)
    command(['docker','run','-d','--init','--name',stack,'--label','project='+OWNER,'--network',network,
        '-p','127.0.0.1:14567:4567','-v',str(session)+':/logs','--entrypoint','/bin/bash',image,'-lc','exec sleep infinity'])
    command(['docker','cp',Path(__file__).resolve(),stack+':/root/ONEFILE_RACER.py'])
    command(['docker','exec',stack,'mkdir','-p','/root/onefile_memory/active_track'])
    for name in('benchmark_racing_line.csv','learned_track.csv','map.yaml','map.pgm'):
        if(active/name).exists():command(['docker','cp',active/name,stack+':/root/onefile_memory/active_track/'+name])
    reference=memory/'benchmark_reference.json'
    if not reference.exists():reference=active.parent/'benchmark_reference.json'
    command(['docker','cp',reference,stack+':/root/onefile_memory/benchmark_reference.json'])
    command(['docker','exec','-d',stack,'bash','-lc',ROS_SETUP+'exec ros2 launch autodrive_roboracer bringup_headless.launch.py > /logs/bridge.log 2>&1'])
    atomic_json(state_path,dict(stack=stack,sim=sim,session=str(session),token=token))
    stop=threading.Event();httpd=None;windows_pid=None
    def request_stop(*unused):
        stop.set();atomic_json(session/'STOP',dict(requested_unix_s=time.time()))
    def heartbeat():
        while not stop.is_set():
            atomic_json(session/'host-heartbeat.json',dict(run_id=token,updated_unix_s=time.time()));stop.wait(.15)
    thread=threading.Thread(target=heartbeat,daemon=True);thread.start()
    nonce=secrets.token_urlsafe(20)
    class Handler(http.server.BaseHTTPRequestHandler):
        def log_message(self,*unused):pass
        def do_GET(self):
            if self.path.startswith('/status'):
                value=read_json(session/'drive/status.json')or dict(running=True,phase='preparing')
                body=json.dumps(value).encode();self.send_response(200);self.send_header('Content-Type','application/json');self.end_headers();self.wfile.write(body);return
            body=("<!doctype html><title>One-file racer</title><style>body{background:#102036;color:white;font:24px system-ui;margin:5vw}button{font-size:30px;background:#ad2020;color:white;padding:18px}pre{font-size:18px}</style><h1>Single-file simulator benchmark</h1><p>Crash-triggered stop "+('ON'if args.stop_on_crash else 'OFF')+". Collision laps are invalid. STOP and stale-data watchdog remain.</p><button onclick=\"fetch('/stop?token="+nonce+"',{method:'POST'})\">STOP</button><pre id='s'>Preparing</pre><script>setInterval(()=>fetch('/status').then(r=>r.json()).then(d=>s.textContent=JSON.stringify(d,null,2)),500)</script>").encode()
            self.send_response(200);self.send_header('Content-Type','text/html');self.end_headers();self.wfile.write(body)
        def do_POST(self):
            if self.path!='/stop?token='+nonce:self.send_error(403);return
            request_stop();self.send_response(200);self.end_headers();self.wfile.write(b'STOP requested')
    for sig in(signal.SIGINT,signal.SIGTERM):signal.signal(sig,request_stop)
    if hasattr(signal,'SIGHUP'):signal.signal(signal.SIGHUP,request_stop)
    try:
        httpd=http.server.ThreadingHTTPServer(('127.0.0.1',14999),Handler)
        threading.Thread(target=httpd.serve_forever,daemon=True).start()
        if not args.no_browser:
            if args.platform=='windows':ps("Start-Process 'http://localhost:14999'")
            else:command(['xdg-open','http://localhost:14999'],check=False)
        if args.platform=='windows':
            exe=root/'simulator-windows-2026-iros-practice/autodrive_simulator/AutoDRIVE Simulator.exe'
            def win(p):
                if os.name=='nt':return str(p)
                return command(['wslpath','-w',str(p)],capture_output=True,text=True).stdout.strip()
            assert exe.exists(),'Official project Windows simulator missing'
            windows_pid=windows_simulator(win(exe),14567,win(session/'unity.log'))
            atomic_json(state_path,dict(stack=stack,sim=sim,session=str(session),token=token,windows_pid=windows_pid,windows_exe=win(exe)))
            windows_connect(windows_pid,win(exe),win(session/'connection.png'))
        else:
            display=os.environ.get('DISPLAY');assert display and Path('/tmp/.X11-unix').is_dir(),'X11 desktop required'
            auth_directory=Path(os.environ.get('XDG_RUNTIME_DIR',str(Path.home()/'.cache')))/('capstone-onefile-'+key)
            auth_directory.mkdir(parents=True,exist_ok=True);auth_directory.chmod(0o700)
            auth=auth_directory/'Xauthority';auth.touch(mode=0o600)
            cookie=command(['xauth','nlist',display],capture_output=True,text=True).stdout
            assert cookie.strip(),'Missing X11 cookie'
            cookie='\n'.join('ffff'+line[4:]for line in cookie.splitlines())+'\n'
            command(['xauth','-f',auth,'nmerge','-'],input=cookie,text=True)
            gpu=['--gpus','all','-e','NVIDIA_DRIVER_CAPABILITIES=graphics,display,utility,compute']if shutil.which('nvidia-smi')else['--device','/dev/dri']
            graphics=gpu+['--ipc=host','-e','DISPLAY='+display,'-e','XAUTHORITY=/run/onefile.Xauthority',
                '-v','/tmp/.X11-unix:/tmp/.X11-unix:ro','-v',str(auth)+':/run/onefile.Xauthority:ro','-v',str(session)+':/logs',
                '-v',str(Path(__file__).resolve())+':/code/ONEFILE_RACER.py:ro']
            command(['docker','run','--rm',*graphics,'--entrypoint','python3',simulator,'/code/ONEFILE_RACER.py','--gpu-check'])
            command(['docker','run','-d','--init','--name',sim,'--label','project='+OWNER,'--network','container:'+stack,
                *graphics,'--entrypoint','python3',simulator,'/code/ONEFILE_RACER.py','--start-unity'])
            command(['docker','exec',sim,'python3','/code/ONEFILE_RACER.py','--native-gui'])
        drive=['python3','/root/ONEFILE_RACER.py','--ros-drive','--memory','/root/onefile_memory',
               '--output','/logs/drive','--token',token]
        for name in('max_speed','lateral','acceleration','braking','scale','maximum_scale','brake_cap','preview','brake_preview',
                    'heading_gain','cross_gain','speed_gain','steering_rate','throttle_rate','refine_sweeps','line_offset',
                    'clearance','run_seconds','stop_after_laps','tracker'):
            drive += ['--'+name.replace('_','-'),str(getattr(args,name))]
        for name in('fixed_scale','stop_on_crash','no_active_brake'):
            if getattr(args,name):drive.append('--'+name.replace('_','-'))
        log=(session/'controller.log').open('w')
        process=subprocess.Popen(['docker','exec','-e','CAPSTONE_SIM_TEST=1',stack,'bash','-lc',ROS_SETUP+'exec "$@"','onefile',*drive],stdout=log,stderr=subprocess.STDOUT)
        print('ONE FILE running. STOP dashboard: http://localhost:14999 ; results:',session,flush=True)
        while process.poll() is None:
            if stop.wait(.2):
                try:process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    command(['docker','exec',stack,'bash','-lc',ROS_SETUP+'exec python3 /root/ONEFILE_RACER.py --ros-stop'],check=False)
                    try:process.wait(timeout=5)
                    except subprocess.TimeoutExpired:process.terminate()
                break
        result=read_json(session/'drive/result.json')
        print(json.dumps(dict(output=str(session),result=result),indent=2),flush=True)
        if process.returncode:raise RuntimeError('Controller exited; inspect '+str(session/'controller.log'))
    finally:
        stop.set();thread.join(timeout=2)
        if owned(stack):command(['docker','exec',stack,'bash','-lc',ROS_SETUP+'exec python3 /root/ONEFILE_RACER.py --ros-stop'],check=False)
        if httpd:httpd.shutdown()
        # Successful/explicitly stopped results remain visible; no automatic
        # restarting after a STOP. Dependency containers are retained for logs.


def vars_to_json(args):return {k:str(v)if isinstance(v,Path)else v for k,v in vars(args).items()}


def ros_stop():
    import rclpy
    from std_msgs.msg import Bool,Float32
    rclpy.init();node=rclpy.create_node('onefile_stop')
    enable=node.create_publisher(Bool,'/roboracer/drive_enable',10)
    throttle=node.create_publisher(Float32,SIM+'throttle_command',1)
    end=time.monotonic()+1
    while time.monotonic()<end:
        enable.publish(Bool(data=False));throttle.publish(Float32(data=0.));rclpy.spin_once(node,timeout_sec=.03)
    node.destroy_node();rclpy.shutdown()


def self_test():
    event=dict(collision=False,teleport=False,transition=False,seen_previous_finish=True,
               distance_m=31.,coverage_fraction=.99,simulator_seconds=7.,wall_seconds=7.02)
    assert valid_lap(event,31.)
    for key in ('collision','teleport','transition'):
        changed=dict(event);changed[key]=True;assert not valid_lap(changed,31.)
    for key,value in [('seen_previous_finish',False),('distance_m',3.),('coverage_fraction',.1),('simulator_seconds',float('nan'))]:
        changed=dict(event);changed[key]=value;assert not valid_lap(changed,31.)
    assert fresh_lease(dict(run_id='test',updated_unix_s=99.8),'test',now=100)
    for value in (None,{},dict(run_id='other',updated_unix_s=100),dict(run_id='test',updated_unix_s=90),dict(run_id='test',updated_unix_s=float('nan'))):
        assert not fresh_lease(value,'test',now=100)
    print('PASS: collision/teleport/transition/warmup/shortcut/invalid-clock laps rejected; stale/corrupt/wrong-owner leases rejected. Synthetic unit fixtures only. No Docker, simulator or driving started.')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ros-drive',action='store_true');parser.add_argument('--ros-watchdog',action='store_true')
    parser.add_argument('--owner-pid',type=int,default=0)
    parser.add_argument('--ros-stop',action='store_true');parser.add_argument('--native-gui',action='store_true')
    parser.add_argument('--gpu-check',action='store_true');parser.add_argument('--start-unity',action='store_true')
    parser.add_argument('--no-browser',action='store_true')
    parser.add_argument('--install',action='store_true')
    parser.add_argument('--self-test',action='store_true')
    parser.add_argument('--memory',type=Path);parser.add_argument('--output',type=Path)
    parser.add_argument('--token',default='onefile')
    parser.add_argument('--max-speed',type=float,default=14.);parser.add_argument('--lateral',type=float,default=12.)
    parser.add_argument('--acceleration',type=float,default=12.);parser.add_argument('--braking',type=float,default=16.)
    parser.add_argument('--scale',type=float,default=.935);parser.add_argument('--maximum-scale',type=float,default=1.3)
    parser.add_argument('--fixed-scale',action='store_true');parser.add_argument('--stop-on-crash',action='store_true')
    parser.add_argument('--no-active-brake',action='store_true');parser.add_argument('--brake-cap',type=float,default=.35)
    parser.add_argument('--preview',type=float,default=.04);parser.add_argument('--brake-preview',type=float,default=.04)
    parser.add_argument('--heading-gain',type=float,default=8.);parser.add_argument('--cross-gain',type=float,default=12.)
    parser.add_argument('--speed-gain',type=float,default=.12);parser.add_argument('--steering-rate',type=float,default=8.)
    parser.add_argument('--throttle-rate',type=float,default=3.)
    parser.add_argument('--tracker',choices=['feedback','pursuit'],default='pursuit')
    parser.add_argument('--refine-sweeps',type=int,default=4);parser.add_argument('--line-offset',type=float,default=.25)
    parser.add_argument('--clearance',type=float,default=.30)
    parser.add_argument('--run-seconds',type=float,default=0);parser.add_argument('--stop-after-laps',type=int,default=0)
    parser.add_argument('--platform',choices=['windows','native'],default='native')
    parser.add_argument('--root',type=Path,default=Path(__file__).resolve().parent)
    parser.add_argument('--stop',action='store_true')
    args=parser.parse_args()
    if args.self_test:return self_test()
    if args.install:return install_shortcuts(args)
    for key in ('max_speed','lateral','acceleration','braking','scale','maximum_scale','preview','brake_preview','heading_gain','cross_gain','speed_gain','steering_rate','throttle_rate','brake_cap','line_offset','clearance','run_seconds'):
        value=getattr(args,key)
        if not math.isfinite(value) or value<0:parser.error('Invalid '+key)
    if not 0<args.max_speed<=25 or not 0<args.lateral<=40 or not 0<args.scale<=args.maximum_scale<=2:parser.error('Invalid benchmark envelope')
    if not 0<args.acceleration<=40 or not 0<args.braking<=40 or not 0<=args.brake_cap<=1:parser.error('Invalid actuator/planner parameters')
    if args.start_unity:
        Path('/run/onefile-unity.pid').write_text(str(os.getpid()));os.chdir('/home/autodrive_simulator')
        os.execv('./AutoDRIVE Simulator.x86_64',['./AutoDRIVE Simulator.x86_64','-force-vulkan','-screen-fullscreen','0',
            '-screen-width','1280','-screen-height','720','-ip','127.0.0.1','-port','4567','-logFile','/logs/unity.log'])
    if args.gpu_check:return gpu_check()
    if args.native_gui:return native_gui_connect(args)
    if args.ros_stop:return ros_stop()
    if args.ros_watchdog:return watchdog(args)
    if args.ros_drive:
        if args.memory is None or args.output is None:parser.error('ROS drive needs genuine --memory and --output')
        return ros_mode(args)
    return launch_host(args)


if __name__=='__main__':main()
