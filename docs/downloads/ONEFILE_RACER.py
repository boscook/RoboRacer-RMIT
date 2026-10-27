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
The default --max-speed 0 means no imposed command-speed ceiling. Cornering,
combined grip, braking/acceleration and finite actuator commands still apply.
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
    px=(c*delta[:,0]+s*delta[:,1])/resolution-.5
    py=clearance.shape[0]-(-s*delta[:,0]+c*delta[:,1])/resolution-.5
    x,y=np.floor(px).astype(int),np.floor(py).astype(int)
    valid=(x>=0)&(y>=0)&(x+1<clearance.shape[1])&(y+1<clearance.shape[0])
    values=np.zeros(len(points));xx,yy=x[valid],y[valid];fx,fy=px[valid]-xx,py[valid]-yy
    values[valid]=((1-fx)*(1-fy)*clearance[yy,xx]+fx*(1-fy)*clearance[yy,xx+1]
                   +(1-fx)*fy*clearance[yy+1,xx]+fx*fy*clearance[yy+1,xx+1])
    # Conservative half-cell diagonal allowance; interpolation does not add
    # unobserved free space or silently treat unknown/outside cells as drivable.
    return np.maximum(0.,values-resolution/math.sqrt(2.))


def reference_speed(points, cap, lateral, acceleration, braking):
    curvature, heading, ds = geometry(points)
    ceiling = math.inf if cap == 0 else cap
    speeds = np.minimum(ceiling, np.sqrt(lateral/np.maximum(abs(curvature), .015)))
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


def vehicle_speed_profile(points, args):
    """Cyclic forward/backward traction/power/braking profile, not a lap result.

    Longitudinal and lateral accelerations share a grip ellipse. Steering-rate
    limits are geometric/actuator constraints, not an arbitrary speed cap.
    """
    curvature, heading, ds = geometry(points)
    ceiling = math.inf if args.max_speed == 0 else args.max_speed
    speed = np.minimum(ceiling, np.sqrt(args.lateral/np.maximum(abs(curvature), 1e-5)))
    steer = np.arctan(.33*curvature/getattr(args,'steering_response_gain',1.))
    steer_gradient = np.abs(np.roll(steer,-1)-steer)/np.maximum(ds,.001)
    speed = np.minimum(speed,args.steering_rate/np.maximum(steer_gradient,1e-5))
    count=len(points)
    # Starting below the total-grip lateral ceiling leaves a finite braking
    # reserve; the actual simulator may have different grip, recorded separately.
    total_grip=max(args.total_grip,args.lateral*1.001)
    for _ in range(9):
        old=speed.copy()
        for i in range(count-1,-1,-1):
            j=(i+1)%count
            lateral_used=min(total_grip,abs(curvature[j])*speed[j]**2)
            available=args.braking*math.sqrt(max(0.,1-(lateral_used/total_grip)**2))
            speed[i]=min(speed[i],math.sqrt(speed[j]**2+2*available*ds[i]))
        for i in range(count):
            j=(i+1)%count
            lateral_used=min(total_grip,abs(curvature[i])*speed[i]**2)
            available=args.acceleration*math.sqrt(max(0.,1-(lateral_used/total_grip)**2))
            # Empirical motor roll-off model; never a hard straight-line cap.
            available/=1+(speed[i]/args.motor_knee)**2
            speed[j]=min(speed[j],math.sqrt(speed[i]**2+2*available*ds[i]))
        if float(np.max(old-speed))<1e-5:break
    # End-point sweeps can slightly exceed a segment's combined budget as
    # curvature changes. Project the complete profile back inside that ellipse.
    for _ in range(2):
        following=np.roll(speed,-1)
        longitudinal=(following**2-speed**2)/(2*np.maximum(ds,.001))
        lateral_mid=(abs(curvature)*speed**2+np.roll(abs(curvature)*speed**2,-1))/2
        mean_speed=(speed+following)/2
        budget=np.where(longitudinal>=0,args.acceleration/(1+(mean_speed/args.motor_knee)**2),args.braking)
        utilisation=np.sqrt((longitudinal/np.maximum(budget,.001))**2+(lateral_mid/total_grip)**2)
        speed/=math.sqrt(max(1.,float(utilisation.max())))
    assert np.isfinite(speed).all() and float(speed.min())>0,'Nonfinite/infeasible speed profile'
    return speed,curvature,heading,ds


def footprint_clearance(points, clearance, origin, resolution):
    """Sample a 0.62 x 0.42 m inflated axle-centred footprint along the line."""
    _,heading,_=geometry(points)
    forward=np.column_stack((np.cos(heading),np.sin(heading)))
    normal=np.column_stack((-np.sin(heading),np.cos(heading)))
    offsets=[(longitudinal,lateral)for longitudinal in np.linspace(-.22,.40,14)for lateral in(-.21,.21)]
    offsets += [(longitudinal,lateral)for longitudinal in(-.22,.40)for lateral in np.linspace(-.21,.21,10)]
    offsets += [(longitudinal,0.)for longitudinal in np.linspace(-.22,.40,8)]
    probes=[points+longitudinal*forward+lateral*normal for longitudinal,lateral in offsets]
    return clearance_at(np.vstack(probes),clearance,origin,resolution).reshape(len(probes),len(points)).min(axis=0)


def joint_racing_line(points,clearance,origin,resolution,args):
    """Optimise estimated complete-lap time and vehicle-feasible track usage.

    Smooth multi-scale moves use the actual saved free-space corridor. Every
    accepted proposal checks the inflated vehicle, offsets, curvature and the
    full cyclic acceleration/braking profile, rather than only local curvature.
    """
    original=resample(points,.12);result=original.copy()
    tangent=np.roll(original,-3,axis=0)-np.roll(original,3,axis=0)
    normal=np.column_stack((-tangent[:,1],tangent[:,0]))
    normal/=np.maximum(np.linalg.norm(normal,axis=1)[:,None],1e-8)
    def evaluate(path):
        center=clearance_at(path,clearance,origin,resolution)
        if float(center.min())<args.clearance:return None
        curve,_,distance=geometry(path)
        if float(abs(curve).max())>getattr(args,'steering_response_gain',1.)*math.tan(.50)/.33:return None
        footprint=footprint_clearance(path,clearance,origin,resolution)
        if float(footprint.min())<args.footprint_margin:return None
        velocity,_,_,_=vehicle_speed_profile(path,args)
        seconds=float(np.sum(2*distance/(velocity+np.roll(velocity,-1))))
        gradient=(np.roll(curve,-1)-curve)/np.maximum(distance,.001)
        smooth=getattr(args,'line_smoothness',.003)*float(np.sum(gradient*gradient*distance))
        edge=.04*float(np.sum(np.maximum(0.,.50-center)**2*distance))
        return seconds+smooth+edge,seconds,float(center.min()),float(footprint.min())
    baseline=evaluate(result)
    dense_baseline=evaluate(resample(points))
    if baseline is None or dense_baseline is None:
        return resample(points),dict(accepted=False,reason='baseline failed joint footprint/steering constraints')
    dense_best=resample(points);dense_value=dense_baseline;accepted_proposals=0;rejected_dense=0
    best=baseline;spacing=float(geometry(original)[2].mean())
    controls=max(1,int(round(.60/spacing)))
    for sweep in range(args.refine_sweeps):
        check=getattr(args,'cancel_check',None)
        if check is not None and check():raise RuntimeError('Line optimisation cancelled; no driving armed')
        width=(1.05,.65,.38)[sweep%3];step=.10*.87**(sweep//3)
        # Alternate sweep direction so a preceding corner is not always favoured.
        control_indices=list(range(0,len(result),controls))
        if sweep%2:control_indices.reverse()
        for index in control_indices:
            separation=np.abs(np.arange(len(result))-index)
            separation=np.minimum(separation,len(result)-separation)*spacing
            influence=np.exp(-.5*(separation/width)**2)
            influence[separation>3*width]=0
            saved=result;chosen=result;value=best
            chosen_dense=dense_best;chosen_dense_value=dense_value
            for sign in (-1,1):
                candidate=saved+sign*step*normal*influence[:,None]
                if np.linalg.norm(candidate-original,axis=1).max()>args.line_offset:continue
                evaluated=evaluate(candidate)
                if evaluated is None or evaluated[0]>=value[0]-1e-6:continue
                # Inspect the actual dense line and retain feasible improvements.
                # Strict search rejects infeasible dense moves. Compatibility
                # search can explore them offline, but never arms such a line.
                dense_candidate=resample(candidate)
                verified=evaluate(dense_candidate)
                strict=getattr(args,'line_search_validation','retain-feasible')=='strict'
                if verified is None:
                    rejected_dense+=1
                    if strict:continue
                if not strict:
                    chosen,value=candidate,evaluated
                if verified is not None and verified[0]<chosen_dense_value[0]-1e-6 and verified[1]<dense_baseline[1]:
                    chosen,value=candidate,evaluated
                    chosen_dense,chosen_dense_value=dense_candidate,verified
                    accepted_proposals+=1
            result,best=chosen,value
            dense_best,dense_value=chosen_dense,chosen_dense_value
    dense=dense_best
    checked=dense_value
    recovered_dense=True
    if getattr(args,'line_search_validation','retain-feasible')=='retain-feasible':
        final_dense=resample(result);final_value=evaluate(final_dense)
        if final_value is not None and final_value[0]<dense_baseline[0] and final_value[1]<dense_baseline[1]:
            dense,checked=final_dense,final_value;recovered_dense=False
    accepted=checked is not None and dense_baseline is not None and checked[1]<dense_baseline[1] and checked[0]<dense_baseline[0]
    if not accepted:dense=resample(points)
    selected=evaluate(dense)
    return dense,dict(accepted=accepted,objective='cyclic vehicle-speed-profile time + curvature-rate/edge penalties',
        original_estimated_profile_s=dense_baseline[1] if dense_baseline else baseline[1],selected_estimated_profile_s=selected[1] if selected else None,
        original_cost=dense_baseline[0] if dense_baseline else baseline[0],selected_cost=selected[0] if selected else None,
        minimum_dense_footprint_clearance_m=float(footprint_clearance(dense,clearance,origin,resolution).min()),
        dense_validation_before_arming=True,search_validation=getattr(args,'line_search_validation','retain-feasible'),
        recovered_prior_dense_feasible_candidate=recovered_dense,
        retained_dense_improvements=accepted_proposals,rejected_dense_proposals=rejected_dense,
        offset_bound_m=args.line_offset,footprint_length_m=.62,footprint_width_m=.42,
        note='All times here are planning estimates, not driven results; physical/model validation is separate.')


def refine_for_time(points, clearance, origin, resolution, args):
    """Map-constrained local search on genuine data, never a supplied fake circuit."""
    original = resample(points, .04); result = original.copy()
    tangent = np.roll(original, -5, axis=0)-np.roll(original, 5, axis=0)
    normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))
    normal /= np.maximum(np.linalg.norm(normal, axis=1)[:, None], 1e-8)
    def cost(path):
        k, _, ds = geometry(path)
        v = np.minimum(math.inf if args.max_speed==0 else args.max_speed, np.sqrt(args.lateral/np.maximum(abs(k), .015)))
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
        optimiser=joint_racing_line if args.line_planner=='joint' else refine_for_time
        points, refinement = optimiser(points, clearance, metadata['origin'], metadata['resolution'], args)
    else:
        points = resample(points); refinement = dict(selected_cost=None)
    checked = clearance_at(points, clearance, metadata['origin'], metadata['resolution'])
    assert checked.min() >= .20, 'Path not clearance-valid in the actual saved map'
    body_clearance=footprint_clearance(points,clearance,metadata['origin'],metadata['resolution'])
    if args.line_planner=='joint':
        assert float(checked.min())>=args.clearance,'Final joint line failed dense centre clearance'
        assert float(body_clearance.min())>=args.footprint_margin,'Final joint line failed dense inflated-footprint clearance'
    if args.line_planner=='joint':
        speed,curvature,heading,ds=vehicle_speed_profile(points,args)
    else:
        speed, curvature, heading, ds = reference_speed(points, args.max_speed, args.lateral, args.acceleration, args.braking)
    proof = dict(source='genuine previously recorded simulator-assisted memory, NOT online LiDAR-only learning',
                 source_path=str(path), source_sha256=original_hash, points=len(points), length_m=float(ds.sum()),
                 minimum_saved_map_clearance_m=float(checked.min()), refinement=refinement,
                 minimum_dense_inflated_footprint_clearance_m=float(body_clearance.min()),
                 line_planner=args.line_planner,imposed_command_speed_cap_mps=args.max_speed if args.max_speed else None,
                 peak_planned_speed_mps=float(speed.max()),lateral_planning_mps2=args.lateral,
                 steering_response_gain=getattr(args,'steering_response_gain',1.),
                 ideal_profile_s=float(np.sum(2*ds/(speed+np.roll(speed, -1)))),
                 note='Ideal profile is a planning estimate, not a driven lap time.')
    return points, speed, curvature, heading, ds, anchor, proof


def valid_lap(event, length):
    return (not event['collision'] and not event['teleport'] and not event['transition']
            and event['seen_previous_finish'] and event['distance_m'] >= .85*length
            and event['coverage_fraction'] >= .80 and math.isfinite(event['simulator_seconds'])
            and event['simulator_seconds'] > 0
            and abs(event['simulator_seconds']-event['wall_seconds']) <= max(.4, .10*event['simulator_seconds']))


def median(values):
    values=sorted(values);count=len(values)
    return (values[(count-1)//2]+values[count//2])/2


def choose_pace(scale,samples,step,failed_scale,maximum,minimum_laps=3):
    """Bracket pace using repeated actual lap medians, not a single lucky best.

    Returns a proposal only; it does not arm/reset a vehicle or certify safety.
    A noisy slower lap must not trap the search below an already observed pace.
    """
    complete=[(median(times[-5:]),float(key))for key,times in samples.items()if len(times)>=minimum_laps]
    if not complete:return scale,step,failed_scale,None,None
    best_median,best_scale=min(complete)
    current=samples.get(round(scale,5),[])
    if len(current)<minimum_laps:return scale,step,failed_scale,best_median,best_scale
    if median(current[-5:])>best_median*1.025 and scale>best_scale+.003:
        failed_scale=min(failed_scale,scale)if failed_scale is not None and failed_scale>best_scale else scale
        step=max(.005,step*.5);proposal=best_scale
    else:
        proposal=scale+step
        if failed_scale is not None and failed_scale>best_scale+.005:
            proposal=min(proposal,(best_scale+failed_scale)/2)
        elif failed_scale is not None:
            failed_scale=None
    return clip(proposal,.4,maximum),step,failed_scale,best_median,best_scale


def export_speed_profile(output,points,speed,curvature,heading,ds):
    distance=np.r_[0.,np.cumsum(ds[:-1])]
    acceleration=(np.roll(speed,-1)**2-speed**2)/(2*np.maximum(ds,.001))
    with (Path(output)/'actually_used_path.csv').open('w',newline='')as stream:
        writer=csv.writer(stream)
        writer.writerow(['x','y','planned_speed','path_s_m','curvature','heading','planned_acceleration'])
        writer.writerows(zip(points[:,0],points[:,1],speed,distance,curvature,heading,acceleration))


def trajectory_acceleration(speed,ds):
    """Continuous spatial acceleration prior; no sampled-target differencing.

    The modest cyclic filter is applied to feedforward only, not to lap clocks,
    measured telemetry or the validated speed/clearance constraints.
    """
    acceleration=(np.roll(speed,-1)**2-speed**2)/(2*np.maximum(ds,.001))
    return sum(weight*np.roll(acceleration,offset)for offset,weight in[(-2,1/9),(-1,2/9),(0,3/9),(1,2/9),(2,1/9)])


def shape_acceleration_prior(prior,scale,speed,reference,steering,path_error,heading_error,requested,args):
    demand=clip(float(prior)*scale**2*clip(speed/max(reference,.5),.5,1.2),-args.braking,args.acceleration)
    if demand>0:
        turning_demand=speed**2*abs(math.tan(steering))/.33
        remaining=math.sqrt(max(0.,1-(turning_demand/args.total_grip)**2))
        demand=min(demand,args.acceleration*remaining/(1+(speed/args.motor_knee)**2))
    if path_error>.12 or abs(heading_error)>.35 or requested<-args.braking*.5:
        demand=min(0.,requested,demand)
    return demand


def predict_vehicle_pose(pose,speed,yaw_rate,acceleration,delay):
    """Predict real measured motion through command transport/steering delay."""
    x,y,angle=pose
    distance=max(0.,(speed+.5*acceleration*delay)*delay)
    rotation=yaw_rate/max(speed,.5)*distance
    gain=math.sin(rotation/2)/(rotation/2)if abs(rotation)>1e-6 else 1.
    return (x+distance*gain*math.cos(angle+rotation/2),
            y+distance*gain*math.sin(angle+rotation/2),wrap(angle+rotation))


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
    shutil.copy2(__file__,output/'controller_source.py')
    args.cancel_check=lambda:Path('/logs/STOP').exists() or not fresh_lease(read_json('/logs/host-heartbeat.json'),args.token)
    points, planned, curvature, heading, ds, anchor, proof = load_memory(args.memory, args)
    proof['controller_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    proof['crash_stop_enabled']=args.stop_on_crash
    proof['safety_policy']='SIMULATOR_DIRECT_BENCHMARK_NO_PROXIMITY_GOVERNOR'
    atomic_json(output/'memory_provenance.json', proof)
    export_speed_profile(output,points,planned,curvature,heading,ds)
    acceleration_prior=trajectory_acceleration(planned,ds)
    rclpy.init()

    class Racer(Node):
        def __init__(self):
            super().__init__('onefile_simulator_racer')
            self.pose = None; self.speed = 0.; self.yaw_rate = 0.; self.pose_time = self.scan_time = 0.
            self.measured_acceleration=0.
            self.last_position = None; self.forward = False; self.last_motion = time.monotonic()
            self.collisions = 0; self.total_collisions = 0; self.count = 0; self.last_time = float('nan')
            self.armed = False; self.stopped = False; self.reason = 'running'; self.index = None
            self.racing_started = False
            self.steering = 0.; self.throttle = 0.; self.reference = 0.; self.integral = 0.
            self.feedforward_acceleration=0.
            self.scale = args.scale; self.best_scale = args.scale; self.best = None
            self.started = time.monotonic(); self.last_tick = self.started; self.lap_started = self.started
            self.lap_collision = False; self.lap_teleport = False; self.lap_transition = False
            self.seen_finish = False; self.distance = 0.; self.coverage = set(); self.pending = []
            self.records = []; self.last_log = 0.; self.last_status = 0.; self.clean_since_tune = 0
            self.tune_step = .04; self.failed_scale = None
            self.pace_samples={};self.best_median=None;self.best_median_scale=None
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
                self.publish(0., self.stopping_steering(), 0.)

        def stopping_steering(self):
            # Zero propulsion immediately, but do not straighten the wheels
            # while coasting out of a curved line: that can run into the wall.
            # This never commands reverse/braking throttle or rearms driving.
            if self.pose is None or time.monotonic()-self.pose_time>.25 or self.speed<.05 or not self.forward:
                return 0.
            x,y,angle=self.pose
            index=int(np.argmin(np.linalg.norm(points-[x,y],axis=1)))
            look=clip(.5+.12*self.speed,.65,1.3);travel=0.;target=index
            while travel<look:
                travel+=ds[target];target=(target+1)%len(points)
            dx,dy=points[target]-[x,y]
            steering=math.atan(.33*2*(-math.sin(angle)*dx+math.cos(angle)*dy)/max(dx*dx+dy*dy,.02))
            return clip(steering,-.5236,.5236)

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
            if args.motion_prediction=='arc'and .008<dt<.15:
                measured=clip((speed-self.speed)/dt,-args.braking,args.acceleration)
                self.measured_acceleration=.75*self.measured_acceleration+.25*measured
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
                observed_best=self.best_median_scale if self.best_median_scale is not None else self.best_scale
                if self.scale>observed_best+.005:
                    self.failed_scale=self.scale if self.failed_scale is None or self.failed_scale<=observed_best else min(self.failed_scale,self.scale)
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
                was_warmup=not self.seen_finish
                self.pending.append(dict(lap=value, collision=self.lap_collision, teleport=self.lap_teleport,
                    transition=self.lap_transition, seen_previous_finish=self.seen_finish,
                    distance_m=self.distance, coverage_fraction=len(self.coverage)/100,
                    wall_seconds=now-self.lap_started, scale=self.scale*(args.warmup_scale if was_warmup else 1.), due=now+.16))
                self.lap_started = now; self.seen_finish = True
                self.lap_collision = False; self.lap_teleport = False
                self.lap_transition = was_warmup and args.warmup_scale!=1.
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
                    key=round(event['scale'],5)
                    self.pace_samples.setdefault(key,[]).append(seconds)
                    self.pace_samples[key]=self.pace_samples[key][-max(9,args.tune_hold_laps):]
                    if self.best is None or seconds < self.best:
                        self.best = seconds; self.best_scale = event['scale']
                        print(f'CLEAN BEST {seconds:.4f} s; scale={event["scale"]:.3f}; collisions overall={self.total_collisions}', flush=True)
                    if not args.fixed_scale and self.clean_since_tune >= args.tune_hold_laps:
                        old_scale=self.scale
                        self.scale,self.tune_step,self.failed_scale,self.best_median,self.best_median_scale=choose_pace(
                            self.scale,self.pace_samples,self.tune_step,self.failed_scale,args.maximum_scale,args.tune_hold_laps)
                        if abs(self.scale-old_scale)>.001:self.lap_transition=True
                        self.clean_since_tune = 0
                        atomic_json(output/'pace-learning.json',dict(policy='repeated-clean-lap medians, independent from fastest single clean lap',
                            minimum_clean_laps=args.tune_hold_laps,samples={str(k):v for k,v in self.pace_samples.items()},
                            selected_scale=self.scale,best_median_s=self.best_median,best_median_scale=self.best_median_scale,
                            failed_scale=self.failed_scale,step=self.tune_step))
                else:
                    print('INVALID/TRANSITION LAP', json.dumps(event), flush=True)
                if args.stop_after_laps and len([r for r in self.records if r['valid_clean_full_lap']]) >= args.stop_after_laps:
                    self.stopped = True; self.armed = False; self.reason = 'requested_clean_laps_complete'

        def publish(self, throttle, steering, reference):
            if not all(math.isfinite(value)for value in(throttle,steering,reference)):
                self.stopped=True;self.armed=False;self.reason='nonfinite_command'
                throttle=steering=reference=0.
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
            requested_acceleration=profile_acceleration=0.
            if self.armed and fresh and not self.stopped and not self.resetting:
                x, y, angle = self.pose
                candidates = np.arange(len(points)) if self.index is None else (self.index+np.arange(-30, 121)) % len(points)
                distances = np.linalg.norm(points[candidates]-[x, y], axis=1)
                self.index = position_index = int(candidates[np.argmin(distances)])
                error = float(distances.min()); self.coverage.add(min(99, int(self.index*100/len(points))))
                # Account for the remaining bridge/actuator delay, without
                # replacing the real-pose error used by timing diagnostics.
                if args.motion_prediction=='arc':
                    px,py,pa=predict_vehicle_pose(self.pose,self.speed,self.yaw_rate,self.measured_acceleration,args.preview)
                else:
                    px=x+self.speed*args.preview*math.cos(angle)
                    py=y+self.speed*args.preview*math.sin(angle)
                    pa=wrap(angle+self.yaw_rate*args.preview)
                future = (self.index+np.arange(0, 51)) % len(points)
                target = int(future[np.argmin(np.linalg.norm(points[future]-[px, py], axis=1))])
                normal = np.array([-math.sin(heading[target]), math.cos(heading[target])])
                cross_track = float(np.dot(points[target]-[px, py], normal))
                feedback_curvature = (curvature[target]+args.heading_gain*wrap(heading[target]-pa)/(self.speed+1.)
                                      +args.cross_gain*cross_track/(self.speed+1.)**2)
                desired = math.atan(.33*feedback_curvature/args.steering_response_gain)
                if args.tracker == 'pursuit':
                    look=clip(args.lookahead_base+args.lookahead_speed*self.speed,args.lookahead_min,args.lookahead_max)
                    if args.curvature_lookahead>0:
                        look=min(look,max(args.lookahead_min,args.curvature_lookahead/(.5+math.sqrt(abs(curvature[target])))))
                    target=target if args.pursuit_origin=='predicted' else self.index
                    travel=0.
                    while travel < look:
                        travel += ds[target]; target=(target+1) % len(points)
                    dx, dy=points[target]-[px,py]
                    desired=math.atan(.33*2*(-math.sin(pa)*dx+math.cos(pa)*dy)/(max(dx*dx+dy*dy,.02)*args.steering_response_gain))
                desired = clip(desired, -.5236, .5236)
                steering = self.steering+clip(desired-self.steering, -args.steering_rate*dt, args.steering_rate*dt)
                preview_distance = max(.08, self.speed*args.brake_preview)
                indices=[self.index]; travel=0.
                while travel < preview_distance:
                    i=indices[-1]; travel+=ds[i]; indices.append((i+1) % len(points))
                reference_index=indices[int(np.argmin(planned[indices]))]
                effective_scale=self.scale*(args.warmup_scale if not self.seen_finish else 1.)
                reference = float(planned[reference_index])*effective_scale
                if args.max_speed>0:reference = min(reference, args.max_speed)
                if error > .15: reference *= max(.35, 1-1.6*(error-.15))
                requested_acceleration = clip((reference-self.reference)/dt, -args.braking, args.acceleration)
                self.reference += requested_acceleration*dt
                # Never let a positive map-profile prior fight the controller's
                # explicit off-path/heading recovery or a falling speed target.
                if args.acceleration_source=='profile':
                    profile_acceleration=shape_acceleration_prior(acceleration_prior[reference_index],effective_scale,
                        self.speed,reference,steering,error,wrap(heading[self.index]-angle),requested_acceleration,args)
                    self.feedforward_acceleration+=(profile_acceleration-self.feedforward_acceleration)*min(1.,dt/.035)
                else:self.feedforward_acceleration=requested_acceleration
                velocity_error = self.reference-self.speed
                self.integral = clip(self.integral+velocity_error*dt, -1.5, 1.5)
                throttle = (args.motor_feedforward*self.reference+args.speed_gain*velocity_error
                            +args.integral_gain*self.integral+args.accel_feedforward*self.feedforward_acceleration)
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
                self.feedforward_acceleration=0.
            if self.resetting:
                self.reset_pub.publish(Bool(data=now < self.reset_until))
                throttle=steering=reference=0.
                if now > self.reset_until+.4: self.resetting=False
            if self.stopped:
                throttle=reference=0.;steering=self.stopping_steering()
            self.publish(throttle,steering,reference)
            if now-self.last_log >= .05:
                row=dict(t=now-self.started,world_map_x=self.pose[0] if self.pose else None,
                    world_map_y=self.pose[1] if self.pose else None,speed=self.speed,reference=reference,
                    steering=steering,throttle=throttle,path_error=error,scale=self.scale,
                    collisions=self.total_collisions,simulator_laps=self.count,armed=self.armed,fresh=bool(fresh),
                    pose_age_s=now-self.pose_time,scan_age_s=now-self.scan_time,yaw_rate=self.yaw_rate,
                    forward_motion_confirmed=self.forward)
                row.update(path_index=position_index,map_yaw=self.pose[2] if self.pose else None,
                           unscaled_path_speed_mps=float(planned[position_index]),
                           effective_speed_scale=self.scale*(args.warmup_scale if not self.seen_finish else 1.),
                           profile_acceleration_mps2=profile_acceleration,target_derivative_mps2=requested_acceleration,
                           feedforward_acceleration_mps2=self.feedforward_acceleration)
                if self.writer is None: self.writer=csv.DictWriter(self.telemetry,fieldnames=list(row));self.writer.writeheader()
                self.writer.writerow(row);self.telemetry.flush();self.last_log=now
            if now-self.last_status >= .5:
                status=dict(running=not self.stopped,armed=self.armed,fresh=bool(fresh),simulator_laps=self.count,
                    total_collisions=self.total_collisions,best_clean_lap_s=self.best,scale=self.scale,
                    valid_laps=sum(r['valid_clean_full_lap'] for r in self.records),reason=self.reason,
                    crash_stop_enabled=args.stop_on_crash,simulator_pose_assisted=True,
                    safety_policy='SIMULATOR_DIRECT_BENCHMARK_NO_PROXIMITY_GOVERNOR',
                    imposed_command_speed_cap_mps=args.max_speed if args.max_speed else None,
                    line_planner=args.line_planner,
                    acceleration_source=args.acceleration_source,
                    pursuit_origin=args.pursuit_origin,
                    warmup_active=not self.seen_finish,
                    best_pace_median_s=self.best_median,best_pace_median_scale=self.best_median_scale,
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
            racer.publish(0.,racer.stopping_steering(),0.);rclpy.spin_once(racer,timeout_sec=.02)
            if racer.speed<.03 and time.monotonic()-racer.pose_time<.3 and time.monotonic()>end-4:break
        result=dict(reason=racer.reason,best_clean_lap_s=racer.best,valid_clean_laps=[r for r in racer.records if r['valid_clean_full_lap']],
                    rejected_laps=[r for r in racer.records if not r['valid_clean_full_lap']],total_collisions=racer.total_collisions,
                    stationary_verified=racer.speed<.03 and time.monotonic()-racer.pose_time<.3,
                    crash_stop_enabled=args.stop_on_crash,simulator_pose_assisted=True,
                    safety_policy='SIMULATOR_DIRECT_BENCHMARK_NO_PROXIMITY_GOVERNOR',
                    imposed_command_speed_cap_mps=args.max_speed if args.max_speed else None,
                    line_planner=args.line_planner,
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
  $client=New-Object OneFileWindow+RECT;$r=New-Object OneFileWindow+RECT;$origin=New-Object OneFileWindow+POINT
  $geometry=$h -and [OneFileWindow]::GetClientRect($h,[ref]$client) -and [OneFileWindow]::GetWindowRect($h,[ref]$r) -and [OneFileWindow]::ClientToScreen($h,[ref]$origin)
  $dx=$origin.X-$r.Left;$dy=$origin.Y-$r.Top
  $geometry=$geometry -and $client.Right -eq 1280 -and $client.Bottom -eq 720 -and $dx -ge 0 -and $dy -ge 0 -and ($dx+1280) -le ($r.Right-$r.Left) -and ($dy+720) -le ($r.Bottom-$r.Top)
  if($geometry){break}
  Start-Sleep -Milliseconds 200
 } while ((Get-Date) -lt $deadline)
 if(!$geometry){throw 'Simulator window geometry is not stable/visible; no click performed'}
 $script:ox=$origin.X-$r.Left;$script:oy=$origin.Y-$r.Top
 $im=New-Object Drawing.Bitmap ($r.Right-$r.Left),($r.Bottom-$r.Top)
 $g=[Drawing.Graphics]::FromImage($im);$dc=$g.GetHdc()
 try{if(![OneFileWindow]::PrintWindow($h,$dc,2)){throw 'Window capture failed'}}finally{$g.ReleaseHdc($dc);$g.Dispose()}
 return $im
}
function White($im,$x,$y){
 $px=$script:ox+$x;$py=$script:oy+$y
 if($px -lt 0 -or $py -lt 0 -or $px -ge $im.Width -or $py -ge $im.Height){return $false}
 $v=$im.GetPixel($px,$py);return $v.R-gt220-and$v.G-gt220-and$v.B-gt220
}
function Scene($im){
 if(($script:ox+300) -ge $im.Width -or ($script:oy+30) -ge $im.Height -or $script:ox -lt 0 -or $script:oy -lt 0){return $false}
 $background=$im.GetPixel($script:ox+300,$script:oy+30)
 return ((White $im 50 30) -or (White $im 1225 30)) -and $background.R -lt 150 -and $background.G -lt 150 -and $background.B -lt 150
}
function Menu($im){return (Scene $im)-and(White $im 219 113)-and(White $im 219 131)-and(White $im 32 113)}
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
do{$im=Capture;$ready=Scene $im;if($ready){break};$im.Dispose();Start-Sleep -Milliseconds 300}while((Get-Date)-lt$deadline)
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
    def scene(im):return (white(im,50,30)or white(im,1225,30))and max(im.getpixel((300,30)))<150
    def menu(im):return scene(im)and all(white(im,x,y)for x,y in[(219,113),(219,131),(32,113)])
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
    while not scene(image)and time.monotonic()<deadline:time.sleep(.2);image=capture()
    assert scene(image),'Scene not ready'
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
                    'clearance','run_seconds','stop_after_laps','tracker','line_planner','total_grip','motor_knee','footprint_margin','line_smoothness','line_search_validation'):
            drive += ['--'+name.replace('_','-'),str(getattr(args,name))]
        drive+=['--tune-hold-laps',str(args.tune_hold_laps)]
        drive+=['--acceleration-source',args.acceleration_source]
        drive+=['--warmup-scale',str(args.warmup_scale)]
        drive+=['--pursuit-origin',args.pursuit_origin]
        drive+=['--steering-response-gain',str(args.steering_response_gain)]
        drive+=['--motion-prediction',args.motion_prediction]
        for name in('lookahead_base','lookahead_speed','lookahead_min','lookahead_max','curvature_lookahead',
                    'motor_feedforward','accel_feedforward','integral_gain'):
            drive+=['--'+name.replace('_','-'),str(getattr(args,name))]
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
        if result is not None and not result.get('stationary_verified'):
            # Never leave an unverified rolling vehicle merely because the
            # controller process ended. Close only this launcher's simulator.
            if args.platform=='windows'and windows_pid:
                current=read_json(state_path)or{}
                ps(r'''
$ErrorActionPreference='Stop'
$p=Get-Process | Where-Object { $_.Id -eq $c.pid -and $_.Path -eq $c.exe }
if($p){$p.CloseMainWindow()|Out-Null;if(!$p.WaitForExit(2000)){Stop-Process -Id $p.Id -Force}}
exit 0
''',dict(pid=windows_pid,exe=current.get('windows_exe','')))
            elif args.platform=='native'and owned(sim):
                command(['docker','stop','--time','3',sim])
            atomic_json(session/'unverified-stop-simulator-closure.json',dict(
                reason='stationary telemetry not verified after zero-propulsion stop',
                result_remains_unverified=True,simulator_closure_requested=True))
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
    assert choose_pace(.95,{.95:[6.]},.04,None,1.3)[0]==.95
    # A single lucky fastest lap does not outweigh a repeated slower median.
    selected=choose_pace(1.,{.95:[6.1,6.2,6.15],1.:[5.7,6.7,6.8]},.04,None,1.3)
    assert selected[0]==.95 and abs(selected[3]-6.15)<1e-6
    selected=choose_pace(.975,{.975:[5.9,6.,5.95]},.02,.90,1.3)
    assert selected[0]>.975 and selected[2] is None,'Stale failure bracket trapped the observed good pace'
    parameters=argparse.Namespace(acceleration=9.,braking=12.,total_grip=12.,motor_knee=7.)
    assert shape_acceleration_prior(3.,1.,4.,4.,0.,0.,0.,0.,parameters)>0
    assert shape_acceleration_prior(3.,1.,4.,4.,0.,.3,0.,-8.,parameters)<=-8.
    assert shape_acceleration_prior(3.,1.,4.,4.,0.,0.,0.,-9.,parameters)<=-9.
    assert shape_acceleration_prior(3.,1.,8.,8.,.5,0.,0.,0.,parameters)<=0
    predicted=predict_vehicle_pose((0.,0.,0.),4.,0.,0.,.1)
    assert abs(predicted[0]-.4)<1e-9 and abs(predicted[1])<1e-9
    predicted=predict_vehicle_pose((0.,0.,0.),4.,2.,0.,.1)
    assert abs(predicted[0]-2*math.sin(.2))<1e-9 and abs(predicted[1]-2*(1-math.cos(.2)))<1e-9
    print('PASS: collision/teleport/transition/warmup/shortcut/invalid-clock laps rejected; stale/corrupt/wrong-owner leases rejected. Synthetic unit fixtures only. No Docker, simulator or driving started.')


def planner_self_test():
    global np
    import numpy as np
    parameters=argparse.Namespace(max_speed=0.,lateral=12.,total_grip=14.,acceleration=12.,braking=16.,steering_rate=8.,motor_knee=7.)
    angle=np.arange(240)*2*math.pi/240
    circle=np.column_stack((9*np.cos(angle),9*np.sin(angle)))
    speed,curve,heading,ds=vehicle_speed_profile(circle,parameters)
    assert np.isfinite(speed).all() and float(speed.min())>6.,'Uncapped planner has an unintended 6 m/s ceiling'
    assert float(np.max(speed*speed*abs(curve)))<=parameters.lateral+1e-5
    parameters.max_speed=2.
    capped,_,_,_=vehicle_speed_profile(circle,parameters)
    assert float(capped.max())<=2.+1e-8
    parameters.max_speed=0.
    oval=np.column_stack((5*np.cos(angle),2*np.sin(angle)))
    speed,curve,heading,ds=vehicle_speed_profile(oval,parameters)
    acceleration=(np.roll(speed,-1)**2-speed**2)/(2*ds)
    assert float(acceleration.max())<=parameters.acceleration+1e-5
    assert float(acceleration.min())>=-parameters.braking-1e-5
    assert float(np.max(speed*speed*abs(curve)))<=parameters.lateral+1e-5
    print('PASS: uncapped/capped cyclic planning, finite speeds, lateral and acceleration/braking bounds including wrap-around. Synthetic geometry only; not track evidence.')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ros-drive',action='store_true');parser.add_argument('--ros-watchdog',action='store_true')
    parser.add_argument('--owner-pid',type=int,default=0)
    parser.add_argument('--ros-stop',action='store_true');parser.add_argument('--native-gui',action='store_true')
    parser.add_argument('--gpu-check',action='store_true');parser.add_argument('--start-unity',action='store_true')
    parser.add_argument('--no-browser',action='store_true')
    parser.add_argument('--install',action='store_true')
    parser.add_argument('--self-test',action='store_true')
    parser.add_argument('--planner-self-test',action='store_true')
    parser.add_argument('--plan-only',action='store_true',help='Export offline planning estimates only; never launch/drive a simulator')
    parser.add_argument('--memory',type=Path);parser.add_argument('--output',type=Path)
    parser.add_argument('--token',default='onefile')
    parser.add_argument('--max-speed',type=float,default=0.,help='Imposed command cap in m/s; 0 disables that ceiling (simulation only)')
    parser.add_argument('--lateral',type=float,default=10.)
    parser.add_argument('--acceleration',type=float,default=9.);parser.add_argument('--braking',type=float,default=12.)
    parser.add_argument('--scale',type=float,default=.95);parser.add_argument('--maximum-scale',type=float,default=1.3)
    parser.add_argument('--warmup-scale',type=float,default=1.,help='Optional first unscored lap pace multiplier; subsequent change lap also excluded')
    parser.add_argument('--fixed-scale',action='store_true');parser.add_argument('--stop-on-crash',action='store_true')
    parser.add_argument('--tune-hold-laps',type=int,default=3,help='Eligible clean laps held at a pace before median-based tuning')
    parser.add_argument('--no-active-brake',action='store_true');parser.add_argument('--brake-cap',type=float,default=.35)
    parser.add_argument('--preview',type=float,default=.04);parser.add_argument('--brake-preview',type=float,default=.10)
    parser.add_argument('--heading-gain',type=float,default=8.);parser.add_argument('--cross-gain',type=float,default=12.)
    parser.add_argument('--speed-gain',type=float,default=.18);parser.add_argument('--steering-rate',type=float,default=8.)
    parser.add_argument('--throttle-rate',type=float,default=3.)
    parser.add_argument('--lookahead-base',type=float,default=.55);parser.add_argument('--lookahead-speed',type=float,default=.14)
    parser.add_argument('--lookahead-min',type=float,default=.65);parser.add_argument('--lookahead-max',type=float,default=1.55)
    parser.add_argument('--curvature-lookahead',type=float,default=1.)
    parser.add_argument('--motor-feedforward',type=float,default=.04)
    parser.add_argument('--accel-feedforward',type=float,default=.035)
    parser.add_argument('--acceleration-source',choices=['command','profile'],default='command')
    parser.add_argument('--pursuit-origin',choices=['measured','predicted'],default='measured')
    parser.add_argument('--motion-prediction',choices=['linear','arc'],default='linear')
    parser.add_argument('--steering-response-gain',type=float,default=1.,help='Measured curvature response divided by commanded bicycle curvature')
    parser.add_argument('--integral-gain',type=float,default=.04)
    parser.add_argument('--tracker',choices=['feedback','pursuit'],default='pursuit')
    parser.add_argument('--refine-sweeps',type=int,default=9);parser.add_argument('--line-offset',type=float,default=.65)
    parser.add_argument('--clearance',type=float,default=.34)
    parser.add_argument('--line-planner',choices=['joint','legacy'],default='joint')
    parser.add_argument('--line-smoothness',type=float,default=.003,help='Curvature-rate penalty in joint line optimisation')
    parser.add_argument('--line-search-validation',choices=['retain-feasible','strict'],default='retain-feasible',help='Preserve the original validated final search result, retaining a dense feasible candidate if it fails; strict rejects each infeasible dense search move')
    parser.add_argument('--total-grip',type=float,default=12.)
    parser.add_argument('--motor-knee',type=float,default=7.)
    parser.add_argument('--footprint-margin',type=float,default=.06)
    parser.add_argument('--run-seconds',type=float,default=0);parser.add_argument('--stop-after-laps',type=int,default=0)
    parser.add_argument('--platform',choices=['windows','native'],default='native')
    parser.add_argument('--root',type=Path,default=Path(__file__).resolve().parent)
    parser.add_argument('--stop',action='store_true')
    args=parser.parse_args()
    if args.self_test:return self_test()
    if args.planner_self_test:return planner_self_test()
    if args.install:return install_shortcuts(args)
    for key in ('max_speed','lateral','acceleration','braking','scale','maximum_scale','preview','brake_preview','heading_gain','cross_gain','speed_gain','steering_rate','throttle_rate','brake_cap','line_offset','clearance','run_seconds','total_grip','motor_knee','footprint_margin','lookahead_base','lookahead_speed','lookahead_min','lookahead_max','curvature_lookahead','motor_feedforward','accel_feedforward','integral_gain'):
        value=getattr(args,key)
        if not math.isfinite(value) or value<0:parser.error('Invalid '+key)
    if not 0<=args.max_speed or not 0<args.lateral<=40 or not 0<args.scale<=args.maximum_scale<=2:parser.error('Invalid benchmark envelope')
    if not 0<args.acceleration<=40 or not 0<args.braking<=40 or not 0<=args.brake_cap<=1:parser.error('Invalid actuator/planner parameters')
    if args.motor_knee<=0 or args.total_grip<args.lateral or args.refine_sweeps<0:parser.error('Invalid joint-profile model')
    if args.tune_hold_laps<3:parser.error('Pace tuning requires at least three eligible clean laps')
    if not math.isfinite(args.warmup_scale)or not 0<args.warmup_scale<=1:parser.error('Invalid warmup pace')
    if not math.isfinite(args.steering_response_gain)or args.steering_response_gain<=0:parser.error('Invalid steering-response model')
    if not math.isfinite(args.line_smoothness)or args.line_smoothness<0:parser.error('Invalid line smoothness')
    if not 0<args.lookahead_min<=args.lookahead_max:parser.error('Invalid lookahead geometry')
    if args.plan_only:
        if args.memory is None or args.output is None:parser.error('--plan-only requires --memory and --output')
        global np
        import numpy as np
        args.output.mkdir(parents=True,exist_ok=False)
        points,speed,curvature,heading,ds,anchor,proof=load_memory(args.memory,args)
        export_speed_profile(args.output,points,speed,curvature,heading,ds)
        atomic_json(args.output/'memory_provenance.json',proof)
        atomic_json(args.output/'planning_configuration.json',vars_to_json(args))
        print(json.dumps(proof,indent=2));return
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
