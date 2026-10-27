FROM ros:humble-ros-base-jammy@sha256:860e53793ccf575e5369b5e9a19939de2bfcfb46a299954d06f54f2af54fed19

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive PIP_DISABLE_PIP_VERSION_CHECK=1 \
    OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 NUMBA_NUM_THREADS=1
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip python3-dev xvfb xauth libgl1-mesa-glx libglu1-mesa \
    libxrandr2 libxi6 ros-humble-ackermann-msgs ros-humble-tf2-ros \
    && rm -rf /var/lib/apt/lists/*
COPY requirements.txt /opt/f1tenth-requirements.txt
RUN python3 -m pip install --no-cache-dir -r /opt/f1tenth-requirements.txt
COPY vendor/f1tenth_gym/gym/ /opt/f1tenth_gym/gym/
COPY vendor/f1tenth_gym/LICENSE /opt/licenses/f1tenth_gym-LICENSE
COPY vendor/f1tenth_gym_ros/f1tenth_gym_ros/ /opt/f1tenth_gym_ros/f1tenth_gym_ros/
COPY vendor/f1tenth_gym_ros/LICENSE /opt/licenses/f1tenth_gym_ros-LICENSE
COPY vendor/f1tenth_racetracks/ /opt/racetracks/
COPY SOURCE_LOCK.json /opt/SOURCE_LOCK.json
ENV PYTHONPATH=/opt/f1tenth_gym/gym:/opt/f1tenth_gym_ros \
    ROS_DOMAIN_ID=77
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-tf2 ros-humble-tf2-py ros-humble-tf2-msgs \
    ros-humble-tf2-ros ros-humble-tf2-ros-py ros-humble-tf2-geometry-msgs \
    ros-humble-tf2-eigen ros-humble-tf2-kdl ros-humble-tf2-sensor-msgs ros-humble-tf2-tools \
    && rm -rf /var/lib/apt/lists/*
# Warm genuine simulator/JIT code without driving or changing the upstream
# physics. The Gym wrapper is removed solely for the legacy positional-reset API.
RUN xvfb-run -a python3 - <<'PY'
import gym, f110_gym, numpy as np
env = gym.make('f110-v0', map='/opt/racetracks/Melbourne/Melbourne_map',
               map_ext='.png', num_agents=1, lidar_dist=.275).unwrapped
line = np.loadtxt('/opt/racetracks/Melbourne/Melbourne_raceline.csv', delimiter=';', comments='#')
pose = np.array([[line[0, 1], line[0, 2], line[0, 3]]])
obs, _, _, _ = env.reset(pose)
for _ in range(3):
    obs, _, _, _ = env.step(np.array([[0., 0.]]))
assert len(obs['scans'][0]) == 1080 and not obs['collisions'][0]
print('GENUINE PHYSICS IMPORT/JIT SMOKE TEST: stationary, 1080 simulated beams, no collision.')
PY
ENTRYPOINT ["/bin/bash", "-lc", "source /opt/ros/humble/setup.bash; exec \"$@\"", "f1tenth"]
