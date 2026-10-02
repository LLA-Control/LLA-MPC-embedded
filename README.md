# LLA-MPC-onboard

See our website [here](https://lla-control.github.io/) for videos and the relevant paper.

LLA-MPC-onboard is an implementation of the LLA-MPC framework that is modularized to accept different dynamics, integrators, and model-based robotics tasks. The framework has been deployed on an F1Tenth with a Single-Track Bicycle Model integrated with Fiala Tire dynamics, and has been validated by both a standard Model Predictive Control task and a Safe Model Predictive Control task. 

## Prerequisites
If you want to deploy the full embedded system on an F1Tenth, you will need the following packages installed on it. Since this implementation has been written to be modular, so you can take individual pieces out of it and use them on their own.

To begin, clone this repo into the src folder of a ROS 2 repo like so:

```
mkdir lla_ws && cd lla_ws
git clone --recurse-submodules git@github.com:LLA-Control/LLA-MPC-onboard.git src
```

`f1tenth_gym`, `f1tenth_gym_ros`, and `natnet_ros2` are submodules pinned to the versions we tested with. If you cloned without `--recurse-submodules`, initialize them with:

```
cd src
git submodule update --init --recursive
```

To change the version of a submodule, check out the commit you want inside it and commit the new pin:

```
cd src/natnet_ros2
git fetch && git checkout <commit-or-tag>
cd .. && git add natnet_ros2 && git commit -m "bump natnet_ros2"
```

### Packages
To use the program as written, you will need to install:
- [acados](https://docs.acados.org/installation/index.html) for the MPC optimizer,
- [JAX](https://docs.jax.dev/en/latest/installation.html) for the parallel accelerated state optimization, and [cuda](https://docs.nvidia.com/jetson/orin-nano-devkit/user-guide/latest/setup_cuda.html) if you are interested in using gpu acceleration,
- [rangelibc](https://github.com/f1tenth/particle_filter/tree/humble-devel) for the particle filter, if you are using onboard state estimation,
- [natnet_ros2](https://github.com/L2S-lab/natnet_ros2) for streaming OptiTrack poses, if you are using motion capture. It is included as a submodule, but follow its README for the NatNet SDK requirements.

If you are running either directly on an embedded system like the Jetson Orin Nano that this framework was deployed on, you may run into issues with `tera_renderer` and `cuda` due to the ARM architecture. You will have to compile them directly on the Jetson.

### Important Notes
- **MPC** will require tuning based on the amount of compute you have available and your objective. The optimizer's version of the Fiala Tire Dynamic Bicycle Model (DBM) can be found in the `export_model` function of `nmpc_gen_fiala_fixed.py`.  The optimizer generation occurs in `create_ocp` in the same file. Options for optimization can be modified by modifying relevant acados optimization variables in `ocp.solver_options`.
- The **safe control optimization** lives in `nmpc_gen_fiala_cbf.py`.
- **LLA Bank creation and rollout structure** is defined in `history.py`.
- **Parallel integrators** used to roll out the models in the LLA Bank are implemented in `rk6.py`.
- **Dynamics** used to roll out models in the LLA Bank can be found in the `llampc/llampc/rollout` folder - the one used in paper is `dynamic_fiala_fixed.py`.
- An **experimental multi-step bank** that regresses ground truth state estimation against **action chunks** instead of action pairs exists in `multi_step_history.py`.
- The `f1tenth_system` package has modifications to enable current-based control. `ackermann_to_vesc.cpp` uses the `drive.jerk` field of the `AckermannDriveStamped` message as a mode gate: `jerk = 2.0` sends `drive.acceleration` to the VESC as motor current in amps (clamped to `current_command_min/max`), `jerk = 1.0` sends it as duty cycle (clamped to `duty_cycle_command_min/max`), and any other value falls back to the usual `drive.speed` control. The controllers use duty cycle with pure pursuit below 0.1 m/s, and switch to current once the MPC takes over.
- **Smoothed velocities** are needed for LLA, since noisy finite-differenced velocities make the bank's model selection jump around. Tune the filter with the `vel_*` parameters in `optitrack_node.py` (mocap) or `particle_filter.py` (onboard).


## Running the LLA-MPC framework

### Building
Build the workspace from its root with `colcon`. The `particle_filter` package will fail to build if `rangelibc` is not installed, so skip it if you are only using motion capture:

```
cd lla_ws
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install` lets you edit the scripts in `llampc/scripts` without rebuilding. You still have to rebuild if you add a new script, since the scripts are registered in `install(PROGRAMS ...)` in `llampc/CMakeLists.txt`. Remember to `source install/setup.bash` in every new terminal.

### State Estimation
The controllers subscribe to `/odometry/filtered`, which is published by a [robot_localization](https://docs.ros.org/en/humble/p/robot_localization/) EKF. There are two options for what feeds the EKF:

- **Onboard (`bringup_launch.py`)** fuses the particle filter pose on `/pf/pose/odom` with the VESC IMU and a zero-velocity update (ZUPT). Its configuration is `llampc/config/ekf.yaml`. This launch file starts the f1tenth stack, the particle filter, `imu_zupt_prep.py`, and the EKF. The particle filter needs a map of your track: add it to `particle_filter/maps` and set its name in `map` in `particle_filter/config/localize.yaml`.
- **Motion capture (`opti_launch.py`)** fuses OptiTrack poses with the VESC IMU and ZUPT. Its configuration is `llampc/config/mocap.yaml`. This launch file starts the f1tenth stack, the [natnet_ros2](https://github.com/L2S-lab/natnet_ros2) client, `imu_zupt_prep.py`, `optitrack_node.py`, and the EKF. Change `serverIP` and `clientIP` in `opti_launch.py` to the IP addresses of your Motive machine and your F1Tenth.

```
ros2 launch llampc bringup_launch.py
ros2 launch llampc opti_launch.py
```

In both configurations the EKF's body frame is `cg` rather than `base_link`. `/set_pose` is remapped to `/initialpose`, so the 2D Pose Estimate tool in RViz resets the EKF.

### OptiTrack Configuration
`optitrack_node.py` converts the NatNet rigid-body pose into the `/optitrack/odom` message that the EKF reads. It also broadcasts a `map` -> `base_link` transform. Most of the work in setting up a new mocap space is in this node:

- **Topic.** The node subscribes to the `mocap_topic` parameter, which defaults to `/f1tenth/pose`. You can set it with `ros2 launch llampc opti_launch.py mocap_topic:=/<name>/pose`. `natnet_ros2` publishes each rigid body on `/<name>/pose`, so `f1tenth` has to match the rigid body's name in Motive.
- **Axis conversion.** Motive streams Y-up coordinates, and ROS uses Z-up. The position is remapped to `(-x, z, y)`, and the orientation is rotated by `R_new = B @ P @ R_orig` in `mocap_callback`. `P` is the axis permutation, and `B` is a yaw offset of `theta = -pi/2` that was found empirically for our space. If the car's position is correct but its heading is off by 90 or 180 degrees, change `theta`. If the car moves in a mirrored direction, change the signs in `P` and in the position remapping together. The easiest way to check is to drive the car forward by hand and confirm in RViz that it moves along its own x-axis.
- **Velocities.** OptiTrack only provides pose, so the node computes velocities by finite difference, rotates them into the body frame, and filters `vx`, `vy`, and `omega` with a causal EMA and a windowed spike clamp. The filter is tuned with the `vel_ema_tau`, `vel_spike_pct`, and `vel_spike_window` parameters. Keep `vel_ema_tau` small, because any lag added here adds to the EKF's own lag.
- **Covariances.** The pose covariance is set low (`1e-6` for position, `1e-4` for orientation) because OptiTrack is accurate. The twist covariance is deliberately moderate because it is still a filtered finite difference. Tune these together with `odom0_config` in `mocap.yaml` if you want the EKF to rely more or less on the mocap velocities.

### Generating Reference Trajectories
Reference trajectories are generated in `llampc/llampc/utils/waypoint_interpolator/interpolator.ipynb`. Each cell in the notebook takes a list of hand-placed waypoints, fits a spline through them, and saves the result to `llampc/llampc/utils/tracks/<track_name>.npz`. The notebook saves to `../tracks` relative to its own folder, so open it from `waypoint_interpolator`. To make a new trajectory, copy one of the existing cells and change its arguments:

- **Waypoints.** `x` and `y` are given in the frame of your state estimate, which is the mocap frame for `opti_launch.py` and the particle filter map for `bringup_launch.py`. The first plot labels each waypoint with its index and coordinates, which makes it easy to move individual points.
- **`wrap`.** Set `wrap = True` for closed loops, and repeat the first waypoint at the end of the list. With `wrap = False`, the path is open and its speed ramps up over the first 20% and down over the last 10%, so the car starts and stops at rest.
- **`vel`.** The reference speed in m/s. The speed is constant along the path apart from the ramps.
- **`mus` (optional).** A list of friction coefficients for a speed bank. Each `mu` gets its own speed profile, scaled from `0.75 * vel` at `mu = 0` to `vel` at `mu = 1`. When `adaptive_planning` is enabled in the controller, the reference speed is interpolated between these profiles using the friction estimated by the LLA Bank. When it is disabled, the controller uses `vel`. Save bank trajectories with `save_track_bank`, and single-speed trajectories with `save_track`.

The spline is resampled to 500 points regardless of how many waypoints you give it. Rebuild with `colcon build` after adding a new trajectory so that it is copied into `install`, then pass its file name to the controller with `track_file_name`.

### Running the Controllers
Once state estimation is running, start one of the two controllers along with `drive_relay.py`. The controllers publish to `/mpc/drive`, and `drive_relay.py` forwards those commands to `/drive`. The relay also acts as a watchdog that brakes if no new commands arrive within `timeout_s`.

```
ros2 run llampc drive_relay.py
ros2 run llampc llampc_fiala_fixed.py
ros2 run llampc llampc_fiala_cbf.py
```

- **`llampc_fiala_fixed.py`** is the standard MPC tracking task. By default it runs a horizon of `N = 30` at 40 Hz with LLA enabled.
- **`llampc_fiala_cbf.py`** is the Safe MPC task. It uses the control barrier function solver in `nmpc_gen_fiala_cbf.py` and runs `N = 20` at 25 Hz by default. Obstacles are set in `self.obstacles` in `declare_params` as a list of `(x, y, radius)` tuples.

Both controllers take the following ROS parameters:
- `track_file_name`: the reference trajectory, an `.npz` file in `llampc/llampc/utils/tracks`. `mocap_*` tracks are meant for motion capture, and `blevel_*` tracks are meant for the particle filter map.
- `odom_topic`: the state estimate, which defaults to `/odometry/filtered`.
- `out_file`: the name of the run log, which is saved to `~/.ros/log/<out_file>_<timestamp>.npz`.

```
ros2 run llampc llampc_fiala_fixed.py --ros-args -p track_file_name:=mocap_square2fast.npz -p out_file:=square_run
```

Everything else, including the horizon, control rate, whether LLA is used (`with_lla`), the LLA window, and the parameter grid for the LLA Bank in `fiala_setup`, is set directly in each script. On startup the controller generates and compiles the acados solver into `llampc/llampc/solvers/default` and JIT-compiles the LLA Bank in JAX. This can take a while on the Jetson, so wait for `LLA BANK COMPILED` before driving. JAX is forced onto the CPU by `JAX_PLATFORM_NAME` at the top of each script, so change that line if you want to use the GPU.

## Running in Simulation
[f1tenth_gym](https://github.com/f1tenth/f1tenth_gym) and its ROS 2 bridge [f1tenth_gym_ros](https://github.com/f1tenth/f1tenth_gym_ros) are included as submodules. Follow their READMEs to install the gym and launch the bridge.

- **State estimate.** The bridge publishes the car's ground-truth odometry on `/ego_racecar/odom` (`ego_namespace` and `ego_odom_topic` in `f1tenth_gym_ros/config/sim.yaml`) instead of running an EKF, so pass `-p odom_topic:=/ego_racecar/odom` to the controller.
- **Map.** Set `map_path` in `sim.yaml` to a map in `particle_filter/maps` (e.g. `blevel2`, with `map_img_ext: '.pgm'`), and use a reference trajectory in that map's frame.
- **Control.** The simulator does not use current-based control. The bridge drives the car from `drive.speed` and `drive.steering_angle` on `/drive`, but the controllers send motor current in `drive.acceleration` with `drive.speed` set to 0. They also read wheel speed from `/sensors/core`, which the simulator does not publish. You will need to change `apply_control` to send a speed command to drive the simulated car.

---

This repository is developed by Henry Liao in collaboration with Maitham F. Al-Sunni. Please contact hzl@andrew.cmu.edu or maitham@cmu.edu if you have questions.
