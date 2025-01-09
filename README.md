# NMPC PX4 ROS2 Trajectory Tracking Controller


![Status](https://img.shields.io/badge/Status-Experimental-orange)
[![ROS 2 Compatible](https://img.shields.io/badge/ROS%202-Humble-blue)](https://docs.ros.org/en/humble/index.html)
[![PX4 Compatible](https://img.shields.io/badge/PX4-Main-pink)](https://github.com/PX4/PX4-Autopilot)
[![Solver: ACADOS](https://img.shields.io/badge/Solver-ACADOS-brightgreen)](https://github.com/acados/acados)

This ROS2 package implements a nonlinear model predictive control (NMPC) pipeline for trajectory tracking of aerial vehicles running PX4 Autopilot. It uses ACADOS to define and solve the optimal control problem (OCP).

The control problem is set up in Python, where system dynamics and constraints are defined. ACADOS generates C code for the solver, which is used in a ROS2 C++ node to compute optimal control inputs based on the drone’s current state and reference trajectory.

The package leverages the PX4 ROS2 Interface library developed by [Auterion](https://github.com/Auterion/px4-ros2-interface-lib), which simplifies communication with PX4 and eliminates the need to directly handle uORB topics. This abstraction allows for easier integration and provides better integration with PX4’s failsafe and arming checks.

This package is designed for developers and researchers working on UAV control, offering a streamlined and powerful solution for integrating NMPC with PX4.

> **⚠ WARNING**
> 
> This package is experimental and intended for research and development purposes only. Users are advised to use this package at their own risk. Ensure thorough testing in a controlled environment before deploying on hardware.

## Features
1. **System Dynamics and Solver Setup**  
   - Utility script to define system dynamics, constraints, and solver options for NMPC.  
   - Automatically generates C code for the solver using ACADOS.  

2. **Trajectory Generation**  
   - Utility script to generate common trajectories (circles, helix, figure-eight, etc).  

3. **Trajectory Tracking Node**  
   - ROS 2 node that utilizes the generated C solver to control the drone through direct actuator inputs through the PX4 ROS2 Interface Library.  

4. **Odometry Republisher Node**  
   - Converts PX4's odometry (provided as a uORB topic) to a standard ROS 2 odometry message for visualization in Rviz.  

5. **External State Estimation Integration**  
   - Node to subscribe to external state estimation topics (motion capture system, visual inertial odometry, lidar inertial odometry, etc)
   - Fuses external state estimation with PX4's state estimation using the navigation interface of the PX4 ROS 2 Interface Library.  

6. **Utility Library**  
   - Functions to convert between NED (North-East-Down) and ENU (East-North-Up) coordinate systems.  
   - Functions to transform between forward-right-down (FRD) and forward-left-up (FLU) body frames.  

## Requirements
This package has been tested with ROS2 Humble on Ubuntu 22.04. Ensure your system meets the following requirements:  
  
1. **ROS2 Humble:** [https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
2. **PX4 Autopilot Source:** [https://docs.px4.io/main/en/dev_setup/building_px4.html](https://docs.px4.io/main/en/dev_setup/building_px4.html) (*Note:* In theory, you can skip installing PX4 source code if you only plan to perform hardware tests on a PX4-compatible board that is flashed with the latest released firmware. However, it is highly discouraged to test directly on hardware without first conducting SITL tests)
3. **ROS2 PX4 Setup:** [https://docs.px4.io/main/en/ros2/user_guide.html](https://docs.px4.io/main/en/ros2/user_guide.html)
4. **ACADOS:** (*Note:* Installation instructions for ACADOS are provided below.
5. **QGroundControl Daily Build:** [https://docs.qgroundcontrol.com/master/en/qgc-dev-guide/getting_started/index.html](https://docs.qgroundcontrol.com/master/en/qgc-dev-guide/getting_started/index.html)

## Install
To install this package follow the instructions below:

1. Make a new ros2 workspace or navigate to your existing workspace
```bash
mkdir -p ~/nmpc_px4_ros2_ws/src
cd nmpc_px4_ros2_ws/src
```
2. Clone package and submodules
> **⚠ WARNING**
> 
> The following commands will clone the submodules as well. If your workspace already contains the *px4_msgs* package or the *px4-ros2-interface-lib* this will cause conflicts. This will also clone ACADOS. Modify the [.gitmodules](https://github.com/kousheekc/nmpc_px4_ros2/blob/main/.gitmodules) file, to exclude packages you may already have.
```bash
git clone https://github.com/kousheekc/nmpc_px4_ros2.git
cd nmpc_px4_ros2
git submodule update --init --recursive
```
3. Install ACADOS
```bash
cd 3rd_party/acados
git submodule update --init --recursive
mkdir -p build
cd build
cmake -DACADOS_WITH_QPOASES=ON ..
make install -j4
```
4. Install ACADOS Python Interface
```bash
cd ..
virtualenv env --python=/usr/bin/python3
source env/bin/activate
pip install -e interfaces/acados_template
```
5. Update LD_LIBRARY_PATH (replace the paths if you have installed acados elsewhere)
```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"~/nmpc_px4_ros2_ws/src/nmpc_px4_ros2/3rd_party/acados/lib"
export ACADOS_SOURCE_DIR="~/nmpc_px4_ros2_ws/src/nmpc_px4_ros2/3rd_party/acados"
```

## Usage

## License
This project is licensed under the BSD 3-Clause License - see the [LICENSE](https://github.com/kousheekc/nmpc_px4_ros2/blob/main/LICENSE) file for details.

## Contact
Kousheek Chakraborty - kousheekc@gmail.com

Project Link: [https://github.com/kousheekc/nmpc_px4_ros2](https://github.com/kousheekc/nmpc_px4_ros2)

If you encounter any difficulties, feel free to reach out through the Issues section. If you find any bugs or have improvements to suggest, don't hesitate to make a pull request.
