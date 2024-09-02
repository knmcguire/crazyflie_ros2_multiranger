# crazyflie_ros2_multiranger
This repository contains different ROS 2 nodes to interact with the multiranger on the Crazyflie for both simulation as the real Crazyflie.

## Installation

Start a workspace and clone the repository:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone git@github.com:knmcguire/crazyflie_ros2_multiranger.git
```

In the same workspace, git clone Crazyswarm2:
```bash

git clone https://github.com/IMRCLab/crazyswarm2 --recursive
```

There is a fix that needs to be merged so it's best to use the odom-tf-fix. For the rest make sure to [install all of Crazyswarm2's dependencies](https://imrclab.github.io/crazyswarm2/installation.html) but skip buildin

In the same workspace also install the ros_gz_crazyflie usage instructions in [the README of the ros_gz_crazyflie repository](https://github.com/knmcguire/ros_gz_crazyflie?tab=readme-ov-file#usage) for sourcing the specific Crazyflie model.
> Make sure to switch to the gazebo-multiranger branch when cloning the [crazyflie-simulation](https://github.com/bitcraze/crazyflie-simulation) repo!

Then build the workspace:
```bash
cd  ~/ros2_ws/
colcon build --cmake-args -DBUILD_TESTING=ON
source ~/ros2_ws/install/setup.bash
```

## Usage

Every terminal were you run the examples in needs to have the setup.bash sourced with:
```bash
source ~/ros2_ws/install/setup.bash
```

Also the simulation model needs to be sourced in every terminal where you run the simulation with:
```bash
export GZ_SIM_RESOURCE_PATH=/home/user/simulation_models/crazyflie_simulation/simulator_files/gazebo/"
```
> Note that the full directory should be sourced as tilde won't be recognized.

The latter is depended on where you have the simulation models stored.

> You might need to enter the full path for sourcing, so with `/home/USER/...` if you get a 'file not found' error.

### Simulated Crazyflie with simple mapper and teleop

To run the simulation with the simple mapper while controlling in with teleop run:

```bash
ros2 launch crazyflie_ros2_multiranger_bringup simple_mapper_simulation.launch.py
```

In another terminal run teleop with
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Have the crazyflie take off by pressing 't' and follow the instructions of the teleop_twist_keyboard to control the Crazyflie. You can see the map being created in Rviz.

### Real Crazyflie with simple mapper and teleop

Go to crazyflie_ros2_multiranger_bringup/confg  and edit the crazyflie_real_crazyswarm2.yaml file to set the uri of the Crazyflie to the correct one.

Then run:
```bash
ros2 launch crazyflie_ros2_multiranger_bringup simple_mapper_real.launch.py
```

In another terminal run teleop with
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Have the crazyflie take off by pressing 't' and follow the instructions of the teleop_twist_keyboard to control the Crazyflie. You can see the map being created in Rviz.

### Simulated Crazyflie with simple mapper and wall following

To run the simulation with the simple mapper while it is autonomously wall following run:

```bash
ros2 launch crazyflie_ros2_multiranger_bringup wall_follower_mapper_simulation.launch.py
```

You don't have to control the  simulated Crazyflie as it is autonmously wallfollowing. You can see the map being created in Rviz.

You can make the simulated Crazyflie stop with calling the following service:
```bash
ros2 service call /crazyflie/stop_wall_following std_srvs/srv/Trigger
```

It will now stop moving and land.

### Real Crazyflie with simple mapper and wall following

Go to crazyflie_ros2_multiranger_bringup/config/  and edit the crazyflie_real_crazyswarm2.yaml file to set the uri of the Crazyflie to the correct one.

Then run:
```bash
ros2 launch crazyflie_ros2_multiranger_bringup wall_follower_mapper_real.launch.py
```

You don't have to control the  simulated Crazyflie as it is autonomously wall following. You can see the map being created in Rviz.

You can make the simulated Crazyflie stop with calling the following service:
```bash
ros2 service call /crazyflie_real/stop_wall_following std_srvs/srv/Trigger
```

The crazyflie will now stop moving and land.
