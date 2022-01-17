# Acoustics ROS

A ROS1 package for acoustics simulation. Helpful for testing acoustic localization algorithms. A work-in-progress for now.

## Installation

### System

* WSL2 with Ubuntu-20.04
* Python 3.8 (3.6+ should work)
* ROS Noetic

### Instructions

1. This package requires catkin build tools, not catkin_make. Install from [here](https://catkin-tools.readthedocs.io/en/latest/installing.html).

2. Create a catkin workspace.

	```
	mkdir -p ros_ws/src
	cd ros_ws
	catkin init
	```

3. Clone this package into src.

4. Install pra_utils from [here](https://github.com/enceladus2000/pra_utils).


## Build and Run Demo

1. Inside the workspace, build using `catkin build acoustics_ros` and then `source devel/setup.bash`.

2. Run the main demo with `roslaunch acoustics_ros basic_test.launch`.

3. You can also run teleop with `roslaunch acoustics_ros teleop.launch`.


