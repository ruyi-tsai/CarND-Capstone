## introduction
Carla use waypoint navigation to drive autonomously while avoiding obstacles and stopping at traffic lights. Waypoints are ordered set of coordinated that Carla uses to plan a path. Each waypoint has a associated target velocity which depends on the desired vehicle behavior. There are 3 modules implementation for this project:
### Perception:

a. Traffic light detection

b. Obstacle detection

### Planning:

Waypoint Updater (Set the velocity for each waypoint)

### Contol:

Controls the car’s throttle, steering, and brake using Dataspeed Drive by wire (DBW) ROS node.
### Code Structure
The following is a system architecture diagram showing the ROS nodes and topics used in the project.
image:

1. This package contains the traffic light detection node: tl_detector.py 
2. This package contains the waypoint updater node: waypoint_updater.py
3.  Carla is equipped with a drive-by-wire (dbw) system, meaning the throttle, brake, and steering have electronic control. This package contains the files that are responsible for control of the vehicle: the node dbw_node.py and the file twist_controller.py, along with a pid and lowpass filter that you can use in your implementation. 




### Usage

1. Clone the project repository
```bash
git clone https://github.com/ruyi-tsai/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator
