# Hummingbird Webots Simulation Package

## System Setup

### 1. Setup ROS2 workspace (only if you don't already have a ros2 workspace)

```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 2. Clone the repositories

```
git clone https://github.com/Olin-HAIR-Lab/hbird_webots.git
```


In the hbird_webots package, the files of importance are:

_hbird_webots folder_

- scripts folder for any supplementary python scripts that are not nodes
- chbird_sim_node.py file which is the ROS node that receives position setpoints
 from the agent_control_node and executes them on the drones in Webots

_launch folder_

- demo.launch.py, the launch file for the flight arena using one HB vehicle

_protos folder_

- meshes folder, holds all .obj files for each CAD (racks, dropoff bins, and
  takeoff pads) along with their respective.mtl files, and the STL_files folder
  holds the STL files for the base body of the drone and the propeller
- textures folder, holds the .jpg images for the colors of the rack bins and the
  image displayed in Webots when the propellers move quickly
- Mark2Assembly.proto is the proto file for the Mark 2 drone

_resource folder_

- hbird_drone.urdf is the URDF file that the webots_ros_driver ROS node uses to
  connect the controller and sensor plugins declared in the URDF to the robot in
  Webots

_worlds folder_

- flight_arena.wbt, the Webots world file that contains a simple, empty flight arena
