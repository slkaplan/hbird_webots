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

### 3. Install Webots and webots_ros2

Follow the instructions
[here](https://cyberbotics.com/doc/guide/installation-procedure#installation-on-linux)

Follow the instructions
[here](https://docs.ros.org/en/iron/Tutorials/Advanced/Simulators/Webots/Installation-Ubuntu.html)
for webots_ros2 (make sure to use the installation from sources approach)

In the hbird_webots package, the files of importance are:

_hbird_webots folder_

- scripts folder for any supplementary python scripts that are not nodes
- crazyflie_sim_node.py file which is the ROS node that receives velocity
  commands from the agent_control_node and executes them on the drones in Webots

_launch folder_

- sim_demo_LPB.launch.py, the launch file for the LPB simulation with 2 drones
  (uses the LPB.wbt Webots world file)
- sim_demo_warehouse_one.launch.py, the launch file for the warehouse simulation
  with one drone, for testing (uses the warehouse_one.wbt world file)
- sim_demo_warehouse_five.launch.py, the launch file for the warehouse
  simulation with five drones, for testing (uses the warehouse_five.wbt world
  file)
- sim_demo_warehouse_ten.launch.py, the launch file for the warehouse simulation
  with ten drones, for testing (uses the warehouse_ten.wbt world file)

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

- LBP.wbt, the Webots world file that contains the simulated Large Project
  Building, the LPB
- warehouse_one.wbt, the Webots world file that contains the 10,000 sq ft.
  section of warehouse, this world has only one drone, for testing
- warehouse_five.wbt, like the warehouse_one file, but with five drones
- warehouse_ten.wbt, like the warehouse_one file, but with ten drones
