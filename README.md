### Autonomous Mobile Robot Navigation

 For the Mater and slave configuration between laptop and Raspberry Pi Reffer the below link: 
1. https://github.com/rigbetellabs/tortoisebot/wiki/

## Installation

1. Create a Catkin workspace (if not already present)
  ```bash
  $ mkdir -p tortoise_ws/src
  ```
2. Change directory to the source space (`src`) of your Catkin workspace
  ```bash
  $ cd tortoise_ws/src
  ```
3. Clone this repository:
  ```bash
  $ git clone https://github.com/AkshayVJ/Autonomus_Mobile_Robot.git
  ```
4. Change directory back to the Catkin workspace:
  ```bash
  $ cd ..
  ```
5. Build the packages:
  ```bash
  $ catkin_make
  ```
## Usage:

# 1. Manual Keyboard Teleoperation:
```bash
$ roslaunch tortoisebot_gazebo tortoisebot_playground.launch
// For running gazebo world
$ roslaunch tortoisebot_control teleop_twist_keyboard.launch
// For running teleop Keyboard
```
# 2. Map Generation( SLAM Gmapping) :
```bash
$ roslaunch tortoisebot_gazebo tortoisebot_playground.launch
$ roslaunch tortoisebot_control teleop_twist_keyboard.launch
$ roslaunch tortoisebot_slam tortoisebot_gmapping.launch
// For running slam gmapping to generate the map
$ roslaunch tortoisebot_slam view_slam_rviz.launch
// For running rviz to visualize the generating map
$ roslaunch tortoisebot_slam map_saver.launch map_name:=<give_your_map_name>
// For saving the generated map
```
# 3. Autonomous Navigation (Simulation):
```bash
$ roslaunch tortoisebot_gazebo tortoisebot_playground.launch
$ roslaunch tortoisebot_navigation tortoisebot_navigation.launch
$ roslaunch tortoisebot_navigation view_navigation_rviz.launch
```
To save generated map to current working directory, run:
```bash
$ rosrun map_server map_saver -f <filename>
```
