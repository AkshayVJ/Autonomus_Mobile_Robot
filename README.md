**Autonomous Mobile Robot Navigation**

## For Initial setups please reffer: 
1. https://www.clearpathrobotics.com/assets/guides/kinetic/ros/Drive%20a%20Husky.html

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
  $ git clone https://github.com/.git
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

1. Manual Keyboard Teleoperation:
```bash
$ roslaunch husky_gazebo husky_playpen.launch
$ roslaunch my_teleoperation_launcher teleop_keyboard.launch
```
2. Map-Less Navigation:
```bash
$ roslaunch cpr_agriculture_gazebo agriculture_world.launch
$ roslaunch my_navigation_launcher map_less_navigation.launch
```
3. Map-Based Navigation:
```bash
$ roslaunch husky_gazebo husky_playpen.launch
$ roslaunch my_navigation_launcher map_based_navigation.launch
```
To save generated map to current working directory, run:
```bash
$ rosrun map_server map_saver -f <filename>
```
