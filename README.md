# rse-nd-ball-chaser
Course projects repository for the Udacity Robotics Software Engineer Nanodegree program.

This project 

This project contains the following Catkin packages :
* my_robot: Implementation of a differential drive robot with lidar sensor and camera, written in URDF.
* ball_chaser: An algorithm using OpenCV to track a ball position and send velocity messages to a simulated robot (my_robot pkg).


## Installation
Clone this repository in **src** folder in your catkin workspace
```
cd ~/catkin_ws/src
git clone https://github.com/rodriguesrenato/rse-nd-ball-chaser.git
```
## Usage
Supposing your catkin workspace is located in ~/
```
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```
Use the .launch files to launch packages.
## License
The contents of this repository are covered under the MIT License.
