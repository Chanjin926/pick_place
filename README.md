# pick_place
for visionless Peg-In-Hole with Spiral Trajectory
## Environment Used
* Ubuntu 18.04
* ROS Melodic

[MoveIt Pick_Place tutorial](http://docs.ros.org/en/melodic/api/moveit_tutorials/html/index.html)

## Execution
Gazebo에Panda Robot Arm과 world 불러오기.

Workspace setup은 [MoveIt Getting Started](http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/getting_started/getting_started.html) 참조
```
roslaunch panda_moveit_config demo_gazebo.launch world:=$(rospack find franka_gazebo)/world/pick_place.sdf
```
manipulation 코드 실행
```
rosrun moveit_tutorials pick_place_tutorial
```
