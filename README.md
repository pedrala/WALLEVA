Description
===================
AMR robots linking with ros2, gazebo, and rviz2 to collect trash around a park



How to excecute
====================
1. multitb_ws

```console
ros2 launch turtlebot3_multi_robot gazebo_multi_custom.launch.py
or ros2 launch turtlebot3_multi_robot gazebo_walleva.launch.py
```
2. amr_ws
```console
ros2 run move_to_goal move_to_goal
```
3. b3_ws
```console
ros2 run dual_bot gui_server
```
