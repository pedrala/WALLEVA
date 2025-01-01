Description
===================
AMR robots linking with ros2, gazebo, and rviz2 to collect trash around a park


Capture
============
<img src="capture/walleva_gui.png" alt="walleva_gui" width="1000">
<img src="capture/walleva_urdf_modify.png" alt="walleva_urdf_modify" width="1000">
<img src="capture/walleva_navigation.png" alt="walleva_navigation" width="1000">
<img src="capture/walleva_nav2.png" alt="walleva_nav2" width="1000">


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
