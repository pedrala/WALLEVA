Description
===================
Two AMR robots are walking around the park. WALL-E walks first and recognizes trash objects with YOLO8. WALL-E picks it up with its manipulator and puts it in the basket of EVE robot following behind. EVE robot is made by modifying URDF of Turtlebot3 base model by removing the manipulator and adding a basket at the back.

Capture
============
## GUI
<img src="capture/walleva_gui.png" alt="walleva_gui" width="1000">

## Modified and applied URDF to remove the manipulator and place a basket at the back to collect waste.
<img src="capture/walleva_urdf_modify.png" alt="walleva_urdf_modify" width="1000">

## Running on Gazebo, Rviz
<img src="capture/walleva_navigation.png" alt="walleva_navigation" width="1000">
<img src="capture/walleva_navi2.png" alt="walleva_nav2" width="1000">

How to execute
====================
## multitb_ws
2 or more AMR spawns, URDF, SDF files modified for special purposes, world file (map) applied
```console
ros2 launch turtlebot3_walleva gazebo_multi_custom.launch.py
or ros2 launch turtlebot3_walleva gazebo_walleva.launch.py
```
## amr_ws
WALL-E goes around looking for trash, and Eve follows him. They go around the park in an endless loop.
```console
ros2 run move_to_goal move_to_goal
```
## b3_ws
Running a GUI server, showing the positions of two AMRs on the minimap and displaying the camera view.
```console
ros2 run dual_bot gui_server
```
Reference
============
To spawn two or more AMR bots in rviz and gazebo, I have referenced the repositories below.
```console
https://github.com/arshadlab/turtlebot3_multi_robot/
```
