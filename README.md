Description
===================
2개의 AMR 로봇이 공원주위를 돌아다닙니다. WALL-E 가 먼저 돌아다니며 쓰레기를 YOLO8로 객체인식합니다. WALL-E 가 매니퓰레이터로 집어서 뒤에 따라다니는 Eve 로봇의 바스켓에 담습니다. Eve 로봇은 터틀봇3 기본 모델에서 매니퓰레이터를 제거하고 뒤에 바스켓을 얹도록 URDF를 수정하였습니다. 

Ref Repo
============
2개 이상의 AMR봇을 rviz, gazebo 에서 스폰하기 위해 아래 리포지토리를 참조하였습니다.
```console
https://github.com/arshadlab/turtlebot3_multi_robot/
```

Capture
============
## GUI 실행모습
<img src="capture/walleva_gui.png" alt="walleva_gui" width="1000">

## 쓰레기를 담기 위해 메니퓰레이터를 떼고 바스켓을 뒤에 얹도록 URDF를 수정하고 적용한 모습
<img src="capture/walleva_urdf_modify.png" alt="walleva_urdf_modify" width="1000">

## Gazebo, Rviz 상에서 Demo 실행 모습
<img src="capture/walleva_navigation.png" alt="walleva_navigation" width="1000">
<img src="capture/walleva_navi2.png" alt="walleva_nav2" width="1000">


How to execute
====================
## multitb_ws
2개 이상의 AMR 스폰, 특수목적에 맞게 개조된 URDF, SDF파일 적용, world파일(map) 적용 
```console
ros2 launch turtlebot3_walleva gazebo_multi_custom.launch.py
or ros2 launch turtlebot3_walleva gazebo_walleva.launch.py
```
## amr_ws
WALL-E 가 쓰레기를 찾아 돌아다니고 Eve 는 WALL-E 를 따라다니도록 함. 공원내에서 무한루프로 돌아다님
```console
ros2 run move_to_goal move_to_goal
```
## b3_ws
GUI 서버 실행, 미니맵 상의 두 AMR의 위치 표시하고 카메라 뷰 디스플레이
```console
ros2 run dual_bot gui_server
```
