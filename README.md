# 0 Intro

This package is used to control the  simulation of UAV with rope load.

# 1 Package structure

```bash
└── src
    ├── CMakeLists.txt
    ├── flightController
    │   ├── CMakeLists.txt
    │   ├── include
    │   │   ├── database.h
    │   │   └── flightController.h
    │   ├── package.xml
    │   ├── README.md
    │   ├── scenes
    │   │   ├── quadcopter.lua
    │   │   └── quadcopter.ttt
    │   ├── src
    │   │   ├── flightController.cpp
    │   │   ├── kTrack.cpp
    │   │   └── main.cpp
    │   └── videos
    │       └── quadcopterController-2020-03-15_01.15.56.mkv
    ├── rviz_plugins # 3D Nav Goal plugin
    └── waypoint_trajectory_generator
        ├── app
        │   └── demo_node.cpp # ROS node 订阅waypoints并生成可视化轨迹
        ├── CMakeLists.txt
        ├── include
        │   └── trajectory_generator_waypoint.h # 轨迹生成的头文件
        ├── launch
        │   ├── rviz_config
        │   │   └── test_traj.rviz
        │   └── test.launch # 示例launch文件
        ├── package.xml
        └── src
            └── trajectory_generator_waypoint.cpp # 轨迹生成的实现文件

```

