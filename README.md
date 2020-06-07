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
    ├── rviz_plugins
    │   ├── CMakeLists.txt
    │   ├── config
    │   │   └── rviz_config.rviz
    │   ├── lib
    │   │   └── librviz_plugins.so
    │   ├── Makefile
    │   ├── package.xml
    │   ├── plugin_description.xml
    │   └── src
    │       ├── aerialmap_display.cpp
    │       ├── aerialmap_display.h
    │       ├── goal_tool.cpp
    │       ├── goal_tool.h
    │       ├── multi_probmap_display.cpp
    │       ├── multi_probmap_display.h
    │       ├── pose_tool.cpp
    │       ├── pose_tool.h
    │       ├── probmap_display.cpp
    │       └── probmap_display.h
    └── waypoint_trajectory_generator
        ├── app
        │   └── demo_node.cpp
        ├── CMakeLists.txt
        ├── include
        │   └── trajectory_generator_waypoint.h
        ├── launch
        │   ├── rviz_config
        │   │   └── test_traj.rviz
        │   └── test.launch
        ├── package.xml
        └── src
            └── trajectory_generator_waypoint.cpp

```

