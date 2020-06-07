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



# 2. 如何使用TrajectoryGeneratorWaypoint类

​	常规包含头文件不再赘述

## 头文件trajectory_generator_waypoint.h

需要关注的

```cpp
private:
    const int d_order = 6;//决定了要最小化的阶次，这里minimum x^(6)
public:
    void init(double Vel,double Acc);//必须初始化速度和加速度

    bool trajGeneration(Eigen::MatrixXd path); //启动轨迹生成函数

	/**
     *
     * @param waypoints 输入的期望路径（中间点）
     * @return
     */
    bool trajGeneration(Eigen::MatrixXd waypoints);//启动轨迹生成函数

    /**
     *
     * @param k : x = 0, y = 1, z = 2
     * @param t_seg 对应轨迹段里的时间点
     * @param order :p=0,v=1,a=2,...
     * @return 轨迹段里的时间点对应order的x,y,z
     */
    Eigen::Vector3d getPolyStates(int k, double t_seg, int order);

    /**
     *
     * @param t 此处t是对应全局时间，即轨迹开始到t的时间
     * @return 包含x,y,z信息的3x3矩阵,组成如下
     * p:x,y,z
     * v:x,y,z
     * a:x,y,z
     */
    Eigen::MatrixXd getTrajectoryStates(double t);

    visualization_msgs::Marker visWayPointPath();//返回用于可视化的路径Marker
    visualization_msgs::Marker visWayPointTraj();//返回用于可视化的轨迹Marker
```

## 节点文件

```cpp
TrajectoryGeneratorWaypoint trajectoryGeneratorWaypoint;

// 初始化初始位置
Vector3d _startPos = Vector3d::Zero();

int main(int argc, char **argv)
{
    //订阅路径节点
_way_pts_sub = nh.subscribe("/waypoint_generator/waypoints", 1, rcvWaypointsCallBack);
    //发布可视化节点
    _wp_traj_vis_pub = nh.advertise<visualization_msgs::Marker>("vis_trajectory", 1);
    _wp_path_vis_pub = nh.advertise<visualization_msgs::Marker>("vis_waypoint_path", 1);
    
}

void rcvWaypointsCallBack(const nav_msgs::Path &wp)
{
    ......将wp转化为waypoints

    //启动轨迹生成
    bool isGenerated = trajectoryGeneratorWaypoint.trajGeneration(waypoints);
    if(isGenerated)
    {
        //获取可视化数据
        _wp_path_vis_pub.publish(trajectoryGeneratorWaypoint.visWayPointPath());
        _wp_traj_vis_pub.publish(trajectoryGeneratorWaypoint.visWayPointTraj());
        
        //获取实时状态变量示例
        double startTime = ros::Time::now().toSec();
        double t = ros::Time::now().toSec()-startTime;
        while (trajectoryGeneratorWaypoint._totalTime > t)
        {
            MatrixXd states = trajectoryGeneratorWaypoint.getTrajectoryStates(t);
            cout << "p=" << states.row(0) << endl;
            cout << "v=" << states.row(1) << endl;
            cout << "a=" << states.row(2) << endl;
            t = ros::Time::now().toSec()-startTime;
        }
    }
}
```



