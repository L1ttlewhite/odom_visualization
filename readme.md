# 多路里程计可视化程序

本程序用来对里程计输出进行可视化,观察运动轨迹和姿态。

最多支持两路可视化输出，格式为 `geometry_msgs/PoseStamped`和`nav_msgs/Odometry`。**暂时不支持两路信号为不同格式的，请统一输出格式**

环境需求：

1. ros
2. Eigen
3. boost 

使用说明：
将此项目拷贝至ros工作区

```
cd catkin_ws/src
git clone https://github.com/L1ttlewhite/odom_visualization.git
cd ..
catkin_make
```

使用时，修改launch文件并运行即可
```
roslaunch odom_visualization odom_visualization.launch
```

launch文件中参数说明

```
num_pose   //geometry_msgs/PoseStamped格式的topic数量，0-2
num_odom   //nav_msgs/Odometry格式的topic数量，0-2

pose_topic_1 
pose_topic_2 //geometry_msgs/PoseStamped格式的topic名称

odom_topic_1 
odom_topic_2 //nav_msgs/Odometry格式的topic名称
```
程序中以Vins的输出的两个odom topic为例，可以根据实际使用需要进行修改 