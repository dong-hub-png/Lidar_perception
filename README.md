#### Note:
这个项目是基于ROS2的激光雷达感知。激光雷达感知的基本流程为：地面滤除、目标检测、目标追踪。目前只实现地面滤除这一部分，代码是参照autoware的ransac_ground_filter_nodelet.cpp，后续会实现多种地面滤除算法（ray_ground_filter、scan_ground_filter以及linefit）。此外目标检测后面会先做个简单实现就采用欧式聚类的方法来做，后续会去做基于深度学习的方法（centerpoint、pointpillars），代码部分基本上参照autoware；目标追踪这部分还没想好参照autoware去做还是apollo。这个项目的目标就是实现一个完整的激光雷达感知模块。

#### 项目结构：

本项目参照autoware的结构，每个功能包就是一个小的模块包括：地面滤除（ground_segmentation）、欧式聚类（euclidean_cluster）、目标追踪（multi_object_tracker）。每个功能包生成对应的组件，最后通过launch文件添加相应的组件到组件容器中，以此实现动态的加载若干的节点组件。



#### 编译：

1.创建ros2的工作空间

```
mkdir -p ~/dev_ws/src
cd ~/dev_ws/src
```

2.拉取Lidar_perception这个项目

```
git clone https://github.com/dong-hub-png/Lidar_perception.git
```

3.进入工作空间下进行编译

```
cd ..
colcon build --packages-up-to ground_segmentation
```

#### 运行：

1.通过launch文件启动：

```
ros2 launch ground_segmentation perception.launch.xml 
```

启动perception.launch.xml 终端后出现以下输出，及代表容器启动成功。

```
[INFO] [launch]: All log files can be found below /home/zpmc/.ros/log/2023-07-28-16-00-13-800024-zpmc-HP-ZBook-Power-15-6-inch-G9-Mobile-Workstation-PC-10314
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [component_container-1]: process started with pid [10327]
[INFO] [rviz2-2]: process started with pid [10329]
[rviz2-2] [INFO] [1690531214.063350669] [rviz2]: Stereo is NOT SUPPORTED
[rviz2-2] [INFO] [1690531214.063455457] [rviz2]: OpenGl version: 4.6 (GLSL 4.6)
[rviz2-2] [INFO] [1690531214.075951419] [rviz2]: Stereo is NOT SUPPORTED
[component_container-1] [INFO] [1690531214.127134190] [perception_pipeline_container]: Load Library: /home/zpmc/ros2_ws/install/ground_segmentation/lib/libground_segmentation.so
[component_container-1] [INFO] [1690531214.155386080] [perception_pipeline_container]: Found class: rclcpp_components::NodeFactoryTemplate<ground_segmentation::RANSACGroundFilterComponent>
[component_container-1] [INFO] [1690531214.155420653] [perception_pipeline_container]: Instantiate class: rclcpp_components::NodeFactoryTemplate<ground_segmentation::RANSACGroundFilterComponent>
[component_container-1] base_frame_:“base_link”
[INFO] [launch_ros.actions.load_composable_nodes]: Loaded node '/common_ground_filter' in container '/perception_pipeline_container
```

同时屏幕会出现RVIZ界面：

2.运行bag包，这里要注意ros1包不可以在ros2下跑，所以需要做一下转换。这里我用自己的数据测试了以下。

![git_readme_1](/home/zpmc/Pictures/git_readme_1.png)

地面滤除的参数写在config/ground_segmentation.param.yaml中，可以根据自己的需求调整。如果使用自己的数据集，首先要改一下input_topic；此外因为没做坐标变化，所以目前坐标系还是数据集本身的，rivz显示的坐标系还需要改一下。
