<div align="center">
    <h1>TwistEstimator</h1>
    <i>An Estimator for Ego-Motion Twist Using 4D-MMWave Radar Doppler and Asynchronous Event Streams</i>
    <br>
    <br>
<div>

[🛠️ Installation](#how-to-use-traj-lo) |
[🎥 Video]() |
[📖 Paper](doc/Radar_and_Event_Camera_Fusion_for_Agile_Robot_Ego_Motion_Estimation.pdf)

</div>
    <br>
    <img src="doc/image/trajectory.png" width="50%" height="auto" alt="Trajectory Image">
    <img src="doc/image/pipeline.png" width="40%" height="auto" alt="Pipeline Image">
<br>
</div>




## What is TwistEstimator
The TwistEstimator is designed to provide complementary 6-DoF velocity estimation. It leverages linear velocity from the millimeter-wave radar and angular velocity from event-based optical flow. By integrating these measurements within a continuous estimation framework, 6-DoF pose estimation is achieved. Additionally, the system simultaneously calibrates the external parameters and time delay between the millimeter-wave radar and event optical flow. A lightweight odometry solution is derived from the velocity loop.


## Supported Dataset
Currently, the released code only supports one LiDAR configuration. We will update it as soon as possible to provide multi-LiDAR support. The provided ROSbag data loader supports different types of LiDAR, including Livox, Ouster, Hesai, Robosense, and Velodyne. We have tested Traj-LO with the following datasets.

| Dataset                                                                                                                                                                                                                 | LiDAR                               | Message Type                                       | Configuration file                       |
|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------|----------------------------------------------------|------------------------------------------|
| [NTU VIRAL](https://ntu-aris.github.io/ntu_viral_dataset/)                                                                                                                                                              | Two Ouster OS1-16                   | sensor_msgs/PointCloud2                            | config_ntu.yaml                          |
| [Hilti 2021](https://www.hilti-challenge.com/dataset-2021.html)                                                                                                                                                         | Ouster OS0-64 <br/> Livox mid-70    | sensor_msgs/PointCloud2 <br/>livox_ros_driver/CustomMsg | config_ouster.yaml<br/>config_livox.yaml |
| [R3LIVE](https://github.com/ziv-lin/r3live_dataset)                                                                                                                                                                     | Livox Avia                          | livox_ros_driver/CustomMsg                            | config_livox.yaml                        |
| [Point-LIO](https://connecthkuhk-my.sharepoint.com/personal/hdj65822_connect_hku_hk/_layouts/15/onedrive.aspx?id=%2Fpersonal%2Fhdj65822%5Fconnect%5Fhku%5Fhk%2FDocuments%2FDataset%20for%20Point%2DLIO%20examples&ga=1) | Livox Avia                          | livox_ros_driver/CustomMsg                            | config_pointlio.yaml                     |
| [New College](https://ori-drs.github.io/newer-college-dataset/)                                                                                                                                                         | Ouster OS-1 64 <br/>Ouster OS-0 128 | sensor_msgs/PointCloud2                            | config_ouster.yaml                       |
| [Hilti 2022 &2023](https://www.hilti-challenge.com/dataset-2022.html)                                                                                                                                                    | Hesai PandarXT-32 | sensor_msgs/PointCloud2                            | config_hesai.yaml                        |
| [SubT-MRS](https://superodometry.com/iccv23_challenge_LiI)                                                                                                                                                    | Velodyne VLP16 | sensor_msgs/PointCloud2                            | config_velodyne.yaml                     |

The corresponding configuration files are located in the "data" directory. For optimal performance, you will need to fine-tune the parameters.

Since Traj-LO is a LiDAR-only method, it may fail in narrow spaces where there are few valid points for a long time.

## Dependency
TwistEstimator is developed based on ROS1 and may be extended to ROS2 in the future. In the author's pc environment, the third-party dependencies required by the code are as follows:
- ROS: Noetic
- Optimization: [Eigen](https://gitlab.com/libeigen/eigen.git), [Ceres-1.14.0](https://github.com/ceres-solver/ceres-solver.git)
- Visual Library: [OpenCV-4.2.0](https://github.com/opencv/opencv/releases/tag/4.2.0) **same as ROS Noetic**
- Datatype: 
    [ti_mmwave_rospkg](https://github.com/radar-lab/ti_mmwave_rospkg.git) 
    [dvs_msgs](https://github.com/davidtr99/dvs_msgs) 
    [pcl_ros / pcl_conversions v1.7.4](https://github.com/ros-perception/perception_pcl.git) **same as ROS Noetic**
- GUI: Rviz, OpenGL
- DataLoader: yaml-cpp
- Logfile: glog

Although major dependencies are included in the third-party folder, you may still need to run the script `install_deps.sh` to install libraries like Boost, etc.

## Build
Some dependencies are configured using terminal commands:
```
sudo apt-get install ros-noetic-pcl-ros ros-noetic-pcl-conversions ros-noetic-cv-bridge yaml-cpp
```
After install [Ceres-1.14.0](https://github.com/ceres-solver/ceres-solver.git) and [OpenCV-4.2.0](https://github.com/opencv/opencv/releases/tag/4.2.0), you can install the TwistEstimator project by following these steps:
```
mkdir -p [path_to_dir]/src
cd [path_to_dir]/src
git clone --recursive https://github.com/ZzhYgwh/TwistEstimator.git
git clone https://github.com/radar-lab/ti_mmwave_rospkg.git
git clone https://github.com/davidtr99/dvs_msgs.git

cd ..
catkin build 

or 

catkin_make
```

## Run
After modifying the config file for your environment, you can run TwistEstimator.
```

```

## Test
### Test Front-End
```
cd <path_to_workspace>
source devel/setup.bash
rosrun twist_estimator test_detector
```
or use GDB and ASan for debug
```
cd <path_to_workspace>/devel/lib/twist_estimator
gdb test_detector
```

If use Dji Experiment data, add this script for combinate with timestamp and pointcloud of Radar
```
python3 <path_to_workspace>/src/TwistEstimator/scripts/combi_radar.py
```

### Test Back-End
```
cd <path_to_workspace>
source devel/setup.bash
rosrun twist_estimator test_estimator
```
or use GDB and ASan for debug
```
cd <path_to_workspace>/devel/lib/twist_estimator
gdb test_estimator
```

If use Dji Experiment data, add this script for combinate with timestamp and pointcloud of Radar
```
python3 <path_to_workspace>/src/TwistEstimator/scripts/combi_radar.py
```

## Citation

If you use this project for any academic work, please cite our RA-L [paper](???).

```bibtex

```

## Contributing

TwistEstimator is currently in develop version, and we are actively working on it. We welcome community users to participate in this project.

## Acknowledgement
Thanks for these pioneering works:
[Basalt](https://cvg.cit.tum.de/research/vslam/basalt) (Batch Optimization), 
[clic](https://github.com/APRIL-ZJU/clic) (Trajctory Manage),
[4DRadarSLAM](ttps://github.com/zhuge2333/4DRadarSLAM) (Radar Ego Estimation).

[![Star History Chart](https://api.star-history.com/svg?repos=ZzhYgwh/TwistEstimator&type=Date)](https://star-history.com/#ZzhYgwh/TwistEstimator&Date)