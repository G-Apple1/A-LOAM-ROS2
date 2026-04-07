# A-LOAM (ROS2 Humble Port)
## Advanced implementation of LOAM (ROS2 Humble Version)

A-LOAM is an Advanced implementation of LOAM (J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time), which uses Eigen and Ceres Solver to simplify code structure.

This version is fully ported to ROS2 Humble.

**Original Project:** [HKUST-Aerial-Robotics/A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM)



**Modifier:** [Tong Qin](http://www.qintonguav.com), [Shaozu Cao](https://github.com/shaozu)

**ROS2 Modifier:**  [Chengguo Xu](https://github.com/G-Apple1)

**Tools used**: Yuanbao AI, Doubao AI

## 1. Prerequisites
### 1.1 **Ubuntu** and **ROS**
Ubuntu 64-bit 22.04 and ROS2 Humble.


### 1.2. **Ceres Solver**
Follow [Ceres Installation](http://ceres-solver.org/installation.html).
```
sudo apt install libceres-dev
```

### 1.3. **PCL**
Follow [PCL Installation](http://www.pointclouds.org/downloads/linux.html).
```
sudo apt install libpcl-dev
```
### 1.4. **Other Dependencies**

```
sudo apt install ros-humble-pcl-conversions ros-humble-tf2 ros-humble-tf2-ros ros-humble-image-transport ros-humble-cv-bridge
```

## 2. Build A-LOAM (ROS2 Humble)
Clone the repository and colcon build:

```
    cd ~/ros2_ws/src
    git clone <your-repo-url> aloam_velodyne_ros2
    cd ~/ros2_ws
    colcon build
    source ~/ros2_ws/install/setup.bash
```

## 3. Velodyne VLP-16 Example
Download [NSH indoor outdoor](https://drive.google.com/file/d/1s05tBQOLNEDDurlg48KiUWxCp-YqYyGH/view) to YOUR_DATASET_FOLDER. 

```
    ros2 launch aloam_velodyne_ros2 aloam_velodyne_VLP_16.launch.py
    ros2 bag play YOUR_DATASET_FOLDER/nsh_indoor_outdoor.bag
```


## 4. Acknowledgements
Thanks for LOAM(J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time) and [LOAM_NOTED](https://github.com/cuitaixiang/LOAM_NOTED).

