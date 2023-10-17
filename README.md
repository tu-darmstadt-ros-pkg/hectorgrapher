# HectorGrapher

## Purpose

HectorGrapher is a robust, continuous-time SLAM framework for mobile rescue robots, forked from [Cartographer](https://github.com/googlecartographer/cartographer).
The framework is developed for the needs of robust and efficient 3D SLAM for the mobile ground robots used for urban search and rescue research at 
[Team Hector](https://www.teamhector.de/) and the 
[German Center for Rescue Robotics (DRZ)](https://rettungsrobotik.de/en/).


![HectorGrapher 3D SLAM Demo](docs/assets/elevated_ramps_5x.gif)

## Paper 

Thank you for citing [HectorGrapher (SSRR-2021)](https://www.sim.informatik.tu-darmstadt.de/publ/download/2021_daun_ssrr_hectorgrapher.pdf) if you use any of this code. 


```latex
@INPROCEEDINGS{daun_hectorgrapher,
  author={Daun, Kevin and Schnaubelt, Marius and Kohlbrecher, Stefan and von Stryk, Oskar},
  booktitle={2021 IEEE International Symposium on Safety, Security, and Rescue Robotics (SSRR)}, 
  title={HectorGrapher: Continuous-time Lidar SLAM with Multi-resolution Signed Distance Function Registration for Challenging Terrain}, 
  year={2021},
  pages={152-159},
  doi={10.1109/SSRR53300.2021.9597690}}
```

## Evaluation

We provide data sets with ground truth data from a Qualisys motion capture system. The data can be downloaded from [TUdatalib DRZ Living Lab Tracked Robot SLAM Dataset](https://tudatalib.ulb.tu-darmstadt.de/handle/tudatalib/3973?locale-attribute=en).

|Continuous Ramps | Woodpile | RoboCup Rescue League: Maneuvering 3 - Traverse | RoboCup Rescue League: Mobility 4 - Elevated Ramps |
|--|--|--|--|
| ![image](/docs/assets/continuous_ramps_photo_500.jpg) |  ![image](/docs/assets/woodpile_photo_close_500.jpg) | ![image](/docs/assets/man3_traverse_photo_500.jpg) | ![image](/docs/assets/mob5_elevated_ramps_photo_500.jpg) |  



## Getting started


If you haven't already installed it, [install ROS](http://wiki.ros.org/noetic/Installation/Ubuntu). Please use **Desktop-Full Install** to run the demo. Noetic is officially supported.

Create a [catkin workspace](https://catkin-tools.readthedocs.io/en/latest/quick_start.html) and compile the repositories: 
```
source /opt/ros/noetic/setup.bash
mkdir -p hectorgrapher/src
cd hectorgrapher 
catkin build 
cd src
git clone https://github.com/tu-darmstadt-ros-pkg/hectorgrapher.git
git clone https://github.com/tu-darmstadt-ros-pkg/hectorgrapher_ros.git
catkin build
```
Create a directory for the bag files
```
cd $HOME
mkdir bag_files
```
Download the zipped bag files from [TUdatalib](https://tudatalib.ulb.tu-darmstadt.de/handle/tudatalib/3973?locale-attribute=en) in $HOME/bag_files 
```
cd bag_files
unzip drz_living_lab_tracked_robot_dataset.zip
```
Launch the demo
```
source $HOME/workspaces/hectorgrapher/devel/setup.bash
roslaunch cartographer_ros demo_asterix.launch bag_filename:=$HOME/bag_files/drz_living_lab_tracked_robot_dataset/continuous_ramps/continuous_ramps_robot_data.bag
```

