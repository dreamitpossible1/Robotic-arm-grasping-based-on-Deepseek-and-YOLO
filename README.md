## Project Title
A Robot System for RB3 Gen2 Utilizing Deepseek and YOLO

## Overview
This project develops a robot arm system for RB3 Gen2 using depth images. RB3 Gen2 serves as the core hardware platform, offering advanced motion control and sensor capabilities. Depth images enable the robot arm to perceive 3D environments for tasks like object recognition and grasping. The Qualcomm Intelligent Robotics Product SDK is utilized for efficient image processing and seamless integration with RB3 Gen2, enhancing the system's performance and reliability.
## Quick Start with QualComm RB3 gen2
Download the precompiled package for RB3 Gen2：

wget https://artifacts.codelinaro.org/artifactory/qli-ci/flashable-binaries/qirpsdk/qcs6490-rb3gen2-vision-kit/arm-qcom-6.6.65-QLI.1.4-Ver.1.1_robotics-product-sdk-1.1.zip

Use the following command to unzip the package:

unzip arm-qcom-6.6.65-QLI.1.4-Ver.1.1_robotics-product-sdk-1.1.zip

The specific content is as follows:[QualComm Intelligent Robotics Product SDK Quick Start]([QIRP User Guide - Qualcomm® Linux Documentation](https://docs.qualcomm.com/bundle/publicresource/topics/80-70018-265/quick-start_3.html?vproduct=1601111740013072&version=1.4&facet=Qualcomm Intelligent Robotics Product (QIRP) SDK)


## 功能包说明packages-overview

* ***sagittarius_descriptions*** : 机械臂的描述功能包。
* ***sagittarius_moveit*** : 机械臂的moveit功能包。
* ***sak_sagittarius_arm*** : 机械臂的SDK源码。

* ***sagittarius_perception*** : 本项目的功能包

## 使用usage

### 系统要求 Prequirement

* System:	Ubuntu 16.04 ,Ubuntu 18.04 or Ubuntu 20.04
* ROS Version:	kinetic, melodic or noetic
* DeekSeek: model="deepseek-r1:14b" or higher,
            url="http://localhost:11434/api/generate"

### 下载安装 Download and install
* 下载工作空间 Download the workspace:
```yaml
    cd ~
    git clone https://github.com/Clipperrr/sagittarius_ws.git
```
* 安装依赖库 Install libraries and dependencies:
```yaml
    cd sagittarius_ws
    ./onekey.sh
    根据提示选择 103 ，然后回车键进行安装，或者执行
    ./src/sagittarius_arm_ros/install.sh
```
### 编译运行 compile and run
```yaml
    cd ~/sagittarius_ws
    catkin_make
```
* 如果编译一切正常，可根据提示运行相关例程。If everything goes fine, test the examples as follow:
```yaml
    roslaunch sagittarius_object_color_detector camera_calibration_hsv.launch 
    完成手眼标定
    roslaunch sagittarius_object_color_detector color_classification_fixed.launch
    输入需求，机械臂会根据需求抓取物体同时会话反馈

```
