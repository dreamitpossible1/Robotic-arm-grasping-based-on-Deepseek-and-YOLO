## Project Title
Robotic Arm Grasping System for RB3 Gen2 Based on Deepseek and YOLO

## Overview
This project presents a robotic arm grasping system for RB3 Gen2, powered by Deepseek and YOLO. The RB3 Gen2 hardware platform provides a solid foundation with its advanced motion control and sensor technology. YOLO is harnessed for real-time object detection, swiftly identifying objects within the environment. Deepseek contributes to the system's decision-making process, analyzing data to determine optimal grasping strategies. Together, these elements enable the robotic arm to perform efficient and precise grasping tasks, with the Qualcomm Intelligent Robotics Product SDK ensuring smooth integration and enhancing the overall reliability of the system.
## Quick Start with QualComm RB3 gen2
Download the precompiled package for RB3 Gen2：

```bash
wget https://artifacts.codelinaro.org/artifactory/qli-ci/flashable-binaries/qirpsdk/qcs6490-rb3gen2-vision-kit/arm-qcom-6.6.65-QLI.1.4-Ver.1.1_robotics-product-sdk-1.1.zip
```

Use the following command to unzip the package:
```bash
unzip arm-qcom-6.6.65-QLI.1.4-Ver.1.1_robotics-product-sdk-1.1.zip
```
 Run Flashing Procedure

```bash
cd <extracted zip directory>/target/qcs6490-rb3gen2-vision-kit/qcom-multimedia-image
<qdl_tool_path>/qdl_2.3.1/QDL_Linux_x64/qdl prog_firehose_ddr.elf rawprogram*.xml patch*.xml
```


Before you start, make sure finish [QualComm Intelligent Robotics Product SDK Quick Start]([QIRP User Guide - Qualcomm® Linux Documentation](https://docs.qualcomm.com/bundle/publicresource/topics/80-70018-265/quick-start_3.html?vproduct=1601111740013072&version=1.4&facet=Qualcomm Intelligent Robotics Product (QIRP) SDK)


## Quick Start with uArm Swift Pro

1. Download [uArm-Python-SDK](https://github.com/uArm-Developer/uArm-Python-SDK.git)


```bash
git clone https://github.com/uArm-Developer/uArm-Python-SDK.git
```
2. Installation
```bash
   python setup.py install
```

## 🦾 Project Overview

This project is based on the uArm Swift Pro robotic arm, which mainly consists of the following components:

- **End Effector**: Equipped with a suction pump, used for picking up and moving Mahjong tiles.
- **Drive System**: Composed of four servo motors, each controlling one degree of freedom (4-DoF), enabling precise and smooth motion control.
- **Control System**: An integrated controller that supports both Arduino and Python development environments, serving as the core control unit of the robotic arm.
 
> ✳️ The uArm Swift Pro is a four-axis (4-DoF) robotic manipulator known for its high precision and repeatability, making it ideal for lightweight object manipulation and interactive gaming tasks.


## packages-overview

* ***sagittarius_descriptions*** : 机械臂的描述功能包。
* ***sagittarius_moveit*** : 机械臂的moveit功能包。
* ***sak_sagittarius_arm*** : 机械臂的SDK源码。

* ***sagittarius_perception*** : 本项目的功能包

## usage

### Prequirement

* System:	Ubuntu 16.04 ,Ubuntu 18.04 or Ubuntu 20.04
* ROS Version:	kinetic, melodic or noetic
* DeekSeek: model="deepseek-r1:14b" or higher,
            url="http://localhost:11434/api/generate"

###  Download and install
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


## Reference

- [Qualcomm Linux](https://www.qualcomm.com/developer/software/qualcomm-linux)

- [QualComm Intelligent Robotics Product SDK](https://docs.qualcomm.com/bundle/publicresource/topics/80-70018-265/introduction_1.html?vproduct=1601111740013072&version=1.4&facet=Qualcomm Intelligent Robotics Product (QIRP) SDK)
