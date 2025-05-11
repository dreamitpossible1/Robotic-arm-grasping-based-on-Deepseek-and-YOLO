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
