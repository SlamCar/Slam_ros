# slam_car
a car to learning slam  
------详情见Wiki------

## 运行环境
`Ubuntu 16.04 ros-kinetic`。

## 项目结构

```
--- src
 |- .gitlab
 |- .vscode
 |- localizer
 |- communcation_mcu
 |- msgs
 |- navigation
 |- 
 |- 
 |- 
 └- 
```

## 下载

```
$ cd ~/catkin_ws/src
$ git clone https://github.com/NOT-old-wang/Slam_car.git
、、、

* 也许你会碰到src非空目录，把src/下的CMakeLists.txt删除即可。

# 依赖
> 项目编译前需要先解决依赖问题。

、、、
# slam_serial的依赖：
$ sudo apt-get install ros-kinetic-serial
```


## 编译
该项目基于catkin工作空间，下载前请先建立catkin工作空间。
- [参考: ROS官方创建工作空间教程](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)

```
# 参考：
$ source /opt/ros/kinetic/setup.bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ catkin_init_workspace
$ cd ~/catkin_ws/
$ catkin_make
$ source devel/setup.bash

```

```
# 需要在catkin工作空间的根目录编译：
$ cd ~/catkin_ws
$ catkin_make

# Try source setup.bash if something wrong happend.
$ source ~/catkin_ws/devel/setup.bash
```

## 运行完整程序

> 程序已通过roslaunch配置，运行完整程序，请使用roslaunch命令。

`communcation_mcu`为程序的主节点，`launch`文件保存在`communcation_mcu`包下的`launch`目录下；

```
# Try tab while finding the launch file.
$ roslaunch communcation_mcu slam_car.launch
```

## 运行单独节点

```
$ source ~/catkin_ws/devel/setup.bash

# Make sure you run the master.
$ roscore

# Replace package_name and node_name
$ rosrun package_name node_name
```
