### Dependencies

First install the required system packages:
```
$ sudo apt-get install python-wstool doxygen python3-pip python3-dev python-virtualenv dh-autoreconf
```
Set up the workspace configuration:
```
$ mkdir -p ~/segmap_ws/src
$ cd ~/segmap_ws
$ catkin init
$ catkin config --merge-devel
$ catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
```
Then use wstool for fetching catkin dependencies:
```
$ cd src
$ git clone https://github.com/cherrybell0912/segmap.git
$ wstool init
$ wstool merge segmap/dependencies.rosinstall
$ wstool update
```

#### Tensorflow
Follow the steps in this blog to install tensorflow 1.8 (https://blog.csdn.net/weixin_37835423/article/details/87295994)

##### Build tensorflow_ros_cpp

Follow step 3 in this blog to build tensorflow_ros_cpp (https://blog.csdn.net/weixin_37835423/article/details/87295994).

```
$ cd ~/segmap_ws
$ catkin build tensorflow_ros_cpp
```

### Build SegMap

Install some packages first：
```
$ sudo apt-get install autoconf automake libtool
```
```
$ git clone https://github.com/doxygen/doxygen.git
$ cd doxygen
$ mkdir build
$ cd build
$ cmake …
$ make
$ sudo make install
```
```
$ git clone https://github.com/ros-perception/pcl_conversions/tree/indigo-devel
$ cd pcl_conversions
$ mkdir build
$ cd build
$ cmake …
$ make
$ sudo make install
```

Finally, build the *segmapper* package which will compile all dependencies and SegMap modules:
```
$ cd ~/segmap_ws
$ catkin build segmapper
```

## Running segmapper Examples

Make sure to source the SegMap workspace before running the segmapper demonstrations:
```
$ source ~/segmap_ws/devel/setup.bash
```

#### Download demonstration files
Download the segmap data from [here](链接:https://pan.baidu.com/s/10ZmnXQMdqTU8qm3SRI1XqQ  密码:gscv). Then you need to change the path of "bag_file" in kitti_loop_closure.launch(segmapper/launch/kitti/kitti_loop_closure.launch) to the path where you store data locally.

#### Run online SLAM example

An online SLAM example can be run with
```
$ roslaunch segmapper kitti_loop_closure.launch
```
