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
To train new models see intructions [here](https://github.com/ethz-asl/segmap/wiki/Training-new-models).

#### Download demonstration files

To download all necessary files, copy the content of the [segmap_data](http://robotics.ethz.ch/~asl-datasets/segmap/segmap_data/) into ```~/.segmap/```. If you installed the segmappy python package you can run the automated download script. **Note: These models have been trained using Tensorflow 1.8 and are only guaranteed to work for that version.**

#### Run online SLAM example

An online SLAM example can be run with
```
$ roslaunch segmapper kitti_loop_closure.launch
```
