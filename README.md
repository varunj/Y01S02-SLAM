# Installalation Manual for Place Recognition and SO-DSO
[Place Recognition](https://github.com/jiawei-mo/so_dso_place_recognition)

[SO-DSO](https://github.com/jiawei-mo/scale_optimization)

# 1. SO-DSO Installation
- To install [SO-DSO](https://github.com/jiawei-mo/scale_optimization), first you need to install [DSO](https://github.com/JakobEngel/dso) and [catkin](http://wiki.ros.org/catkin)

- Here are steps to install DSO. Only listed the steps that are necessary. 

#### 1.1 DSO Installation Requirement
Reference [DSO](https://github.com/JakobEngel/dso)

##### suitesparse and eigen3.
Install with

	sudo apt-get install libsuitesparse-dev libeigen3-dev libboost-all-dev

##### OpenCV.
Install with

	sudo apt-get install libopencv-dev

##### Pangolin
Install from [https://github.com/stevenlovegrove/Pangolin](https://github.com/stevenlovegrove/Pangolin)

##### Build

    git clone https://github.com/JakobEngel/dso
    cd dso
    mkdir build
    cd build
    cmake ..
    make -j4

This will compile a library libdso.a, which can be linked from external projects. If there is any confusion please refer to the original [DSO](https://github.com/JakobEngel/dso) repo

#### 1.2 catkin Installation Requirement 
Reference [catkin](http://wiki.ros.org/catkin)

##### Install prebuilt package from the ubuntu
```
sudo apt-get install ros-melodic-catkin
```
If it does not work, you may need to install from source

##### Install from source


    git clone https://github.com/ros/catkin
    sudo apt-get install cmake python-catkin-pkg python-empy python-nose python-setuptools 
    libgtest-dev build-essential
    cd catkin
    mkdir build && cd build && cmake ../ && make && sudo make install

Please refer to the reference [catkin](http://wiki.ros.org/catkin) if there is anything goes wrong. The tricky part is that you still need to use ` sudo apt-get install ` to  install some ROG related packages. You may install the corresponding packages when you see the errors during the installation of SO-DSO.

#### 1.3 SO-DSO Intallation
Reference [SO-DSO](https://github.com/jiawei-mo/scale_optimization)

```
export DSO_PATH=[PATH_TO_DSO]/dso
```

You need to set ` DSO_PATH ` to the dso folder before you install the SO-DSO. There would also be a error suggestion if you did not set the path correctly. Then


    mkdir ~/catkin
    mkdir ~/catkin/src
    cd ~/catkin_ws/src
    git clone https://github.com/jiawei- 
    mo/scale_optimization.git
    cd ~/catkin_ws
    catkin_make



# Place Recognition
Reference [Place Recognition](https://github.com/jiawei-mo/so_dso_place_recognition)
Run the matlab files (test\_kitti.m test\_robotcar.m) in [Place Recognition](https://github.com/jiawei-mo/so_dso_place_recognition/tree/master/place_recognition/match_signatures)
