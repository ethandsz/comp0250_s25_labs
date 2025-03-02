## Task 1
```bash
sudo apt install ros-noetic-franka-ros ros-noetic-libfranka
```
Gazebo physics simluator is also needed (http://gazebosim.org/). This can be installed and then run with:
```bash
curl -sSL http://get.gazebosim.org | sh
gazebo
```

## Task 2
```bash
git clone --recurse-submodules https://github.com/surgical-vision/comp0250_s25_labs.git
```
```bash
cd comp0250_s25_labs
```
```bash
git submodule update --init --recursive
```
```bash
catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release
```
```bash
catkin build
```

## Task 3
```bash
source devel/setup.bash
```
```bash
roslaunch panda_description description.launch
```
