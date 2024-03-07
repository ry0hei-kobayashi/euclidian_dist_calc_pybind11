# Description


# How to use
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/ry0hei-kobayashi/euclidian_dist_calc_pybind11.git
cd euclidian_dist_calc_pybind11
pip install -r requirments.txt
cd ~/catkin_ws
rosdep install -i --from-paths ./
catkin_make
source ~/catkin_ws/devel/setup.bash

roscore
rosrun deformation_detector call_cpp.py
```
