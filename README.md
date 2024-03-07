
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/ry0hei-kobayashi/deformation_detector.git
cd deformation_detector
pip install -r requirments.txt
cd ~/catkin_ws
rosdep install -i --from-paths ./
catkin_make

roscore
rosrun deformation_detector call_cpp.py
