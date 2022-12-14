#!/bin/sh
python3 -m venv venv
cd venv/bin
./pip install --ignore-installed  --extra-index-url https://rospypi.github.io/simple/ rospy rosbag sensor-msgs geometry-msgs cv_bridge
./pip install opencv-python
./pip install pyyaml
./pip install numpy
./pip install pandas
./pip install open3d
echo "Dependencias y paquetes instalados"