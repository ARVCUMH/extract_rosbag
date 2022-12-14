# EXTRACT ROSBAG
Extracts a rosbag file (ROS) and converts it to a EUROC/ASL format.

The basic structure of the format is a set of directories containing:
```
├── robot0
   ├── camera
   │    ├── /*.png
   ├── gps0
   │    ├── /data.csv
   ├── gps_filtered
   │     ├── /data.csv
   ├── ground_truth
   │    ├── /data.csv
   ├── imu0
   │    ├── /data.csv
   ├── lidar
   │    ├── /*.pcd
   ├──odom
   │    ├── /data.csv
   ├── odometry_gps
   │    ├── /data.csv  
   ├──tf
   │    ├── /data.csv
   └── tf_static
        ├── /data.csv
```

# INSTALL
Install the requirements with:
- ./install.sh
# USAGE
First we activate the virtual environment created during installation.
- source setup.sh

Now, we just run the python file once the YAML file was properly configured.
- python3 extract_rosbag.py


# Setup YAML

Configure the following variables in config.yaml, for example:

- rosbag_path: "/home/user/file.bag"

- output_path: "/home/user/extract_rosbag"


Next: configure the topics that will be read and extracted

topic_name_point_cloud: "/ouster/points" # /os1/pointCloud\
topic_name_odometry: "/odometry/filtered"\
topic_name_ground_truth: "/ground_truth/state"\
topic_name_gps: "/fix"\
topic_name_imu: "/imu/data"\
topic_name_camera: "/aravis_cam/aravis_cam/image_raw"\
topic_name_tf: "/tf"\
topic_name_tf_static: "/tf_static"\
topic_name_gps_filtered: "/gps/filtered"\
topic_name_odometry_gps: "/odometry/gps"\

Finally, decide whether to extract those topics:\
extract_topics:\
  odometry: True\
  lidar2pcd: True\
  lidar2csv: False\
  ground_truth: False #just for simulation\
  gps: True\
  imu: True\
  camera: False\
  tf: True\
  tf_static: True\
  gps_filtered: True\
  odometry_gps: True\
