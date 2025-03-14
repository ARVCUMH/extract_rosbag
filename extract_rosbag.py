#!/usr/bin/env python
# encoding: utf-8
"""
Extracts a rosbag file (ROS) and converts it to a EUROC/ASL format.

The basic structure of the format is a set of directories containing:

robot0/
    gps0/data.csv
    lidar/data.csv
         /data/timestamp.pcd
    odom/
    ground_truth/
    camera/data.csv
           /data/images.png

@Authors: Antonio Santo and Arturo Gil
          arturo.gil@umh.es
@Time: November 2022
"""
import rosbag
import yaml
from eurocsaver.eurocsaver import EurocSaver
import sys
import getopt


def find_options(argv):
    rosbag_path = None
    output_path = None
    try:
        opts, args = getopt.getopt(argv, "hi:o:", ["ifile=", "odir="])
    except getopt.GetoptError:
        print('extract_rosbag.py -i <rosbagfile> -o <outputdirectory>')
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print('extract_rosbag.py -i <rosbagfile> -o <outputdirectory>')
            sys.exit()
        elif opt in ("-i", "--ifile"):
            rosbag_path = arg
        elif opt in ("-o", "--odir"):
            output_path = arg
    print('Input ROSBAG file is: ', rosbag_path)
    print('Output directory is: ', output_path)
    return rosbag_path, output_path


if __name__ == '__main__':
    # TOPICS TO EXTRACT AND PATH CONFIGURATION FROM YAML
    with open(r'config.yaml') as file:
        param_list = yaml.load(file, Loader=yaml.FullLoader)
        print(param_list)
    # get paths of input filename and output directory
    # CAUTION: if run without arguments, the input and output are read from the yaml
    # alternatively, both arguments can be passed as command line arguments
    rosbag_path, output_path = find_options(sys.argv[1:])
    if rosbag_path is None or output_path is None:
        rosbag_path = param_list.get('rosbag_path')
        output_path = param_list.get('output_path')
    else:
        print('Processing file: ', rosbag_path)
        print('Output directory: ', output_path)

    topic_name_ground_truth = param_list.get('topic_name_ground_truth')
    topic_name_point_cloud = param_list.get('topic_name_point_cloud')
    topic_name_odometry = param_list.get('topic_name_odometry')
    topic_name_gps = param_list.get('topic_name_gps')
    topic_name_imu = param_list.get('topic_name_imu')
    topic_name_camera = param_list.get('topic_name_camera')
    topic_name_tf = param_list.get('topic_name_tf')
    topic_name_tf_static = param_list.get('topic_name_tf_static')
    topic_name_gps_filtered = param_list.get('topic_name_gps_filtered')
    topic_name_odometry_gps = param_list.get('topic_name_odometry_gps')
    topic_name_aruco_observations = param_list.get('topic_name_aruco_observations')

    # extract topics?
    save_point_cloud_as_csv = param_list.get('save_point_cloud_as_csv')
    save_point_cloud_as_pcd = param_list.get('save_point_cloud_as_pcd')

    print('POINT CLOUD TOPIC (LiDAR): ', topic_name_point_cloud)
    print('GROUND TRUTH TOPIC: ', topic_name_ground_truth)
    print('ODOMETRY TOPIC: ', topic_name_odometry)
    print('GPS TOPIC: ', topic_name_gps)
    print('IMU TOPIC: ', topic_name_imu)
    print('CAMERA TOPIC: ', topic_name_camera)
    print('ARUCO OBSERVATIONS : ', topic_name_aruco_observations)

    # WRITE ROBOT0 DIRECTORY
    eurocsaver = EurocSaver(euroc_directory=output_path)
    print("Loading Rosbag file!")
    # READ ROSBAG FILE
    bag = rosbag.Bag(rosbag_path)
    print("Rosbag loaded!")
    # this topic may only be available if using simulation in ROS/Gazebo
    if topic_name_ground_truth:
        try:
            print('Saving topic: ', topic_name_ground_truth)
            eurocsaver.save_ground_truth(bag, topic_name_ground_truth)
        except IndexError:
            print('Error saving', topic_name_ground_truth)
            print('The topic name may be non-existent')
            pass
    if topic_name_odometry:
        try:
            print('Saving topic_name_odometry: ', topic_name_odometry)
            eurocsaver.save_odometry(bag, topic_name_odometry)
        except IndexError:
            print('Error saving', topic_name_odometry)
            print('The topic name may be non-existent')
            pass
    if topic_name_gps:
        try:
            print('Saving topic_name_gps: ', topic_name_gps)
            eurocsaver.save_gps(bag, topic_name_gps)
        except IndexError:
            print('Error saving', topic_name_gps)
            print('The topic name may be non-existent')
            pass
    if topic_name_gps_filtered:
        try:
            print('Saving: ', topic_name_gps_filtered)
            eurocsaver.save_gps_filtered(bag, topic_name_gps_filtered)
        except IndexError:
            print('Error saving', topic_name_gps_filtered)
            print('The topic name may be non-existent')
            pass
    if topic_name_odometry_gps:
        try:
            print('Saving: ', topic_name_odometry_gps)
            eurocsaver.save_odometry_gps(bag, topic_name_odometry_gps)
        except IndexError:
            print('Error saving', topic_name_odometry_gps)
            print('The topic name may be non-existent')
            pass
    if topic_name_imu:
        try:
            print('Saving topic_name_imu: ', topic_name_imu)
            eurocsaver.save_imu(bag, topic_name_imu)
        except IndexError:
            print('Error saving', topic_name_imu)
            print('The topic name may be non-existent')
            pass
    if topic_name_camera:
        try:
            print('Saving: ', topic_name_camera)
            eurocsaver.save_camera(bag, topic_name_camera)
        except IndexError:
            print('Error saving', topic_name_camera)
            print('The topic name may be non-existent')
            pass
    if topic_name_aruco_observations:
        try:
            print('Saving: ', topic_name_aruco_observations)
            eurocsaver.save_aruco_observations(bag, topic_name_aruco_observations)
        except IndexError:
            print('Error saving', topic_name_aruco_observations)
            print('The topic name may be non-existent')
            pass
    if topic_name_point_cloud:
        print('Saving LiDAR points as csv: ', save_point_cloud_as_csv)
        print('Saving LiDAR points as pcd: ', save_point_cloud_as_pcd)
        try:
            print('Saving: ', topic_name_point_cloud)
            eurocsaver.save_lidar(bag, topic_name_point_cloud,
                                  to_csv=save_point_cloud_as_csv,
                                  to_pcd=save_point_cloud_as_pcd)
        except IndexError:
            print('Error saving', topic_name_point_cloud)
            pass
    if topic_name_tf:
        try:
            print('Saving topic_name_tf: ', topic_name_tf)
            eurocsaver.save_tf(bag, topic_name_tf)
        except IndexError:
            print('Error saving', topic_name_tf)
            pass
    if topic_name_tf_static:
        try:
            print('Saving topic_name_tf_static: ', topic_name_tf_static)
            eurocsaver.save_tf_static(bag, topic_name_tf_static)
        except IndexError:
            print('Error saving', topic_name_tf_static)
            pass

    print('FINISHED!')
