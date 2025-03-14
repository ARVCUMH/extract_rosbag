#!/usr/bin/env python
# encoding: utf-8
"""
EurocSaver class

this defines the mapping from the ROS messages to EUROC/ASL format

@Authors: Antonio Santo and Arturo Gil
          Universidad Miguel Hernández de Elche
          arturo.gil@umh.es
@Time: November 2022
"""
import os
import numpy as np
import pandas as pd
import open3d as o3d
import sensor_msgs.point_cloud2 as pc2
import cv2
from cv_bridge import CvBridge, CvBridgeError


class EurocSaver():
    """
    Class that saves data in EUROC format.
    """
    def __init__(self, euroc_directory=None):
        self.euroc_directory = euroc_directory
        self.odometry_directory = euroc_directory + '/robot0/odom'
        self.imu_directory = euroc_directory + '/robot0/imu0'
        self.tf_directory = euroc_directory + '/robot0/tf'
        self.tf_static_directory = euroc_directory + '/robot0/tf_static'
        self.gps_filtered_directory = euroc_directory + '/robot0/gps_filtered'
        self.odometry_gps_directory = euroc_directory + '/robot0/odometry_gps'
        self.gps_directory = euroc_directory + '/robot0/gps0'
        self.lidar_directory = euroc_directory + '/robot0/lidar'
        self.ground_truth_directory = euroc_directory + '/robot0/ground_truth'
        self.camera_directory = euroc_directory + '/robot0/camera'
        self.aruco_directory = euroc_directory + '/robot0/aruco'

    def save_tf(self, bag_file, topic):
        """
        Saves tf to EUROC/ASL/UMH directory
        """
        try:
            os.makedirs(self.tf_directory)
        except OSError:
            print("Directory exists or creation failed", self.tf_directory)
        epoch_list = []
        translation_list = []
        rotation_list = []
        frame_id_list = []
        child_frame_id_list = []

        for topic, msg, t in bag_file.read_messages(topics=[topic]):
            time_str = str(msg.transforms[0].header.stamp)
            epoch_list.append(time_str)
            frame_id_list.append(msg.transforms[0].header.frame_id)
            child_frame_id_list.append(msg.transforms[0].child_frame_id)
            rotation_list.append([msg.transforms[0].transform.rotation.x, msg.transforms[0].transform.rotation.y, msg.transforms[0].transform.rotation.z, msg.transforms[0].transform.rotation.w])
            translation_list.append([msg.transforms[0].transform.translation.x, msg.transforms[0].transform.translation.y,
                                  msg.transforms[0].transform.translation.z])

        rotation_list = np.array(rotation_list)
        translation_list = np.array(translation_list)
        raw_data = {'timestamp': epoch_list,
                    'frame_id': frame_id_list,
                    'child_frame_id': child_frame_id_list,
                    'x': translation_list[:, 0],
                    'y': translation_list[:, 1],
                    'z': translation_list[:, 2],
                    'qx': rotation_list[:, 0],
                    'qy': rotation_list[:, 1],
                    'qz': rotation_list[:, 2],
                    'qw': rotation_list[:, 3]
                    }
        df = pd.DataFrame(raw_data, columns=['timestamp', 'frame_id', 'child_frame_id', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
        df.to_csv(self.tf_directory + '/data.csv', index=False, header=['timestamp', 'frame_id', 'child_frame_id', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
        print('\n---')
        return True

    def save_tf_static(self, bag_file, topic):
        """
        Saves static transforms.
        """
        try:
            os.makedirs(self.tf_static_directory)
        except OSError:
            print("Directory exists or creation failed", self.tf_static_directory)
        epoch_list = []
        translation_list = []
        rotation_list = []
        frame_id_list = []
        child_frame_id_list = []

        for topic, msg, t in bag_file.read_messages(topics=[topic]):
            for i in range(0, len(msg.transforms)):
                time_str = str(msg.transforms[i].header.stamp)
                epoch_list.append(time_str)
                frame_id_list.append(msg.transforms[i].header.frame_id)
                child_frame_id_list.append(msg.transforms[i].child_frame_id)
                rotation_list.append([msg.transforms[i].transform.rotation.x, msg.transforms[i].transform.rotation.y,
                                      msg.transforms[i].transform.rotation.z, msg.transforms[i].transform.rotation.w])
                translation_list.append(
                    [msg.transforms[i].transform.translation.x, msg.transforms[i].transform.translation.y,
                     msg.transforms[i].transform.translation.z])

        rotation_list = np.array(rotation_list)
        translation_list = np.array(translation_list)
        raw_data = {'timestamp': epoch_list,
                    'frame_id': frame_id_list,
                    'child_frame_id': child_frame_id_list,
                    'x': translation_list[:, 0],
                    'y': translation_list[:, 1],
                    'z': translation_list[:, 2],
                    'qx': rotation_list[:, 0],
                    'qy': rotation_list[:, 1],
                    'qz': rotation_list[:, 2],
                    'qw': rotation_list[:, 3]
                    }
        df = pd.DataFrame(raw_data,
                          columns=['timestamp', 'frame_id', 'child_frame_id', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
        df.to_csv(self.tf_static_directory + '/data.csv', index=False,
                  header=['timestamp', 'frame_id', 'child_frame_id', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
        print('\n---')
        return True

    def save_odometry_gps(self, bag_file, topic):
        """
        Saves GPS information using XYZ Q. Considering a ENU UTM reference system.
        """
        try:
            os.makedirs(self.odometry_gps_directory)
        except OSError:
            print("Directory exists or creation failed", self.odometry_gps_directory)
        epoch_list = []
        # list of xyz positions and quaternions
        gps_position_list = []
        gps_orientation_list = []
        covariance_list = []

        for topic, msg, t in bag_file.read_messages(topics=[topic]):
            time_str = str(msg.header.stamp)
            epoch_list.append(time_str)
            gps_position_list.append([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
            gps_orientation_list.append([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
            covariance = np.reshape(msg.pose.covariance, (6, 6))
            covariance_list.append(np.diag(covariance))

        gps_position_list = np.array(gps_position_list)
        gps_orientation_list = np.array(gps_orientation_list)
        covariance_list = np.array(covariance_list)
        raw_data = {'timestamp': epoch_list,
                    'x': gps_position_list[:, 0],
                    'y': gps_position_list[:, 1],
                    'z': gps_position_list[:, 2],
                    'qx': gps_orientation_list[:, 0],
                    'qy': gps_orientation_list[:, 1],
                    'qz': gps_orientation_list[:, 2],
                    'qw': gps_orientation_list[:, 3],
                    'covariance_d1': covariance_list[:, 0],
                    'covariance_d2': covariance_list[:, 1],
                    'covariance_d3': covariance_list[:, 2],
                    'covariance_d4': covariance_list[:, 3],
                    'covariance_d5': covariance_list[:, 4],
                    'covariance_d6': covariance_list[:, 5]
                    }
        df = pd.DataFrame(raw_data,
                          columns=['timestamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw', 'covariance_d1', 'covariance_d2',
                                   'covariance_d3', 'covariance_d4', 'covariance_d5',
                                   'covariance_d6'])
        df.to_csv(self.odometry_gps_directory + '/data.csv', index=False,
                  header=['timestamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw', 'covariance_d1', 'covariance_d2',
                                   'covariance_d3', 'covariance_d4', 'covariance_d5',
                                   'covariance_d6'])
        print('\n---')
        print('Saving topic odometry GPS data.csv: ')
        print(df)
        return True

    def save_gps_filtered(self, bag_file, topic):
        try:
            os.makedirs(self.gps_filtered_directory)
        except OSError:
            print("Directory exists or creation failed", self.gps_filtered_directory)
        epoch_list = []
        # list of xyz positions and quaternions
        gps_coords_list = []
        covariance_list = []
        mode = []
        for topic, msg, t in bag_file.read_messages(topics=[topic]):
            if np.isnan(msg.latitude) == False:
                time_str = str(msg.header.stamp)
                epoch_list.append(time_str)
                gps_coords_list.append([msg.latitude, msg.longitude, msg.altitude])
                mode.append(msg.status.status)
                covariance = np.reshape(msg.position_covariance, (3, 3))
                covariance_list.append(np.diag(covariance))
            else:
                time_str = str(msg.header.stamp)
                # GPS coords.
                epoch_list.append(time_str)
                gps_coords_list.append([0.0, 0.0, 0.0])
                mode.append(msg.status.status)
                covariance = np.reshape(msg.position_covariance, (3, 3))
                covariance_list.append(np.diag(covariance))

        gps_coords_list = np.array(gps_coords_list)
        covariance_list = np.array(covariance_list)
        raw_data = {'timestamp': epoch_list,
                    'latitude': gps_coords_list[:, 0],
                    'longitude': gps_coords_list[:, 1],
                    'altitude': gps_coords_list[:, 2],
                    'covariance_d1': covariance_list[:, 0],
                    'covariance_d2': covariance_list[:, 1],
                    'covariance_d3': covariance_list[:, 2],
                    'status': mode
                    }
        df = pd.DataFrame(raw_data,
                          columns=['timestamp', 'latitude', 'longitude', 'altitude', 'covariance_d1', 'covariance_d2',
                                   'covariance_d3', 'status'])
        df.to_csv(self.gps_filtered_directory + '/data.csv', index=False,
                  header=['#timestamp [ns]', 'latitude', 'longitude', 'altitude',
                          'covariance_d1', 'covariance_d2', 'covariance_d3', 'status'])
        print('Saving topic GPS filtered data.csv: ')
        print(df)
        print('\n---')
        return True

    def save_odometry(self, bag_file, topic):
        """
        Saves odometry from a robot/odom topic
        """
        try:
            os.makedirs(self.odometry_directory)
        except OSError:
            print("Directory exists or creation failed", self.odometry_directory)
        epoch_list = []
        xyz_list = []
        q_list = []
        for topic, msg, t in bag_file.read_messages(topics=[topic]):
            time_str = str(msg.header.stamp)
            epoch_list.append(time_str)
            xyz_list.append([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
            q_list.append([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])

        xyz_list = np.array(xyz_list)
        q_list = np.array(q_list)
        raw_data = {'timestamp': epoch_list,
                    'x': xyz_list[:, 0],
                    'y': xyz_list[:, 1],
                    'z': xyz_list[:, 2],
                    'qx': q_list[:, 0],
                    'qy': q_list[:, 1],
                    'qz': q_list[:, 2],
                    'qw': q_list[:, 3]
                    }
        df = pd.DataFrame(raw_data, columns=['timestamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
        df.to_csv(self.odometry_directory + '/data.csv', index=False, header=['#timestamp [ns]',
                                                                              'x', 'y', 'z',
                                                                              'qx', 'qy', 'qz', 'qw'])
        print('Saving topic odometry data.csv: ')
        print(df)
        print('\n---')
        return True

    def save_imu(self, bag_file, topic):
        """
        Saving, separately, in three different directories:
        orientation
        linar_acceleration
        linear_velocity
        angular_velocity
        """
        try:
            os.makedirs(self.imu_directory)
        except OSError:
            print("Directory exists or creation failed", self.imu_directory)
        print('Saving separately orientation, linear acceleration and linear velocity')
        print('Saving orientation')
        self.save_imu_orientation(bag_file, topic)
        print('Saving linear acceleration')
        self.save_imu_linear_acceleration(bag_file, topic)
        print('Saving angular velocity')
        self.save_imu_angular_velocity(bag_file, topic)
        print('\n---')
        return True

    def save_imu_orientation(self, bag_file, topic):
        try:
            os.makedirs(self.imu_directory + '/orientation')
        except OSError:
            print("Directory exists or creation failed", self.imu_directory)
        epoch_list = []
        q_list = []
        covariance_list = []
        for topic, msg, t in bag_file.read_messages(topics=[topic]):
            time_str = str(msg.header.stamp)
            epoch_list.append(time_str)
            q_list.append([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
            covariance = np.reshape(msg.orientation_covariance, (3, 3))
            covariance_list.append(np.diag(covariance))
        q_list = np.array(q_list)
        covariance_list = np.array(covariance_list)
        raw_data = {'timestamp': epoch_list,
                    'qx': q_list[:, 0],
                    'qy': q_list[:, 1],
                    'qz': q_list[:, 2],
                    'qw': q_list[:, 3],
                    'covariance_d1': covariance_list[:, 0],
                    'covariance_d2': covariance_list[:, 1],
                    'covariance_d3': covariance_list[:, 2]
                    }
        df = pd.DataFrame(raw_data, columns=['timestamp', 'qx', 'qy', 'qz', 'qw',
                                             'covariance_d1',
                                             'covariance_d2',
                                             'covariance_d3'])
        print('Saving IMU orientation data.csv')
        df.to_csv(self.imu_directory + '/orientation/data.csv', index=False,
                  header=['#timestamp [ns]', 'qx', 'qy', 'qz', 'qw', 'covariance_d1', 'covariance_d2', 'covariance_d3'])
        print(df)
        print('\n---')
        return True

    def save_imu_linear_acceleration(self, bag_file, topic):
        try:
            os.makedirs(self.imu_directory + '/linear_acceleration')
        except OSError:
            print("Directory exists or creation failed", self.imu_directory + '/linear_acceleration')
        epoch_list = []
        q_list = []
        covariance_list = []
        for topic, msg, t in bag_file.read_messages(topics=[topic]):
            time_str = str(msg.header.stamp)
            epoch_list.append(time_str)
            q_list.append([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
            covariance = np.reshape(msg.linear_acceleration_covariance, (3, 3))
            covariance_list.append(np.diag(covariance))
        q_list = np.array(q_list)
        covariance_list = np.array(covariance_list)
        raw_data = {'timestamp': epoch_list,
                    'x': q_list[:, 0],
                    'y': q_list[:, 1],
                    'z': q_list[:, 2],
                    'covariance_d1': covariance_list[:, 0],
                    'covariance_d2': covariance_list[:, 1],
                    'covariance_d3': covariance_list[:, 2]
                    }
        df = pd.DataFrame(raw_data, columns=['timestamp', 'x', 'y', 'z',
                                             'covariance_d1',
                                             'covariance_d2',
                                             'covariance_d3'])
        print('Saving IMU linear acceleration data data.csv')
        df.to_csv(self.imu_directory + '/linear_acceleration/data.csv', index=False, header=['#timestamp [ns]',
                                                                                             'x', 'y', 'z',
                                                                                             'covariance_d1',
                                                                                             'covariance_d2',
                                                                                             'covariance_d3'])
        print(df)
        print('\n---')
        return True

    def save_imu_angular_velocity(self, bag_file, topic):
        try:
            os.makedirs(self.imu_directory + '/angular_velocity')
        except OSError:
            print("Directory exists or creation failed", self.imu_directory + '/angular_velocity')
        epoch_list = []
        q_list = []
        covariance_list = []
        for topic, msg, t in bag_file.read_messages(topics=[topic]):
            time_str = str(msg.header.stamp)
            epoch_list.append(time_str)
            q_list.append([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
            covariance = np.reshape(msg.angular_velocity_covariance, (3, 3))
            covariance_list.append(np.diag(covariance))
        q_list = np.array(q_list)
        covariance_list = np.array(covariance_list)
        raw_data = {'timestamp': epoch_list,
                    'x': q_list[:, 0],
                    'y': q_list[:, 1],
                    'z': q_list[:, 2],
                    'covariance_d1': covariance_list[:, 0],
                    'covariance_d2': covariance_list[:, 1],
                    'covariance_d3': covariance_list[:, 2]
                    }
        df = pd.DataFrame(raw_data, columns=['timestamp', 'x', 'y', 'z',
                                             'covariance_d1',
                                             'covariance_d2',
                                             'covariance_d3'])
        print('Saving IMU angular velocity data.csv')
        df.to_csv(self.imu_directory + '/angular_velocity/data.csv', index=False, header=['#timestamp [ns]',
                                                                                          'x', 'y', 'z',
                                                                                          'covariance_d1',
                                                                                          'covariance_d2',
                                                                                          'covariance_d3'])
        print(df)
        print('\n---')
        return True

    def save_gps(self, bag_file, topic):
        """
        Saves raw lat/lng and altitude along with the covariance (x, y, h)
        """
        try:
            os.makedirs(self.gps_directory)
        except OSError:
            print("Directory exists or creation failed", self.gps_directory)
        epoch_list = []
        # list of xyz positions and quaternions
        gps_coords_list = []
        covariance_list = []
        mode = []
        for topic, msg, t in bag_file.read_messages(topics=[topic]):
            if np.isnan(msg.latitude) == False:
                time_str = str(msg.header.stamp)
                epoch_list.append(time_str)
                gps_coords_list.append([msg.latitude, msg.longitude, msg.altitude])
                mode.append(msg.status.status)
                covariance = np.reshape(msg.position_covariance, (3, 3))
                covariance_list.append(np.diag(covariance))
            else:
                time_str = str(msg.header.stamp)
                # GPS coords.
                epoch_list.append(time_str)
                gps_coords_list.append([0.0, 0.0, 0.0])
                mode.append(msg.status.status)
                covariance = np.reshape(msg.position_covariance, (3, 3))
                covariance_list.append(np.diag(covariance))

        gps_coords_list = np.array(gps_coords_list)
        covariance_list = np.array(covariance_list)
        raw_data = {'timestamp': epoch_list,
                    'latitude': gps_coords_list[:, 0],
                    'longitude': gps_coords_list[:, 1],
                    'altitude': gps_coords_list[:, 2],
                    'covariance_d1': covariance_list[:, 0],
                    'covariance_d2': covariance_list[:, 1],
                    'covariance_d3': covariance_list[:, 2],
                    'status': mode
                    }
        df = pd.DataFrame(raw_data,
                          columns=['timestamp', 'latitude', 'longitude', 'altitude', 'covariance_d1', 'covariance_d2',
                                   'covariance_d3', 'status'])
        print('Saving GPS data.csv')
        print('Number of GPS coordinates found: ', len(epoch_list))
        df.to_csv(self.gps_directory + '/data.csv', index=False,
                  header=['#timestamp [ns]', 'latitude', 'longitude', 'altitude',
                          'covariance_d1', 'covariance_d2', 'covariance_d3', 'status'])
        print(df)
        print('\n---')
        return True

    def save_lidar(self, bag_file, topic, to_csv, to_pcd):
        """
        Saves both the pointclouds in either csv or pcd format.
        Saving also the timestamps in data.csv to ease reading the data.
        """
        try:
            os.makedirs(self.lidar_directory + '/data')
        except OSError:
            print("Directory exists or creation failed", self.lidar_directory)
        print('SAVING LIDAR DATA!')
        epoch_list = []
        print('Finding the number of lidar clouds')
        for topic, msg, t in bag_file.read_messages(topics=[topic]):
            # time_str = str(msg.header.stamp)
            # caution! msg.header.stamp may fail if using rostopic drop
            time_str = str(t)
            # print(time_str)
            epoch_list.append(time_str)
        # TIMESTAMPS ONLY
        raw_data2 = {'timestamp': epoch_list}
        df = pd.DataFrame(raw_data2, columns=['timestamp'])
        df.to_csv(self.lidar_directory + "/data.csv", index=False, header=['#timestamp [ns]'])
        print('Saving data.csv for LiDAR which only contains timestamps')
        print(df)
        print('\n---')
        print('Now saving POINTCLOUDS')
        k = 0
        N = len(epoch_list)
        for topic, msg, t in bag_file.read_messages(topics=[topic]):
            print('Percentage complete: ', 100*k/N, '%')
            k += 1
            # time_str = str(msg.header.stamp)
            time_str = str(t)
            # epoch_list.append(time_str)
            field_names = [field.name for field in msg.fields]
            points = list(pc2.read_points(msg, skip_nans=True, field_names=field_names))
            if len(points) == 0:
                print("CAUTION!!! Converting an empty cloud!!!!!!!!!!!!")
                break

            # saving also reflectivity
            #  Lidar and pointcloud simple tutorial for open3D
            pcd_array = np.asarray(points)
            colors = np.zeros((pcd_array.shape[0], 3))
            if to_pcd:
                points = pcd_array[:, 0:3]
                # saving reflectivity only
                colors[:, 0] = pcd_array[:, 5]
                pointcloud = o3d.geometry.PointCloud()
                pointcloud.points = o3d.utility.Vector3dVector(points)
                pointcloud.colors = o3d.utility.Vector3dVector(colors)
                output_directory = self.lidar_directory + '/data/'
                output_filename = output_directory + time_str + ".pcd"
                print('Wrote: ', output_filename)
                o3d.io.write_point_cloud(output_filename, pointcloud)
            if to_csv:
                raw_data = {'x': pcd_array[:, 0],
                            'y': pcd_array[:, 1],
                            'z': pcd_array[:, 2],
                            'intensity': pcd_array[:, 3],
                            't': pcd_array[:, 4],
                            'reflectivity': pcd_array[:, 5],
                            'ring': pcd_array[:, 6],
                            'ambient': pcd_array[:, 7],
                            'range': pcd_array[:, 8]}
                df = pd.DataFrame(raw_data, columns=['x', 'y', 'z', 'intensity', 't', 'reflectivity','ring', 'ambient', 'range'])
                df.to_csv(self.lidar_directory + '/data/' + time_str + ".csv", index=False,
                          header=['x', 'y', 'z', 'intensity', 't',
                                  'reflectivity', 'ring', 'ambient', 'range'])
        print('NUMBER OF POINTCLOUD SCANS FOUND: ', k)
        return True

    def save_ground_truth(self, bag_file, topic):
        """
        Saves the ground truth
        """
        try:
            os.makedirs(self.ground_truth_directory)
        except OSError:
            print("Directory exists or creation failed", self.ground_truth_directory)
        epoch_list = []
        # list of xyz positions and quaternions
        xyz_list = []
        q_list = []

        for topic, msg, t in bag_file.read_messages(topics=[topic]):
            time_str = str(msg.header.stamp)
            # Odometry.
            epoch_list.append(time_str)
            xyz_list.append([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
            q_list.append(
                [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,
                 msg.pose.pose.orientation.w])

        xyz_list = np.array(xyz_list)
        q_list = np.array(q_list)
        raw_data = {'timestamp': epoch_list,
                    'x': xyz_list[:, 0],
                    'y': xyz_list[:, 1],
                    'z': xyz_list[:, 2],
                    'qx': q_list[:, 0],
                    'qy': q_list[:, 1],
                    'qz': q_list[:, 2],
                    'qw': q_list[:, 3]
                    }
        df = pd.DataFrame(raw_data, columns=['timestamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
        df.to_csv(self.ground_truth_directory + '/data.csv', index=False, header=['#timestamp [ns]',
                                                                                  'x', 'y', 'z',
                                                                                  'qx', 'qy', 'qz', 'qw'])
        print('Saving ground truth data.csv')
        print(df)
        print('\n---')

    def save_camera(self, bag_file, topic):
        try:
            os.makedirs(self.camera_directory + '/data')
        except OSError:
            print("Directory exists or creation failed", self.camera_directory)

        for topic, msg, t in bag_file.read_messages(topics=[topic]):
            bridge = CvBridge()
            cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
            print('Saving image: ', self.camera_directory + '/data/' + str(msg.header.stamp) + '.png')
            cv2.imwrite(self.camera_directory + '/data/' + str(msg.header.stamp)+'.png', cv_image)

    def save_aruco_observations(self, bag_file, topic):
        """
        Saves ARUCO OBSERVATIONS in the camera reference frame
        A relative observations is saved, along with the ARUCO identifier
        """
        try:
            os.makedirs(self.aruco_directory)
        except OSError:
            print("Directory exists or creation failed", self.aruco_directory)
        epoch_list = []
        xyz_list = []
        q_list = []
        aruco_id_list = []
        for topic, msg, t in bag_file.read_messages(topics=[topic]):
            time_str = str(msg.header.stamp)
            epoch_list.append(time_str)
            xyz_list.append([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
            q_list.append([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
            # caution. the aruco id is saved as frame_id in a Pose message
            aruco_id_list.append(int(msg.header.frame_id))
        print('FOUND: ', len(aruco_id_list), 'ARUCO OBSERVATIONS!')
        xyz_list = np.array(xyz_list)
        q_list = np.array(q_list)
        aruco_id_list = np.array(aruco_id_list)
        raw_data = {'timestamp': epoch_list,
                    'x': xyz_list[:, 0],
                    'y': xyz_list[:, 1],
                    'z': xyz_list[:, 2],
                    'qx': q_list[:, 0],
                    'qy': q_list[:, 1],
                    'qz': q_list[:, 2],
                    'qw': q_list[:, 3],
                    'aruco_id': aruco_id_list
                    }
        df = pd.DataFrame(raw_data, columns=['timestamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw', 'aruco_id'])
        df.to_csv(self.aruco_directory + '/data.csv', index=False, header=['#timestamp [ns]',
                                                                              'x', 'y', 'z',
                                                                              'qx', 'qy', 'qz', 'qw', 'aruco_id'])
        print('Saving topic ARUCO data.csv: ')
        print(df)
        print('\n---')
        return True