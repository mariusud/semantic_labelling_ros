#!/usr/bin/env python3.8
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Image, PointCloud2, CameraInfo, PointField
from image_geometry import PinholeCameraModel
import tf
from geometry_msgs.msg import PointStamped
import tf2_msgs.msg
import sensor_msgs.point_cloud2 as pc2
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import struct
import timeit
import math
import geometry_msgs.msg as gsm_msg
import tf2_ros
import tf2_geometry_msgs
from tf2_ros import Buffer
import time
import os

class DatasetWriter:
    def __init__(self, directory):
        self.directory = directory
        rospy.init_node('dataset_writer_node')
        rospy.Subscriber("/colored_points", PointCloud2, self.pc_callback)
        self.name_incrementer = 0
        try:
            os.chdir(directory)
            print("Current working directory: {0}".format(os.getcwd()))
        except FileNotFoundError:
            print("Directory: {0} does not exist".format(directory))
        except NotADirectoryError:
            print("{0} is not a directory".format(directory))
        except PermissionError:
            print("You do not have permissions to change to {0}".format(directory))
        self.spin()

    def spin(self):
        rospy.spin()

    def pc_callback(self, cloud):
        print("callback")
        cc2 = pc2.read_points_list(cloud, skip_nans=True)
        print(cc2[0])
        readcloud = pc2.read_points_list(cloud, skip_nans=True, field_names=("x", "y", "z", "intensity", "label"))
        N = len(readcloud)
        arr = np.zeros((N,4),dtype=np.float32)
        label = np.zeros((N,1),dtype=np.float32)
        for n, point in enumerate(readcloud):
            arr[n,0] = point[0] #might be different xyz
            arr[n,1] = point[1] #might be different xyz
            arr[n,2] = point[2] #might be different xyz
            arr[n,3] = point[3] # reflectivity
            label[n] = point[4] #might be different xyz
        arr.astype('float32').tofile(self.directory +  '/velodyne/' +  str(self.name_incrementer) + '.bin') # add location
        label.astype('float32').tofile(self.directory + '/labels/' + str(self.name_incrementer) + '.label') # add location
        self.name_incrementer += 1

