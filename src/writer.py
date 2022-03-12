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

class DatasetWriter:
    def __init__(self, directory):
        rospy.init_node('dataset_writer_node')
        rospy.Subscriber("/lidar/points", PointCloud2, self.pc_callback)
        self.spin()
        name_incrementer = 0
        try:
            os.chdir(directory)
            print("Current working directory: {0}".format(os.getcwd()))
        except FileNotFoundError:
            print("Directory: {0} does not exist".format(path))
        except NotADirectoryError:
            print("{0} is not a directory".format(path))
        except PermissionError:
            print("You do not have permissions to change to {0}".format(path))

    def spin(self):
        rospy.spin()

    def pc_callback(self, cloud):
        self.gen = pc2.read_points(cloud, skip_nans=True, field_names=("x", "y", "z", "label"))
        N = len(self.gen)
        arr = np.zeros((N,3),dtype=FLOAT32)
        label = np.zeros((N,1),dtype=FLOAT32)
        for n, point in enumerate(self.gen):
            arr[n] = point[0] #might be different xyz
            arr[n+1] = point[1] #might be different xyz
            arr[n+2] = point[2] #might be different xyz
            label[n] = point[3] #might be different xyz

        arr.astype('float32').tofile( '/ouster/' +  name_incrementer + '.bin') # add location
        label.astype('float32').tofile('/label' + name_incrementer + '.label') # add location

