#!/usr/bin/env python3.8
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Image, PointCloud2, CameraInfo, PointField
import sensor_msgs, geometry_msgs
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
import tf2_kdl
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point
import PyKDL
import os
import threading
import message_filters
import timeit
import sys

class LabellingNode():
    def __init__(self, directory):
        self.directory = directory
        self.node = rospy.init_node('labelling_node', anonymous=True,disable_signals=True)
        self.camera = PinholeCameraModel()
        self.bridge = CvBridge()
        self.name_incrementer = 0
        self.i = 0
        self.pcl_colorized = []

        self.raw_cloud = None
        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer, queue_size=1)
        #try:
        #    self.latest_trans = self.tfBuffer.lookup_transform('vehicle_blue/semantic_camera_link_1','vehicle_blue',rospy.Time(0))
        #except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        #    pass
        self._init_pcl()
        self.red = [0,0,255]

        self.stairs_color = 50
        a = 50
        self.stairs_rgb = struct.unpack('I', struct.pack('BBBB', self.stairs_color, self.stairs_color, self.stairs_color, a))[0]

        camera_info = rospy.wait_for_message("/panoptic/camera_info", CameraInfo)
        self.update_camera(camera_info)

        self.lidarcb = message_filters.Subscriber("/lidar/points", PointCloud2)#self.pc2_callback)
        self.pan1cb = message_filters.Subscriber("/panoptic/labels_map", Image)#self.IM_callback)
        self.pan2cb = message_filters.Subscriber("/panoptic2/labels_map", Image)# self.IM_callback_2)
        self.pan3cb = message_filters.Subscriber("/panoptic3/labels_map", Image)# self.IM_callback_3)
        self.pan4cb = message_filters.Subscriber("/panoptic4/labels_map", Image)# self.IM_callback_4)
        self.pan5cb = message_filters.Subscriber("/panoptic5/labels_map", Image)# self.IM_callback_5)
        self.pan6cb = message_filters.Subscriber("/panoptic6/labels_map", Image)# self.IM_callback_6)
        self.pan7cb = message_filters.Subscriber("/panoptic7/labels_map", Image)# self.IM_callback_7)
        self.pan8cb = message_filters.Subscriber("/panoptic8/labels_map", Image)# self.IM_callback_8)
        self.pan9cb = message_filters.Subscriber("/panoptic9/labels_map", Image)# self.IM_callback_8)
        self.pan10cb = message_filters.Subscriber("/panoptic10/labels_map", Image)# self.IM_callback_8)
        self.pan11cb = message_filters.Subscriber("/panoptic11/labels_map", Image)# self.IM_callback_8)
        self.pan12cb = message_filters.Subscriber("/panoptic12/labels_map", Image)# self.IM_callback_8)
        self.pan13cb = message_filters.Subscriber("/panoptic13/labels_map", Image)# self.IM_callback_8)
        self.pan14cb = message_filters.Subscriber("/panoptic14/labels_map", Image)# self.IM_callback_8)
        self.pan15cb = message_filters.Subscriber("/panoptic15/labels_map", Image)# self.IM_callback_8)
        self.pan16cb = message_filters.Subscriber("/panoptic16/labels_map", Image)# self.IM_callback_8)

        self.pub = rospy.Publisher("/colored_points", PointCloud2, queue_size=10)
        #ts2 = message_filters.ApproximateTimeSynchronizer([self.lidarcb, self.pan2cb], 10, 1.5, allow_headerless=True)
        #ts3 = message_filters.ApproximateTimeSynchronizer([self.lidarcb, self.pan3cb], 10, 1.5, allow_headerless=True)
        #ts4 = message_filters.ApproximateTimeSynchronizer([self.lidarcb, self.pan4cb], 10, 1.5, allow_headerless=True)
        #ts5 = message_filters.ApproximateTimeSynchronizer([self.lidarcb, self.pan5cb], 10, 1.5, allow_headerless=True)
        #ts6 = message_filters.ApproximateTimeSynchronizer([self.lidarcb, self.pan6cb], 10, 1.5, allow_headerless=True)
        #ts7 = message_filters.ApproximateTimeSynchronizer([self.lidarcb, self.pan7cb], 10, 1.5, allow_headerless=True)
        #ts8 = message_filters.ApproximateTimeSynchronizer([self.lidarcb, self.pan8cb], 10, 1.5, allow_headerless=True)

        tst = message_filters.ApproximateTimeSynchronizer([self.lidarcb, self.pan1cb,self.pan2cb,self.pan3cb,self.pan4cb,self.pan5cb,self.pan6cb,self.pan7cb,self.pan8cb, self.pan9cb,self.pan10cb,self.pan11cb,self.pan12cb,self.pan13cb,self.pan14cb, self.pan15cb,self.pan16cb], 1, 0.2, allow_headerless=True)
        tst.registerCallback(self.pcl2points)
        print("STARTING NODE")
        try:
            os.chdir(directory)
            print("Current working directory: {0}".format(os.getcwd()))
        except FileNotFoundError:
            print("Directory: {0} does not exist".format(directory))
        except NotADirectoryError:
            print("{0} is not a directory".format(directory))
        except PermissionError:
            print("You do not have permissions to change to {0}".format(directory))
        rospy.spin()

    def update_camera(self,camera_info):
        self.camera.fromCameraInfo(camera_info)
        self.width = camera_info.width
        self.height = camera_info.height # parse this from cam info
        #camera_info.D = [0.0, 0.0, 0.0, 0.0, 0.0]
        #focal_length = (camera_info.width/2) / math.tan( 1.57/2)
        #camera_info.K = [focal_length, 0.0, self.width/2, 0.0, focal_length, self.height/2, 0.0, 0.0, 1.0]
        ###camera_info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        #camera_info.P = [focal_length, 0.0, self.width/2, -0.0, 0.0, focal_length, self.height/2, 0.0, 0.0, 0.0, 1.0, 0.0]
        #camera_info.binning_x = 0
        #camera_info.binning_y = 0
        print("Camera Updated\n",camera_info)

    def pc2_callback(self,data):
        raw_cloud = pc2.read_points_list(data,skip_nans=True,field_names=("x", "y", "z", "intensity"))
        raw_cloud = list(filter(lambda num: not math.isinf(num[0]), raw_cloud))
        self.raw_cloud = raw_cloud
        # if self.i % 1 == 0:
            # self.pcl2points(data)
        # self.i += 1

    def IM_callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
            self.width, self.height, channels = cv_image.shape 
            self.last_image = np.array(cv_image)
        except CvBridgeError as error:
            print(error)
    
    def pcl1_cb(self,cloud,image):
        print("CB")
        try:
            self.transform_link_1 = self.tfBuffer.lookup_transform('vehicle_blue/semantic_camera_link_1/semantic_segmentation_camera_1','vehicle_blue/semantic_camera_link_1',rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("Error while receiving tf transform")
        raw_cloud = pc2.read_points_list(cloud,skip_nans=True,field_names=("x", "y", "z", "intensity"))
        raw_cloud = list(filter(lambda num: not math.isinf(num[0]), raw_cloud))
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image, desired_encoding='passthrough')
            self.width, self.height, channels = cv_image.shape 
            self.last_image = np.array(cv_image)
        except CvBridgeError as error:
            print(error)

        self.pc(raw_cloud, self.transform_link_1, self.last_image)

    def pcl2points(self, cloud, im1,im2,im3,im4,im5,im6,im7,im8, im9, im10, im11, im12, im13, im14, im15, im16):
        print("LISTENING")
        raw_cloud = pc2.read_points_list(cloud,skip_nans=True,field_names=("x", "y", "z", "intensity"))
        self.raw_cloud = list(filter(lambda num: not math.isinf(num[0]), raw_cloud))
        try:
            self.last_image = np.array(self.bridge.imgmsg_to_cv2(im1, desired_encoding='passthrough'))
            self.last_image_2 = np.array(self.bridge.imgmsg_to_cv2(im2, desired_encoding='passthrough'))
            self.last_image_3 = np.array(self.bridge.imgmsg_to_cv2(im3, desired_encoding='passthrough'))
            self.last_image_4 = np.array(self.bridge.imgmsg_to_cv2(im4, desired_encoding='passthrough'))
            self.last_image_5 = np.array(self.bridge.imgmsg_to_cv2(im5, desired_encoding='passthrough'))
            self.last_image_6 = np.array(self.bridge.imgmsg_to_cv2(im6, desired_encoding='passthrough'))
            self.last_image_7 = np.array(self.bridge.imgmsg_to_cv2(im7, desired_encoding='passthrough'))
            self.last_image_8 = np.array(self.bridge.imgmsg_to_cv2(im8, desired_encoding='passthrough'))
            self.last_image_9 = np.array(self.bridge.imgmsg_to_cv2(im9, desired_encoding='passthrough'))
            self.last_image_10 = np.array(self.bridge.imgmsg_to_cv2(im10, desired_encoding='passthrough'))
            self.last_image_11 = np.array(self.bridge.imgmsg_to_cv2(im11, desired_encoding='passthrough'))
            self.last_image_12 = np.array(self.bridge.imgmsg_to_cv2(im12, desired_encoding='passthrough'))
            self.last_image_13 = np.array(self.bridge.imgmsg_to_cv2(im13, desired_encoding='passthrough'))
            self.last_image_14 = np.array(self.bridge.imgmsg_to_cv2(im14, desired_encoding='passthrough'))
            self.last_image_15 = np.array(self.bridge.imgmsg_to_cv2(im15, desired_encoding='passthrough'))
            self.last_image_16 = np.array(self.bridge.imgmsg_to_cv2(im16, desired_encoding='passthrough'))

        except CvBridgeError as error:
            print(error)
        try:
            self.transform_link_1 = self.tfBuffer.lookup_transform('vehicle_blue/semantic_camera_link_1/semantic_segmentation_camera_1','vehicle_blue/semantic_camera_link_1',rospy.Time(0))
            self.transform_link_2 = self.tfBuffer.lookup_transform('vehicle_blue/semantic_camera_link_2/semantic_segmentation_camera_2','vehicle_blue/semantic_camera_link_1',rospy.Time(0))
            self.transform_link_3 = self.tfBuffer.lookup_transform('vehicle_blue/semantic_camera_link_3/semantic_segmentation_camera_3','vehicle_blue/semantic_camera_link_1',rospy.Time(0))
            self.transform_link_4 = self.tfBuffer.lookup_transform('vehicle_blue/semantic_camera_link_4/semantic_segmentation_camera_4','vehicle_blue/semantic_camera_link_1',rospy.Time(0))
            self.transform_link_5 = self.tfBuffer.lookup_transform('vehicle_blue/semantic_camera_link_5/semantic_segmentation_camera_5','vehicle_blue/semantic_camera_link_1',rospy.Time(0))
            self.transform_link_6 = self.tfBuffer.lookup_transform('vehicle_blue/semantic_camera_link_6/semantic_segmentation_camera_6','vehicle_blue/semantic_camera_link_1',rospy.Time(0))
            self.transform_link_7 = self.tfBuffer.lookup_transform('vehicle_blue/semantic_camera_link_7/semantic_segmentation_camera_7','vehicle_blue/semantic_camera_link_1',rospy.Time(0))
            self.transform_link_8 = self.tfBuffer.lookup_transform('vehicle_blue/semantic_camera_link_8/semantic_segmentation_camera_8','vehicle_blue/semantic_camera_link_1',rospy.Time(0))
            self.transform_link_9 = self.tfBuffer.lookup_transform('vehicle_blue/semantic_camera_link_9/semantic_segmentation_camera_9','vehicle_blue/semantic_camera_link_1',rospy.Time(0))
            self.transform_link_10 = self.tfBuffer.lookup_transform('vehicle_blue/semantic_camera_link_10/semantic_segmentation_camera_10','vehicle_blue/semantic_camera_link_1',rospy.Time(0))
            self.transform_link_11 = self.tfBuffer.lookup_transform('vehicle_blue/semantic_camera_link_11/semantic_segmentation_camera_11','vehicle_blue/semantic_camera_link_1',rospy.Time(0))
            self.transform_link_12 = self.tfBuffer.lookup_transform('vehicle_blue/semantic_camera_link_12/semantic_segmentation_camera_12','vehicle_blue/semantic_camera_link_1',rospy.Time(0))
            self.transform_link_13 = self.tfBuffer.lookup_transform('vehicle_blue/semantic_camera_link_13/semantic_segmentation_camera_13','vehicle_blue/semantic_camera_link_1',rospy.Time(0))
            self.transform_link_14 = self.tfBuffer.lookup_transform('vehicle_blue/semantic_camera_link_14/semantic_segmentation_camera_14','vehicle_blue/semantic_camera_link_1',rospy.Time(0))
            self.transform_link_15 = self.tfBuffer.lookup_transform('vehicle_blue/semantic_camera_link_15/semantic_segmentation_camera_15','vehicle_blue/semantic_camera_link_1',rospy.Time(0))
            self.transform_link_16 = self.tfBuffer.lookup_transform('vehicle_blue/semantic_camera_link_16/semantic_segmentation_camera_16','vehicle_blue/semantic_camera_link_1',rospy.Time(0))
        
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("Error while receiving tf transform")
        self.pcl_colorized = []
        
        t1 = threading.Thread(target=self.pc(self.raw_cloud, self.transform_link_1, self.last_image))
        t2 = threading.Thread(target=self.pc(self.raw_cloud, self.transform_link_2, self.last_image_2))
        t3 = threading.Thread(target=self.pc(self.raw_cloud, self.transform_link_3, self.last_image_3))
        t4 = threading.Thread(target=self.pc(self.raw_cloud, self.transform_link_4, self.last_image_4))
        t5 = threading.Thread(target=self.pc(self.raw_cloud, self.transform_link_5, self.last_image_5))
        t6 = threading.Thread(target=self.pc(self.raw_cloud, self.transform_link_6, self.last_image_6))
        t7 = threading.Thread(target=self.pc(self.raw_cloud, self.transform_link_7, self.last_image_7))
        t8 = threading.Thread(target=self.pc(self.raw_cloud, self.transform_link_8, self.last_image_8))

        t9 = threading.Thread(target=self.pc(self.raw_cloud, self.transform_link_9, self.last_image_9))
        t10 = threading.Thread(target=self.pc(self.raw_cloud, self.transform_link_10, self.last_image_10))
        t11 = threading.Thread(target=self.pc(self.raw_cloud, self.transform_link_11, self.last_image_11))
        t12 = threading.Thread(target=self.pc(self.raw_cloud, self.transform_link_12, self.last_image_12))
        t13 = threading.Thread(target=self.pc(self.raw_cloud, self.transform_link_13, self.last_image_13))
        t14 = threading.Thread(target=self.pc(self.raw_cloud, self.transform_link_14, self.last_image_14))
        t15 = threading.Thread(target=self.pc(self.raw_cloud, self.transform_link_15, self.last_image_15))
        t16 = threading.Thread(target=self.pc(self.raw_cloud, self.transform_link_16, self.last_image_16))

        t1.start(), t2.start(), t3.start(), t4.start(), t5.start(), t6.start(), t7.start(), t8.start(), t9.start(), t10.start(), t11.start(), t12.start(), t13.start(), t14.start(), t15.start(), t16.start()

        t1.join(), t2.join(), t3.join(), t4.join(), t5.join(), t6.join(), t7.join(), t8.join(), t9.join(), t10.join(), t11.join(), t12.join(), t13.join(), t14.join(), t15.join(), t16.join()
        pcp = pc2.create_cloud(self.header, self.fields, self.pcl_colorized)
        if len(self.pcl_colorized) > 100:
            self.write_cloud(pcp)
        self.publish_pcl(pcp)
        
    def pc(self, cloud, transform, image, overlay=False): #separate thread?
        for p in cloud:
            transformed_p = tf2_kdl.transform_to_kdl(transform) * PyKDL.Vector(p[0], p[1], p[2]) # transform point based on tf transform
            if transformed_p[0] > 0:
                projected_p = self.project_point(transformed_p)
                try:
                    h, w= round(projected_p[0]), round(projected_p[1])
                    if w < self.width and w > 0:
                        if h < self.height and h > 0:
                            if image[w,h][2] != 0:
                                self.pcl_colorized.append(list(p) + [image[w,h][0]] + [image[w,h][2]])
                            if overlay:
                                image[w,h] = self.red
                except:
                    pass
        if overlay:
           cv2.imshow("Image window", image)
           cv2.imwrite("result.png",image)
           cv2.waitKey(1)

    def publish_pcl(self,pcp):
            pcp.header.stamp = rospy.Time.now()
            self.pub.publish(pcp)

    def _init_pcl(self):
        self.fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1),
                  PointField('intensity', 12, PointField.FLOAT32, 1),
                  PointField('instance', 16, PointField.UINT32, 1),
                  PointField('label', 20, PointField.UINT32, 1),
                  ]
        self.header = Header()
        self.header.frame_id = "vehicle_blue/lidar_link/gpu_lidar" #add eventually world ?

    def project_point(self,point):
        x = -point[1] # -y
        y = -point[2] # -z
        z = point[0] # x
        return self.camera.project3dToPixel((x, y, z))

    def overlay_image(self,gen):
        '''
        For visualization and debugging purposes
        '''
        image = np.copy(self.last_image)
        red = [0,0,255]

        segm_pcl = []
        for p in gen:
            projected_p = self.project_point(p)
            if projected_p[1] < self.height and projected_p[1] > 0:
                if projected_p[0] < self.width and projected_p[0] > 0:
                    #segmented_pcl[int(projected_p[1]), int(projected_p[0])] = red
                    image[int(projected_p[1]), int(projected_p[0])] = red

        cv2.imshow("Image window", image)
        cv2.imwrite("result.png",image)
        cv2.waitKey(1)

    def publish_image(self, image):
        try:
           self.image_pub.publish(self.bridge.cv2_to_imgmsg(image, encoding="passthrough")) #"bgr8"))
        except CvBridgeError as error:
           print(error)

    def write_cloud(self, cloud):
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

    def shutdown(self):
        del self.node
        del self.tfBuffer
        del self.lidarcb
        del self.pan1cb
        del self.pan2cb
        del self.pan3cb
        del self.pan4cb
        del self.pan5cb
        del self.pan6cb
        del self.pan7cb
        del self.pan8cb

if __name__ == '__main__':
    LabellingNode(rospy.get_param('directory_param'))