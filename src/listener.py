#!/usr/bin/env python3.8
from turtle import end_poly
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
import math
import geometry_msgs.msg as gsm_msg
import tf2_ros
from tf2_ros import Buffer
import tf2_kdl
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point
import PyKDL
import os
import message_filters
import multiprocessing
import time

class LabellingNode():
    def __init__(self, directory):
        self.save_dataset = False
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

        self._init_pcl()
        self.red = [0,0,255]

        self.stairs_color = 50 # todo all colors
        a = 50
        #self.stairs_rgb = struct.unpack('I', struct.pack('BBBB', self.stairs_color, self.stairs_color, self.stairs_color, a))[0]

        camera_info = rospy.wait_for_message("/panoptic/camera_info", CameraInfo)
        self.update_camera_info(camera_info)

        self.lidarcb = message_filters.Subscriber("/lidar/points", PointCloud2)
        self.pan1cb = message_filters.Subscriber("/panoptic/labels_map", Image)
        self.pan2cb = message_filters.Subscriber("/panoptic2/labels_map", Image)
        self.pan3cb = message_filters.Subscriber("/panoptic3/labels_map", Image)
        self.pan4cb = message_filters.Subscriber("/panoptic4/labels_map", Image)
        self.pan5cb = message_filters.Subscriber("/panoptic5/labels_map", Image)
        self.pan6cb = message_filters.Subscriber("/panoptic6/labels_map", Image)
        self.pan7cb = message_filters.Subscriber("/panoptic7/labels_map", Image)
        self.pan8cb = message_filters.Subscriber("/panoptic8/labels_map", Image)
        self.pan9cb = message_filters.Subscriber("/panoptic9/labels_map", Image)
        self.pan10cb = message_filters.Subscriber("/panoptic10/labels_map", Image)
        self.pan11cb = message_filters.Subscriber("/panoptic11/labels_map", Image)
        self.pan12cb = message_filters.Subscriber("/panoptic12/labels_map", Image)
        self.pan13cb = message_filters.Subscriber("/panoptic13/labels_map", Image)
        self.pan14cb = message_filters.Subscriber("/panoptic14/labels_map", Image)
        self.pan15cb = message_filters.Subscriber("/panoptic15/labels_map", Image)
        self.pan16cb = message_filters.Subscriber("/panoptic16/labels_map", Image)

        self.pub = rospy.Publisher("/colored_points", PointCloud2, queue_size=10)
        cb = message_filters.ApproximateTimeSynchronizer([self.lidarcb, self.pan1cb,self.pan2cb,self.pan3cb,self.pan4cb,self.pan5cb,self.pan6cb,self.pan7cb,self.pan8cb, self.pan9cb,self.pan10cb,self.pan11cb,self.pan12cb,self.pan13cb,self.pan14cb, self.pan15cb,self.pan16cb], 1, 0.1, allow_headerless=True)
        cb.registerCallback(self.pcl2points)
        try:
            os.chdir(directory)
            print("Current working directory: {0}".format(os.getcwd()))
        except FileNotFoundError:
            print("Directory: {0} does not exist".format(directory))
        except NotADirectoryError:
            print("{0} is not a directory".format(directory))
        except PermissionError:
            print("You do not have permissions to change to {0}".format(directory))
        print("Starting Labelling node")
        rospy.spin()

    def update_camera_info(self,camera_info):
        self.camera.fromCameraInfo(camera_info)
        self.width = camera_info.width
        self.height = camera_info.height # parse this from cam info
        print("Camera Updated\n",camera_info)

    def IM_callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
            self.width, self.height, channels = cv_image.shape 
            self.last_image = np.array(cv_image)
        except CvBridgeError as error:
            print(error)
    
    def pcl2points(self, cloud, im1,im2,im3,im4,im5,im6,im7,im8, im9, im10, im11, im12, im13, im14, im15, im16):
        print("LISTENING")
        start = time.time()
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

        with multiprocessing.Manager() as manager:
            self.pcl_colorized = manager.list()

            processes = [
                         multiprocessing.Process(target=self.pc, args=(self.raw_cloud, self.transform_link_1, self.last_image)),
                         multiprocessing.Process(target=self.pc, args=(self.raw_cloud, self.transform_link_2, self.last_image_2)),
                         multiprocessing.Process(target=self.pc, args=(self.raw_cloud, self.transform_link_3, self.last_image_3)),
                         multiprocessing.Process(target=self.pc, args=(self.raw_cloud, self.transform_link_4, self.last_image_4)),
                         multiprocessing.Process(target=self.pc, args=(self.raw_cloud, self.transform_link_5, self.last_image_5)),
                         multiprocessing.Process(target=self.pc, args=(self.raw_cloud, self.transform_link_6, self.last_image_6)),
                         multiprocessing.Process(target=self.pc, args=(self.raw_cloud, self.transform_link_7, self.last_image_7)),
                         multiprocessing.Process(target=self.pc, args=(self.raw_cloud, self.transform_link_8, self.last_image_8)),
                         multiprocessing.Process(target=self.pc, args=(self.raw_cloud, self.transform_link_9, self.last_image_9)),
                          multiprocessing.Process(target=self.pc, args=(self.raw_cloud, self.transform_link_10, self.last_image_10)),
                          multiprocessing.Process(target=self.pc, args=(self.raw_cloud, self.transform_link_11, self.last_image_11)),
                          multiprocessing.Process(target=self.pc, args=(self.raw_cloud, self.transform_link_12, self.last_image_12)),
                          multiprocessing.Process(target=self.pc, args=(self.raw_cloud, self.transform_link_13, self.last_image_13)),
                          multiprocessing.Process(target=self.pc, args=(self.raw_cloud, self.transform_link_14, self.last_image_14)),
                          multiprocessing.Process(target=self.pc, args=(self.raw_cloud, self.transform_link_15, self.last_image_15)),
                          multiprocessing.Process(target=self.pc, args=(self.raw_cloud, self.transform_link_16, self.last_image_16)),
                        ]
            #start processes
            for process in processes:
                process.start()
            # wait for finish
            for process in processes:
                process.join()        
           # publish pcl
            pcp = pc2.create_cloud(self.header, self.fields, self.pcl_colorized)
            if len(self.pcl_colorized) > 100 and self.save_dataset:
                self.write_cloud(pcp)
            self.publish_pcl(pcp)
            end = time.time()
            print(end - start)



    def pc(self, cloud, transform, image, overlay=False): #separate thread?
        start = time.time()
        for p in cloud:
            transformed_p = tf2_kdl.transform_to_kdl(transform) * PyKDL.Vector(p[0], p[1], p[2]) # transform point based on tf transform
            if transformed_p[0] > 0:
                projected_p = self.project_point(transformed_p)
                try:
                    h, w= round(projected_p[0]), round(projected_p[1])
                    if w > 0 and h > 0:
                        if w < self.width :
                            if h < self.height:
                                if image[w,h][2] != 0:
                                    self.pcl_colorized.append(list(p) + [image[w,h][0]] + [image[w,h][2]])
                                #if overlay:
                                #    image[w,h] = self.red
                except:
                    pass
        #if overlay:
        #   cv2.imshow("Image window", image)
        #   cv2.imwrite("result.png",image)
        #   cv2.waitKey(1)
        end = time.time()
        #print("time  ",start-end )

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
