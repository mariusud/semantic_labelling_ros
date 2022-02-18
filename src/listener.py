#!/usr/bin/env python3.8
from this import d
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
import tf2_kdl
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point
import PyKDL

class RosNode:
    def __init__(self):
        rospy.init_node('labelling_node')
        self.camera = PinholeCameraModel()
        self.bridge = CvBridge()

        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer)
        try:
            self.latest_trans = self.tfBuffer.lookup_transform('vehicle_blue/semantic_camera_link','vehicle_blue',rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass
        self._init_pcl()

        self.stairs_color = 50
        a = 50
        self.stairs_rgb = struct.unpack('I', struct.pack('BBBB', self.stairs_color, self.stairs_color, self.stairs_color, a))[0]

        camera_info = rospy.wait_for_message("/semantic/camera_info", CameraInfo)
        self.update_camera(camera_info)

        rospy.Subscriber("/lidar/points", PointCloud2, self.pc2_callback, queue_size=1)
        rospy.Subscriber("/semantic/labels_map", Image, self.IM_callback)
        self.pub = rospy.Publisher("/colored_points", PointCloud2, queue_size=10)
        self.spin()


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
        self.pcl2points(data)
    
    def IM_callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
            self.width, self.height, channels = cv_image.shape 
            self.last_image = np.copy(cv_image)
        except CvBridgeError as error:
            print(error)
    
    def pcl2points(self,cloud):
        #overlay = True
        #image = np.copy(self.last_image)
        #red = [0,0,255]
        self.gen = pc2.read_points(cloud, skip_nans=True, field_names=("x", "y", "z"))
        segm_pcl = []
        try:
            self.latest_transform = self.tfBuffer.lookup_transform('vehicle_blue/semantic_camera_link','vehicle_blue',rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass
        for p in self.gen:
            if not math.isinf(p[0]): 
                p = tf2_kdl.transform_to_kdl(self.latest_transform) * PyKDL.Vector(p[0], p[1], p[2])
                projected_p = self.project_point(p)
                try:
                    h, w= int(projected_p[0]), int(projected_p[1])
                    if w < self.width and w > 0:
                        if h < self.height and h > 0:
                            if self.last_image[w,h][0] == self.stairs_color:
                                segm_pcl.append(p)
                            #if overlay:
                            #    image[w,h] = red
                except:
                    pass
        colored_pcl = [list(tup)+[self.stairs_rgb] for tup in segm_pcl]
        pcp = pc2.create_cloud(self.header, self.fields, colored_pcl)
        self.publish_pcl(pcp)
        #if overlay:
        #    cv2.imshow("Image window", image)
        #    cv2.imwrite("result.png",image)
        #    cv2.waitKey(1)

    def publish_pcl(self,pcp):
            pcp.header.stamp = rospy.Time.now()
            rospy.loginfo("publishing points")
            self.pub.publish(pcp)

    def _init_pcl(self):
        self.fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1),
                  # PointField('rgb', 12, PointField.UINT32, 1),
                  PointField('rgba', 12, PointField.UINT32, 1),
                  ]
        
        self.header = Header()
        self.header.frame_id = "vehicle_blue/lidar_link/gpu_lidar"

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

        stairs = [50,50,50]
        #segmented_pcl = np.zeros([320,240,3],dtype=np.uint8)
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

    def spin(self):
        rospy.spin()



if __name__ == '__main__':
    RosNode()