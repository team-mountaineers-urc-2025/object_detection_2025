# Subscribes to the yolo node and publishes the estimated position of the object

import pickle
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from robot_interfaces.msg import ObjectPoint
from robot_interfaces.msg import ImageMetadata
from sensor_msgs.msg import Image
import math
from ultralytics import YOLO
import torch
import os
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np



class DistanceNode(Node):
   
    class_dict = [100, 0.36, 0.36, 0.215]  # Misc, Misc, Keyboard Height, Bottle Height

    
    def __init__(self):
        super().__init__(f'distance_node')

        # Topics 
        self.declare_parameter('global_origin_frame', 'base_link')
        self.declare_parameter('camera_metadata', '/CM_cam_meta')
        self.declare_parameter('cam_images', 'image_topic')
        self.declare_parameter('cam_object', '/cam_object_pose')

        # Metadata Parameters
        self.focal_length = 3.67
        self.sensor_height = 1.0
        self.image_height = 1
        self.image_width = 1


        #Setup Camera Info Subscriber-Republisher
        self.cam_images = self.get_parameter('cam_images').get_parameter_value().string_value
        self.create_subscription(
            Image,
            self.cam_images,
            self.cam_images_callback,
            10
        )

        # Setup Camera Metadata Subscriber
        self.camera_metadata= self.get_parameter('camera_metadata').get_parameter_value().string_value
        self.create_subscription(
            ImageMetadata,
            self.camera_metadata,
            self.cam_metadata_callback,
            10
        )

        # Publisher for Objects Location Relative to Camera
        self.distance_publisher = self.create_publisher(ObjectPoint, '/autonomy/cam_object', 10)
    

       
    def cam_metadata_callback(self, msg):
        self.focal_length = msg.foc_len_mm 
        self.sensor_height = msg.sensor_height
        self.image_height = msg.im_height
        self.image_width = msg.im_width



    def cam_images_callback(self, msg):

        # Execute the YOLO model
        device = torch.device("cpu")

        config_file = os.path.join('./launches/config/', 'best.pt')
        model = YOLO(config_file)

      
        # Convert ROS image message to OpenCV (BGR format)
        try:
            cv_image = CvBridge().imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'Error converting image: {str(e)}')
            return


        predict = model(cv_image, device=device, imgsz = 1920 )    

        for p in predict:
            # Get info on prediction box for object
            boxes = p.boxes

            for box in boxes:
                # Get Bounds for bounding box
                x1, y1, x2, y2 = box.xyxy[0]
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

                # Get  object height and width for real and image
                box_width = x2-x1
                box_height = y2-y1

                img_object_height = box_width if box_width > box_height else box_height 
                
             
                object_class = int(box.cls.item())
                self.get_logger().info(f"Object  = {object_class}")
                real_object_height = self.class_dict[ object_class]


                diagonal_resolution = math.sqrt(float(self.image_width)**2 + float(self.image_height)**2)
                self.focal_length = diagonal_resolution / (2 * math.tan(math.radians(78.0) / 2))

                # Calculate distance estimation
                distance = (real_object_height * self.focal_length) / float(img_object_height)
                self.get_logger().info(f"Distance  = {distance}")

                # Get horizontal center of box. 
                box_horizontal_center = float(x2+x1) / 2
                box_vertical_center = float(y2+y1) / 2

                img_center = [float(self.image_width)/2, float(self.image_height)/2]

                #Calculate angle from camera
                horizontal_angle = math.radians(((box_horizontal_center - img_center[0])/(img_center[0]))*(70.42/2)) # Horizonal FOV is 70.42 degrees
                vertical_angle =  math.radians(((box_vertical_center - img_center[1])/(img_center[1]))*(43.3/2))
                
                #Apply to object distance 
                object_pose = ObjectPoint()
               

                object_pose.point.point.x =  distance * math.cos(vertical_angle) * math.sin(horizontal_angle)   # Up and Down 
                object_pose.point.point.y = - distance * math.sin(vertical_angle)                           # Left and right of camera
                object_pose.point.point.z = distance * math.cos(vertical_angle) * math.cos(horizontal_angle)    # Forward and Backward

                #Set object ID to either hammer or bottle 
                if(object_class == 3):        #   bottle
                    object_pose.object_id = 7
                elif (object_class == 1):       #hammer
                  object_pose.object_id = 8


                #Set Frame ID
                object_pose.point.header.frame_id = msg.header.frame_id
                object_pose.point.header.stamp = msg.header.stamp

                self.get_logger().info(f"CAMERA  = {msg.header.frame_id}")
                self.get_logger().info(f"Object Detected: Y = {object_pose.point.point.y}   Z = {object_pose.point.point.z}")
                self.get_logger().info(f"Object Class = {box.cls[0]}")

                #Publish the pose of the object from the camera. To be used by position estimator
                self.distance_publisher.publish(object_pose)




def main(args=None):
    rclpy.init(args=args)
    
    subscriber = DistanceNode()

    try:
        rclpy.spin(subscriber)
    except KeyboardInterrupt:
        subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
