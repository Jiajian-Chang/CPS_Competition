import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
import numpy as np
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from rclpy.qos import HistoryPolicy, QoSProfile
import math

depth_topic = "/camera/aligned_depth_to_color/image_raw"
detected_bbox_topic = "yolo_detector/detected_bounding_boxes"
fx = 302.1692199707031
cx = 214.1420135498047
fy = 302.1572570800781
cy = 120.7688980102539
class lidar_filter(Node):
    def __init__(self):
        super().__init__("lidar_filter_node")
        self.depth_img_received = False
        # subscriber
        self.br = CvBridge()
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10)        
        self.depth_sub = self.create_subscription(Image,depth_topic,self.depth_callback,qos_profile=qos)
        self.detected_bbox_sub = self.create_subscription(Detection2DArray,detected_bbox_topic,self.detected_bbox_callback,qos_profile=qos)        
       # publisher
        self.angle_pub = self.create_publisher(Float32MultiArray,"filter_angle",  10)
    def depth_callback(self, msg):
        self.depth_img = self.br.imgmsg_to_cv2(msg, "passthrough")
        self.depth_img_received = True

    def detected_bbox_callback(self, msg):
        filter_msg = Float32MultiArray()
        if msg.detections and self.depth_img_received:
            self.detected_bbox_array = msg.detections            
            filter_msg.data = self.filter()
        self.angle_pub.publish(filter_msg)


    def filter(self):
         filter_angle = []
         for detected_bbox in self.detected_bbox_array:
              # transform the x y to arrary index
              u = detected_bbox.bbox.center.position.x
              v = detected_bbox.bbox.center.position.y
              size_x = detected_bbox.bbox.size_x
              bbox_depth = self.depth_img[int(v)-10:int(v)+10, int(u)-10:int(u)+10]              
              depth = np.nanmedian(bbox_depth)
              # pixel to camera frame(Front-Right-Up)
              x = depth
              y = (u - cx)/fx*depth
              z = -(v - cy)/fy*depth
              transformed_size_x = size_x/fx*depth*1.5
              # get the filter start angle and end angle
              start_angle = math.atan2(depth, (y+0.5*transformed_size_x))*180/math.pi
              end_angle = math.atan2(depth, (y-0.5*transformed_size_x))*180/math.pi
              filter_angle.append(start_angle)
              filter_angle.append(end_angle)
              #print(x,y,z)
              #print(bbox_depth)
         #print(filter_angle)
         return filter_angle


def main():
	rclpy.init()	
	rclpy.spin(lidar_filter())

if __name__=="__main__":
	main()