import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import qos_profile_system_default
import math
import numpy as np
laserScan_topic = "front/scan"
filter_angle = "filter_angle"
class lidar_relay(Node):

    def __init__(self):
        super().__init__('lidar_relay_node')
        qos = qos_profile_sensor_data
        self.laserScan_sub = self.create_subscription(LaserScan,laserScan_topic,self.laserScan_callback,qos_profile=qos)
        self.filterAngle_sub = self.create_subscription(Float32MultiArray,filter_angle,self.filterAngle_callback,qos_profile=qos_profile_system_default)
       # publisher
        self.laserScan_pub = self.create_publisher(LaserScan,"scan",  10)
        self.laserScan_received = False

    def laserScan_callback(self, msg):
        self.laserScan = msg
        self.laserScan_ranges = np.asarray(msg.ranges)
        #print(self.laserScan_ranges)
        self.angle_min = msg.angle_min/math.pi*180+180
        self.angle_max = msg.angle_max/math.pi*180+180
        self.angle_increment = msg.angle_increment/math.pi*180
        self.laserScan_received = True

    def filterAngle_callback(self, msg):
        if self.laserScan_received:
            filter_angle = msg.data
            for i in range(0,len(filter_angle),2):
                if i+1<=len(filter_angle) and not math.isnan(filter_angle[i]) and not math.isnan(filter_angle[i+1]):
                    #print(filter_angle[i],filter_angle[i+1])
                    # 30 is the offset of the lidar start angle check lidar data sheet
                    start_index = round((filter_angle[i]+self.angle_min)/self.angle_increment)
                    end_index = round((filter_angle[i+1]+self.angle_min)/self.angle_increment)
                    #print("start_index is", start_index)
                    #print("end index is", end_index)
                    #print("angle min is", self.angle_min)
                    #print("angle max is", self.angle_max)
                    #print("self.laserScan_ranges", len(self.laserScan_ranges))
                    #print(start_index>=0 and end_index<=len(self.laserScan_ranges))
                    if start_index>=0 and end_index<=len(self.laserScan_ranges):
                        self.laserScan_ranges[start_index:end_index] = 0
                        #print(self.laserScan_ranges[start_index:end_index])
                        self.laserScan.ranges = self.laserScan_ranges.tolist() 
                        #print(self.laserScan.ranges[start_index:end_index])
            
            self.laserScan_pub.publish(self.laserScan)  
        


def main(args=None):
    rclpy.init(args=args)

    rclpy.spin(lidar_relay())

    rclpy.shutdown()


if __name__ == '__main__':
    main()