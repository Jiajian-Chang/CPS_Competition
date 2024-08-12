#!/usr/bin/env python

import rclpy
import numpy as np
from yolo_ros2.yolo_detector import *


def main():
	rclpy.init()
	yolo_detector()
	rclpy.spin()

if __name__=="__main__":
	main()
	
