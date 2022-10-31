import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import CompressedImage, LaserScan
from geometry_msgs.msg import Point, Twist
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

import sys

import numpy as np
import cv2
from cv_bridge import CvBridge

class ObjectRange(Node):

    def __init__(self):

        # Creates the node.
        super().__init__('object_range')

        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE

        self.fov = 62.2
        self.total_xframe = 320

        self.pos_x = 0
        self.pos_y = 0

        self.angle_pos = Float32()
        self.dist = Float32()


        self.object_pos = self.create_subscription(Point, '/object_pos', self.get_pixel_pos, 1)
        self.object_pos

        self.lidar_data = self.create_subscription(LaserScan, '/scan', self.get_lidar_data, qos_profile)

        self.publish_angle_pos = self.create_publisher(Float32, '/angle_pos', 5)
        self.publish_distance = self.create_publisher(Float32, '/dist', 5)

        # timer_period = 0.5
        # self.timer = self.create_timer(timer_period, self.rotate_robot)
        # self.i = 0

    

    def get_lidar_data(self, msg):
        ranges = msg.ranges #use this list
        
        size_ranges = len(ranges)
        ratio = size_ranges/360

        print('Angle Pos: ', self.angle_pos.data)

        if self.angle_pos.data == float('inf'):
            self.dist.data = float('inf')
        elif self.angle_pos.data != float('inf'):        
            idx = round(ratio*self.angle_pos.data)
            self.dist.data = ranges[idx]

        self.publish_distance.publish(self.dist)


    def get_pixel_pos(self, msg):
        self.pos_x = msg.x
        self.pos_y = msg.y

        print('X position: ', self.pos_x)
        print('Y position: ', self.pos_y)
        print()
        
        if self.pos_x == float('inf') and self.pos_y == float('inf'):
            self.angle_pos.data = float('inf')
        else:
            self.angle_pos.data = (self.fov * self.pos_x/self.total_xframe) - self.fov/2

        # print('Angle: ', self.angle_pos.data)

        self.publish_angle_pos.publish(self.angle_pos)



def main():
	rclpy.init() #init routine needed for ROS2.
	# video_subscriber = MinimalVideoSubscriber() #Create class object to be used.
	object_range = ObjectRange() #Create class object to be used.
	
	rclpy.spin(object_range) # Trigger callback processing.		

	#Clean up and shutdown.
	object_range.destroy_node()  
	rclpy.shutdown()


if __name__ == '__main__':
	main()