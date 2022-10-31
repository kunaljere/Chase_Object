import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import CompressedImage, LaserScan
from geometry_msgs.msg import Point, Twist

import sys
import time
import math
import numpy as np
import cv2
from cv_bridge import CvBridge

class ChaseObject(Node):

    def __init__(self):

        # Creates the node.
        super().__init__('object_range')

        self.object_angle = 0
        self.object_dist = 0

        self.goal_distance = 0.4
        self.goal_angle = 0

        self.prev_dist = 0.0
        self.prev_ang = 0.0

        self.max_linear_velocity = 0.8
        self.max_angular_velocity = 0.8

        self.min_linear_velocity = 0.2
        self.min_angular_velocity = 0.16

        self.kp_linear = 0.4
        self.kp_angular = 0.6

        self.kd_linear = 0.2 # derivate, future
        self.kd_angular = 0.03 # derivate, future 
        self.dt = 0.01

        self.angle_sub = self.create_subscription(Float32, '/angle_pos', self.angle_callback, 1)
        self.angle_sub

        self.dist_sub = self.create_subscription(Float32, '/dist', self.dist_callback, 1)
        self.dist_sub

        self.move_robot_pub = self.create_publisher(Twist, '/cmd_vel', 5)

        self.start_time = time.time()


    def angle_callback(self, msg):
        self.object_angle = msg.data

    def dist_callback(self, msg):
        self.object_dist = msg.data
        self.follow_object()

    def follow_object(self):
        # print('Angle: ', str(self.object_angle))
        # print('Dist: ', str(self.object_dist))
        move = Twist()

        deriv_dist_err =  (self.object_dist - self.prev_dist)/self.dt
        deriv_ang_err = (self.object_angle - self.prev_ang)/self.dt

        # prev_error_angle = 0
        # prev_error_dist = 0
        

        if self.object_dist == float('inf') and self.object_angle == float('inf'):
            move.linear.x = 0.0
            move.angular.z = 0.0

            print('Angular Velocity: ', move.angular.z)
            print('Linear Velocity: ', move.linear.x)
            print()   

        else:

            error_distance = self.object_dist - self.goal_distance
            error_angle = self.object_angle

            print('Error Distance: ', error_distance)
            print('Error Angle: ', error_angle)

            w_distance = self.kp_linear * error_distance #+ self.kd_linear * deriv_dist_err
            w_angular = self.kp_angular * error_angle/31.1 #+ self.kd_angular * deriv_ang_err

            

            threshold_linear = 0.05
            threshold_angular = 0.05

            bool_dist = abs(error_distance) < threshold_linear
            bool_ang = abs(error_angle) < threshold_angular

            
            print("test ",bool_dist, bool_ang)
            
            #limits on simultaneous large linear and angular movements
            if (abs(error_distance) < threshold_linear): # and abs(error_angle) > threshold_angular):
                print('Stop Linear')
                w_distance = 0.0

                # if abs(w_angular) > self.max_angular_velocity:
                #     if w_angular < 0:
                #         w_angular = -1*self.max_angular_velocity
                #     else:
                #         w_angular = self.max_angular_velocity

                # elif abs(w_angular) < self.min_angular_velocity:
                #     if w_angular < 0:
                #         w_angular = -1*self.min_angular_velocity
                #     else:
                #         w_angular = self.min_angular_velocity

            
            if (abs(error_angle) < threshold_angular): # and abs(error_distance) > threshold_linear):
                w_angular = 0.0
                print('Stop Angular')

                # if abs(w_distance) > self.max_linear_velocity:
                #     if w_distance < 0:
                #         w_distance = -1*self.max_linear_velocity
                #     else:
                #         w_distance = self.max_linear_velocity

                # elif abs(w_distance) < self.min_linear_velocity:
                #     if w_distance < 0:
                #         w_distance = -1*self.min_linear_velocity
                #     else:
                #         w_distance = self.min_linear_velocity

            if (abs(error_distance) > threshold_linear and abs(error_angle) > threshold_angular):
                print('MOVE')
                if abs(w_distance) > self.max_linear_velocity:
                    if w_distance < 0:
                        w_distance = -1*self.max_linear_velocity
                    else:
                        w_distance = self.max_linear_velocity

                elif abs(w_distance) < self.min_linear_velocity:
                    if w_distance < 0:
                        w_distance = -1*self.min_linear_velocity
                    else:
                        w_distance = self.min_linear_velocity   



                if abs(w_angular) > self.max_angular_velocity:
                    if w_angular < 0:
                        w_angular = -1*self.max_angular_velocity
                    else:
                        w_angular = self.max_angular_velocity

                elif abs(w_angular) < self.min_angular_velocity:
                    if w_angular < 0:
                        w_angular = -1*self.min_angular_velocity
                    else:
                        w_angular = self.min_angular_velocity

                # if error_distance < 0:
                #     w_angular = -1*w_angular
                            

            elif (abs(error_angle) < threshold_angular and abs(error_distance) < threshold_linear): 
                w_distance = 0.0
                w_angular = 0.0

            move.linear.x = w_distance
            move.angular.z = w_angular

            self.prev_dist = self.object_dist
            self.prev_ang = self.object_angle


            print('Angular Velocity: ', w_angular)
            print('Linear Velocity: ', w_distance)
            print()        
        

        self.move_robot_pub.publish(move)

        



def main():
	rclpy.init() #init routine needed for ROS2.
	# video_subscriber = MinimalVideoSubscriber() #Create class object to be used.
	chase_object = ChaseObject() #Create class object to be used.
	
	rclpy.spin(chase_object) # Trigger callback processing.		

	#Clean up and shutdown.
	chase_object.destroy_node()  
	rclpy.shutdown()


if __name__ == '__main__':
	main()

