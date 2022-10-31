import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point, Twist

import sys

import numpy as np
import cv2
from cv_bridge import CvBridge

# class MinimalVideoSubscriber(Node):
class VideoProcessing(Node):

	def __init__(self):		
		# Creates the node.
		# super().__init__('minimal_video_subscriber')
		super().__init__('video_processing')

	
		#Declare that the minimal_video_subscriber node is subcribing to the /camera/image/compressed topic.
		self._video_subscriber = self.create_subscription(
				CompressedImage,
				'/camera/image/compressed',
				self._image_callback,
				1)
		self._video_subscriber # Prevents unused variable warning.

		self.publisher_object_pos = self.create_publisher(Point, '/object_pos', 10)

	def _image_callback(self, CompressedImage):	
		# The "CompressedImage" is transformed to a color image in BGR space and is store in "_imgBGR"
		self._imgBGR = CvBridge().compressed_imgmsg_to_cv2(CompressedImage, "bgr8")


		# black_lower = (0,0,0)
		# black_upper = (360,255,50)

		# pink_lower = (161,155,84)
		# pink_upper = (179,255,255)

		pink_lower = (351,100,86)
		pink_upper = (351,29,100)

		yellow_lower = (20,100,100)
		yellow_upper = (30,255,255)

		orange_lower = (22,62,111)
		orange_upper = (56,255,255)


		blurred_frame = cv2.GaussianBlur(self._imgBGR, (11, 11), 0)
		hsv = cv2.cvtColor(blurred_frame, cv2.COLOR_BGR2HSV)

		#Create mask for specific color
		kernel = np.ones((5, 5), np.uint8)

		mask = cv2.inRange(hsv, yellow_lower, yellow_upper)
		mask = cv2.erode(mask, kernel, iterations=2)
		mask = cv2.dilate(mask, kernel, iterations=2)

		
		#Find Contours
		contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

		if (len(contours) > 0):

			# find the largest contour in the mask, then use
			# it to compute the minimum enclosing circle and centroid
			max_contour = max(contours, key=cv2.contourArea)
			((x, y), radius) = cv2.minEnclosingCircle(max_contour)

		else:
			x = float('inf')
			y = float('inf')
		
		self.publish_position(x, y, 0.0)		



	def publish_position(self, x, y, z):
		point = Point()
		point.x = x
		point.y = y
		point.z = z

		self.publisher_object_pos.publish(point)		 


	def get_image(self):
		return self._imgBGR

	def get_user_input(self):
		return self._user_input

	def show_image(self, img):
		cv2.imshow(self._titleOriginal, img)
		# Cause a slight delay so image is displayed
		self._user_input=cv2.waitKey(10) #Use OpenCV keystroke grabber for delay.



def main():
	rclpy.init() #init routine needed for ROS2.
	video_subscriber = VideoProcessing() #Create class object to be used.
	while rclpy.ok():
		rclpy.spin_once(video_subscriber) # Trigger callback processing.

	#Clean up and shutdown.
	video_subscriber.destroy_node()  
	rclpy.shutdown()

if __name__ == '__main__':
	main()
