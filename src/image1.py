#!/usr/bin/env python3
import math

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError


class image_converter:

  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('image_processing', anonymous=True)
    # initialize a publisher to send images from camera1 to a topic named image_topic1
    self.image_pub1 = rospy.Publisher("image_topic1",Image, queue_size = 1)
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw",Image,self.callback1)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()
    # joints
    self.robot_joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
    self.robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
    self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
    self.robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)
    # initialize the start time
    self.first_time = rospy.get_time()
  # Recieve data from camera 1, process it, and publish
  def callback1(self,data):
    # Recieve the image
    try:
      self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    
    # Uncomment if you want to save the image
    #cv2.imwrite('image_copy.png', cv_image)

    # make the joint move
    time = rospy.get_time() - self.first_time
    joint1 = Float64()
    joint1.data = np.pi * np.sin(np.pi / 15. * time)
    joint1.data = 0.
    joint2 = Float64()
    joint2.data = np.pi / 2 * np.sin(np.pi / 15. * time)
    # joint2.data = 0
    joint3 = Float64()
    joint3.data = np.pi / 2 * np.sin(np.pi / 18. * time)
    # joint3.data = 1
    joint4 = Float64()
    joint4.data = np.pi / 2 * np.sin(np.pi / 20. * time)  # without direction
    # joint4.data = 1

    # ==========test==============
    theta0 = math.pi / 2  # ja1
    T_0_g = self.get_T(alpha_pre=0, a_pre=0, d_i=0, theta_i=theta0)
    theta1 = joint2.data + math.pi / 2
    T_1_0 = self.get_T(alpha_pre=math.pi / 2, a_pre=0, theta_i=theta1, d_i=0)
    theta2 = joint3.data
    T_2_1 = self.get_T(alpha_pre=math.pi / 2, a_pre=0, d_i=0, theta_i=theta2)
    link3 = 3.5
    theta3 = joint4.data
    T_3_2 = self.get_T(alpha_pre=-math.pi / 2, a_pre=link3, d_i=0, theta_i=theta3)
    link1 = 2.5
    link3 = 3
    a = np.array([link3, 0, 0, 1])  # slide on z axis //zhongduan
    a = T_3_2.dot(a)
    a = T_2_1.dot(a)
    # on axis 0
    a = T_1_0.dot(a) + np.array([0, 0, link1, 0])
    # print(a)
    # on axis global
    on_global = (T_0_g.dot(a))

    # =================std test
    a = np.array([0, 0, 0, 1])
    theta4 = joint4.data
    a4 = 3
    T_4_3 = self.get_T_std(alpha=0, a=a4, d=0, theta=theta4)
    a = T_4_3.dot(a)
    theta3 = joint3.data
    a3 = 3.5
    T_3_2 = self.get_T_std(alpha=-math.pi / 2, a=a3, d=0, theta=theta3)
    a = T_3_2.dot(a)

    theta2 = joint2.data + math.pi / 2
    a2 = 0
    T_2_1 = self.get_T_std(alpha=math.pi / 2, a=a2, d=0, theta=theta2)
    a = T_2_1.dot(a)
    print(a)
    # a = np.array([1, 0, 0, 1])
    theta1 = joint1.data + math.pi / 2
    a1 = 2.5
    T_1_0 = self.get_T_std(alpha=math.pi / 2, a=0, d=0, theta=theta1)
    a = T_1_0.dot(a)
    a = a + np.array([0, 0, a1, 0])
    on_global_std = a
    print("----")
    print(on_global)
    print(on_global_std)
    im1=cv2.imshow('window1', self.cv_image1)
    cv2.waitKey(1)
    # Publish the results
    try: 
      self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image1, "bgr8"))
      self.robot_joint1_pub.publish(joint1)
      self.robot_joint2_pub.publish(joint2)
      self.robot_joint3_pub.publish(joint3)
      self.robot_joint4_pub.publish(joint4)
    except CvBridgeError as e:
      print(e)

  def get_T (self, alpha_pre, theta_i, d_i, a_pre):
    return np.array([
      [math.cos(theta_i), -math.sin(theta_i), 0, a_pre],
      [math.sin(theta_i) * math.cos(alpha_pre), math.cos(theta_i) * math.cos(alpha_pre), -math.sin(alpha_pre),
       -math.sin(alpha_pre) * d_i],
      [math.sin(theta_i) * math.sin(alpha_pre), math.cos(theta_i) * math.sin(alpha_pre), math.cos(alpha_pre),
       math.cos(alpha_pre) * d_i],
      [0., 0., 0, 1.]
    ])
  def get_T_std(self, alpha, theta, d, a):
    return np.array([
      [math.cos(theta), -math.sin(theta) * math.cos(alpha), math.sin(theta) * math.sin(alpha), a * math.cos(theta)],
      [math.sin(theta), math.cos(theta) * math.cos(alpha), -math.cos(theta) * math.sin(alpha), a * math.sin(theta)],
      [0., math.sin(alpha), math.cos(alpha), d],
      [0, 0, 0, 1]
    ])
# call the class
def main(args):
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)


