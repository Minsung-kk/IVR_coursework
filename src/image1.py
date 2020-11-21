#!/usr/bin/env python3
import math
import os

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError
import cv_utils

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
    # the message from x camera
    self.x_came_pos_pub = rospy.Publisher("x_cam_pos", Float64MultiArray, queue_size=10)
    self.fk_end_pos = rospy.Publisher("fk_end_pos", Float64MultiArray, queue_size=10)
    self.template = cv2.imread("image_crop.png", 0)
    self.time = 1
    if self.template is None:
      print("load the templete Failde, \n Please check the image_crop.png is in"+os.getcwd())
      exit(1)
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
    joint1 = Float64()
    joint2 = Float64()
    joint3 = Float64()
    joint4 = Float64()

    time = rospy.get_time() - self.first_time
    # joint1.data = np.pi * np.sin(np.pi / 15. * time)
    # joint1.data = 0
    # joint2.data = np.pi / 2 * np.sin(np.pi / 15. * time)
    # joint2.data = 0
    # joint3.data = np.pi / 2 * np.sin(np.pi / 18. * time)
    # joint3.data = 0
    # joint4.data = np.pi / 2 * np.sin(np.pi / 20. * time)  # without direction
    # joint4.data = 0



    on_global_std = Float64MultiArray()
    on_global_std.data = self.calcu_fk_end_pos(joint1.data, joint2.data, joint3.data, joint4.data)
    start_point = np.array([0,0,6.5])
    desire_goal = np.array([0,5,0])
    dt = 5.
    v_point = (desire_goal-start_point)/dt
    J, Jp, Jo = self.calcu_jocabian(0,0,0,0)
    J = Jp
    assert J.shape == (3,4)
    J_inv= np.linalg.pinv(J)
    assert J_inv.shape ==(4,3)
    q_speed = J_inv.dot(v_point.T)
    delt_q = q_speed*dt
    joint1.data = delt_q[0] + 0
    joint2.data = delt_q[1] + 0
    joint3.data = delt_q[2] + 0
    joint4.data = delt_q[3] + 0
    print(delt_q)

    # =========find circles========
    x_cam_pos = self.get_cirlces()

    im1=cv2.imshow('window1', self.cv_image1)
    cv2.waitKey(1)

    # Publish the results
    try: 
      self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image1, "bgr8"))
      self.robot_joint1_pub.publish(joint1)
      self.robot_joint2_pub.publish(joint2)
      self.robot_joint3_pub.publish(joint3)
      self.robot_joint4_pub.publish(joint4)
      self.x_came_pos_pub.publish(x_cam_pos)
      self.fk_end_pos.publish(on_global_std)
      exit(0)
    except CvBridgeError as e:
      print(e)


  def get_T_std(self, alpha, theta, d, a):
    """
    get the FK matrix
    :param alpha:
    :param theta:
    :param d:
    :param a:
    :return:
    """
    return np.array([
      [math.cos(theta), -math.sin(theta) * math.cos(alpha), math.sin(theta) * math.sin(alpha), a * math.cos(theta)],
      [math.sin(theta), math.cos(theta) * math.cos(alpha), -math.cos(theta) * math.sin(alpha), a * math.sin(theta)],
      [0., math.sin(alpha), math.cos(alpha), d],
      [0, 0, 0, 1]
    ])
  def get_cirlces(self):
    # x cam position
    x_cam_pos = Float64MultiArray()
    yellow_pos = cv_utils.detect_yellow(self.cv_image1)
    blue_pos = cv_utils.detect_blue(self.cv_image1)
    green_pos = cv_utils.detect_green(self.cv_image1)
    red_pos = cv_utils.detect_red(self.cv_image1)
    mask = cv_utils.detect_orange(self.cv_image1)
    orange_pos = cv_utils.find_target(mask, self.template)
    x_cam_pos.data = np.array(
      [yellow_pos[0], yellow_pos[1], blue_pos[0], blue_pos[1], green_pos[0], green_pos[1], red_pos[0], red_pos[1],
       orange_pos[0], orange_pos[1]])
    return x_cam_pos

  def calcu_fk_end_pos(self,ja1,ja2,ja3,ja4):
    a = np.array([0, 0, 0, 1])
    theta4 = ja4
    a4 = 3
    T_4_3 = self.get_T_std(alpha=0, a=a4, d=0, theta=theta4)
    a = T_4_3.dot(a)
    theta3 = ja3
    a3 = 3.5
    T_3_2 = self.get_T_std(alpha=-math.pi / 2, a=a3, d=0, theta=theta3)
    a = T_3_2.dot(a)

    theta2 = ja2 + math.pi / 2
    a2 = 0
    T_2_1 = self.get_T_std(alpha=math.pi / 2, a=a2, d=0, theta=theta2)
    a = T_2_1.dot(a)
    # a = np.array([1, 0, 0, 1])
    theta1 = ja1 + math.pi / 2
    a1 = 2.5
    T_1_0 = self.get_T_std(alpha=math.pi / 2, a=0, d=0, theta=theta1)
    a = T_1_0.dot(a)
    return a + np.array([0, 0, a1, 0])

  def calcu_jocabian(self,ja1, ja2, ja3, ja4):
    theta1 = ja1 + math.pi / 2
    theta2 = ja2 + math.pi / 2
    theta3 = ja3
    theta4 = ja4
    a1 = 2.5
    a2 = 0
    a3 = 3.5
    a4 = 3
    T_1_0 = self.get_T_std(alpha=math.pi / 2, a=0, d=0, theta=theta1)
    T_2_1 = self.get_T_std(alpha=math.pi / 2, a=a2, d=0, theta=theta2)
    T_3_2 = self.get_T_std(alpha=-math.pi / 2, a=a3, d=0, theta=theta3)
    T_4_3 = self.get_T_std(alpha=0, a=a4, d=0, theta=theta4)

    T_2_0 = T_1_0.dot(T_2_1)
    T_3_0 = T_2_0.dot(T_3_2)
    T_4_0 = T_3_0.dot(T_4_3)

    z_0 = np.array([0, 0, 1]).T
    z_1 = T_1_0[0:3, 0:3].dot(z_0)
    z_2 = T_2_0[0:3, 0:3].dot(z_0)
    z_3 = T_3_0[0:3, 0:3].dot(z_0)
    z_4 = T_4_0[0:3, 0:3].dot(z_0)

    p_0 = np.array([0, 0, 0, 1]).T
    p_1_0 = T_1_0.dot(p_0)
    assert p_1_0.shape == (4,)
    p_1_0 = p_1_0[0:3]
    p_2_0 = T_2_0.dot(p_0)[0:3]
    p_3_0 = T_3_0.dot(p_0)[0:3]
    p_4_0 = T_4_0.dot(p_0)[0:3]

    J_11 = np.cross(z_0, (p_4_0 - p_0[0:3]))  # the first col
    J_12 = np.cross(z_1, (p_4_0 - p_1_0))
    J_13 = np.cross(z_2, (p_4_0 - p_2_0))
    J_14 = np.cross(z_3, (p_4_0 - p_3_0))

    Jp = np.array([J_11, J_12, J_13, J_14])
    Jo = np.array([z_0, z_1, z_2, z_3])
    J = np.concatenate([Jp, Jo], axis=-1).T
    return J,Jp.T,Jo.T


def get_T_std(alpha, theta, d, a):
  """
  get the FK matrix
  :param alpha:
  :param theta:
  :param d:
  :param a:
  :return:
  """
  return np.array([
    [math.cos(theta), -math.sin(theta) * math.cos(alpha), math.sin(theta) * math.sin(alpha), a * math.cos(theta)],
    [math.sin(theta), math.cos(theta) * math.cos(alpha), -math.cos(theta) * math.sin(alpha), a * math.sin(theta)],
    [0., math.sin(alpha), math.cos(alpha), d],
    [0, 0, 0, 1]
  ])
def calcu_jocabian(ja1, ja2, ja3, ja4):
  theta1 = ja1+math.pi/2
  theta2 = ja2+math.pi/2
  theta3 = ja3
  theta4 = ja4
  a1 = 2.5
  a2 = 0
  a3 = 3.5
  a4 = 3
  T_1_0 = get_T_std(alpha=math.pi / 2, a=0, d=0, theta=theta1)
  T_2_1 = get_T_std(alpha=math.pi / 2, a=a2, d=0, theta=theta2)
  T_3_2 = get_T_std(alpha=-math.pi / 2, a=a3, d=0, theta=theta3)
  T_4_3 = get_T_std(alpha=0, a=a4, d=0, theta=theta4)

  T_2_0 = T_1_0.dot(T_2_1)
  T_3_0 = T_2_0.dot(T_3_2)
  T_4_0 = T_3_0.dot(T_4_3)

  z_0 = np.array([0,0,1]).T
  z_1 = T_1_0[0:3, 0:3].dot(z_0)
  z_2 = T_2_0[0:3, 0:3].dot(z_0)
  z_3 = T_3_0[0:3, 0:3].dot(z_0)
  z_4 = T_4_0[0:3, 0:3].dot(z_0)

  p_0 = np.array([0,0,0,1]).T
  p_1_0 = T_1_0.dot(p_0)
  assert  p_1_0.shape == (4,)
  p_1_0 = p_1_0[0:3]
  p_2_0 = T_2_0.dot(p_0)[0:3]
  p_3_0 = T_3_0.dot(p_0)[0:3]
  p_4_0 = T_4_0.dot(p_0)[0:3]

  J_11 = np.cross(z_0, (p_4_0-p_0[0:3])) # the first col
  J_12 = np.cross(z_1, (p_4_0-p_1_0))
  J_13 = np.cross(z_2, (p_4_0-p_2_0))
  J_14 = np.cross(z_3, (p_4_0-p_3_0))

  Jp = np.array([J_11, J_12, J_13, J_14])
  Jo = np.array([z_0, z_1, z_2, z_3])
  J = np.concatenate([Jp,Jo], axis=-1).T
  return J


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

