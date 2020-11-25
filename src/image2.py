#!/usr/bin/env python3
import math

import roslib
import sys
import rospy
import cv2
import numpy as np
from scipy.optimize import least_squares
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError
import cv_utils
from math import *
class image_converter:

  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing

    rospy.init_node('image_processing', anonymous=True)
    # initialize a publisher to send images from camera2 to a topic named image_topic2
    self.image_pub2 = rospy.Publisher("image_topic2",Image, queue_size = 1)
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw",Image,self.callback2)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()

    # x camera msg
    self.x_cam_pos_sub = rospy.Subscriber("x_cam_pos", Float64MultiArray, self.x_cam_callback)
    # target 3D postion
    self.target_3Dposition_pub = rospy.Publisher("/target/position_estimation", Float64MultiArray, queue_size=10)
    self.cv_end_pos_pub = rospy.Publisher("cv_end_pos", Float64MultiArray, queue_size=10)
    self.template = cv2.imread("image_crop.png", 0)
    # joint control
    self.robot_joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
    self.robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
    self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
    self.robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)

    # joint estimate
    self.robot_joint_angle1_pub = rospy.Publisher("joint_angle1_cv", Float64, queue_size=10)
    self.robot_joint_angle2_pub = rospy.Publisher("joint_angle2_cv", Float64,queue_size=10)
    self.robot_joint_angle3_pub = rospy.Publisher("joint_angle3_cv", Float64, queue_size=10)
    self.robot_joint_angle4 = rospy.Publisher("joint_angle4_cv", Float64, queue_size=10)
    self.robot_joint_angles_pub = rospy.Publisher("joint_angles_cv", Float64MultiArray, queue_size=10)

    # get time
    self.start_time = rospy.get_time()

  def x_cam_callback(self, data):
    try:
      x_cam_pos = np.array(data.data)
      self.x_cam_pos_yellow = x_cam_pos[0:2]
      self.x_cam_pos_blue = x_cam_pos[2:4]
      self.x_cam_pos_green = x_cam_pos[4:6]
      self.x_cam_pos_red = x_cam_pos[6:8]
      self.x_cam_pos_tar = x_cam_pos[8:10]
    except Exception as e:
      print(e)

  # Recieve data, process it, and publish
  def callback2(self,data):
    # Recieve the image
    try:
      self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    # get all cirlce's 3d position
    self.global_circle_pos = self.estimate_global_pos(self.cv_image2)
    self.global_target_pos = self.estimate_target_3Dposition()
    target_pub = Float64MultiArray()
    target_pub.data = self.global_target_pos

    # get end postion
    cv_end_pos = Float64MultiArray()
    cv_end_pos.data = self.global_circle_pos["red"]
    # estimate the angles:
    joints_angle_esti = self.calcu_jas_from_vision(self.global_circle_pos)
    ja1_esti = Float64()
    ja1_esti.data = joints_angle_esti[0]
    ja2_esti = Float64()
    ja2_esti.data = joints_angle_esti[1]
    ja3_esti = Float64()
    ja3_esti.data = joints_angle_esti[2]
    ja4_esti = Float64()
    ja4_esti.data = joints_angle_esti[3]

    jas_cv = Float64MultiArray()
    jas_cv.data = joints_angle_esti

    ja1 = Float64()
    ja2 = Float64()
    ja3 = Float64()
    ja4 = Float64()
    time = rospy.get_time() - self.start_time

    # =========================================
    # If the accuracy of joint angles not stable enough,
    # maybe you can control by a numerical IK like follows, and set a nearly start point from joint angles estimate
    # The follow one is start from (0,0,0)
    # jas4_cv = self.fit_joint_angles(joints_angle_esti)
    # jas4_cv = self.angle_conv_T2J(jas4_cv)
    # ja1.data = jas4_cv[0]
    # ja2.data = jas4_cv[1]
    # ja3.data = jas4_cv[2]
    # ja4.data = jas4_cv[3]
    # ==================================

    # create publish var

    # print(time)
    # ja1.data = np.pi * np.sin(np.pi / 15. * time)
    
    
    # ja2.data = np.pi / 2 * np.sin(np.pi / 15. * time)
    
    # ja3.data = np.pi / 2 * np.sin(np.pi / 18. * time)
    
    # ja4.data = np.pi / 2 * np.sin(np.pi / 20. * time)  # without direction
    #
    jas = np.array([ja1.data, ja2.data, ja3.data, ja4.data])


    im2=cv2.imshow('window2', self.cv_image2)
    cv2.waitKey(1)

    # Publish the results
    try:
      self.image_pub2.publish(self.bridge.cv2_to_imgmsg(self.cv_image2, "bgr8"))
      self.cv_end_pos_pub.publish(cv_end_pos)
      self.robot_joint1_pub.publish(ja1)
      self.robot_joint2_pub.publish(ja2)
      self.robot_joint3_pub.publish(ja3)
      self.robot_joint4_pub.publish(ja4)

      self.robot_joint_angle1_pub.publish(ja1_esti)
      self.robot_joint_angle2_pub.publish(ja2_esti)
      self.robot_joint_angle3_pub.publish(ja3_esti)
      self.robot_joint_angle4.publish(ja4_esti)
      self.robot_joint_angles_pub.publish(jas_cv)

      self.target_3Dposition_pub.publish(target_pub)

    except CvBridgeError as e:
      print(e)

  def angle_conv_J2T(self, ja):
    ret = np.clip(ja+np.array([pi/2, pi/2, 0, 0]),[-pi/2, 0, -pi/2, -pi/2], [pi/2*3 , pi, pi/2, pi/2])
    return ret


  def angle_conv_T2J(self, theta):
    return theta- np.array([pi/2, pi/2, 0, 0])

  # don't use thhis alone
  def get_global_pos(self, x_cam_pos, y_cam_pos, x_cam_pos_yellow, y_cam_pos_yellow, a):
    if x_cam_pos[0] == -1:
      x = a * (y_cam_pos[0] - y_cam_pos_yellow[0])
      y = 0
      z = -a * (y_cam_pos[1] - y_cam_pos_yellow[1])
    elif y_cam_pos[0] == -1:
      x = 0
      y = a * (x_cam_pos[0] - x_cam_pos_yellow[0])
      z = -a * (x_cam_pos[1] - x_cam_pos_yellow[1])
    else:
      x = a * (y_cam_pos[0] - y_cam_pos_yellow[0])
      y = a * (x_cam_pos[0] - x_cam_pos_yellow[0])
      z = -a * (x_cam_pos[1] - y_cam_pos_yellow[1])

    ret = np.array([x, y, z])
    return ret

  def estimate_global_pos(self, image):
    """
    Estimate all Joints positions
    :param image:
    :return: Directionary
    """
    self.y_cam_pos_yellow = cv_utils.detect_yellow(self.cv_image2)
    self.y_cam_pos_blue = cv_utils.detect_blue(self.cv_image2)
    self.y_cam_pos_green = cv_utils.detect_green(self.cv_image2)
    self.y_cam_pos_red = cv_utils.detect_red(self.cv_image2)
    a =cv_utils.pixel2meter(image)
    yellow_global = np.array([0,0,0])
    blue_global = np.array([0,0,2.5])
    # green circle, Not visiable in camera x
    green_global = self.get_global_pos(self.x_cam_pos_green, self.y_cam_pos_green, self.x_cam_pos_yellow, self.y_cam_pos_yellow,a)
    red_global = self.get_global_pos(self.x_cam_pos_red, self.y_cam_pos_red, self.x_cam_pos_yellow, self.y_cam_pos_yellow, a)
    ret = {"yellow":yellow_global,
           "blue":blue_global,
           "green":green_global,
           "red":red_global}
    return ret

# call the class
  def move_from(self, ja, start_pos, end_pos):
    dt = 10
    delt_angle = np.array([math.pi / 2, math.pi / 2, 0, 0])
    theta = ja + delt_angle
    v_point = (end_pos-start_pos) / dt
    # J, Jp, Jo = self.calcu_jocabian(theta[0], theta[1], theta[2], theta[3])
    # J = Jp
    J = self.jocabian40(theta)
    assert J.shape == (3, 4)
    J_inv = np.linalg.pinv(J)
    assert J_inv.shape == (4, 3)
    q_speed = J_inv.dot(v_point.T)
    delt_q = q_speed * dt
    theta = theta + delt_q
    # convert to global axis
    ja = theta - delt_angle
    return ja

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

  def calcu_jocabian(self,theta1, theta2, theta3, theta4):
    d1 = 2.5
    a2 = 0
    a3 = 3.5
    a4 = 3
    T_1_0 = self.get_T_std(alpha=math.pi / 2, a=0, d=d1, theta=theta1)
    T_2_1 = self.get_T_std(alpha=math.pi / 2, a=a2, d=0, theta=theta2)
    T_3_2 = self.get_T_std(alpha=-math.pi / 2, a=a3, d=0, theta=theta3)
    T_4_3 = self.get_T_std(alpha=0, a=a4, d=0, theta=theta4)

    T_2_0 = T_1_0.dot(T_2_1)
    T_3_0 = T_2_0.dot(T_3_2)
    T_4_0 = T_3_0.dot(T_4_3)

    z_0 = np.array([0, 0, 1])
    z_1 = T_1_0[0:3,2]
    z_2 = T_2_0[0:3,2]
    z_3 = T_3_0[0:3,2]

    p_0 = np.array([0, 0, 0, 1]).T
    p_1_0 = T_1_0.dot(p_0)[0:3]
    p_2_0 = T_2_0.dot(p_0)[0:3]
    p_3_0 = T_3_0.dot(p_0)[0:3]
    p_4_0 = T_4_0.dot(p_0)[0:3]

    p_0 = np.array([0,0,0])
    J_11 = np.cross(z_0, (p_4_0 - p_0))  # the first col
    J_12 = np.cross(z_1, (p_4_0 - p_1_0))
    J_13 = np.cross(z_2, (p_4_0 - p_2_0))
    J_14 = np.cross(z_3, (p_4_0 - p_3_0))

    Jp = np.array([J_11, J_12, J_13, J_14])
    Jo = np.array([z_0, z_1, z_2, z_3])
    J = np.concatenate([Jp, Jo], axis=-1).T
    Jp = Jp.T
    Jo = Jo.T
    return J,Jp, Jo


  def jocabian40(self, q):
    theta1 = q[0]
    theta2 = q[1]
    theta3 = q[2]
    theta4 = q[3]
    a3 = 3.5
    a4 = 3
    # generate from matlab
    Jocabian = np.array([
      [a4 * cos(theta4) * (cos(theta1) * sin(theta3) - cos(theta2) * cos(theta3) * sin(theta1)) + a3 * cos(
        theta1) * sin(theta3) - a3 * cos(theta2) * cos(theta3) * sin(theta1) + a4 * sin(theta1) * sin(theta2) * sin(
        theta4),
       - a3 * cos(theta1) * cos(theta3) * sin(theta2) - a4 * cos(theta1) * cos(theta2) * sin(theta4) - a4 * cos(
         theta1) * cos(theta3) * cos(theta4) * sin(theta2),
       a4 * cos(theta4) * (cos(theta3) * sin(theta1) - cos(theta1) * cos(theta2) * sin(theta3)) + a3 * cos(
         theta3) * sin(theta1) - a3 * cos(theta1) * cos(theta2) * sin(theta3),
       - a4 * sin(theta4) * (sin(theta1) * sin(theta3) + cos(theta1) * cos(theta2) * cos(theta3)) - a4 * cos(
         theta1) * cos(theta4) * sin(theta2)],
      [a4 * cos(theta4) * (sin(theta1) * sin(theta3) + cos(theta1) * cos(theta2) * cos(theta3)) + a3 * sin(
        theta1) * sin(theta3) + a3 * cos(theta1) * cos(theta2) * cos(theta3) - a4 * cos(theta1) * sin(theta2) * sin(
        theta4),
       - a3 * cos(theta3) * sin(theta1) * sin(theta2) - a4 * cos(theta2) * sin(theta1) * sin(theta4) - a4 * cos(
         theta3) * cos(theta4) * sin(theta1) * sin(theta2),
       - a4 * cos(theta4) * (cos(theta1) * cos(theta3) + cos(theta2) * sin(theta1) * sin(theta3)) - a3 * cos(
         theta1) * cos(theta3) - a3 * cos(theta2) * sin(theta1) * sin(theta3),
       a4 * sin(theta4) * (cos(theta1) * sin(theta3) - cos(theta2) * cos(theta3) * sin(theta1)) - a4 * cos(
         theta4) * sin(theta1) * sin(theta2)],
      [0,
       a3 * cos(theta2) * cos(theta3) - a4 * sin(theta2) * sin(theta4) + a4 * cos(theta2) * cos(theta3) * cos(theta4),
       - a3 * sin(theta2) * sin(theta3) - a4 * cos(theta4) * sin(theta2) * sin(theta3),
       a4 * cos(theta2) * cos(theta4) - a4 * cos(theta3) * sin(theta2) * sin(theta4)]])
    return Jocabian


  def jocabian30(self, q):
    theta1 = q[0]
    theta2 = q[1]
    theta3 = q[2]
    a3 = 3.5
    # generate from matlab
    return np.array([
      [a3 * cos(theta1) * sin(theta3) - a3 * cos(theta2) * cos(theta3) * sin(theta1),
       -a3 * cos(theta1) * cos(theta3) * sin(theta2),
       a3 * cos(theta3) * sin(theta1) - a3 * cos(theta1) * cos(theta2) * sin(theta3)],
      [a3 * sin(theta1) * sin(theta3) + a3 * cos(theta1) * cos(theta2) * cos(theta3),
       -a3 * cos(theta3) * sin(theta1) * sin(theta2),
       - a3 * cos(theta1) * cos(theta3) - a3 * cos(theta2) * sin(theta1) * sin(theta3)],
      [0, a3 * cos(theta2) * cos(theta3), -a3 * sin(theta2) * sin(theta3)]])

  def K30(self, q):
    theta1 = q[0]
    theta2 = q[1]
    theta3 = q[2]
    a3 = 3.5
    # generate from matlab
    return np.array([
      a3 * sin(theta1) * sin(theta3) + a3 * cos(theta1) * cos(theta2) * cos(theta3),
      a3 * cos(theta2) * cos(theta3) * sin(theta1) - a3 * cos(theta1) * sin(theta3),
      a3 * cos(theta3) * sin(theta2) + 5 / 2])

  def K30_for_esti(self, q):
    rel_pos = self.K30(q)
    return rel_pos - self.global_circle_pos["green"]
  def K40_for_esti(self,q):
    rel_pos = self.K40(q)
    return rel_pos - self.global_target_pos

  def K40(self, q):
    theta1 = q[0]
    theta2 = q[1]
    theta3 = q[2]
    theta4 = q[3]
    a3 = 3.5
    a4 = 3
    return np.array([
      a4 * cos(theta4) * (sin(theta1) * sin(theta3) + cos(theta1) * cos(theta2) * cos(theta3)) + a3 * sin(theta1) * sin(
        theta3) + a3 * cos(theta1) * cos(theta2) * cos(theta3) - a4 * cos(theta1) * sin(theta2) * sin(theta4),
      a3 * cos(theta2) * cos(theta3) * sin(theta1) - a3 * cos(theta1) * sin(theta3) - a4 * cos(theta4) * (
              cos(theta1) * sin(theta3) - cos(theta2) * cos(theta3) * sin(theta1)) - a4 * sin(theta1) * sin(
        theta2) * sin(theta4),
      a3 * cos(theta3) * sin(theta2) + a4 * cos(theta2) * sin(theta4) + a4 * cos(theta3) * cos(theta4) * sin(
        theta2) + 5 / 2
    ])

  def estimate_target_3Dposition(self):
    """
    Estimate the target orange ball
    :return: [x,y,z]
    """
    a = cv_utils.pixel2meter(self.cv_image2)
    orange_mask = cv_utils.detect_orange(self.cv_image2)
    target_proj_pos2 = cv_utils.find_target(orange_mask, self.template)
    target_proj_pos2[0] = target_proj_pos2[0] + 24  # the temeple is 48*48
    target_proj_pos2[1] = target_proj_pos2[1] + 24
    # mark the target with a red circle
    # cv2.circle(self.cv_image2, (target_proj_pos2[0], target_proj_pos2[1]),5,(0,0,255))
    target_x = (target_proj_pos2[0] - self.y_cam_pos_yellow[0]) * a * 0.8
    target_x = np.clip(target_x, -3, 3)
    target_y = (self.x_cam_pos_tar[0] - self.x_cam_pos_yellow[0]) * a * 0.9 + 1.7  # the 1.7 is camera view offset
    # target_y = target_y * 0.8
    target_y = np.clip(target_y, -2.5, 2.5)
    target_z = (self.x_cam_pos_yellow[1] - self.x_cam_pos_tar[1]) * a
    target_z = (target_z - 7) * 0.8 + 7 - 0.5
    target_z = np.clip(target_z, 6, 8)
    return np.array([target_x, target_y, target_z])

  def calcu_fk_end_pos(self,ja1,ja2,ja3,ja4):
    a = np.array([0, 0, 0, 1])
    d1 = 2.5
    theta4 = ja4
    a4 = 3
    theta3 = ja3
    a3 = 3.5
    theta2 = ja2 + math.pi / 2
    a2 = 0
    theta1 = ja1 + math.pi / 2
    T_1_0 = self.get_T_std(alpha=math.pi / 2, a=0, d=d1, theta=theta1)
    T_2_1 = self.get_T_std(alpha=math.pi / 2, a=a2, d=0, theta=theta2)
    T_3_2 = self.get_T_std(alpha=-math.pi / 2, a=a3, d=0, theta=theta3)
    T_4_3 = self.get_T_std(alpha=0, a=a4, d=0, theta=theta4)

    T_2_0 = T_1_0.dot(T_2_1)
    T_3_0 = T_2_0.dot(T_3_2)
    T_4_0 = T_3_0.dot(T_4_3)
    a = T_4_0.dot(a)
    return a[0:3]

  def calcu_jas_from_vision(self, global_pos):
    """
    This function is used to calcu the joint2, joint3, joint4 angles, when Join1 angle = 0
    :param global_pos:
    :return: np array, [0, ja2, ja3, ja4]
    """
    ja2 = -np.arctan2(global_pos["green"][1] - global_pos["blue"][1], global_pos["green"][2] - global_pos["blue"][2])
    ja2 = np.clip(ja2.data, -np.pi / 2, np.pi / 2)
    ja3 = np.arctan2(global_pos["green"][0] - global_pos["blue"][0], global_pos["green"][2] - global_pos["blue"][2])
    ja3 = np.clip(ja3.data, -np.pi / 2, np.pi / 2)
    # calculate 3D vector of link3 and link4
    v3 = global_pos["green"] - global_pos["blue"]
    v4 = global_pos["red"] - global_pos["green"]
    v_c = np.cross(v3, v4)
    sin_theta = np.linalg.norm(v_c) / (np.linalg.norm(v3) * np.linalg.norm(v4))
    cos_theta = v3.dot(v4) / (np.linalg.norm(v3) * np.linalg.norm(v4))
    ja4 = np.arctan2(sin_theta, cos_theta)
    inv_flag = 0
    if v_c[0] < 0: # because the direct is about x axis
      ja4 = -ja4

    return np.array([0, ja2, ja3, ja4])


  def fit_joint_angles(self, q):
    """
    To estimate the Joint1 ~ Joint4 angles use Neural Network
    :param green: green position
    :param red: red position
    :return: np array [ja1,ja2,ja3,ja4]
    """
    theta = self.angle_conv_J2T(q)
    res = least_squares(self.K40_for_esti, (theta[0], theta[1], theta[2], theta[3]), self.jocabian40, bounds=([-pi/2-0.1, 0, -pi/2, -pi/2], [pi/2*3+0.1, pi, pi/2, pi/2]))
    theta = res.x

    return theta



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
