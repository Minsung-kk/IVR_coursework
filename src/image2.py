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
import torch
import torch.nn as nn
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
    self.robot_joint_angle_pub = rospy.Publisher("joint_angle_cv", Float64MultiArray, queue_size=10)
    self.robot_joint_angle4 = rospy.Publisher("joint_angle4_cv", Float64, queue_size=10)
    self.start_time = rospy.get_time()
    self.last_ja4 = 0
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
    global_target_pos = self.estimate_target_3Dposition()
    cv_end_pos = Float64MultiArray()
    cv_end_pos.data = self.global_circle_pos["red"]


    start_angle = np.array([0.,0.,0.2,0.])
    start_pos = self.calcu_fk_end_pos(start_angle[0], start_angle[1], start_angle[2], start_angle[3])
    print("target:{}".format(global_target_pos))
    fk_pos = self.calcu_fk_end_pos(0,0,0.2,0.2)
    print("fk:pos:{}".format(fk_pos))
    global_target_pos = fk_pos
    esti_angle = self.joint_angles_estimation()
    print("esti_angle:{}".format(esti_angle))
    # global_target_pos = np.array([4.3,-3.69, 1.87])
    ja = self.move_from(start_angle, start_pos, global_target_pos)
    fk_pos = self.calcu_fk_end_pos(ja[0], ja[1], ja[2], ja[3])
    print("fk2:pos:{}".format(fk_pos))
    print("ja {}".format(ja))

    # create publish var
    ja1 = Float64()
    ja2 = Float64()
    ja3 = Float64()
    ja4 = Float64()
    time = rospy.get_time() - self.start_time
    ja1.data = ja[0]
    ja2.data = ja[1]
    ja3.data = ja[2]
    ja4.data = ja[3]
    # # ja1.data = np.pi * np.sin(np.pi / 15. * time)
    # ja1.data = 0
    # ja2.data = np.pi / 2 * np.sin(np.pi / 15. * time)
    # # joint2.data = 0
    # ja3.data = np.pi / 2 * np.sin(np.pi / 18. * time)
    # # joint3.data = 0
    # ja4.data = np.pi / 2 * np.sin(np.pi / 20. * time)  # without direction
    # # ja4.data = 0

    jas_cv = self.calcu_jas_from_vision(global_pos=self.global_circle_pos)
    ja_pub = Float64MultiArray()
    ja_pub.data = jas_cv
    # print(jas_cv)
    # ja1.data=0
    # ja2.data=1
    # ja3.data=1
    # ja4.data = 1
    #
    ja4_pub = Float64()
    ja4_pub.data = jas_cv[3]
    #cv2.imwrite('image_copy.png', cv_image)
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
      self.robot_joint_angle_pub.publish(ja_pub)
      self.robot_joint_angle4.publish(ja4_pub)
    except CvBridgeError as e:
      print(e)
  def angle_conv_J2T(self, ja):
    return ja+np.array([pi/2, pi/2, 0, 0])

  def angle_conv_T2J(self, theta):
    return theta- np.array([pi/2, pi/2, 0, 0])
  # don't use alone
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
    a = cv_utils.pixel2meter(self.cv_image2)
    orange_mask = cv_utils.detect_orange(self.cv_image2)
    target_proj_pos2 = cv_utils.find_target(orange_mask, self.template)
    target_proj_pos2[0] = target_proj_pos2[0] + 24 # the temeple is 48*48
    target_proj_pos2[1]= target_proj_pos2[1] + 24
    # mark the target with a red circle
    # cv2.circle(self.cv_image2, (target_proj_pos2[0], target_proj_pos2[1]),5,(0,0,255))
    target_x = (target_proj_pos2[0] - self.y_cam_pos_yellow[0]) * a
    target_y = (self.x_cam_pos_tar[0] - self.x_cam_pos_yellow[0]) *a
    target_z = (self.x_cam_pos_yellow[1] - self.x_cam_pos_tar[1]) * a
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
    # ja4.data = np.arccos(v3.dot(v4)/(np.sqrt(np.sum(v3**2)) * np.sqrt(np.sum(v4**2))))
    ja4 = np.arctan2(sin_theta, cos_theta)
    inv_flag = 0
    if v_c[0] < 0: # because the direct is about x axis
      ja4 = -ja4

    return np.array([0, ja2, ja3, ja4])
  def joint_angles_estimation(self):
    res1 = least_squares(self.K30_for_esti,(0,0,0),self.jocabian30,bounds = (-math.pi / 2, math.pi / 2))
    self.j1 = res1.x[0]
    self.j2 = res1.x[1]
    self.j3 = res1.x[2]
    # res2 = least_squares(self.K40, 0,bounds = (-math.pi / 2, math.pi / 2))
    # self.j4 = res2.x[0]
    return np.array([self.j1,self.j2,self.j3,])

class Esti_Angel_Model(nn.Module):
  def __init__(self, input_dim, output_dim):
    super(Esti_Angel_Model, self).__init__()
    self.input_dim = input_dim
    self.output_dim = output_dim

    self.model = nn.Sequential(
      nn.Linear(input_dim, 100),
      nn.ReLU(),
      nn.Linear(100, 100),
      nn.ReLU(),
      nn.Linear(100, output_dim),
      nn.Sigmoid()
    )
  def forward(self, input_data):
    out = self.model(input_data)
    out = out * pi
    return out

def K30(q):
  theta1 = q[0]
  theta2 = q[1]
  theta3 = q[2]
  a3 = 3.5
  # generate from matlab
  return np.array([
    a3 * sin(theta1) * sin(theta3) + a3 * cos(theta1) * cos(theta2) * cos(theta3),
    a3 * cos(theta2) * cos(theta3) * sin(theta1) - a3 * cos(theta1) * sin(theta3),
    a3 * cos(theta3) * sin(theta2) + 5 / 2])

def K40(q):
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

def train():
  batch_size = 256
  # model = Esti_Angel_Model(6, 4)
  model = torch.load("model.pth")
  loss_fn = torch.nn.MSELoss()
  optimazer = torch.optim.Adam(model.parameters(), lr = 0.0001)
  time = 0
  loss_all = 0
  save_thredshold = 0.5
  for ep in range(100):
    for t in range(1000):

      if time > 100000:
        print("reset time")
        time = 0
      y = []
      x = []
      for _ in range(batch_size):
        time +=1
        ja1 = np.pi * np.sin(np.pi / 15. * time)
        ja2 = np.pi / 2 * np.sin(np.pi / 15. * time)
        ja3 = np.pi / 2 * np.sin(np.pi / 18. * time)
        ja4 = np.pi / 2 * np.sin(np.pi / 20. * time)  # without direction
        q = np.array([ja1, ja2, ja3, ja4])
        q = q + np.array([pi/2,pi/2,0,0])
        green_circle = K30(q[0:3])
        red_circle = K40(q)
        x.append(np.concatenate([green_circle,red_circle], axis=-1))
        y.append(q)
      # BP
      x = torch.as_tensor(x, dtype=torch.float)
      y = torch.as_tensor(y, dtype=torch.float)
      model.zero_grad()
      y_pred = model(x)
      loss = loss_fn(y_pred, y)
      loss_all += loss
      loss.backward()
      optimazer.step()
      if (t+1)%100 ==0:
        everage_loss = loss_all/100
        print("ep:{}, loss: {}".format(ep,everage_loss))
        if everage_loss < save_thredshold:
          save_thredshold-=0.05
          torch.save(model, "model_{}".format(everage_loss)+".pth")
        loss_all = 0

def test():
  model = torch.load("model.pth")
  for time in range(100000):
    ja1 = np.pi * np.sin(np.pi / 15. * time)
    ja2 = np.pi / 2 * np.sin(np.pi / 15. * time)
    ja3 = np.pi / 2 * np.sin(np.pi / 18. * time)
    ja4 = np.pi / 2 * np.sin(np.pi / 20. * time)  # without direction
    q = np.array([ja1, ja2, ja3, ja4])
    q = q + np.array([pi / 2, pi / 2, 0, 0])
    print("q:{}".format(q))
    green_circle = K30(q[0:3])
    red_circle = K40(q)
    x = np.concatenate([green_circle, red_circle], axis=-1)
    x = torch.as_tensor(x, dtype=torch.float)
    y = model(x)
    print("y_pred:{}".format(y.data))
    print("-------------------------")
def main(args):
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    # main(sys.argv)
  train()
  # test()