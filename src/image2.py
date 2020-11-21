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
import cv_utils

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
    global_circle_pos = self.estimate_global_pos(self.cv_image2)
    global_target_pos = self.estimate_target_3Dposition()
    cv_end_pos = Float64MultiArray()
    cv_end_pos.data = global_circle_pos["red"]
    start_pos = np.array([0.,0.,9])
    print("target:{}".format(global_target_pos))
    global_target_pos = np.array([-1,0.5, 7.5])
    ja = self.move_from(np.array([0.,0.,0.,0.]),start_pos, global_target_pos)
    print(ja)
    # create publish var
    ja1 = Float64()
    ja1.data = ja[0]
    ja2 = Float64()
    ja2.data = ja[1]
    ja3 = Float64()
    ja3.data = ja[2]
    ja4 = Float64()
    ja4.data = ja[3]

    # ja1.data=0
    # ja2.data=1
    # ja3.data=0
    # ja4.data = 0

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

    except CvBridgeError as e:
      print(e)

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
    a1 = 2.5
    offset_z = np.array([0,0,a1])
    start_pos = start_pos-offset_z
    end_pos = end_pos - offset_z
    v_point = (start_pos - end_pos) / dt
    J, Jp, Jo = self.calcu_jocabian(0, 1, 0, 0)
    J = Jp
    assert J.shape == (3, 4)
    J_inv = np.linalg.pinv(J)
    assert J_inv.shape == (4, 3)
    q_speed = J_inv.dot(v_point.T)
    delt_q = q_speed * dt
    ja = ja+delt_q
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

    p_0 = np.array([0, 0, 0, 1]).T
    p_1_0 = T_1_0.dot(p_0)
    assert p_1_0.shape == (4,)
    p_1_0 = p_1_0[0:3]
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
    return J,Jp.T,Jo.T
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


