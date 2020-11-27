#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError
import cv_utils
import message_filters
class image_converter:

  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('image_processing', anonymous=True)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()

    # publish and subscribe images
    self.image_pub1 = rospy.Publisher("image_topic1",Image, queue_size = 1)
    self.image_pub2 = rospy.Publisher("image_topic2",Image, queue_size = 1)
    self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw",Image,self.callback_image1)
    self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw",Image,self.callback_image2)    

    # joint control
    self.robot_joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
    self.robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
    self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
    self.robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)

    # publish end-effector position for plot
    self.ee_pos_x_pub = rospy.Publisher("end_effector_x", Float64, queue_size=10)
    self.ee_pos_y_pub = rospy.Publisher("end_effector_y", Float64, queue_size=10)
    self.ee_pos_z_pub = rospy.Publisher("end_effector_z", Float64, queue_size=10)

    # publish and subscribe the target position
    self.target_position_pub = rospy.Publisher("target_position", Float64MultiArray, queue_size=10)
    self.target_position_sub = rospy.Subscriber("target_position", Float64MultiArray, self.callback_target_position)

    # actual robot joint angles
    self.joint_states_sub = rospy.Subscriber("/robot/joint_states", JointState, self.callback_joint_states)
    self.joint_states_pub = rospy.Publisher("/jss", Float64MultiArray, queue_size=10)

    # task 3.1: vision estimation and forward kinematics estimation
    self.ee_pos_cv_pub = rospy.Publisher("/ee_pos_cv", Float64MultiArray, queue_size=10)
    self.ee_pos_fk_pub = rospy.Publisher("/ee_pos_fk", Float64MultiArray, queue_size=10)


    # record the begining time
    self.time_trajectory = rospy.get_time()
    # initialize errors
    self.time_previous_step = np.array([rospy.get_time()], dtype='float64')
    # errors
    self.error = np.zeros(3, dtype='float64')  
    self.error_d = np.zeros(3, dtype='float64') 
    # template target
    self.template = cv2.imread("image_crop.png", 0)
    self.global_target_pos = np.zeros(3, dtype='float64')
    self.pos = np.zeros(3, dtype='float64')

  def callback_joint_states(self, data):
    self.joint_states_origin = data.position
    joint_states_pub = Float64MultiArray()
    joint_states_pub.data = self.joint_states_origin
    self.joint_states_pub.publish(joint_states_pub)
  # get images
  def callback_image1(self, data):
    try:
      self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image1, "bgr8"))
  def callback_image2(self, data):
    try:
      self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    self.image_pub2.publish(self.bridge.cv2_to_imgmsg(self.cv_image2, "bgr8"))

    # find the target position via vision
    x_cam_pos = self.get_cirlces()
    self.x_cam_pos_yellow = x_cam_pos[0:2]
    self.x_cam_pos_blue = x_cam_pos[2:4]
    self.x_cam_pos_green = x_cam_pos[4:6]
    self.x_cam_pos_red = x_cam_pos[6:8]
    self.x_cam_pos_tar = x_cam_pos[8:10]
    self.global_circle_pos = self.estimate_global_pos(self.cv_image2)
    self.global_target_pos = self.estimate_target_3Dposition()
    cv_end_pos = Float64MultiArray()
    cv_end_pos.data = self.global_circle_pos["red"]
    tar_pos = Float64MultiArray()
    tar_pos.data = self.global_target_pos
    # Publish the results
    try:
      self.ee_pos_cv_pub.publish(cv_end_pos)
      self.target_position_pub.publish(tar_pos)
    except CvBridgeError as e:
      print(e)

  def callback_target_position(self, data):
    tar_pos = np.array(data.data)
    # closed-loop
    t_m = self.T_matrices(self.joint_states_origin)
    ee_pos = self.forward_kinematics(t_m)
    # save the end-effector position data to publish
    self.ee_pos_fk = Float64MultiArray()
    self.ee_pos_fk.data = ee_pos 
    self.ee_pos_x = Float64()
    self.ee_pos_x.data = ee_pos[0]
    self.ee_pos_y = Float64()
    self.ee_pos_y.data = ee_pos[1]
    self.ee_pos_z = Float64()
    self.ee_pos_z.data = ee_pos[2]

    # closed-loop joint angles

    q_d = self.control_closed(ee_pos, tar_pos)
    self.joint1=Float64()
    self.joint1.data= q_d[0]
    self.joint2=Float64()
    self.joint2.data= q_d[1]
    self.joint3=Float64()
    self.joint3.data= q_d[2]
    self.joint4=Float64()
    self.joint4.data= q_d[3]

    # publish the robot using the joint angles via closed-loop and the end-effector position
    try:
      self.robot_joint1_pub.publish(self.joint1)
      self.robot_joint2_pub.publish(self.joint2)
      self.robot_joint3_pub.publish(self.joint3)
      self.robot_joint4_pub.publish(self.joint4)
      self.ee_pos_fk_pub.publish(self.ee_pos_fk)
      self.ee_pos_x_pub.publish(self.ee_pos_x)
      self.ee_pos_y_pub.publish(self.ee_pos_y)
      self.ee_pos_z_pub.publish(self.ee_pos_z)
    except CvBridgeError as e:
      print(e)

  # find the position of objects from image1
  def get_cirlces(self):
    # x cam position
    # x_cam_pos = Float64MultiArray()
    yellow_pos = cv_utils.detect_yellow(self.cv_image1)
    blue_pos = cv_utils.detect_blue(self.cv_image1)
    green_pos = cv_utils.detect_green(self.cv_image1)
    red_pos = cv_utils.detect_red(self.cv_image1)
    mask = cv_utils.detect_orange(self.cv_image1)
    orange_pos = cv_utils.find_target(mask, self.template)
    x_cam_pos = np.array(
      [yellow_pos[0], yellow_pos[1], blue_pos[0], blue_pos[1], green_pos[0], green_pos[1], red_pos[0], red_pos[1],
       orange_pos[0], orange_pos[1]])
    return x_cam_pos

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

  # given angle
  def trajectory_joint_angles(self, time):
    # a1 = 180*np.sin(np.pi/15*t)
    a1 = 0
    a2 = 90*np.sin(np.pi/15*time)
    a3 = 90*np.sin(np.pi/18*time)
    a4 = 90*np.sin(np.pi/20*time)
    return np.array([a1, a2, a3, a4])

  # Transformation matrices
  def T_matrices(self, joint_angles):
    # D-H table
    # theta, alpha, r, d 
    l1 = [joint_angles[0]+np.pi/2, np.pi/2, 0, 2.5]
    l2 = [joint_angles[1]+np.pi/2, np.pi/2, 0, 0]
    l3 = [joint_angles[2], -np.pi/2, 3.5, 0]
    l4 = [joint_angles[3], 0, 3, 0]

    Ls = np.array([l1, l2, l3, l4])  
    n = len(Ls)
    Ts = []

    for i in range(n):
      Ts.append(
        np.dot(
              [
               [np.cos(Ls[i][0]), -np.sin(Ls[i][0]),0,0], 
               [np.sin(Ls[i][0]), np.cos(Ls[i][0]),0,0],
               [0,0,1,Ls[i][3]],
               [0,0,0,1],
              ], 
              [
               [1,0,0,Ls[i][2]],
               [0,np.cos(Ls[i][1]),-np.sin(Ls[i][1]),0],
               [0,np.sin(Ls[i][1]),np.cos(Ls[i][1]),0],
               [0,0,0,1]
              ]
              )
        )
    return np.array(Ts)

  # Forward Kinematics using transformation matrices
  def forward_kinematics(self, Ts):
    temp = np.identity(4)
    for i in Ts:
      temp = np.dot(temp, i)
    return np.array(temp[:3,3])

  # Jacobian using transformation matrices
  def jacobian(self, Ts):
    # get tranformation matrices for frame 0 to 1, 0 to 2, 0 to 3, 0 to 4
    T_0 = []
    T_0.append(Ts[0])  
    for i in range(1,len(Ts)):
      T_0.append(np.dot(T_0[i-1], Ts[i]))
    # T_0[0]: T_1_0
    # T_0[1]: T_2_0
    # T_0[2]: T_3_0
    # T_0[3]: T_4_0
    T_0 = np.array(T_0)

    # Rotation matrices
    R_0 = []
    R_0.append(np.eye(3))
    for i in range(3):
      R_0.append(T_0[i][:3,:3])
    # R_0[0]: R_0_0
    # R_0[1]: R_1_0
    # R_0[2]: R_2_0
    # R_0[3]: R_3_0
    R_0 = np.array(R_0)

    # Linear matrices
    D_0 = []
    D_0.append(np.zeros(3))
    for i in T_0:
      D_0.append(i[:3,3])
    # D_0[0]: D_0_0
    # D_0[1]: D_1_0
    # D_0[2]: D_2_0
    # D_0[3]: D_3_0
    # D_0[4]: D_4_0
    D_0 = np.array(D_0)

    # Jacobian
    jacobian = np.zeros((3,4))
    z = np.array([0, 0, 1])
    for i in range(len(Ts)):
      # Linear part
      D_diff = D_0[len(D_0)-1] - D_0[i]
      linear = np.cross(np.dot(R_0[i], z), D_diff)
      # print(linear)
      for j in range(3):
        jacobian[j][i] = linear[j] 
      # Rotation part
      # rotation = np.dot(R_0[i], z)
      # for k in range(3):
      #   jacobian[k+3][i] = rotation[k]
    return jacobian

  def control_closed(self, ee_pos, tar_pos):
    # p and d gains
    p = 1
    d = 0.2
    # diagonal matrices
    K_p = np.zeros((3,3))
    K_d = np.zeros((3,3))
    np.fill_diagonal(K_p, p)
    np.fill_diagonal(K_d, d)
    # estimate time step
    cur_time = np.array([rospy.get_time()])
    dt = cur_time - self.time_previous_step
    self.time_previous_step = cur_time

    # end-effector position
    pos = ee_pos
    # target position
    pos_d = tar_pos
    # estimate derivative of error
    self.error_d = ((pos_d - pos) - self.error)/dt
    # estimate error
    self.error = pos_d-pos
    q = self.joint_states_origin # estimate initial value of joints'
    t_m = self.T_matrices(q)
    j = self.jacobian(t_m)
    J_inv = np.linalg.pinv(j)  # calculating the psudeo inverse of Jacobian
    dq_d =np.dot(J_inv, ( np.dot(K_d,self.error_d.transpose()) + np.dot(K_p,self.error.transpose())))  # control input (angular velocity of joints)
    q_d = q + (dt * dq_d)  # control input (angular position of joints)
    return q_d
    
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


