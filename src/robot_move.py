#!/usr/bin/env python3

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
import message_filters
class image_converter:

  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('image_processing', anonymous=True)
    # joint control
    self.robot_joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
    self.robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
    self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
    self.robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)

    # subscribe robot joint angles
    self.robot_joint_angle1_sub = rospy.Subscriber("joint_angle1_cv", Float64, self.callback_ja1)
    self.robot_joint_angle2_sub = rospy.Subscriber("joint_angle2_cv", Float64, self.callback_ja2)
    self.robot_joint_angle3_sub = rospy.Subscriber("joint_angle3_cv", Float64, self.callback_ja3)
    self.robot_joint_angle4_sub = rospy.Subscriber("joint_angle4_cv", Float64, self.callback_ja4)
    # subscribe end-effector positon via vision
    self.cv_end_pos_sub = rospy.Subscriber("cv_end_pos", Float64MultiArray, self.callback_cv_end_pos)
    # publish end-effector position via forward kinematics 
    self.end_effector_position_fk_pub = rospy.Publisher("ee_fk", Float64MultiArray, queue_size = 10)

    # subscribe target position by vision for closed-loop control
    self.target_position_sub = rospy.Subscriber("/target/position_estimation", Float64MultiArray, self.callback)
    # subscribe actual end-effector and target position to test
    # self.ee_sub = rospy.Subscriber("/target/joint_states", Float64MultiArray, self.callback_tp)
    # self.tp_sub = rospy.Subscriber("/target/joint_states", Float64MultiArray, self.callback_tp)

    # record the begining time
    self.time_trajectory = rospy.get_time()
    # initialize errors
    self.time_previous_step = np.array([rospy.get_time()], dtype='float64')
    # erros
    self.error = np.zeros(3, dtype='float64')  
    self.error_d = np.zeros(3, dtype='float64') 
    

    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()
  # def callback_cv_end_pos(self, data):
  #   self.cv_end_pos = data.data
  # def callback_tp(self, data):
  #   self.tp = np.array(data.data[0])
  def callback_cv_end_pos(self, data):
    self.cv_end_pos = np.array(data.data)
  def callback_ja1(self, data):
    self.joint_angle1 = data.data
  def callback_ja2(self, data):
    self.joint_angle2 = data.data
  def callback_ja3(self, data):
    self.joint_angle3 = data.data
  def callback_ja4(self, data):
    self.joint_angle4 = data.data
  def callback(self, data):
    self.target_pos = np.array(data.data)
    self.joint_angles_cv = np.array([self.joint_angle1, self.joint_angle2, self.joint_angle3, self.joint_angle4])

    # end-effecor position by forward kinematics
    # cur_time = rospy.get_time() - self.time_trajectory
    # joint_angles = self.joint_angles_a
    # self.cv_end_pos = np.array(data.data)
    # tm = self.T_matrices(joint_angles)
    # self.end_pos_fk = self.forward_kinematics(tm)
    # fk_pub = Float64MultiArray()
    # fk_pub.data = self.fk_trajectory[:3,3]
    # closed_loop
    q_d = self.control_closed()
    self.joint1=Float64()
    self.joint1.data= q_d[0]
    self.joint2=Float64()
    self.joint2.data= q_d[1]
    self.joint3=Float64()
    self.joint3.data= q_d[2]
    self.joint4=Float64()
    self.joint4.data= q_d[3]

    try:
      # self.end_effector_position_fk_pub.publish(fk_pub)
      self.robot_joint1_pub.publish(self.joint1)
      self.robot_joint2_pub.publish(self.joint2)
      self.robot_joint3_pub.publish(self.joint3)
      self.robot_joint4_pub.publish(self.joint4)
    except CvBridgeError as e:
      print(e)
  
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
    return np.array(temp)

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
    jacobian = np.zeros((6,4))
    z = np.array([0, 0, 1])
    for i in range(len(Ts)):
      # Linear part
      D_diff = D_0[len(D_0)-1] - D_0[i]
      linear = np.cross(np.dot(R_0[i], z), D_diff)
      # print(linear)
      for j in range(3):
        jacobian[j][i] = linear[j] 
      # Rotation part
      rotation = np.dot(R_0[i], z)
      for k in range(3):
        jacobian[k+3][i] = rotation[k]
    return jacobian

  def control_closed(self):
    # p and d gains
    p = 20
    d = 6
    K_p = np.zeros((6,3))
    K_d = np.zeros((6,3))
    np.fill_diagonal(K_p, p)
    np.fill_diagonal(K_d, d)

    # ([[3, 0, 0],[0, 3, 0],[0, 0, 3], 
    #   [3,0,0], [0,3,0], [0,0,3]] )
    # K_d = np.array([[0.2,0,0],[0,0.2,0],[0,0,0.2], 
    #   [0.2,0,0], [0,0.2,0], [0,0,0.2]])

    # estimate time step
    cur_time = np.array([rospy.get_time()])
    dt = cur_time - self.time_previous_step
    self.time_previous_step = cur_time
    # robot end-effector position
    pos = self.cv_end_pos
    # pos = self.cv_end_pos
    # desired trajectory
    pos_d = self.target_pos
    # estimate derivative of error
    self.error_d = ((pos_d - pos) - self.error)/dt
    # estimate error
    self.error = pos_d-pos
    q = self.joint_angles_cv # estimate initial value of joints'
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


