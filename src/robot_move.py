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

class image_converter:

  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('image_processing', anonymous=True)
    # subscribe robot joint angles
    self.robot_joint_angles_sub = rospy.Subscriber("joint_angles_cv", Float64MultiArray, self.callback)
    self.forward_kinematics_pub = rospy.Publisher("fk", Float64MultiArray, queue_size = 10)
    self.forward_kinematics_cv_pub = rospy.Publisher("fk_cv", Float64MultiArray, queue_size = 10)
    self.time_trajectory = rospy.get_time()
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()

  def callback(self, data):
    cur_time = rospy.get_time() - self.time_trajectory
    joint_angles = self.trajectory_joint_angles(cur_time)
    joint_angles_cv = np.array(data.data)
    # for i in data.data:
    #   joint_angles.append(i)
    tm = self.T_matrices(joint_angles)
    fk = self.forward_kinematics(tm)
    tm_cv = self.T_matrices(joint_angles)
    fk_cv = self.forward_kinematics(tm)

    # print(fk)
    fk_pub = Float64MultiArray()
    fk_pub.data = fk[:3,3]
    fk_cv_pub = Float64MultiArray()
    fk_cv_pub.data = fk_cv[:3,3]


    try:
      self.forward_kinematics_pub.publish(fk_pub)
      self.forward_kinematics_cv_pub.publish(fk_cv_pub)
    except CvBridgeError as e:
      print(e)
  
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
    
    # T_2_0 = T_0[1]
    # T_3_0 = T_0[2]
    # T_4_0 = T_0[3]

    # Rotation matrices
    R_0 = []
    for i in T_0:
      R_0.append(i[:3,:3])
    # R_1_0 = R_0[0]
    # R_2_0 = R_0[1]
    # R_3_0 = R_0[2]
    # R_4_0 = R_0[3]
    
    # Linear matrices
    D_0 = []
    for i in T_0:
      D_0.append(i[:3,3])
    # D_1_0 = Ts[0][:3,3]
    # D_2_0 = T_2_0[:3,3]
    # D_3_0 = T_3_0[:3,3]
    # D_4_0 = T_4_0[:3,3]

    # Jacobian
    jacobian = np.zeros((6,4))
    z = np.array([0, 0, 1])
    for i in range(len(Ts)):
      # Linear part
      D_diff = D_0[len(Ts)-1] - D_0[i]
      linear = np.cross(np.dot(R_0[i], z), D_diff)
      for j in range(3):
        jacobian[j][i] = linear[j] 
      # Rotation part
      rotation = np.dot(R_0[i], z)
      for k in range(3):
        jacobian[k+3][i] = rotation[k]

    return jacobian

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


