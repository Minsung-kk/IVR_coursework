#!/usr/bin/env python3


import rospy
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Float64

# Joint angles given time
def joint_angles(time):
  # a1 = 180*np.sin(np.pi/15*t)
  a1 = 0
  a2 = 90*np.sin(np.pi/15*time)
  a3 = 90*np.sin(np.pi/18*time)
  a4 = 90*np.sin(np.pi/20*time)
  return [a1, a2, a3, a4]

# Transformation matrices
def T_matrices(joint_angles):
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
  return Ts

# Forward Kinematics using transformation matrices
def forward_kinematics(Ts):
  temp = np.identity(4)
  for i in Ts:
        temp = np.dot(temp, i)
  return temp

# Jacobian using transformation matrices
def jacobian(Ts):
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

  # pseudo inverse
  # np.linalg.pinv(jacobian)

# Publish data
def move():
  rospy.init_node('target_pos_cmd', anonymous=True)
  rate = rospy.Rate(30) # 30hz
  # initialize a publisher to send joints' angular position to the robot
  robot_joint1_pub = rospy.Publisher("/target/x_position_controller/command", Float64, queue_size=10)
  robot_joint2_pub = rospy.Publisher("/target/y_position_controller/command", Float64, queue_size=10)
  robot_joint3_pub = rospy.Publisher("/target/z_position_controller/command", Float64, queue_size=10)
  robot_joint4_pub = rospy.Publisher("/target2/x2_position_controller/command", Float64, queue_size=10)
  robot_joint5_pub = rospy.Publisher("/target2/y2_position_controller/command", Float64, queue_size=10)
  robot_joint6_pub = rospy.Publisher("/target2/z2_position_controller/command", Float64, queue_size=10)

  # Initialize a publisher for the robot
  robot_joint11_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
  robot_joint22_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
  robot_joint33_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
  robot_joint44_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)

  t0 = rospy.get_time()
  while not rospy.is_shutdown():
    cur_time = np.array([rospy.get_time()])-t0
    #y_d = float(6 + np.absolute(1.5* np.sin(cur_time * np.pi/100)))
    x_d = 2.5* np.cos(cur_time * np.pi/15)
    y_d = 2.5* np.sin(cur_time * np.pi/15)
    z_d = 1* np.sin(cur_time * np.pi/15)
    joint1=Float64()
    joint1.data= 0.5 + x_d
    joint2=Float64()
    joint2.data= 0 + y_d
    joint3=Float64()
    joint3.data= 7 + z_d
    robot_joint1_pub.publish(joint1)
    robot_joint2_pub.publish(joint2)
    robot_joint3_pub.publish(joint3)
    x_d = 2+ 2* np.cos(cur_time * np.pi/15)
    y_d = 2.5+ 1.5* np.sin(cur_time * np.pi/15)
    joint4=Float64()
    joint4.data=  x_d
    joint5=Float64()
    joint5.data=  y_d
    joint6=Float64()
    joint6.data= 7.5
    robot_joint4_pub.publish(joint4)
    robot_joint5_pub.publish(joint5)
    robot_joint6_pub.publish(joint6)
    rate.sleep()

    j2 = ((90*np.sin(np.pi/15*cur_time))/180.0)*np.pi
    j3 = ((90*np.sin(np.pi/18*cur_time))/180.0)*np.pi
    j4 = ((90*np.sin(np.pi/20*cur_time))/180.0)*np.pi
    joint22=Float64()
    joint22.data = j2
    joint33=Float64()
    joint33.data = j3
    joint44=Float64()
    joint44.data = j4
    robot_joint22_pub.publish(joint22)
    robot_joint33_pub.publish(joint33)
    robot_joint44_pub.publish(joint44)
    rate.sleep()

# run the code if the node is called
if __name__ == '__main__':
  try:
    # ja = joint_angles(0)
    # Ts = T_matrices(ja)
    # # print(Ts)
    # jacobian(Ts)
    move()
  except rospy.ROSInterruptException:
    pass


