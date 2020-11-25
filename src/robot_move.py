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


class image_converter:

  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('image_processing', anonymous=True)
    # initialize a publisher to send messages to a topic named image_topic
    self.image_pub = rospy.Publisher("image_topic",Image, queue_size = 1)
    # initialize a publisher to send joints' angular position to a topic called joints_pos
    self.joints_pub = rospy.Publisher("joints_pos",Float64MultiArray, queue_size=10)
    # initialize a publisher to send robot end-effector position
    self.end_effector_pub = rospy.Publisher("end_effector_prediction",Float64MultiArray, queue_size=10)
    # initialize a publisher to send desired trajectory
    self.trajectory_pub = rospy.Publisher("trajectory",Float64MultiArray, queue_size=10)
    # initialize a publisher to send joints' angular position to the robot
    self.robot_joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
    self.robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
    self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
    self.robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub = rospy.Subscriber("/robot/camera1/image_raw",Image,self.callback)
    # record the begining time
    self.time_trajectory = rospy.get_time()
    # initialize errors
    self.time_previous_step = np.array([rospy.get_time()], dtype='float64')     
    self.time_previous_step2 = np.array([rospy.get_time()], dtype='float64')   
    # initialize error and derivative of error for trajectory tracking  
    self.error = np.array([0.0,0.0], dtype='float64')  
    self.error_d = np.array([0.0,0.0], dtype='float64') 

    
  # In this method you can focus on detecting the centre of the red circle
  def detect_red(self,image):
      # Isolate the blue colour in the image as a binary image
      mask = cv2.inRange(image, (0, 0, 100), (0, 0, 255))
      # This applies a dilate that makes the binary region larger (the more iterations the larger it becomes)
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      # Obtain the moments of the binary image
      M = cv2.moments(mask)
      # Calculate pixel coordinates for the centre of the blob
      cx = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
      return np.array([cx, cy])

  # Detecting the centre of the green circle
  def detect_green(self,image):
      mask = cv2.inRange(image, (0, 100, 0), (0, 255, 0))
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      M = cv2.moments(mask)
      cx = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
      return np.array([cx, cy])

  # Detecting the centre of the blue circle
  def detect_blue(self,image):
      mask = cv2.inRange(image, (100, 0, 0), (255, 0, 0))
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      M = cv2.moments(mask)
      cx = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
      return np.array([cx, cy])

  # Detecting the centre of the yellow circle
  def detect_yellow(self,image):
      mask = cv2.inRange(image, (0, 100, 100), (0, 255, 255))
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      M = cv2.moments(mask)
      cx = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
      return np.array([cx, cy])

  # Calculate the conversion from pixel to meter
  def pixel2meter(self,image):
      # Obtain the centre of each coloured blob
      circle1Pos = self.detect_blue(image)
      circle2Pos = self.detect_green(image)
      # find the distance between two circles
      dist = np.sum((circle1Pos - circle2Pos)**2)
      return 3 / np.sqrt(dist)

    # Calculate the relevant joint angles from the image
  def detect_joint_angles(self,image):
    a = self.pixel2meter(image)
    # Obtain the centre of each coloured blob 
    center = a * self.detect_yellow(image)
    circle1Pos = a * self.detect_blue(image) 
    circle2Pos = a * self.detect_green(image) 
    circle3Pos = a * self.detect_red(image)
    # Solve using trigonometry
    ja1 = np.arctan2(center[0]- circle1Pos[0], center[1] - circle1Pos[1])
    ja2 = np.arctan2(circle1Pos[0]-circle2Pos[0], circle1Pos[1]-circle2Pos[1]) - ja1
    ja3 = np.arctan2(circle2Pos[0]-circle3Pos[0], circle2Pos[1]-circle3Pos[1]) - ja2 - ja1
    return np.array([ja1, ja2, ja3])

    # detect robot end-effector from the image
  def detect_end_effector(self,image):
    a = self.pixel2meter(image)
    endPos = a * (self.detect_yellow(image) - self.detect_red(image))
    return endPos

  # Define a circular trajectory
  def trajectory(self):
    # get current time
    cur_time = np.array([rospy.get_time() - self.time_trajectory])
    x_d = float(6* np.cos(cur_time * np.pi/100))
    y_d = float(6 + np.absolute(1.5* np.sin(cur_time * np.pi/100)))
    return np.array([x_d, y_d])

  # Calculate the forward kinematics
  def forward_kinematics(self,image):
    joints = self.detect_joint_angles(image)
    end_effector = np.array([3 * np.sin(joints[0]) + 3 * np.sin(joints[0]+joints[1]) + 3 *np.sin(joints.sum()), 3 * np.cos(joints[0]) + 3 * np.cos(joints[0]+joints[1]) + 3 * np.cos(joints.sum())])
    return end_effector

  # Calculate the robot Jacobian
  # def calculate_jacobian(self,image):
  #   joints = self.detect_joint_angles(image)
  #   jacobian = np.array([[3 * np.cos(joints[0]) + 3 * np.cos(joints[0]+joints[1]) + 3 *np.cos(joints.sum()), 3 * np.cos(joints[0]+joints[1]) + 3 *np.cos(joints.sum()),  3 *np.cos(joints.sum())], [-3 * np.sin(joints[0]) - 3 * np.sin(joints[0]+joints[1]) - 3 * np.sin(joints.sum()), - 3 * np.sin(joints[0]+joints[1]) - 3 * np.sin(joints.sum()), - 3 * np.sin(joints.sum())]])
  #   return jacobian
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

  def target_position(cur_time):
    x_d = Float64(2.5* np.cos(cur_time * np.pi/15)) + 0.5
    y_d = Float64(2.5* np.sin(cur_time * np.pi/15)) + 0
    z_d = Float64(1* np.sin(cur_time * np.pi/15)) + 7
    return [x_d, y_d, z_d]
  # Estimate control inputs for open-loop control
  def control_open(self,image):
    # estimate time step
    cur_time = rospy.get_time()
    dt = cur_time - self.time_previous_step2
    self.time_previous_step2 = cur_time
    q = self.detect_joint_angles(image) # estimate initial value of joints'
    J_inv = np.linalg.pinv(self.calculate_jacobian(image))  # calculating the psudeo inverse of Jacobian
    # desired trajectory
    pos_d= self.trajectory() 
    # estimate derivative of desired trajectory
    self.error_d = (pos_d - self.error)/dt
    self.error = pos_d
    q_d = q + (dt * np.dot(J_inv, self.error_d.transpose()))  # desired joint angles to follow the trajectory
    return q_d


  def control_closed(self,image):
    # P gain
    K_p = np.array([[10,0],[0,10]])
    # D gain
    K_d = np.array([[0.1,0],[0,0.1]])
    # estimate time step
    cur_time = np.array([rospy.get_time()])
    dt = cur_time - self.time_previous_step
    self.time_previous_step = cur_time
    # robot end-effector position
    pos = self.detect_end_effector(image)
    # desired trajectory
    pos_d= self.trajectory() 
    # estimate derivative of error
    self.error_d = ((pos_d - pos) - self.error)/dt
    # estimate error
    self.error = pos_d-pos
    q = self.detect_joint_angles(image) # estimate initial value of joints'
    J_inv = np.linalg.pinv(self.calculate_jacobian(image))  # calculating the psudeo inverse of Jacobian
    dq_d =np.dot(J_inv, ( np.dot(K_d,self.error_d.transpose()) + np.dot(K_p,self.error.transpose()) ) )  # control input (angular velocity of joints)
    q_d = q + (dt * dq_d)  # control input (angular position of joints)
    return q_d

  # Recieve data, process it, and publish
  def callback(self,data):
    # Recieve the image
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    
    # Perform image processing task (your code goes here)
    # The image is loaded as cv_imag
    # rospy.init_node('target_pos_cmd', anonymous=True)
    # rate = rospy.Rate(30) # 30hz
    # Uncomment if you want to save the image
    #cv2.imwrite('image_copy.png', cv_image)

    cv2.imshow('window', cv_image)
    cv2.waitKey(3)
    
    # publish robot joints angles (lab 1 and 2)
    self.joints=Float64MultiArray()
    self.joints.data= self.detect_joint_angles(cv_image)

    
    # compare the estimated position of robot end-effector calculated from images with forward kinematics(lab 3)
    x_e = self.forward_kinematics(cv_image)
    x_e_image = self.detect_end_effector(cv_image)
    self.end_effector=Float64MultiArray()
    self.end_effector.data= x_e_image   

    # send control commands to joints (lab 3)
    q_d = self.control_closed(cv_image)
    #q_d = self.control_open(cv_image)
    self.joint1=Float64()
    self.joint1.data= q_d[0]
    self.joint2=Float64()
    self.joint2.data= q_d[1]
    self.joint3=Float64()
    self.joint3.data= q_d[2]

    # Publishing the desired trajectory on a topic named trajectory(lab 3)
    x_d = self.trajectory()    # getting the desired trajectory
    self.trajectory_desired= Float64MultiArray()
    self.trajectory_desired.data=x_d

    # Publish the results
    try: 
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
      self.joints_pub.publish(self.joints)
      self.end_effector_pub.publish(self.end_effector)
      self.trajectory_pub.publish(self.trajectory_desired)
      self.robot_joint1_pub.publish(self.joint1)
      self.robot_joint2_pub.publish(self.joint2)
      self.robot_joint3_pub.publish(self.joint3)
    except CvBridgeError as e:
      print(e)
    # rate.sleep()

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


