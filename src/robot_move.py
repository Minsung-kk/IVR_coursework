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
    # initialize a publisher to send images from camera1 to a topic named image_topic1
    # self.image_pub1 = rospy.Publisher("image_topic1",Image, queue_size = 1)
    # self.image_pub2 = rospy.Publisher("image_topic2",Image, queue_size = 1)
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw",Image,self.callback1)
    self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw",Image,self.callback2)
    # camera 1 data and msg
    self.x_cam_pos_pub = rospy.Publisher("x_cam_pos", Float64MultiArray, queue_size=10)
    self.red_pub = rospy.Publisher("red_pos", Float64, queue_size=10)
    self.x_cam_pos_sub = rospy.Subscriber("x_cam_pos", Float64MultiArray, self.x_cam_callback)
    self.template = cv2.imread("image_crop.png", 0)
    
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()
  
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

  # 3D positions
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
    a = cv_utils.pixel2meter(image)
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
    a = cv_utils.pixel2meter(self.cv_image2)
    orange_mask = cv_utils.detect_orange(self.cv_image2)
    target_proj_pos2 = cv_utils.find_target(orange_mask, self.template)
    target_proj_pos2[0] = target_proj_pos2[0] + 24 
    target_proj_pos2[1]= target_proj_pos2[1] + 24
    target_x = (target_proj_pos2[0] - self.y_cam_pos_yellow[0]) * a
    target_y = (self.x_cam_pos_tar[0] - self.x_cam_pos_yellow[0]) *a
    target_z = (self.x_cam_pos_yellow[1] - self.x_cam_pos_tar[1]) * a
    return np.array([target_x, target_y, target_z])  

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
      inv_flag = 1

    # if np.abs(v_c[0])<0.5 and np.abs(self.last_ja4-ja4)>0.5 and inv_flag == 1:
    #   ja4 = -ja4
    # self.last_ja4 = ja4
    return np.array([0, ja2, ja3, ja4])

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
  # Recieve data from camera 1, process it, and publish
  def callback1(self,data):
    # Recieve the image
    try:
      self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
      # self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # find circles for camera1  
    x_cam_pos = self.get_cirlces()
    # red = Float64()
    # red = cv_utils.detect_red(self.cv_image1)
    
    # Publish the results
    try: 
      # self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image1, "bgr8"))
      self.x_cam_pos_pub.publish(x_cam_pos)
      # self.red_pub.publish(red)
    except CvBridgeError as e:
      print(e)

  def callback2(self,data):
    # Recieve the image
    try:
      self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)


    # estimate 3D circles' positions 
    global_circle_pos = self.estimate_global_pos(self.cv_image2)
    # 3D orange sphere position
    global_target_pos = self.estimate_target_3Dposition()
    jas_cv = self.calcu_jas_from_vision(global_pos=global_circle_pos)
# 
    # # Publish the results
    # try: 
    #   # self.image_pub2.publish(self.bridge.cv2_to_imgmsg(self.cv_image2, "bgr8"))
    # except CvBridgeError as e:
    #   print(e)

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


