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

    # Uncomment if you want to save the image
    self.y_cam_pos_yellow = cv_utils.detect_yellow(self.cv_image2)
    self.y_cam_pos_blue = cv_utils.detect_blue(self.cv_image2)
    self.y_cam_pos_green = cv_utils.detect_green(self.cv_image2)
    self.y_cam_pos_red = cv_utils.detect_red(self.cv_image2)

    global_pos = self.estimate_global_pos(self.cv_image2)
    cv_end_pos = Float64MultiArray()
    cv_end_pos.data = global_pos["red"]
    # print(cv_end_pos)
    #cv2.imwrite('image_copy.png', cv_image)
    im2=cv2.imshow('window2', self.cv_image2)
    cv2.waitKey(1)

    # Publish the results
    try: 
      self.image_pub2.publish(self.bridge.cv2_to_imgmsg(self.cv_image2, "bgr8"))
      self.cv_end_pos_pub.publish(cv_end_pos)
    except CvBridgeError as e:
      print(e)

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


