#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
class Follower:
  def __init__(self):
    #ROS图像转换为OpenCV图像
    self.bridge = cv_bridge.CvBridge()
    cv2.namedWindow("window", 1)
    self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
  def image_callback(self, msg):
    
    image = self.bridge.imgmsg_to_cv2(msg) #CvBridge模块将ROS sensor_msgs / Image消息转换为OpenCV图像格式
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) #将OpenCV图像传递给cvtColor（）函数，以在RGB表示及其在HSV空间中的等效表示之间进行转换
    lower_yellow = numpy.array([ 50, 50, 170])
    upper_yellow = numpy.array([255, 255, 190])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    masked = cv2.bitwise_and(image, image, mask=mask)
    cv2.imshow("window", mask )
    cv2.waitKey(3)
rospy.init_node('follower')#订阅follower
follower = Follower()#follower等于类Follower
rospy.spin()
