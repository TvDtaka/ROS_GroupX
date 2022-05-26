#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import rospy
import cv2 # OpenCVライブラリのインポート
from std_msgs.msg import String
from sensor_msgs.msg import Image # ROSの画像の型のインポート
from cv_bridge import CvBridge, CvBridgeError # CvBridgeのインポート
import numpy as np
from geometry_msgs.msg import Twist
#import message_filters  #　複数メッセージの同期

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("masked_image",Image)
    self.bridge = CvBridge() # CvBridgeクラスの宣言
    self.image_sub = rospy.Subscriber("image_raw",Image,self.callback)
    self.vel_pub = rospy.Publisher("cmd_vel",Twist) # 速度のPublish
    self.yolo_sub = rospy.Subscriber("yolox",float,self.callback)  #yoloから送られてくる情報のsubscribej #メッセージ型に依存
    


  def callback(self,data,yolo):
    try:
      # 画像の型を変換．同時に色順をOpenCVの標準の並びに直す
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    


    h, w, _ = cv_image.shape   #画像データの取得

    #yolo側から送られてきた数値を実際の画像データに変換する．
    image_x=yolo.x*w

    
    cmd_vel = Twist() # 速度のメッセージ型を用意

    #角度の設定
    if (w/3 > image_x) and (image_x >= 0):   # 左側
      rospy.loginfo("Left side")
      cmd_vel.angular.z = 0.20 # 角速度の設定
    elif (w*2/3 > image_x) and (image_x >= w/3): # 真ん中
      rospy.loginfo("Center")
      cmd_vel.angular.z = 0.00 # 角速度の設定
    elif (w >= image_x) and (image_x >= w*2/3): # 右側
      rospy.loginfo("Right side")
      cmd_vel.angular.z = -0.20 # 角速度の設定
    else:
      cmd_vel.angular.z = 0.00 # 角速度の設定
    
    
    # 直進速度の設定  座標のみが送られてきた場合．
    '''
    if (image_y >= 0) and (h/3 >= image_y):   # 画像の上
      cmd_vel.linear.x  = 0.00 # 直進速度の設定
    elif (h*2/3 > image_y) and (image_y > h/3): # 画像の真ん中
      cmd_vel.linear.x  = 0.05 # 直進速度の設定
    elif (image_y > h*2/3) and (h >= image_y): # 画像の下
      cmd_vel.linear.x  = 0.00  # 直進速度の設定
    else:
      cmd_vel.linear.x  = 0.00 # 直進速度の設定
    '''

    # 直進速度の設定　比率が送られてきた場合

    if (yolo.w<0.75) or (yolo.h<0.50):
        cmd_vel.linear.x  = 0.05 # 直進速度の設定
    elif (yolo.w > 0.75) or (yolo.h>0.50):
        cmd_vel.linear.x  = 0.00  # 直進速度の設定
    else:
        cmd_vel.linear.x  = 0.00 # 直進速度の設定
    

    
    self.vel_pub.publish( cmd_vel)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)




def main(args):
  ic = image_converter()
  rospy.init_node('follow', anonymous=False)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

