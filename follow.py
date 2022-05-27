#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import rospy
from std_msgs.msg import String
import numpy as np
from geometry_msgs.msg import Twist
from bbox_msgs.msg import BoundingBox

class conjection_converter:

  def __init__(self):
    self.vel_pub = rospy.Publisher("cmd_vel",Twist) # 速度のPublish
    self.yolo_sub = rospy.Subscriber("lookingbbox",BoundingBox,self.callback)  #yoloから送られてくる情報のsubscribej #メッセージ型に依存
    


  def callback(self,data):

    #yolo側からの情報整理
    pkmon_xmin=data.xmin
    pkmon_xmax=data.xmax
    pkmon_ymin=data.ymin
    pkmon_ymax=data.ymax
    pkmon_center=data.center
    w=640
    h=480
    area=w*h


    
    cmd_vel = Twist() # 速度のメッセージ型を用意


    #角度の設定

    if ( w/4> pkmon_center[0]) and (pkmon_center[0] >= 0):   # 左側
      rospy.loginfo("Left side")
      cmd_vel.angular.z = 0.20 # 角速度の設定
    elif (w*3/4 > pkmon_center[0]) and (pkmon_center[0] >= w/4): # 真ん中
      rospy.loginfo("Center")
      cmd_vel.angular.z = 0.00 # 角速度の設定
    elif (w >= pkmon_center[0] ) and (pkmon_center[0] >= w*3/4): # 右側
      rospy.loginfo("Right side")
      cmd_vel.angular.z = -0.20 # 角速度の設定
    else:
      cmd_vel.angular.z = 0.00 # 角速度の設定
    
    


    #画像上での物体の比率計算
    verticl=pkmon_ymax-pkmon_ymin
    besaide=pkmon_xmax-pkmon_xmin
    area_obj=verticl*besaide
    area_rate=float(area_obj)/float(area)
    pre_area_rate=area_rate




     

    # 直進速度の設定　比率が小さきときは進む．大きいときは止まる．

    if (area_rate<0.15):
        cmd_vel.linear.x  = 0.05 # 直進速度の設定
    elif (area_rate >= 0.15):
        cmd_vel.linear.x  = 0.00  # 直進速度の設定
    else:
        cmd_vel.linear.x  = 0.00 # 直進速度の設定
    

    #比率が急激に変化したとき止まる.

    

    
    self.vel_pub.publish( cmd_vel)





def main(args):
  ic = conjection_converter()
  rospy.init_node('follow', anonymous=False)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  
  
if __name__ == '__main__':
    main(sys.argv)

