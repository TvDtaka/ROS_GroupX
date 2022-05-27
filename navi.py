#!/usr/bin/env python
# -*- coding: utf-8 -*-

import actionlib
from actionlib_msgs.msg import GoalStatus
#座標や回転角に関わるメッセージ
from geometry_msgs.msg import Point, PoseStamped, Quaternion
#move_baseに関わるメッセージ
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import rospy
import tf.transformations
import numpy as np


def navi(place_mark,place_agora):
    rospy.init_node('navigation')
    #move_baseのクライアントの宣言
    cli = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
    cli.wait_for_server()
    

    while place_mark < 4:
        #目標地点の設定  
        if place_mark < 4:
            goal_x = place_agora[place_mark,0]
            goal_y = place_agora[place_mark,1]
            goal_yaw = place_agora[place_mark,2]
        else:
            goal_x = 0.0
            goal_y = 0.0
            goal_yaw = 0.0

        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        #基準となるフレームの宣言
        pose.header.frame_id = "map"
        #目標地点の座標の宣言
        pose.pose.position = Point(goal_x, goal_y, 0)
        #目標地点到着時の向きの宣言
        quat = tf.transformations.quaternion_from_euler(0, 0, goal_yaw)
        pose.pose.orientation = Quaternion(*quat)
        #move baseの目標値に関するクラスの宣言
        goal = MoveBaseGoal()
        goal.target_pose = pose

        cli.send_goal(goal)
        cli.wait_for_result()

        action_state = cli.get_state()
        if action_state == GoalStatus.SUCCEEDED:
            rospy.loginfo("Navigation Succeeded.Next place  GO GO GO!")
            place_mark+=1
        
            


if __name__ == '__main__':

    count = 0
    place_agora=np.zeros((4,3),dtype=float)
    # 配列の意味：4地点，ｘ，ｙ，yaw角の３情報
    
    # １つ目の地点
    place_agora[0,0]=0.538268230631
    place_agora[0,1]= 0.0938405069591
    place_agora[0,2]=2.92475126107
    # 2つ目の地点
    place_agora[1,0]=0
    place_agora[1,1]=0
    place_agora[1,2]=0
    # 3つ目の地点
    place_agora[2,0]=0
    place_agora[2,1]=0
    place_agora[2,2]=0
    # 4つ目の地点
    place_agora[3,0]=0
    place_agora[3,1]=0
    place_agora[3,2]=0
    while not rospy.is_shutdown():
        navi(count,place_agora)



