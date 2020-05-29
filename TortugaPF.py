#! /usr/bin/env python
#Tortuga Proyecto Final

import rospy
import math
import time
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn
from std_srvs.srv import Empty
from turtlesim.srv import SetPen

from turtlesim.msg import *
from turtlesim.srv import *
from std_srvs.srv import *
import random

objetivoX=0      #posicion final x
objetivoY=0      #posicion fianl y

x = 0
y = 0
z = 0
theta = 0

x2 = 0
y2 = 0
theta2 = 0

x3 = 0
y3 = 0
theta3 = 0

x4 = 0
y4 = 0
theta4 = 0

R = 0
G = 0
B = 0

def colores():
    global R
    global G
    global B
    R = random.randint(0,255)
    G = random.randint(0,255)
    B = random.randint(0,255)

def poseCallback(pose_message):
    global x
    global y
    global theta

    x = pose_message.x
    y = pose_message.y
    theta = pose_message.theta
    #print ('x1=', x, 'y1=', y)

def poseCallback2(pose_message):
    global x2
    global y2
    global theta2

    x2 = pose_message.x
    y2 = pose_message.y
    theta2 = pose_message.theta
    #print ('x2=', x2, 'y2=', y2)

def poseCallback3(pose_message):
    global x3
    global y3
    global theta3

    x3 = pose_message.x
    y3 = pose_message.y
    theta3 = pose_message.theta
    #print ('x3=', x3, 'y3=', y3)

def poseCallback4(pose_message):
    global x4
    global y4
    global theta4

    x4 = pose_message.x
    y4 = pose_message.y
    theta4 = pose_message.theta

def orientate (xgoal, ygoal):
    global x4
    global y4
    global theta4

    velocity_message = Twist()
    cmd_vel_topic = '/turtle4/cmd_vel'

    while(True):
        ka = 4.0
        desired_angle_goal = math.atan2(ygoal-y4, xgoal-x4)
        dtheta = desired_angle_goal-theta4
        angular_speed = ka * (dtheta)

        velocity_message.linear.x = 0.0
        velocity_message.angular.z = angular_speed
        velocity_publisher.publish(velocity_message)
        print ('x=', x, 'y=', y)

        if (dtheta < 0.01):
            break

def go_to_goal_col (xgoal, ygoal):

    global x4
    global y4
    global theta4
    global R
    global G
    global B
    global objetivoX
    global oojetivoY

    orientate(xgoal,ygoal)

    velocity_message = Twist()
    cmd_vel_topic = '/turtle4/cmd_vel'

    while(True):
        kv = 0.5				
        distance = abs(math.sqrt(((xgoal-x4)**2)+((ygoal-y4)**2)))
        linear_speed = kv * distance

        ka = 4.0
        desired_angle_goal = math.atan2(ygoal-y4, xgoal-x4)
        dtheta = desired_angle_goal-theta4
        angular_speed = ka * (dtheta)

        velocity_message.linear.x = linear_speed
        velocity_message.angular.z = angular_speed
        velocity_publisher.publish(velocity_message)
        print ('x=', x, 'y=', y)

        colores()
        turtle4Pen(R,G,B,5,0)

        if (distance < 0.01):
            orientate(objetivoX,objetivoY)
            time.sleep(1)
            objetivo(objetivoX,objetivoY)
            break


def go_to_goal (xgoal, ygoal):

    global x4
    global y4
    global theta4
    global R
    global G
    global B
    global objetivoX
    global oojetivoY

    orientate(xgoal,ygoal)

    velocity_message = Twist()
    cmd_vel_topic = '/turtle4/cmd_vel'

    while(True):
        kv = 0.5				
        distance = abs(math.sqrt(((xgoal-x4)**2)+((ygoal-y4)**2)))
        linear_speed = kv * distance

        ka = 4.0
        desired_angle_goal = math.atan2(ygoal-y4, xgoal-x4)
        dtheta = desired_angle_goal-theta4
        angular_speed = ka * (dtheta)

        velocity_message.linear.x = linear_speed
        velocity_message.angular.z = angular_speed
        velocity_publisher.publish(velocity_message)
        print ('x=', x, 'y=', y)

        colores()
        turtle4Pen(R,G,B,5,0)

        if (distance > 0.01):
            objetivo(objetivoX,objetivoY)
            break

        if (distance < 0.01):
            break


def objetivo(xgoal,ygoal):
    global x
    global y
    global theta
    global x2
    global y2
    global theta2
    global x3
    global y3
    global theta3
    global x4
    global y4
    global theta4


    while(True):
        distance = abs(math.sqrt(((xgoal-x4)**2)+((ygoal-y4)**2)))
        dx1=0 #distancia en x tortuga 4-1
        dx2=0 #distancia en x tortuga 4-2
        dx3=0 #distancia en x tortuga 4-3
        dy1=0 #distancia en y tortuga 4-1
        dy2=0 #distancia en y tortuga 4-2
        dy3=0 #distancia en y tortuga 4-3

        dx1=x2-x4
        dx2=x3-x4
        dx3=x-x4

        dy1=y2-y4
        dy2=y3-y4
        dy3=y-y4

        d1=abs(math.sqrt(((dx1)**2)+((dy1)**2)))
        d2=abs(math.sqrt(((dx2)**2)+((dy2)**2)))
        d3=abs(math.sqrt(((dx3)**2)+((dy3)**2)))

        NPx=0
        NPy=0


        if (d1<1):
            if(objetivoY>y4):
                NPx=x4-0.25
                NPy=y4-0.25
                go_to_goal_col(NPx,NPy)
                break
            if(objetivoY<y4):
                NPx=x4+0.25
                NPy=y4+0.25
                go_to_goal_col(NPx,NPy)
                break
            break

        if (d2<1):
            if(objetivoY>y4):
                NPx=x4-0.25
                NPy=y4-0.25
                go_to_goal_col(NPx,NPy)
                break
            if(objetivoY<y4):
                NPx=x4+0.25
                NPy=y4+0.25
                go_to_goal_col(NPx,NPy)
                break
            break

        if (d3<1):
            if(objetivoY>y4):
                NPx=x4-0.25
                NPy=y4-0.25
                orientate(NPx,NPy)
                go_to_goal_col(NPx,NPy)
                break
            if(objetivoY<y4):
                NPx=x4+0.25
                NPy=y4+0.25
                orientate(NPx,NPy)
                go_to_goal_col(NPx,NPy)
                break
            break

        if (distance<0.5):
            break

        orientate(xgoal,ygoal)
        go_to_goal(xgoal,ygoal)



if __name__ == '__main__':
    try:

        rospy.init_node('turtlesim_motion_pose', anonymous = True)

        cmd_vel_topic = '/turtle4/cmd_vel'
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size = 1)

        position_topic1 = "/turtle1/pose"
        pose_subscriber = rospy.Subscriber(position_topic1, Pose, poseCallback)
        time.sleep(2)

        position_topic2 = "/turtle2/pose"
        pose_subscriber = rospy.Subscriber(position_topic2, Pose, poseCallback2)
        time.sleep(2)

        position_topic3 = "/turtle3/pose"
        pose_subscriber = rospy.Subscriber(position_topic3, Pose, poseCallback3)
        time.sleep(2)

        position_topic4 = "/turtle4/pose"
        pose_subscriber = rospy.Subscriber(position_topic4, Pose, poseCallback4)
        time.sleep(2)

	rospy.wait_for_service('turtle4/set_pen')
	turtle4Pen = rospy.ServiceProxy('/turtle4/set_pen', SetPen)
	
	objetivoX=10      #posicion final x
	objetivoY=1
	orientate(objetivoX,objetivoY)
	time.sleep(1)
	objetivo(objetivoX,objetivoY)
	time.sleep(1)
	
	
	objetivoX=4      #posicion final x
	objetivoY=10
	orientate(objetivoX,objetivoY)
	time.sleep(1)
	objetivo(objetivoX,objetivoY)
	time.sleep(1)
	
	objetivoX=1      #posicion final x
	objetivoY=1
	orientate(objetivoX,objetivoY)
	time.sleep(1)
	objetivo(objetivoX,objetivoY)
	time.sleep(1)
	
	objetivoX=7      #posicion final x
	objetivoY=10
	orientate(objetivoX,objetivoY)
	time.sleep(1)
	objetivo(objetivoX,objetivoY)
	time.sleep(1)
	
    except rospy.ROSInterruptException:
	pass
