#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Float64
from math import sin,cos,atan2,sqrt,fabs
from std_msgs.msg import String
import time
import numpy
deg=57.295779513
#Initiate node for controlling joint1 and joint2 positions.
rospy.init_node('joint_positions_node', anonymous=True)
#Define publishers for each joint position controller commands.
pub1 = rospy.Publisher('/sj605/joint1_position_controller/command', Float64, queue_size=20)
pub2 = rospy.Publisher('/sj605/joint2_position_controller/command', Float64, queue_size=20)
pub3 = rospy.Publisher('/sj605/joint3_position_controller/command', Float64, queue_size=20)
pub4 = rospy.Publisher('/sj605/joint4_position_controller/command', Float64, queue_size=20)
pub5 = rospy.Publisher('/sj605/joint5_position_controller/command', Float64, queue_size=20)
pub6 = rospy.Publisher('/sj605/joint6_position_controller/command', Float64, queue_size=20)
rate = rospy.Rate(100) #100 Hz	
#Define a RRBot joint positions publisher for joint controllers.
def rrbot_joint_positions_publisher(p1,p2,p3,p4,p5):
	pub1.publish(p1)
	pub2.publish(p2)
	pub3.publish(p3)
	pub4.publish(p4)
        pub5.publish(p5)
	rospy.loginfo("Jb deg= %.2f,J1 deg= %.2f,J2 deg= %.2f,J3 deg= %.2f,J4 deg= %.2f",p1*deg,p2*deg,p3*deg,p4*deg,p5*deg)
	#rate.sleep() #sleep for rest of rospy.Rate(100)
def position():
	for i in numpy.arange(-1.57,3.14,1.57):
		rrbot_joint_positions_publisher(i,-0.35,-0.53,0,-0.67)
		time.sleep(3)
		rrbot_joint_positions_publisher(i,-0.1,-0.3,0,-1.14)
		time.sleep(3)
		rrbot_joint_positions_publisher(i,-0.76,0.28,0,-1.07)
		time.sleep(3)		
def init():
	time.sleep(5)
	rospy.loginfo("========initing========")
	rrbot_joint_positions_publisher(0,0,0,0,0)
	time.sleep(5)			
#Main section of code that will continuously run unless rospy receives interuption (ie CTRL+C)
if __name__ == '__main__':	
    try:
	init()		
	position()
	init()
	rospy.loginfo("finish")
    except rospy.ROSInterruptException:
        pass
