#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import Float64
from math import sin,cos,atan2,sqrt,fabs
from std_msgs.msg import String
import time
deg=0.017453293
#Define a RRBot joint positions publisher for joint controllers.
def sj605_joint_positions_publisher():
	
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
	while not rospy.is_shutdown():
	
		angle1 = input("Input your jointb angle(180~-180):")
		angle2 = input("Input your joint1 angle(90~-45):")
		angle3 = input("Input your joint2 angle(180~-90):")
		angle4 = input("Input your joint3 angle(180~-180):")
		angle5 = input("Input your joint4 angle(90~-90):")
		position1 = angle1*deg
		position2 = angle2*deg
		position3 = angle3*deg
		position4 = angle4*deg
		position5 = angle5*deg			
		pub1.publish(position1)
		pub2.publish(position2)
		pub3.publish(position3)
		pub4.publish(position4)
	        pub5.publish(position5)
		rate.sleep() #sleep for rest of rospy.Rate(100)
		print("Please Wait Move")
		time.sleep(3)
		cs=input("Continue or Shutdowm?(1 or 0):")
		if cs==0:
    			break
		else:
			continue	
		
#Main section of code that will continuously run unless rospy receives interuption (ie CTRL+C)
if __name__ == '__main__':	
    try:
        sj605_joint_positions_publisher()
    except rospy.ROSInterruptException:
        pass
