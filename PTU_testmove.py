#!/usr/bin/env python
# 
#
# Program that
# moves the pan tilt unit
# to close of program by ^C
# dml 2022
#
#
import time
import math
import random
import rospy # needed for ROS
import numpy as np # for map arrays
import matplotlib.pyplot as plt


# ROS message types
from sensor_msgs.msg import JointState       # ROS DP PTU command
from geometry_msgs.msg import Twist      # ROS Twist message


# ROS topics

motionTopic='/P2/RosAria/cmd_vel' 
ptuCmdTopic = '/P2/ptu/cmd'          # check that these are right
ptuPosTopic = '/joint_states'

#globals

gPTU = [0.0, 0.0]
gCtr=0
gWait=False

# get current PT position from PTU
# ignores velocity, effort and finished flag
#
def ptuCallback(data):
    global gPTU,gWait,gCtr
    gPTU=data.position
    gCtr += 1
    temp_pan=math.degrees(gPTU[0])
    temp_tilt=math.degrees(gPTU[1])
    if gCtr>10 and not gWait: # output the PTU position every few secs
        print("PT position:",temp_pan,temp_tilt)
        gCtr=0
    return

#
# publish a command to move the ptu unit
# pub=publisher, seq = message sequence number (monotinic increasing)
#
def sendPTcommand(pub,p,t,seq):
    msg = JointState()
    msg.header.frame_id='0'
    msg.header.stamp=rospy.Time.now()
    msg.name=["pan","tilt"]
    msg.position = [p,t]
    msg.velocity=[1,1]
    msg.effort=[.8,.8]
    pub.publish(msg)

    return seq+1

#
# move the PT unit
# ask user for input pan and tilt settings and
# send them to the PT

def PTU_test_node():
    '''send pt commands to the pt unit'''
    global gWait
    # all ROS 'nodes' (ie your program) have to do the following
    rospy.init_node('PTU_test_Node', anonymous=True)

    # register as a ROS publisher for the PTU position
    ptu_pub = rospy.Publisher(ptuCmdTopic, JointState, queue_size=10)
    ptu_sub = rospy.Subscriber(ptuPosTopic, JointState,ptuCallback)

    msg_seq_num=0 
    while not rospy.is_shutdown():
        gWait=True # don't be interrupted with messages
        print("Enter the pan and tilt values as two real numbers with a space between:")
        pan= math.radians(float(input()))
        tilt= math.radians(float(input()))
	msg_seq_num = sendPTcommand(ptu_pub,pan,tilt,msg_seq_num)
	pan=math.degrees(pan)
	tilt=math.degrees(tilt)
        print("    || Sent command in Degrees: "+ " Tilt value: "+ str(tilt) + " 	Pan value: " + str(pan)+ " ||")
        #print("   Sent command:{tilt:.2f}".format(tilt))
        #print("   Sent command:{pan:.2f}".format(pan))
        gWait=False # allow messages
        rospy.sleep(5)
      
      
      
    return

#
# This function is called by ROS when you stop ROS
# Here we use it to send a zero velocity to robot
# in case it was moving when you stopped ROS
#

def callback_shutdown():
    print("Shutting down")
    pub = rospy.Publisher(motionTopic, Twist, queue_size=1)
    msg = Twist()
    msg.angular.z=0.0
    msg.linear.x=0.0
    pub.publish(msg) 
    return



#-------------------------------MAIN  program----------------------
if __name__ == '__main__':
    try:
	rospy.on_shutdown(callback_shutdown)
	PTU_test_node()
    except rospy.ROSInterruptException:
        pass

