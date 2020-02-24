#!/usr/bin/env python


## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy  
#Jointstates package contains the states of our robot's joints etc vel,pos,effect
from sensor_msgs.msg import JointState 
# Header- to publish a message 
from std_msgs.msg import Header


def talker():
# initialise class publisher - instance to publish in the topic; the message between the publisher and subscriber
    
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)

    rospy.init_node('custom_joint_state_publisher')
    rate = rospy.Rate(10) # 10hz

     # Joint 1:
    joint = JointState()# creating an instance
    joint.header = Header() # creating a header
    joint.header.stamp = rospy.Time.now() # a header file contains the time.stamp which tells you when the message was published to ensure the subscriber takes the most recent message from publisher(talker)
    x = 0
    while not rospy.is_shutdown(): # user input
	joint.name = ['base_link_link_1', 'my_first_link1_link_2','link2_link_3','link3_link_4']	
	
	#Initialize Joint State
	if x == 0:
		print"x == 0"
		joint.name = ['base_link_link_1', 'my_first_link1_link_2','link2_link_3','link3_link_4']
		joint.position = [0.0, 0.0, 0.0, 0.0]
		joint.header.stamp =rospy.Time.now()
		pub.publish(joint)
		x += 1

	# Joint 1:
	print "User input for joint 1:"
	input_val = raw_input()
	angle1 = float(input_val)

	# Joint 2:
	print "User input for joint 2:"
	input_val = raw_input()
	angle2 = float(input_val)

	while angle2 < -1.35 or angle2 > 1.35:
		print"Error, beyond joint limit, must be between pi/2 and -pi/2. Please enter again:" 
		input_val = raw_input()
		angle2 = float(input_val)

	# Joint 3
	print "User input for joint 3:"
	input_val = raw_input()
	angle3 = float(input_val)

	while angle3 < -1.571 or angle3 > 1.571:
		print"Error, beyond joint limit, must be between pi/2 and -pi/2. Please enter again:" 
		input_val = raw_input()
		angle3 = float(input_val)

	# Joint 4
	print "User input for joint 4:"
	input_val = raw_input()
	angle4 = float(input_val)

	while angle4 < -1.571 or angle4 > 1.571:
		print"Error, beyond joint limit, must be between pi/2 and -pi/2. Please enter again:" 
		input_val = raw_input()
		angle4 = float(input_val)

	# Publish
	joint.position = [angle1, angle2, angle3, angle4]
	joint.header.stamp =rospy.Time.now()
	pub.publish(joint)
	print "Position updated!"
	print 
	print

	rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
