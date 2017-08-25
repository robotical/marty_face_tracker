#!/usr/bin/env python

import rospy
from marty_msgs.msg import CentroidMsg, ServoMsg, ServoMsgArray
from std_msgs.msg import Bool
from time import sleep

#Publisher for servo commands
pub = rospy.Publisher('/marty/servo_array', ServoMsgArray, queue_size=10)

#Variable to keep track of the current/desired position
global current_position
current_position = 0

def tracker(data):
    global current_position
    servo_cmd_array = ServoMsgArray()

    servo_cmd_1 = ServoMsg()
    servo_cmd_2 = ServoMsg()

    twist_size = 10 #The resolution/speed of the movement to track the face

    if(any(data.x)):    #If a face is detected
        servo_cmd_1.servo_id = 1    #Left Leg
        servo_cmd_2.servo_id = 4    #Right Leg
        if(data.x[0] < 130 and current_position < 120):
            current_position += twist_size  #Adjust current_position to get to desired position
        elif(data.x[0] > 170 and current_position > -120):
            current_position -= twist_size  #Adjust current_position to get to desired position
        else:
            pass
        servo_cmd_1.servo_cmd = current_position
        servo_cmd_2.servo_cmd = current_position
    	servo_cmd_array.servo_msg.append(servo_cmd_1)
    	servo_cmd_array.servo_msg.append(servo_cmd_2)
    	pub.publish(servo_cmd_array)   #Publis commands

def callback(data):
    #Enable motors
    rospy.Publisher('/marty/enable_motors', Bool, queue_size=1).publish(True)
    tracker(data)


def listener():
	rospy.init_node('echoer', anonymous=True)
	rospy.Subscriber("marty/face_tracking/faces_centroid/", CentroidMsg, callback)
	rospy.spin()


if __name__ == '__main__':
	listener()
