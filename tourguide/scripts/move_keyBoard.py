#! /usr/bin/env python

import rospy
from msgs.msg import CmdVel
import sys, select, termios, tty

helpDoc = """
Reading from the keyboard  and Publishing to cmd_vel!
---------------------------
Moving around:
           w :[forward]
a :[left]  s :[back]    d :[right]

r :[reset]
---------------------------
CTRL-C to quit
"""

moveKeys = {
    'w':(0.1,0),
    's':(-0.1,0),
    'a':(0,10),
    'd':(0,-10),
	'q':(0,0)
}

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

def vel(speed, angle):
	return ("currently:\tspeed %s\tangle %s " % (speed,angle))

if __name__ == "__main__":
    	settings = termios.tcgetattr(sys.stdin)
		
	rospy.init_node('move_keyBoard')
	cmdVelPub = rospy.Publisher('cmd_vel', CmdVel, queue_size = 1)

	speed = rospy.get_param("~speed", 0.0)
	angle = rospy.get_param("~angle", 0.0)

	try:
		print helpDocr
		print vel(speed,angle)
		while(1):
			key = getKey()
			print(key)
			# if key in moveKeys.keys():
			# 	speed = moveKeys[key][0]
			# 	angle = moveKeys[key][1]

			# 	print(vel(speed,angle))

			# else:
			# 	speed = 0
			# 	angle = 0
			# 	if (key == '\x03'):
			# 		break

			cmdVel = CmdVel()
			cmdVel.driverVelocity = speed
			cmdVel.steeringAngle = angle
			cmdVelPub.publish(cmdVel)

	except:
		print("\n error!!! \n")

	finally:
		cmdVel = CmdVel()
		cmdVel.driverVelocity = speed
		cmdVel.steeringAngle = angle
		cmdVelPub.publish(cmdVel)

    		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)	
