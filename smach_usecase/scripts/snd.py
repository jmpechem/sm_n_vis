#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('transition',String,queue_size=10)
    rospy.init_node('sender_test',anonymous=True)
    pub.publish("shutdown")

if __name__== '__main__':
   try:
	talker()
   except rospy.ROSInterruptException:
        pass
