#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
from std_msgs.msg import String
states_of_machines = ''
# define state Foo
class Foo(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2','outcome3','outcome4'])
	
    def execute(self, userdata):
	#try:
	msg = rospy.wait_for_message('transition',String)
    	states_of_machines = msg.data	
	#except rospy.ROSInterruptException: pass
        rospy.loginfo('Executing state FOO')
        if states_of_machines == "yes":	    
            return 'outcome1'
        elif states_of_machines == "no":
            return 'outcome2'
	elif states_of_machines == "shutdown":
	    return 'outcome4'
	else:
	    return 'outcome3'

# define state Bar
class Bar(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])

    def execute(self, userdata):
	#try:
	msg = rospy.wait_for_message('transition',String)
    	states_of_machines = msg.data	
	#except rospy.ROSInterruptException: pass
        rospy.loginfo('Executing state BAR')
	if states_of_machines == "return":
           return 'outcome1'
	else:
	   return 'outcome2'
        
def endhook():
    print "shutdown time!"


def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['yes','no','none','shutdown'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('FOO', Foo(), 
                               transitions={'outcome1':'yes', 'outcome2':'no', 'outcome3':'BAR','outcome4':'shutdown'})
        smach.StateMachine.add('BAR', Bar(), 
                               transitions={'outcome1':'FOO','outcome2':'BAR'})

    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    # Execute SMACH plan
    #msg = rospy.wait_for_message('transition',String)
    #states_of_machines = msg.data
    outcome = sm.execute()
    rospy.spin()
    sis.stop()
    rospy.signal_shutdown('All done.')


if __name__ == '__main__':
    try:
	main()
    except rospy.ROSInterruptException('rospy shutdown'):
        pass
