#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
from std_msgs.msg import String
trans_tag = "";
# define state Foo
class Foo(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['outcome1','outcome2'],
                             input_keys=['foo_counter_in'],
                             output_keys=['foo_counter_out'])

    def execute(self, userdata):
	rospy.sleep(3.0)
        rospy.loginfo('Executing state FOO')
        if userdata.foo_counter_in == 3:
            #userdata.foo_counter_out = userdata.foo_counter_in + 1
            return 'outcome1'
        elif userdata.foo_counter_in == 10:
            return 'outcome2'


# define state Bar
class Bar(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['outcome1'],
                             input_keys=['bar_counter_in'])
        
    def execute(self, userdata):
	rospy.sleep(3.0)
        rospy.loginfo('Executing state BAR')
        rospy.loginfo('Counter = %f'%userdata.bar_counter_in)        
        return 'outcome1'
        
def cb(data):
    global trans_tag
    sm.userdata.sm_counter = data.data
    trans_tag = data.data
    rospy.loginfo(data.data)
    rospy.loginfo(trans_tag)


def main():
    rospy.init_node('smach_example_state_machine')
    rospy.Subscriber("/test",String,cb)
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4'])
    sm.userdata.sm_counter = 0

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('FOO', Foo(), 
                               transitions={'outcome1':'BAR', 
                                            'outcome2':'outcome4'},
                               remapping={'foo_counter_in':'sm_counter', 
                                          'foo_counter_out':'sm_counter'})
        smach.StateMachine.add('BAR', Bar(), 
                               transitions={'outcome1':'FOO'},
                               remapping={'bar_counter_in':'sm_counter'})


    # Execute SMACH plan
   #sis = smach_ros.IntrospectionServer('test_machine', sm, '/SM_LOAD')
    #sis.start()
    outcome = sm.execute()
    rospy.spin()
    #sis.stop()


if __name__ == '__main__':
    main()

