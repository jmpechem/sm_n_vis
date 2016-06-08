#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
from std_msgs.msg import String

#define state Stand_By
class Stand_By(smach.State):
      def __init__(self):
	  smach.State.__init__(self, outcomes=['power_on','loop','shutdown'])

      def execute(self, userdata):
	  trans_tag = rospy.wait_for_message('transition',String)
	  if trans_tag.data == "power_on":
	     return 'power_on'
	  elif trans_tag.data == "shutdown":
	     return 'shutdown'
          else:
	     return 'loop'

#define state Power_On
class Power_On(smach.State):
      def __init__(self):
	  smach.State.__init__(self, outcomes=['auto_on','manu_on','loop','shutdown'])

      def execute(self, userdata):
	  trans_tag = rospy.wait_for_message('transition',String)	  
          if trans_tag.data == "auto_on":
             return 'auto_on'
          elif trans_tag.data == "manu_on":
             return 'manu_on'
	  elif trans_tag.data == "shutdown":
	     return 'shutdown'
	  else:
	     return 'loop'

#define state Auto
class Auto(smach.State):
      def __init__(self):
	  smach.State.__init__(self, outcomes=['mission1','mission2','mission3','loop','shutdown'])

      def execute(self, userdata):
	  trans_tag = rospy.wait_for_message('transition',String)
          if trans_tag.data == "mission1":
             return 'mission1'
          elif trans_tag.data == "mission2":
             return 'mission2'
	  elif trans_tag.data == "mission3":
	     return 'mission3'
	  elif trans_tag.data == "shutdown":
	     return 'shutdown'
	  else:
	     return 'loop'

#define state Manual
class Manual(smach.State):
      def __init__(self):
	  smach.State.__init__(self, outcomes=['activate_jctrl','activate_tctrl','activate_recog','loop','shutdown'])

      def execute(self, userdata):
	  trans_tag = rospy.wait_for_message('transition',String)
          if trans_tag.data == "activate_jctrl":
             return 'activate_jctrl'
          elif trans_tag.data == "activate_tctrl":
             return 'activate_tctrl'
	  elif trans_tag.data == "activate_recog":
	     return 'activate_recog'
	  elif trans_tag.data == "shutdown":
	     return 'shutdown'
	  else:
	     return 'loop'
#define state JointCtrl
class JointCtrl(smach.State):
      def __init__(self):
	  smach.State.__init__(self, outcomes=['jctrl_back','cmd_modechg','loop','shutdown'])

      def execute(self, userdata):
	  trans_tag = rospy.wait_for_message('transition',String)
	  if trans_tag.data == "jctrl_back":
	     return 'jctrl_back'
          elif trans_tag.data == "cmd_modechg":
             return 'cmd_modechg'
	  elif trans_tag.data == "shutdown":
	     return 'shutdown'
          else:
             return 'loop'

#define state task
class TaskCtrl(smach.State):
      def __init__(self):
	  smach.State.__init__(self, outcomes=['tctrl_back','cmd_modechg','loop','shutdown'])

      def execute(self, userdata):
	  trans_tag = rospy.wait_for_message('transition',String)
	  if trans_tag.data == "tctrl_back":
	     return 'tctrl_back'
          elif trans_tag.data == "cmd_modechg":
             return 'cmd_modechg'
	  elif trans_tag.data == "shutdown":
	     return 'shutdown'
          else:
             return 'loop'
#define state recog
class Recog(smach.State):
      def __init__(self):
	  smach.State.__init__(self, outcomes=['recog_back','cmd_modechg','loop','shutdown'])

      def execute(self, userdata):
	  trans_tag = rospy.wait_for_message('transition',String)
	  if trans_tag.data == "recog_back":
	     return 'recog_back'
          elif trans_tag.data == "cmd_modechg":
             return 'cmd_modechg'
	  elif trans_tag.data == "shutdown":
	     return 'shutdown'
          else:
             return 'loop'

#define state Mode_Chg
class Mode_Chg(smach.State):
	def __init__(self):
	    smach.State.__init__(self, outcomes=['manu_on','auto_on','loop','shutdown'])

	def execute(self, userdata):
	    trans_tag = rospy.wait_for_message('transition',String)
	    if trans_tag.data == "manu_on":
		return 'manu_on'
	    elif trans_tag.data == "auto_on":
		return 'auto_on'
	    elif trans_tag.data == "shutdown":
	        return 'shutdown'
	    else:
		return 'loop'

#define state Valve_Mission
class Valve_Mission(smach.State):
	def __init__(self):
	    smach.State.__init__(self, outcomes=['v_init','v_ready','v_approach','v_reach','v_close','loop','shutdown'])

	def execute(self, userdata):
  	    trans_tag = rospy.wait_for_message('transition',String)
	    if trans_tag.data == "v_init":	    
	       return 'v_init'
	    elif trans_tag.data == "v_ready":
		return 'v_ready'
	    elif trans_tag.data == "v_reach":
		return 'v_reach'
	    elif trans_tag.data == "v_approach":
		return 'v_approach'
	    elif trans_tag.data == "v_close":
		return 'v_close'
 	    elif trans_tag.data == "shutdown":
	        return 'shutdown'
	    else:
		return 'loop'

#define state Valve_Init
class Valve_Init(smach.State):
      def __init__(self):
	  smach.State.__init__(self, outcomes=['v_ready','v_approach','cmd_modechg','loop','shutdown'])

      def execute(self, userdata):
	  trans_tag = rospy.wait_for_message('transition',String)
	  if trans_tag.data == "v_ready":
	     return 'v_ready'
          elif trans_tag.data == "v_approach":
	     return 'v_approach'
	  elif trans_tag.data == "cmd_modechg":
	     return 'cmd_modechg'
	  elif trans_tag.data == "shutdown":
	     return 'shutdown'
	  else:
	     return 'loop'
#define state Valve_Ready
class Valve_Ready(smach.State):
      def __init__(self):
	  smach.State.__init__(self, outcomes=['v_reach','cmd_modechg','loop','shutdown'])

      def execute(self, userdata):
	  trans_tag = rospy.wait_for_message('transition',String)
	  if trans_tag.data == "v_reach":
	     return 'v_reach'
	  elif trans_tag.data == "cmd_modechg":
	     return 'cmd_modechg'
	  elif trans_tag.data == "shutdown":
	     return 'shutdown'
	  else:
             return 'loop'	 
#define state Valve_Reach
class Valve_Reach(smach.State):
      def __init__(self):
	  smach.State.__init__(self, outcomes=['v_close','cmd_modechg','loop','shutdown'])

      def execute(self, userdata):
	  trans_tag = rospy.wait_for_message('transition',String)
	  if trans_tag.data == "v_close":
	     return 'v_close'
	  elif trans_tag.data == "cmd_modechg":
	     return 'cmd_modechg'
	  elif trans_tag.data == "shutdown":
	     return 'shutdown'
	  else:
             return 'loop'	 
#define state Valve_Close
class Valve_Close(smach.State):
      def __init__(self):
	  smach.State.__init__(self, outcomes=['v_init','cmd_modechg','loop','shutdown'])

      def execute(self, userdata):
	  trans_tag = rospy.wait_for_message('transition',String)
	  if trans_tag.data == "v_init":
	     return 'v_init'
	  elif trans_tag.data == "cmd_modechg":
	     return 'cmd_modechg'
	  elif trans_tag.data == "shutdown":
	     return 'shutdown'
	  else:
             return 'loop'
#define State Valve_Approach
class Valve_Approach(smach.State):
      def __init__(self):
	  smach.State.__init__(self, outcomes=['v_init','cmd_modechg','loop','shutdown'])

      def execute(self, userdata):
	  trans_tag = rospy.wait_for_message('transition',String)
	  if trans_tag.data == "v_init":
	     return 'v_init'
	  elif trans_tag.data == "cmd_modechg":
	     return 'cmd_modechg'
	  elif trans_tag.data == "shutdown":
	     return 'shutdown'
	  else:
             return 'loop'  
#define State Door_Mission
class Door_Mission(smach.State):
	def __init__(self):
	    smach.State.__init__(self, outcomes=['d_init','d_ready','d_reach','d_open','d_push','loop','shutdown'])

	def execute(self,userdata):
	    trans_tag = rospy.wait_for_message('transition',String)
	    if trans_tag.data == "d_init":
		return 'd_init'
	    elif trans_tag.data == "d_ready":
		return 'd_ready'
	    elif trans_tag.data == "d_reach":
		return 'd_reach'
	    elif trans_tag.data == "d_open":
		return 'd_open'
	    elif trans_tag.data == "d_push":
		return 'd_push'
	    elif trans_tag.data == "shutdown":
	        return 'shutdown'
	    else:
		return 'loop'
#define State Door_Init
class Door_Init(smach.State):
	def __init__(self):
	    smach.State.__init__(self,outcomes=['cmd_dready','cmd_modechg','loop','shutdown'])

	def execute(self,userdata):
	    trans_tag = rospy.wait_for_message('transition',String)
	    if trans_tag.data == "cmd_dready":
		return 'cmd_dready'
	    elif trans_tag.data == "cmd_modechg":
		return 'cmd_modechg'
	    elif trans_tag.data == "shutdown":
	        return 'shutdown'
	    else:
		return 'loop'

#define State Door_Ready
class Door_Ready(smach.State):
	def __init__(self):
	    smach.State.__init__(self,outcomes=['cmd_dreach','cmd_modechg','loop','shutdown'])

	def execute(self,userdata):
 	    trans_tag = rospy.wait_for_message('transition',String)
	    if trans_tag.data == "cmd_dreach":
		return 'cmd_dreach'
	    elif trans_tag.data == "cmd_modechg":
		return 'cmd_modechg'
	    elif trans_tag.data == "shutdown":
	        return 'shutdown'
	    else:
		return 'loop'
#define State Door_Reach
class Door_Reach(smach.State):
	def __init__(self):
	    smach.State.__init__(self,outcomes=['cmd_dopen','cmd_modechg','loop','shutdown'])

	def execute(self,userdata):
	    trans_tag = rospy.wait_for_message('transition',String)
	    if trans_tag.data == "cmd_dopen":
		return 'cmd_dopen'
	    elif trans_tag.data == "cmd_modechg":
		return 'cmd_modechg'
	    elif trans_tag.data == "shutdown":
	        return 'shutdown'
	    else:
		return 'loop'
#define State Door_Open
class Door_Open(smach.State):
	def __init__(self):
	    smach.State.__init__(self,outcomes=['cmd_dpush','cmd_modechg','loop','shutdown'])

	def execute(self,userdata):
	    trans_tag = rospy.wait_for_message('transition',String)
	    if trans_tag.data == "cmd_dpush":
		return 'cmd_dpush'
	    elif trans_tag.data == "cmd_modechg":
		return 'cmd_modechg'
	    elif trans_tag.data == "shutdown":
	        return 'shutdown'
	    else:
		return 'loop'
#define State Door_Push
class Door_Push(smach.State):
	def __init__(self):
	    smach.State.__init__(self,outcomes=['cmd_dinit','cmd_modechg','loop','shutdown'])

	def execute(self,userdata):
	    trans_tag = rospy.wait_for_message('transition',String)
	    if trans_tag.data == "cmd_dinit":
		return 'cmd_dinit'
	    elif trans_tag.data == "cmd_modechg":
		return 'cmd_modechg'
	    elif trans_tag.data == "shutdown":
	        return 'shutdown'
	    else:
		return 'loop'

#define State Wall_Mission
class Wall_Mission(smach.State):
	def __init__(self):
	    smach.State.__init__(self, outcomes=['w_init','w_approach','w_ready','w_reach','w_grab','w_ungrab','w_rotatedrill','w_wallready','w_contact','w_cut','w_push','loop','shutdown'])

	def execute(self,userdata):
	    trans_tag = rospy.wait_for_message('transition',String)
	    if trans_tag.data == "w_init":
		return 'w_init'
	    elif trans_tag.data == "w_approach":
		return 'w_approach'
	    elif trans_tag.data == "w_ready":
		return 'w_ready'
	    elif trans_tag.data == "w_reach":
		return 'w_reach'
	    elif trans_tag.data == "w_grab":
		return 'w_grab'
	    elif trans_tag.data == "w_ungrab":
		return 'w_ungrab'
	    elif trans_tag.data == "w_rotatedrill":
		return 'w_rotatedrill'
	    elif trans_tag.data == "w_wallready":
		return 'w_wallready'
	    elif trans_tag.data == "w_contact":
		return 'w_contact'
	    elif trans_tag.data == "w_cut":
		return 'w_cut'
	    elif trans_tag.data == "w_push":
		return 'w_push'
	    elif trans_tag.data == "shutdown":
	        return 'shutdown'
	    else:
		return 'loop'
#define State Wall_Init
class Wall_Init(smach.State):
	def __init__(self):
	    smach.State.__init__(self,outcomes=['cmd_wready','cmd_wapproach','cmd_modechg','loop','shutdown'])

	def execute(self,userdata):
	    trans_tag = rospy.wait_for_message('transition',String)
	    if trans_tag.data == "cmd_wready":
		return 'cmd_wready'
	    elif trans_tag.data == "cmd_wapproach":
		return 'cmd_wapproach'
	    elif trans_tag.data == "cmd_modechg":
		return 'cmd_modechg'
	    elif trans_tag.data == "shutdown":
	        return 'shutdown'
	    else:
		return 'loop'
#define State Wall_Approach
class Wall_Approach(smach.State):
	def __init__(self):
	    smach.State.__init__(self,outcomes=['cmd_winit','cmd_modechg','loop','shutdown'])

	def execute(self,userdata):
	    trans_tag = rospy.wait_for_message('transition',String)
	    if trans_tag.data == "cmd_winit":
		return 'cmd_wready'
	    elif trans_tag.data == "cmd_modechg":
		return 'cmd_modechg'
	    elif trans_tag.data == "shutdown":
	        return 'shutdown'
	    else:
		return 'loop'
#define State Wall_Ready
class Wall_Ready(smach.State):
	def __init__(self):
	    smach.State.__init__(self,outcomes=['cmd_wreach','cmd_modechg','loop','shutdown'])

	def execute(self,userdata):
	    trans_tag = rospy.wait_for_message('transition',String)
	    if trans_tag.data == "cmd_wreach":
		return 'cmd_wreach'
	    elif trans_tag.data == "cmd_modechg":
		return 'cmd_modechg'
	    elif trans_tag.data == "shutdown":
	        return 'shutdown'
	    else:
		return 'loop'
#define State Wall_Reach
class Wall_Reach(smach.State):
	def __init__(self):
	    smach.State.__init__(self,outcomes=['cmd_wgrab','cmd_modechg','loop','shutdown'])

	def execute(self,userdata):
	    trans_tag = rospy.wait_for_message('transition',String)
	    if trans_tag.data == "cmd_wgrab":
		return 'cmd_wgrab'
	    elif trans_tag.data == "cmd_modechg":
		return 'cmd_modechg'
	    elif trans_tag.data == "shutdown":
	        return 'shutdown'
	    else:
		return 'loop'
#define State Wall_Grab
class Wall_Grab(smach.State):
	def __init__(self):
	    smach.State.__init__(self,outcomes=['cmd_wungrab','cmd_wroratedrill','cmd_wwallready','cmd_modechg','loop','shutdown'])

	def execute(self,userdata):
	    trans_tag = rospy.wait_for_message('transition',String)
	    if trans_tag.data == "cmd_wungrab":
		return 'cmd_wungrab'
	    elif trans_tag.data == "cmd_wroratedrill":
		return 'cmd_wroratedrill'
	    elif trans_tag.data == "cmd_wwallready":
		return 'cmd_wwallready'
	    elif trans_tag.data == "cmd_modechg":
		return 'cmd_modechg'
	    elif trans_tag.data == "shutdown":
	        return 'shutdown'
	    else:
		return 'loop'
#define State Wall_Ungrab
class Wall_Ungrab(smach.State):
	def __init__(self):
	    smach.State.__init__(self,outcomes=['cmd_wgrab','cmd_wroratedrill','cmd_modechg','loop','shutdown'])

	def execute(self,userdata):
	    trans_tag = rospy.wait_for_message('transition',String)
	    if trans_tag.data == "cmd_wgrab":
		return 'cmd_wgrab'
	    elif trans_tag.data == "cmd_wroratedrill":
		return 'cmd_wroratedrill'
	    elif trans_tag.data == "cmd_modechg":
		return 'cmd_modechg'
	    elif trans_tag.data == "shutdown":
	        return 'shutdown'
	    else:
		return 'loop'
#define State Wall_Rotatedrill
class Wall_Rotatedrill(smach.State):
	def __init__(self):
	    smach.State.__init__(self,outcomes=['cmd_wgrab','cmd_wungrab','cmd_modechg','loop','shutdown'])

	def execute(self,userdata):
	    trans_tag = rospy.wait_for_message('transition',String)
	    if trans_tag.data == "cmd_wgrab":
		return 'cmd_wgrab'
	    elif trans_tag.data == "cmd_wungrab":
		return 'cmd_wungrab'
	    elif trans_tag.data == "cmd_modechg":
		return 'cmd_modechg'
	    elif trans_tag.data == "shutdown":
	        return 'shutdown'
	    else:
		return 'loop'
#define State Wall_wallready
class Wall_Wallready(smach.State):
	def __init__(self):
	    smach.State.__init__(self,outcomes=['cmd_wcontact','cmd_modechg','loop','shutdown'])

	def execute(self,userdata):
	    trans_tag = rospy.wait_for_message('transition',String)
	    if trans_tag.data == "cmd_wcontact":
		return 'cmd_wcontact'
	    elif trans_tag.data == "cmd_modechg":
		return 'cmd_modechg'
	    elif trans_tag.data == "shutdown":
	        return 'shutdown'
	    else:
		return 'loop'
#define State Wall_Contact
class Wall_Contact(smach.State):
	def __init__(self):
	    smach.State.__init__(self,outcomes=['cmd_wcut','cmd_modechg','loop','shutdown'])

	def execute(self,userdata):
	    trans_tag = rospy.wait_for_message('transition',String)
	    if trans_tag.data == "cmd_wcut":
		return 'cmd_wcut'
	    elif trans_tag.data == "cmd_modechg":
		return 'cmd_modechg'
	    elif trans_tag.data == "shutdown":
	        return 'shutdown'
	    else:
		return 'loop'
#define State Wall_Cut
class Wall_Cut(smach.State):
	def __init__(self):
	    smach.State.__init__(self,outcomes=['cmd_wpush','cmd_modechg','loop','shutdown'])

	def execute(self,userdata):
	    trans_tag = rospy.wait_for_message('transition',String)
	    if trans_tag.data == "cmd_wpush":
		return 'cmd_wpush'
	    elif trans_tag.data == "cmd_modechg":
		return 'cmd_modechg'
	    elif trans_tag.data == "shutdown":
	        return 'shutdown'
	    else:
		return 'loop'
#define State Wall_Push
class Wall_Push(smach.State):
	def __init__(self):
	    smach.State.__init__(self,outcomes=['cmd_winit','cmd_modechg','loop','shutdown'])

	def execute(self,userdata):
	    trans_tag = rospy.wait_for_message('transition',String)
	    if trans_tag.data == "cmd_winit":
		return 'cmd_winit'
	    elif trans_tag.data == "cmd_modechg":
		return 'cmd_modechg'
	    elif trans_tag.data == "shutdown":
	        return 'shutdown'
	    else:
		return 'loop'

def main():
    rospy.init_node('jimin_state_machine')

    # Create a SMACH state machine
    sm_top = smach.StateMachine(outcomes=['END','Wall','Joint','Task','Recognize'])
#    rospy.Subscriber("/transition",String,cb)
    
    # Open the container
    with sm_top:
        # Stand By state
	smach.StateMachine.add('Stand_By',Stand_By(),
			       transitions={'power_on':'Power_On','loop':'Stand_By','shutdown':'END'})

        # Power on state
	smach.StateMachine.add('Power_On',Power_On(),
			       transitions={'auto_on':'Auto','manu_on':'Manual','loop':'Power_On','shutdown':'END'})
        # Auto state
        smach.StateMachine.add('Auto',Auto(),
			       transitions={'mission1':'Valve_Mission','mission2':'Door_Mission','mission3':'Wall','loop':'Auto','shutdown':'END'})
        # Manual state
	smach.StateMachine.add('Manual',Manual(),
			       transitions={'activate_jctrl':'JointCtrl','activate_tctrl':'TaskCtrl','activate_recog':'Recog','loop':'Manual','shutdown':'END'})
	# JointCtrl state
	smach.StateMachine.add('JointCtrl',JointCtrl(),
			       transitions={'jctrl_back':'Manual','cmd_modechg':'Mode_Chg','loop':'JointCtrl','shutdown':'END'})
	# TaskCtrl state
	smach.StateMachine.add('TaskCtrl',TaskCtrl(),
			       transitions={'tctrl_back':'Manual','cmd_modechg':'Mode_Chg','loop':'TaskCtrl','shutdown':'END'})
	# Recog state
	smach.StateMachine.add('Recog',Recog(),
			       transitions={'recog_back':'Manual','cmd_modechg':'Mode_Chg','loop':'Recog','shutdown':'END'})

	# Mode_Chg state
	smach.StateMachine.add('Mode_Chg',Mode_Chg(),transitions={'manu_on':'Manual','auto_on':'Auto','loop':'Mode_Chg','shutdown':'END'})



	# Valve_Mission state
	smach.StateMachine.add('Valve_Mission', Valve_Mission(), transitions={'v_init':'Valve_Init','v_ready':'Valve_Ready','v_approach':'Valve_Approach','v_reach':'Valve_Reach','v_close':'Valve_Close','loop':'Valve_Mission','shutdown':'END'})
        # Valve_init state
	smach.StateMachine.add('Valve_Init', Valve_Init(), transitions={'v_ready':'Valve_Ready','v_approach':'Valve_Approach','cmd_modechg':'Mode_Chg','loop':'Valve_Init','shutdown':'END'})
	# Valve_ready state
	smach.StateMachine.add('Valve_Ready', Valve_Ready(), transitions={'v_reach':'Valve_Reach','cmd_modechg':'Mode_Chg','loop':'Valve_Ready','shutdown':'END'})
	# Valve_reach state
	smach.StateMachine.add('Valve_Reach', Valve_Reach(), transitions={'v_close':'Valve_Close','cmd_modechg':'Mode_Chg','loop':'Valve_Reach','shutdown':'END'})   
	# Valve_close state
	smach.StateMachine.add('Valve_Close', Valve_Close(), transitions={'v_init':'Valve_Init','cmd_modechg':'Mode_Chg','loop':'Valve_Close','shutdown':'END'})   
	# Valve_approach state
	smach.StateMachine.add('Valve_Approach', Valve_Approach(), transitions={'v_init':'Valve_Init','cmd_modechg':'Mode_Chg','loop':'Valve_Approach','shutdown':'END'})   

	#Door_Mission state	
	smach.StateMachine.add('Door_Mission',Door_Mission(),transitions={'d_init':'Door_Init','d_ready':'Door_Ready','d_reach':'Door_Reach','d_open':'Door_Open','d_push':'Door_Push','loop':'Door_Mission','shutdown':'END'})
	#Door_Init state
	smach.StateMachine.add('Door_Init',Door_Init(),transitions={'cmd_dready':'Door_Ready','cmd_modechg':'Mode_Chg','loop':'Door_Init','shutdown':'END'})
	#Door_Ready state
	smach.StateMachine.add('Door_Ready',Door_Ready(),transitions={'cmd_dreach':'Door_Reach','cmd_modechg':'Mode_Chg','loop':'Door_Ready','shutdown':'END'})
	#Door_Reach state
	smach.StateMachine.add('Door_Reach',Door_Reach(),transitions={'cmd_dopen':'Door_Open','cmd_modechg':'Mode_Chg','loop':'Door_Reach','shutdown':'END'})
	#Door_Open state
	smach.StateMachine.add('Door_Open',Door_Open(),transitions={'cmd_dpush':'Door_Push','cmd_modechg':'Mode_Chg','loop':'Door_Open','shutdown':'END'})
	#Door_Push state
	smach.StateMachine.add('Door_Push',Door_Push(),transitions={'cmd_dinit':'Door_Init','cmd_modechg':'Mode_Chg','loop':'Door_Push','shutdown':'END'})

	#Wall_Mission state	
	smach.StateMachine.add('Wall_Mission',Wall_Mission(),transitions={'w_init':'Wall_Init','w_approach':'Wall_Approach','w_ready':'Wall_Ready','w_reach':'Wall_Reach','w_grab':'Wall_Grab','w_ungrab':'Wall_Ungrab','w_rotatedrill':'Wall_Rotatedrill','w_wallready':'Wall_Wallready','w_contact':'Wall_Contact','w_cut':'Wall_Cut','w_push':'Wall_Push','loop':'Wall_Mission','shutdown':'END'})
	#Wall_Init state	
	smach.StateMachine.add('Wall_Init',Wall_Init(),transitions={'cmd_wready':'Wall_Ready','cmd_wapproach':'Wall_Approach','cmd_modechg':'Mode_Chg','loop':'Wall_Init','shutdown':'END'})
	#Wall_Approach state	
	smach.StateMachine.add('Wall_Approach',Wall_Approach(),transitions={'cmd_winit':'Wall_Init','cmd_modechg':'Mode_Chg','loop':'Wall_Approach','shutdown':'END'})
	#Wall_Ready state
	smach.StateMachine.add('Wall_Ready',Wall_Ready(),transitions={'cmd_wreach':'Wall_Reach','cmd_modechg':'Mode_Chg','loop':'Wall_Ready','shutdown':'END'})
	#Wall_Reach state	
	smach.StateMachine.add('Wall_Reach',Wall_Reach(),transitions={'cmd_wgrab':'Wall_Grab','cmd_modechg':'Mode_Chg','loop':'Wall_Reach','shutdown':'END'})
	#Wall_Grab state	
	smach.StateMachine.add('Wall_Grab',Wall_Grab(),transitions={'cmd_wungrab':'Wall_Ungrab','cmd_wroratedrill':'Wall_Rotatedrill', 'cmd_wwallready':'Wall_Wallready','cmd_modechg':'Mode_Chg','loop':'Wall_Grab','shutdown':'END'})
	#Wall_Ungrab state	
	smach.StateMachine.add('Wall_Ungrab',Wall_Ungrab(),transitions={'cmd_wgrab':'Wall_Grab','cmd_wroratedrill':'Wall_Rotatedrill','cmd_modechg':'Mode_Chg','loop':'Wall_Ungrab','shutdown':'END'})
	#Wall_Rotatedrill state	
	smach.StateMachine.add('Wall_Rotatedrill',Wall_Rotatedrill(),transitions={'cmd_wgrab':'Wall_Grab','cmd_wungrab':'Wall_Ungrab','cmd_modechg':'Mode_Chg','loop':'Wall_Rotatedrill','shutdown':'END'})	
	#Wall_Wallready state
	smach.StateMachine.add('Wall_Wallready',Wall_Wallready(),transitions={'cmd_wcontact':'Wall_Contact','cmd_modechg':'Mode_Chg','loop':'Wall_Wallready','shutdown':'END'})	
	#Wall_Contact state	
	smach.StateMachine.add('Wall_Contact',Wall_Contact(),transitions={'cmd_wcut':'Wall_Cut','cmd_modechg':'Mode_Chg','loop':'Wall_Contact','shutdown':'END'})	
	#Wall_Cut state	
	smach.StateMachine.add('Wall_Cut',Wall_Cut(),transitions={'cmd_wpush':'Wall_Push','cmd_modechg':'Mode_Chg','loop':'Wall_Cut','shutdown':'END'})	
	#Wall_Push state	
	smach.StateMachine.add('Wall_Push',Wall_Push(),transitions={'cmd_winit':'Wall_Init','cmd_modechg':'Mode_Chg','loop':'Wall_Push','shutdown':'END'})	

    # Execute SMACH plan
    sis = smach_ros.IntrospectionServer('Jimin_machine', sm_top, '/SM_START')
    sis.start()
    outcome = sm_top.execute()    
    rospy.spin()
    sis.stop()
    
    rospy.signal_shutdown('All done.')

if __name__ == '__main__':
    main()

