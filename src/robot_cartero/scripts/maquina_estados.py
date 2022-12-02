#!/usr/bin/env python3

import roslib; roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros
from geometry_msgs.msg import Pose
from kobuki_msgs.msg import ButtonEvent
from std_msgs.msg import String

# define state Foo
class Reposo(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['outcome1','outcome2'],
                             output_keys=['direccion_out'])
                             
        rospy.Subscriber("/camera", String, self.__camera_cb)
        rospy.Subscriber("/mobile_base/events/button", ButtonEvent, button_cb)
        
        self.__pose = Pose()
        self.__prev_pose = Pose()
        self.__prev_pose.position.z = -1.0

        
        self.__is_dir = False
        
        self.__bt = False

    def execute(self, userdata):
        rospy.loginfo('Executing state FOO')
        
        while not rospy.is_shutdown():
            if self.__is_dir == True:
                userdata.foo_counter_out = self.__pose
            	
                self.__prev_pose = self.__pose
                self.__is_dir = False
		        
                return 'outcom1'
            	 
            elif self.__is_bt == True and self.__prev_pose.position.z != -1.0:
                userdata.foo_counter_out = self.__prev_pose
                print("1. Going to Ir a destino (Destino anterior)")
                return 'outcome2'
            	
                	
    
def __camera_cb(self, data):
    print("Message from camera recevied")
    message = data.data.split()
    self.__pose.position.x = float(message[0])
    self.__pose.position.y = float(message[1])
    self.__pose.orientation.w = float(message[2])
    self.__is_dir = True


	# TODO    
def button_cb(self, data):
    print("TODO: hacer el codigo para pillar el valor del B0 en self.__bt")
    print("Message from button received")
    	
    	
def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4'])
    sm.userdata.sm_counter = 0

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('FOO', Reposo(), 
                               transitions={'outcome1':'BAR', 
                                            'outcome2':'outcome4'},
                               remapping={'foo_counter_in':'sm_counter', 
                                          'foo_counter_out':'sm_counter'})
        smach.StateMachine.add('BAR', Bar(), 
                               transitions={'outcome1':'FOO'},
                               remapping={'bar_counter_in':'sm_counter'})


    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()
