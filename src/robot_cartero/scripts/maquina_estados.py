#!/usr/bin/env python3

import rospy
import smach
import smach_ros
from geometry_msgs.msg import Pose
from kobuki_msgs.msg import ButtonEvent, Led, Sound
from std_msgs.msg import String
import time
import actionlib
from geometry_msgs.msg import PoseStamped
import move_base_msgs.msg

bt0 = False
bt1 = False
bt2 = False

    
# Callback del boton con el laboratorio
def button_cb(data):
    global bt0, bt1, bt2
    
    if data.button == 0:
        bt0 = True
    elif data.button == 1:
        bt1 = True
    elif data.button == 2:
        bt2 = True

# rospy.Subscriber("/teclas", String, button_cb)
rospy.Subscriber("/mobile_base/events/button", ButtonEvent, button_cb)
led1 = rospy.Publisher("/mobile_base/commands/led1", Led, queue_size=10)
led2 = rospy.Publisher("/mobile_base/commands/led2", Led, queue_size=10)
sound = rospy.Publisher("/mobile_base/commands/sound", Sound, queue_size=10)

arm = rospy.Publisher("/arm", String, queue_size=10)

wait_time = 6

rospy.Subscriber("/mobile_base/events/button", ButtonEvent, button_cb)


class Reposo(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['outcome1','outcome2', 'outcome3'],
                             input_keys=['prev_direccion_in'],
                             output_keys=['prev_direccion_out'])

        self.__home_pose = Pose()

        self.__home_pose.position.x = 0.0
        self.__home_pose.position.y = 0.0
        
        self.__home_pose.orientation.x = 0.0
        self.__home_pose.orientation.y = 0.0
        self.__home_pose.orientation.z = 0.0
        self.__home_pose.orientation.w = 1
            
    def execute(self, userdata):
        global bt0, bt1, bt2
        print("--- Reposo ---")
        
        led1.publish(0)
        led2.publish(0)
        sound.publish(0)
        
        time.sleep(0.8)
        
        while not rospy.is_shutdown():
            if bt0 == True:
                bt0 = False
                return 'outcome1'
            	 
            if bt1 == True:
                bt1 = False
                userdata.prev_direccion_out = self.__home_pose
                return 'outcome3'
            
            if bt2 == True:
                userdata.prev_direccion_out = userdata.prev_direccion_in
                bt2 = False
                return 'outcome3'



class Detectar(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['outcome1','outcome2'],
                             output_keys=['direccion_out'])
        
        rospy.Subscriber("/teclas", String, self.__camera_cb)
                
        self.__pose = Pose()

        self.__is_dir = False
        
        self.positions = []
        for i in range(3):
            self.positions.append(Pose())
        
        self.positions[0].position.x = 0.925
        self.positions[0].position.y = 1.848
        
        self.positions[0].orientation.x = 0.0
        self.positions[0].orientation.y = 0.0
        self.positions[0].orientation.z = 0.875
        self.positions[0].orientation.w = 0.484
        
        self.positions[1].position.x = 1.698
        self.positions[1].position.y = 2.1999
        
        self.positions[1].orientation.x = 0.0
        self.positions[1].orientation.y = 0.0
        self.positions[1].orientation.z = 0.461
        self.positions[1].orientation.w = 0.887

        self.positions[2].position.x = 2.0999951362609863
        self.positions[2].position.y = 3.279998540878296
        
        self.positions[2].orientation.x = 0.0
        self.positions[2].orientation.y = 0.0
        self.positions[2].orientation.z = -0.998
        self.positions[2].orientation.w = 0.05
        
        

    def execute(self, userdata):
        print("--- Detectando imagen ---")
        global bt0, bt1, bt2, sound, led1, led2
        
        led1.publish(1)
        led2.publish(1)
        sound.publish(1)
        
        time.sleep(0.8)
        
        while not rospy.is_shutdown():
            if self.__is_dir == True:
                print("is_dir")
                userdata.direccion_out = self.__pose
            	
                self.__is_dir = False
		        
                return 'outcome1'
            	 
            elif bt1 == True:
                bt1 = False
                return 'outcome2'
            
    def __camera_cb(self, data):        
        if data.data == "a":
            self.__pose = self.positions[0]
            self.__is_dir = True
        
        elif data.data == "b": 
            self.__pose = self.positions[1]
            self.__is_dir = True
        
        elif data.data == "c":
            self.__pose = self.positions[2]	
            self.__is_dir = True

    

class Img_leida(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['outcome1',
                                       'outcome2'],
                             input_keys=['direccion_in'],
                             output_keys=['direccion_out'])
        
    def execute(self, userdata):
        print("--- Imagen leida ---")
        global bt0, bt1, bt2, sound
        
        sound.publish(2)
        
        time.sleep(0.8)
        
        if bt1 == False:
            userdata.direccion_out = userdata.direccion_in
   
            return 'outcome1'
        else:
            return 'outcome2'


class Recoger_carta(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['outcome1',
                                       'outcome2'],
                             input_keys=['direccion_in'],
                             output_keys=['direccion_out'])
        
        
    def execute(self, userdata):
        print("--- Recoger carta ---")
        global bt0, bt1, bt2, arm, led1, led2
        
        time.sleep(wait_time)
        
        print("------ Cerrar pinza ------")
        arm.publish("recoger")
        
        time.sleep(wait_time)
        
        print("------ Luz verde ------")
        led1.publish(1)
        led2.publish(1)
        
        time.sleep(0.8)
        
        if bt1 == False:
            userdata.direccion_out = userdata.direccion_in
            return 'outcome1'
        else:
            bt1 = False
            return 'outcome2'


    
class Ir_destino(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['outcome1'],
                             input_keys=['direccion_in'],
                             output_keys=['prev_direccion_out'])
        
        self.__go_pose_pub = rospy.Publisher("/go_pose", PoseStamped, queue_size=10)
        rospy.Subscriber("/arrive", Pose, self.__get_prev_pose_)
        
        self.__prev_pose = Pose()
        self.__get_prev_pose = False
        
    def __get_prev_pose_(self, data):
        self.__prev_pose = data.pose
        self.__get_prev_pose = True
        
    
    def execute(self, userdata):
        global bt0, bt1, bt2, led1, led2
        
        print("--- Ir destino ---")
        self.__get_prev_pose = False 
        
        led1. publish(1)
        led2.publish(1)
        
        time.sleep(0.8)
        '''
        client = actionlib.SimpleActionClient('move_base', move_base_msgs.msg.MoveBaseAction)
        
        client.wait_for_server()
        
        desiredPose = PoseStamped()

        desiredPose.header.frame_id = "map"
            # desiredPose.header.seq = 12
            # desiredPose.header.stamp = rospy.Time.now()
            
        desiredPose.pose = userdata.direccion_in
            
        goal = move_base_msgs.msg.MoveBaseGoal(desiredPose)
        print(goal)
        client.send_goal(goal)

        client.wait_for_result()'''
        
        desiredPose = PoseStamped()

        desiredPose.header.frame_id = "map"
        # desiredPose.header.seq = 12
        # desiredPose.header.stamp = rospy.Time.now()
            
        desiredPose.pose = userdata.direccion_in
        
        self.__go_pose_pub.publish(desiredPose)
        
        while not self.__get_prev_pose:
            pass
        
        
        userdata.prev_direccion_out = self.__prev_pose

        return 'outcome1'
    
            

class Llega_destino(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['outcome1'],
                             input_keys=['prev_direccion_in'],
                             output_keys=['prev_direccion_out'])
        
        
    def execute(self, userdata):
        global bt0, bt1, bt2, sound, arm
        print("--- Llega destino ---")
        
        time.sleep(1)
        
        print("------ Sonido ------")
        sound.publish(1)
        time.sleep(0.8)
        
        arm.publish("soltar")        
        time.sleep(wait_time)
        
        userdata.prev_direccion_out = userdata.prev_direccion_in
        return 'outcome1'


class Recogida(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['outcome1'],
                             input_keys=['prev_direccion_in'],
                             output_keys=['prev_direccion_out'])
        
        
    def execute(self, userdata):
        global bt0, bt1, bt2, arm
        print("--- Recogida ---")
        
        while not rospy.is_shutdown():
            if bt0 == True:
                arm.publish("abrir")        # Sujeto a cambios
                
                userdata.prev_direccion_out = userdata.prev_direccion_in
                bt0 = False
                return 'outcome1'


def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4'])
    
    sm.userdata.pose = Pose()

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Reposo', Reposo(), 
                               transitions={'outcome1':'Detectar', 
                                            'outcome2':'Recoger_carta',
                                            'outcome3':'Recoger_carta'},
                               remapping={'prev_direccion_in':'pose',
                                          'prev_direccion_out':'pose'})
        
        smach.StateMachine.add('Detectar', Detectar(), 
                               transitions={'outcome1':'Img_leida',
                                            'outcome2':'Reposo'},
                               remapping={'direccion_out':'pose'})
        
        smach.StateMachine.add('Img_leida', Img_leida(), 
                               transitions={'outcome1':'Recoger_carta',
                                            'outcome2':'Reposo'},
                               remapping={'direccion_in':'pose',
                                          'direccion_out':'pose'})
        
        smach.StateMachine.add('Recoger_carta', Recoger_carta(), 
                               transitions={'outcome1':'Ir_destino',
                                            'outcome2':'Reposo'},
                               remapping={'direccion_in':'pose',
                                          'direccion_out':'pose'})
        
        smach.StateMachine.add('Ir_destino', Ir_destino(), 
                               transitions={'outcome1':'Llega_destino'},
                               remapping={'direccion_in':'pose',
                                          'prev_direccion_out':'pose'})

        smach.StateMachine.add('Llega_destino', Llega_destino(), 
                               transitions={'outcome1':'Recogida'},
                               remapping={'prev_direccion_in':'pose',
                                          'prev_direccion_out':'pose'})
        
        smach.StateMachine.add('Recogida', Recogida(), 
                               transitions={'outcome1':'Reposo'},
                               remapping={'prev_direccion_in':'pose',
                                          'prev_direccion_out':'pose'})


    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    print("hola")
    main()
