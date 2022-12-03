#!/usr/bin/env python3

import roslib; roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros
from geometry_msgs.msg import Pose
from kobuki_msgs.msg import ButtonEvent
from std_msgs.msg import String


####################################################### TODO #######################################################
# Publicar en el nodo de los leds y del sonido


bt0 = False
bt1 = False
bt2 = False


	# TODO    
def button_cb(self, data):
    print("TODO: hacer el codigo para pillar el valor del B0 en self.__bt")
    print("Message from button received")


rospy.Subscriber("/mobile_base/events/button", ButtonEvent, button_cb)


# define state Foo
class Reposo(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['outcome1','outcome2', 'outcome3'],
                             input_keys=['prev_direccion_in'],
                             output_keys=['prev_direccion_out'])

        ######################### TODO ##################################
        ####### Suscribirse al nodo de la odometría o de la posicion actual
    
        self.__home_pose = Pose()
        self.__get_home_pose = False
        
    
    def __get_home_pose(self, data):
        if self.__get_home_pose == False:
            
            ################### TODO #########################
            ###### Obtener info de la posicion inicial ########
            
            self.__get_home_pose = True
            
    def execute(self, userdata):
        rospy.loginfo('Executing state FOO')
        
        while not rospy.is_shutdown():
            if bt0 == True:
                bt0 = False
                return 'outcome1'
            	 
            if bt1 == True:
                bt1 = False
                userdata.prev_pose_out = self.__home_pose
                return 'outcome3'
            
            if bt2 == True:
                userdata.prev_pose_out = userdata.prev_pose_in
                bt2 = False
                return 'outcome3'



class Detectar(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['outcome1','outcome2'],
                             output_keys=['direccion_out'])
        
        rospy.Subscriber("/camera", String, self.__camera_cb)
        
        self.__pose = Pose()
        self.__prev_pose = Pose()
        self.__prev_pose.position.z = -1.0

        self.__is_dir = False
        
        
    def execute(self, userdata):
        rospy.loginfo('Executing state FOO')
        
        
        # TODO: luz naranja
        
        
        while not rospy.is_shutdown():
            if self.__is_dir == True:
                userdata.direccion_out = self.__pose
            	
                self.__prev_pose = self.__pose
                self.__is_dir = False
		        
                return 'outcome1'
            	 
            elif bt1 == True:
                bt1 = False
                return 'outcome2'
            
    def __camera_cb(self, data):
        print("Message from camera recevied")
        message = data.data.split()
        self.__pose.position.x = float(message[0])
        self.__pose.position.y = float(message[1])
        self.__pose.orientation.w = float(message[2])
        self.__is_dir = True   	
    
    

class Img_leida(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['outcome1',
                                       'outcome2'],
                             input_keys=['direccion_in'],
                             output_keys=['direccion_out'])
        
    def execute(self, userdata):
        rospy.loginfo('Executing state FOO')
        
        ################################# TODO ########################
        ###################### sonido #################################
        
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
        
        ########################## TODO ###########################
        ########### Suscribirse a topics del brazo ###############
        
    def execute(self, userdata):
        
        ############################# TODO ########################
        ########## Esperar Xs, cerrar pinza, esperar Xs, luz verde #################
        ############# Fuera de los ifs ##### y comprobar el boton todo el rato
        
        if bt1 == False:
            return 'outcome1'
        else:
            return 'outcome2'


class Ir_destino(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['outcome1'],
                             input_keys=['direccion_in'],
                             output_keys=['prev_direccion_out'])
        
        
        ######################### TODO ######################################
        ####### Suscribirse al nodo de la posición actual y el objetivo #####
        
        
        self.__prev_pose = Pose()
        self.__get_prev_pose = False
        
        
    def execute(self, userdata):
        
        ########################## TODO ##############################
        ############## Publicar en el topic de destino de SLAM #######
        
        self.__get_prev_pose = True   # Esto dentro del if, justo antes de acabar la ejecución
        return 'outcome1'
    
    def __get_pose(self, data):
        if self.__get_prev_pose == False:
            
            ################### TODO #########################
            ###### Obtener info de la posicion actual ########
            
            self.__get_prev_pose = True
            

class Llega_destino(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['outcome1'],
                             input_keys=['prev_direccion_in'],
                             output_keys=['prev_direccion_out'])
        
        
        ##################### TODO ##################################
        ######### Suscribirse al nodo del brazo #####################
        
        
    def execute(self, userdata):
        
        ########################## TODO ##############################
        ############## Espera Xs, sonido, brazo, espera Xs #######
        
        userdata.prev_direccion_out = userdata.prev_direccion_in
        return 'outcome1'


class Recogida(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['outcome1'],
                             input_keys=['prev_direccion_in'],
                             output_keys=['prev_direccion_out'])
        
        
        ##################### TODO ##################################
        ######### Suscribirse al nodo del brazo #####################
        
        
    def execute(self, userdata):
        
        ########################## TODO ##############################
        ############## Sonido, Espera Xs, abre pinza, espera Xs, brazo #######
        
        userdata.prev_direccion_out = userdata.prev_direccion_in
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
                               transitions={'outcome1':'Deteccion', 
                                            'outcome2':'Recoger',
                                            'outcome3':'Recoger'},
                               remapping={'prev_direccion_in':'pose',
                                          'prev_direccion_out':'pose'})
        
        smach.StateMachine.add('Detectar', Detectar(), 
                               transitions={'outcome1':'Img_leida',
                                            'outcome2':'Reposo'},
                               remapping={'direccion_out':'pose'})
        
        smach.StateMachine.add('Img_Leida', Img_leida(), 
                               transitions={'outcome1':'Recoger_carta',
                                            'outcome2':'Reposo'},
                               remapping={'direccion_in':'pose',
                                          'direccion_out':'pose'})
        
        smach.StateMachine.add('Recoger_carta', Recoger_carta(), 
                               transitions={'outcome1':'Ir_destion',
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
    main()
