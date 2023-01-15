#!/usr/bin/env python3

import rospy
import smach
from smach_ros import SimpleActionState
from geometry_msgs.msg import Pose
from kobuki_msgs.msg import ButtonEvent, Led, Sound
from std_msgs.msg import String
import time
import actionlib
from geometry_msgs.msg import PoseStamped
import move_base_msgs.msg

# Variables para los botones
bt0 = False
bt1 = False
bt2 = False

    
# Callback del boton con el laboratorio
def button_cb(data):
    global bt0, bt1, bt2
    
    if data.state == 1:
        if data.button == 0:
            bt0 = True
            bt1 = False
            bt2 = False
        elif data.button == 1:
            bt1 = True
            bt0 = False
            bt2 = False
        elif data.button == 2:
            bt2 = True
            bt0 = False
            bt1 = False
    
# Publishers y subscribers de los elementos del botón
rospy.Subscriber("/mobile_base/events/button", ButtonEvent, button_cb)          # Subscriber de los botones
led1 = rospy.Publisher("/mobile_base/commands/led1", Led, queue_size=10)        # Publishers de los leds
led2 = rospy.Publisher("/mobile_base/commands/led2", Led, queue_size=10)        
sound = rospy.Publisher("/mobile_base/commands/sound", Sound, queue_size=10)    # Publisher del sonido
go_pose_pub = rospy.Publisher("/go_pose", PoseStamped, queue_size=10)           # Publisher para el nodo de /go_pose
arm = rospy.Publisher("/arm_controller", String, queue_size=10)                            # Publisher para el nodo del brazo

# Tiempo de espera
wait_time = 5


# Reposo
class Reposo(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['outcome1','outcome2', 'outcome3'],
                             input_keys=['prev_direccion_in'],
                             output_keys=['prev_direccion_out'])

        # Posición de Home
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
        
        # Apagar los leds y hacer sonido
        led1.publish(0)
        led2.publish(0)
        sound.publish(0)
        
        time.sleep(0.8)
        
        # Espera a que se pulse uno de los botones para cambiar de estado
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



# Detectar imágenes
class Detectar(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['outcome1','outcome2'],
                             output_keys=['direccion_out'])
        
        rospy.Subscriber("/text_rec", String, self.__camera_cb)      
        
        self.__start_rec = rospy.Publisher("/text_rec_start", String, queue_size=10)

        # Mensaje de Pose
        self.__pose = Pose()

        # Flag para indicar si se recibió una dirección
        self.__is_dir = False
        
        # Lista de direcciones
        self.positions = []
        for i in range(3):
            self.positions.append(Pose())
        
        self.positions[0].position.x = 0.47
        self.positions[0].position.y = 0.24
        
        self.positions[0].orientation.x = 0.0
        self.positions[0].orientation.y = 0.0
        self.positions[0].orientation.z = 0.0
        self.positions[0].orientation.w = 1
        
        self.positions[1].position.x = 1.0
        self.positions[1].position.y = 1.7
        
        self.positions[1].orientation.x = 0.0
        self.positions[1].orientation.y = 0.0
        self.positions[1].orientation.z = 0.0
        self.positions[1].orientation.w = 1

        self.positions[2].position.x = -0.46
        self.positions[2].position.y = 2.84
        
        self.positions[2].orientation.x = 0.0
        self.positions[2].orientation.y = 0.0
        self.positions[2].orientation.z = 0
        self.positions[2].orientation.w = 1

        self.positions[3].position.x = 2.45
        self.positions[3].position.y = -0.84
        
        self.positions[3].orientation.x = 0.0
        self.positions[3].orientation.y = 0.0
        self.positions[3].orientation.z = 0.0
        self.positions[3].orientation.w = 1
        
        self.positions[4].position.x = 2.76
        self.positions[4].position.y = 1.43
        
        self.positions[4].orientation.x = 0.0
        self.positions[4].orientation.y = 0.0
        self.positions[4].orientation.z = 0
        self.positions[4].orientation.w = 1

        self.positions[5].position.x = 3.6
        self.positions[5].position.y = 1.17
        
        self.positions[5].orientation.x = 0.0
        self.positions[5].orientation.y = 0.0
        self.positions[5].orientation.z = 0
        self.positions[5].orientation.w = 1

        self.positions[6].position.x = 3.95
        self.positions[6].position.y = -0.39
        
        self.positions[6].orientation.x = 0.0
        self.positions[6].orientation.y = 0.0
        self.positions[6].orientation.z = 0.0
        self.positions[6].orientation.w = 1
        
        self.positions[7].position.x = 1.1
        self.positions[7].position.y = 2.66
        
        self.positions[7].orientation.x = 0.0
        self.positions[7].orientation.y = 0.0
        self.positions[7].orientation.z = 0
        self.positions[7].orientation.w = 1

        self.positions[8].position.x = 0.82
        self.positions[8].position.y = 4.16
        
        self.positions[8].orientation.x = 0.0
        self.positions[8].orientation.y = 0.0
        self.positions[8].orientation.z = 0
        self.positions[8].orientation.w = 1

        self.positions[9].position.x = 2.68
        self.positions[9].position.y = 4.41
        
        self.positions[9].orientation.x = 0.0
        self.positions[9].orientation.y = 0.0
        self.positions[9].orientation.z = 0
        self.positions[9].orientation.w = 1
        
        self.positions[10].position.x = 3.12
        self.positions[10].position.y = 2.98
        
        self.positions[10].orientation.x = 0.0
        self.positions[10].orientation.y = 0.0
        self.positions[10].orientation.z = 0
        self.positions[10].orientation.w = 1

        self.positions[11].position.x = 5.64
        self.positions[11].position.y = 1
        
        self.positions[11].orientation.x = 0.0
        self.positions[11].orientation.y = 0.0
        self.positions[11].orientation.z = 0
        self.positions[11].orientation.w = 1

        self.positions[12].position.x = 5.17
        self.positions[12].position.y = 1.156
        
        self.positions[12].orientation.x = 0.0
        self.positions[12].orientation.y = 0.0
        self.positions[12].orientation.z = 0
        self.positions[12].orientation.w = 1
        
        self.positions[13].position.x = 4.5
        self.positions[13].position.y = 2.55
        
        self.positions[13].orientation.x = 0.0
        self.positions[13].orientation.y = 0.0
        self.positions[13].orientation.z = 0
        self.positions[13].orientation.w = 1

        self.positions[14].position.x = 3.6
        self.positions[14].position.y = 4.5
        
        self.positions[14].orientation.x = 0.0
        self.positions[14].orientation.y = 0.0
        self.positions[14].orientation.z = 0
        self.positions[14].orientation.w = 1
        
        self.positions[15].position.x = 3
        self.positions[15].position.y = 6.13
        
        self.positions[15].orientation.x = 0.0
        self.positions[15].orientation.y = 0.0
        self.positions[15].orientation.z = 0
        self.positions[15].orientation.w = 1
        
        self.positions[16].position.x = 4.23
        self.positions[16].position.y = 4.3
        
        self.positions[16].orientation.x = 0.0
        self.positions[16].orientation.y = 0.0
        self.positions[16].orientation.z = 0.0
        self.positions[16].orientation.w = 1

        self.positions[17].position.x = 4.47
        self.positions[17].position.y = -0.7
        
        self.positions[17].orientation.x = 0.0
        self.positions[17].orientation.y = 0.0
        self.positions[17].orientation.z = 0
        self.positions[17].orientation.w = 1

        self.positions[18].position.x = 6.61
        self.positions[18].position.y = 4.4
        
        self.positions[18].orientation.x = 0.0
        self.positions[18].orientation.y = 0.0
        self.positions[18].orientation.z = 0.0
        self.positions[18].orientation.w = 1
        
        self.positions[19].position.x = 6.61
        self.positions[19].position.y = 4.4
        
        self.positions[19].orientation.x = 0.0
        self.positions[19].orientation.y = 0.0
        self.positions[19].orientation.z = 0.0
        self.positions[19].orientation.w = 1

        self.positions[20].position.x = -2.82
        self.positions[20].position.y = 6.02
        
        self.positions[20].orientation.x = 0.0
        self.positions[20].orientation.y = 0.0
        self.positions[20].orientation.z = 0
        self.positions[20].orientation.w = 1

        self.positions[21].position.x = -0.75
        self.positions[21].position.y = 7.53
        
        self.positions[21].orientation.x = 0.0
        self.positions[21].orientation.y = 0.0
        self.positions[21].orientation.z = 0.0
        self.positions[21].orientation.w = 1
        
        

    def execute(self, userdata):
        print("------ Detectando imagen ------")
        global bt0, bt1, bt2, sound, led1, led2
        
        self.__start_rec.publish("start")

        # Enciende leds en naranja y emite sonido
        led1.publish(2)
        led2.publish(2)
        sound.publish(1)
        
        time.sleep(0.8)
        
        # Hasta que detecte una imagen o se pulse el botón no cambia de estado
        while not rospy.is_shutdown():
            if self.__is_dir == True:
                self.__start_rec.publish("stop")
                # Se pasa la dirección al siguiente estado
                userdata.direccion_out = self.__pose            	
                self.__is_dir = False
		        
                return 'outcome1'
            	 
            elif bt1 == True:
                bt1 = False
                return 'outcome2'
    
    # Callback de la cámara
    def __camera_cb(self, data):        
        
        self.__pose = self.positions[ord(data.data) - ord('a')]
        self.__is_dir = True
        self.__start_rec.publish("stop")
        

    

# Image leída 
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
        
        # Sonido
        sound.publish(2)
        
        time.sleep(0.8)
        
        # Pasa la dirección si no se pulsó el botón 1
        if bt1 == False:
            userdata.direccion_out = userdata.direccion_in
   
            return 'outcome1'
        else:
            return 'outcome2'



# Recoger la carta
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
        
        # Espera unos segundos y publica en el nodo de la pinza para recoger la carta
        time.sleep(wait_time)
        
        print("------ Cerrar pinza ------")
        arm.publish("recoger")
        
        time.sleep(wait_time)
        
        # Enciende los leds en verde
        print("------ Luz verde ------")
        led1.publish(2)
        led2.publish(2)
        
        time.sleep(0.8)
        
        # Pasa la dirección al siguiente estado si no se pulsa el botón 1
        if bt1 == False:
            userdata.direccion_out = userdata.direccion_in
            return 'outcome1'
        else:
            bt1 = False
            return 'outcome2'


    
# Ir al destino
class Ir_destino(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['outcome1'],
                             input_keys=['direccion_in'],
                             output_keys=['prev_direccion_out'])
        
        # Suscribirse al topic de la posición de inicio de movimiento
        rospy.Subscriber("/arrive", Pose, self.__get_prev_pose_)
        
        # Pose anterior al movimiento y flag para indicar cuando se recibe
        self.__prev_pose = Pose()
        self.__get_prev_pose = False
        
    # Callback para cuando se reciba la posición anterior
    def __get_prev_pose_(self, data):
        self.__prev_pose = data
        self.__get_prev_pose = True
        
    
    def execute(self, userdata):
        global bt0, bt1, bt2, led1, led2, go_pose
    
        print("------ Ir destino ------")
        self.__get_prev_pose = False 

        # Enciende los leds en naranja        
        led1. publish(1)
        led2.publish(1)
        
        time.sleep(0.8)
        
        # Crea el mensaje
        desiredPose = PoseStamped()
        desiredPose.header.frame_id = "map"
        desiredPose.pose = userdata.direccion_in
        
        # Publica el mensaje
        go_pose_pub.publish(desiredPose)
        
        # Mientras no reciba el mensaje de que ha llegado al objetivo espera
        while not self.__get_prev_pose:
            pass
        
        # Pasa la posición anterior al movimiento al siguiente estado
        userdata.prev_direccion_out = self.__prev_pose

        return 'outcome1'
    
            

# Llega al destino
class Llega_destino(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['outcome1'],
                             input_keys=['prev_direccion_in'],
                             output_keys=['prev_direccion_out'])
        
        
    def execute(self, userdata):
        global bt0, bt1, bt2, sound, arm
        print("------ Llega destino ------")
        
        time.sleep(1)
        
        # Sonido
        print("------ Sonido ------")
        sound.publish(1)
        time.sleep(0.8)
        
        # Pasa la posición anterior al movimiento al siguiente estado
        userdata.prev_direccion_out = userdata.prev_direccion_in

        return 'outcome1'



# Recogida
class Recogida(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['outcome1'],
                             input_keys=['prev_direccion_in'],
                             output_keys=['prev_direccion_out'])
        
        
    def execute(self, userdata):
        global bt0, bt1, bt2, arm
        print("--- Recogida ---")
        
        # Bucle mientras espera la pulsación del botón
        while not rospy.is_shutdown():
            if bt0 == True:
                
                # Publica el mensaje soltar 
                time.sleep(wait_time/2)
                arm.publish("soltar")       # Sujeto a cambios
                time.sleep(wait_time)

                print("Entregado")
                # Pasa la posición anterior al movimiento al siguiente estado
                userdata.prev_direccion_out = userdata.prev_direccion_in

                bt0 = False
                
                return 'outcome1'



# Función para el main
def main():
    
    # Nodo
    rospy.init_node('smach_example_state_machine')

    # Máquina de estados
    sm = smach.StateMachine(outcomes=['outcome4'])

    # Los mensajes entre estados son de tipo Pose
    sm.userdata.pose = Pose()

    with sm:
        
        # Añade los estados 
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

    outcome = sm.execute()
# Main
if __name__ == '__main__':
    main()
