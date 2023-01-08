#!/usr/bin/env python3

import rospy
import smach
from geometry_msgs.msg import Pose
from std_msgs.msg import String
import time
from geometry_msgs.msg import PoseStamped

# Variables para los botones
bt0 = False
bt1 = False
bt2 = False

    
# Callback de los botones como pulsaciones de teclas
def button_cb(data):
    global bt0, bt1, bt2

    if data.data == "0":
        bt0 = True
    elif data.data == "1":
        bt1 = True
    elif data.data == "2":
        bt2 = True
    
# Variables de ROS
rospy.Subscriber("/teclas", String, button_cb)                              # Suscriptor a las pulsaciones de teclas
go_pose_pub = rospy.Publisher("/go_pose", PoseStamped, queue_size=10)       # Publisher para publicar el destino
arm = rospy.Publisher("/arm", String, queue_size=10)                        # Publisher para el nodo del brazo

# Tiempo de espera
wait_time = 1

# Reposo
class Reposo(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['outcome1','outcome2', 'outcome3'],
                             input_keys=['prev_direccion_in'],
                             output_keys=['prev_direccion_out'])

        # Posición HOME
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
        
        time.sleep(0.8)
        
        # Pasará al siguiente estado según pulsaciones del botón
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



# Detectar imagen: estado donde se detectan imágenes por la cámara
class Detectar(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['outcome1','outcome2'],
                             output_keys=['direccion_out'])
        
        # TODO: ###################### CAMBIAR EL TOPIC DE LAS TECLAS POR EL DE LA CÁMARA #########################
        rospy.Subscriber("/text_rec", String, self.__camera_cb)      
        
        self.__start_rec = rospy.Publisher("/text_rec_start", String)

        # Posición objetivos
        self.__pose = Pose()

        # Flag por si se detectó la imagen
        self.__is_dir = False
        
        # Lista de posiciones pre - definidas
        self.positions = []

        for i in range(22):
            self.positions.append(Pose())
        
        self.positions[0].position.x = 5.33
        self.positions[0].position.y = 6
        self.positions[0].orientation.z = 0
        self.positions[0].orientation.w = 1

        self.positions[1].position.x = 5.019
        self.positions[1].position.y = 5.5
        self.positions[1].orientation.z = -0.707
        self.positions[1].orientation.w = 0.707

        self.positions[2].position.x = 5.07
        self.positions[2].position.y = 3.77
        self.positions[2].orientation.z = 0
        self.positions[2].orientation.w = 1

        self.positions[3].position.x = 4.62
        self.positions[3].position.y = 5.4
        self.positions[3].orientation.z = 0.707
        self.positions[3].orientation.w = 0.707

        self.positions[4].position.x = 2.84
        self.positions[4].position.y = 5.18
        self.positions[4].orientation.z = 0.0
        self.positions[4].orientation.w = 1

        # F
        self.positions[5].position.x = 2.47
        self.positions[5].position.y = 5.73
        self.positions[5].orientation.z = 0.707
        self.positions[5].orientation.w = 0.707 

        self.positions[6].position.x = 2.7
        self.positions[6].position.y = 7.26
        self.positions[6].orientation.z = -0.18
        self.positions[6].orientation.w = 0.98

        self.positions[7].position.x = 2.163
        self.positions[7].position.y = 5.27
        self.positions[7].orientation.z = -0.707
        self.positions[7].orientation.w = 0.707 

        self.positions[8].position.x = 1.977
        self.positions[8].position.y = 3.41
        self.positions[8].orientation.z = 1
        self.positions[8].orientation.w = 0

        self.positions[9].position.x = -0.03
        self.positions[9].position.y = 3.64
        self.positions[9].orientation.z = 0.72
        self.positions[9].orientation.w = 0.69

        self.positions[10].position.x = 0.33
        self.positions[10].position.y = 5.726
        self.positions[10].orientation.z = 0.0
        self.positions[10].orientation.w = 1

        self.positions[11].position.x = -0.99
        self.positions[11].position.y = 7.33
        self.positions[11].orientation.z = 1
        self.positions[11].orientation.w = 0

        self.positions[12].position.x = -0.996
        self.positions[12].position.y = 5.777
        self.positions[12].orientation.z = 0.7
        self.positions[12].orientation.w = 0.717

        self.positions[13].position.x = -1.25
        self.positions[13].position.y = 5.23
        self.positions[13].orientation.z = -1
        self.positions[13].orientation.w = 0.0

        self.positions[14].position.x = -2.92
        self.positions[14].position.y = 5.08
        self.positions[14].orientation.z = -0.711
        self.positions[14].orientation.w = 0.702

        self.positions[15].position.x = -3.286
        self.positions[15].position.y = 3.085
        self.positions[15].orientation.z = 1
        self.positions[15].orientation.w = 0

        self.positions[16].position.x = -3.28
        self.positions[16].position.y = 5.405
        self.positions[16].orientation.z = 1
        self.positions[16].orientation.w = 0

        self.positions[17].position.x = -0.97
        self.positions[17].position.y = 7.46
        self.positions[17].orientation.z = 0.696
        self.positions[17].orientation.w = 0.717

        self.positions[18].position.x = -5.36
        self.positions[18].position.y = 5.55
        self.positions[18].orientation.z = -1
        self.positions[18].orientation.w = 0.0

        self.positions[19].position.x = -6.66
        self.positions[19].position.y = 8.04
        self.positions[19].orientation.z = 0.71
        self.positions[19].orientation.w = 0.698

        self.positions[20].position.x = 6.73
        self.positions[20].position.y = 1.01
        self.positions[20].orientation.z = -0.71
        self.positions[20].orientation.w = 0.7

        self.positions[21].position.x = 4.48
        self.positions[21].position.y = -0.15
        self.positions[21].orientation.z = 1
        self.positions[21].orientation.w = 0

        

    def execute(self, userdata):
        print("------ Detectando imagen ------")
        global bt0, bt1, bt2, sound, led1, led2
        
        self.__is_dir = False
        self.__start_rec.publish("start")
        
        time.sleep(0.8)
        
        # Cuando haya una dirección se pasa al siguiente estado, ...
        # ... si se pulsael botón vuelve al reposo
        while not rospy.is_shutdown():
            if self.__is_dir == True:

                userdata.direccion_out = self.__pose            	
                self.__is_dir = False
		        
                return 'outcome1'
            	 
            elif bt1 == True:
                self.__start_rec.publish("stop")
                bt1 = False
                return 'outcome2'
    
    # Callback del nodo de la cámara
    def __camera_cb(self, data):        
        self.__pose = self.positions[ord(data.data) - ord('a')]
        self.__is_dir = True
        self.__start_rec.publish("stop")


    
# Imagen Leída
class Img_leida(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['outcome1',
                                       'outcome2'],
                             input_keys=['direccion_in'],
                             output_keys=['direccion_out'])
    

    # Cuando se lee una imagen emite sonido y pasa la posición
    def execute(self, userdata):
        print("--- Imagen leida ---")
        global bt0, bt1, bt2, sound
                
        time.sleep(0.8)
        
        if bt1 == False:
            userdata.direccion_out = userdata.direccion_in
   
            return 'outcome1'
        else:
            return 'outcome2'



# Recoger carta
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
        
        # Espera
        time.sleep(wait_time)

        # Publica para recoger la carta con el brazo
        print("------ Cerrar pinza ------")
        arm.publish("recoger")
        time.sleep(wait_time)
        
        time.sleep(0.8)
        
        # Pasa la posición al siguiente estado o vuelve al inicio
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
        
        # Suscriptor al topic que avisa si llega al destino
        rospy.Subscriber("/arrive", Pose, self.__get_prev_pose_)
        
        # Posición previa y flag por posición previa
        self.__prev_pose = Pose()
        self.__get_prev_pose = False
    
    # Callback del topic /arrive
    def __get_prev_pose_(self, data):
        self.__prev_pose = data
        self.__get_prev_pose = True
        
    
    def execute(self, userdata):
        global bt0, bt1, bt2, led1, led2, go_pose
        
        print("------ Ir destino ------")
        self.__get_prev_pose = False 
        
        time.sleep(0.8)
        
        # Se crea el mensaje a enviar por el topic /go_pose y se publixa
        desiredPose = PoseStamped()
        desiredPose.header.frame_id = "map"
        desiredPose.pose = userdata.direccion_in
        
        go_pose_pub.publish(desiredPose)
        
        # Espera a recibir mensaje de llegada
        while not self.__get_prev_pose:
            pass
        
        # Pasa la dirección previa, la de inicio
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
        
        # Se pasa la posición anterior al movimiento
        userdata.prev_direccion_out = userdata.prev_direccion_in
        return 'outcome1'



# Estado de recogida
class Recogida(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['outcome1'],
                             input_keys=['prev_direccion_in'],
                             output_keys=['prev_direccion_out'])
        

    def execute(self, userdata):
        global bt0, bt1, bt2, arm
        print("--- Recogida ---")
        
        # Bucle en el que se espera la pulsación del botón 0
        while not rospy.is_shutdown():
            
            if bt0 == True:
                # Se publica en el topic del brazo para soltar    
                time.sleep(wait_time/2)
                arm.publish("soltar")       
                time.sleep(wait_time)

                # Se pasa la posición anterior al movimiento
                userdata.prev_direccion_out = userdata.prev_direccion_in
                bt0 = False
                return 'outcome1'


# Main
def main():
    rospy.init_node('smach_example_state_machine')

    # Se crea una máquina de estados
    sm = smach.StateMachine(outcomes=['outcome4'])

    # Los mensajes son de tipo Pose
    sm.userdata.pose = Pose()

    with sm:
        # Se añaden los estados
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

if __name__ == '__main__':
    main()
