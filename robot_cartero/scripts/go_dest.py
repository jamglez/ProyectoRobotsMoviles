#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Bool
import move_base_msgs.msg
import actionlib

# Nodo
rospy.init_node("go_node")

# Publisher de arrive
pub = rospy.Publisher("/arrive", Pose, queue_size=10)
pub2 = rospy.Publisher("/go_pose", PoseStamped, queue_size=10)

# Flag para cuando se reciba la primera posición en el /action/feedback
get_prev_pose = False

# Posición de inicio del movimiento
prev_pose = Pose()


# Callback para el feedback del action_server: almacena el primer envío
def cb(data):
    global get_prev_pose, prev_pose
    if get_prev_pose == False:                      # Si no se ha registrado ninguna
            get_prev_pose = True                    # Se poner el falg a True
            prev_pose = data.base_position.pose     # Se almacena la posición

# Callback para la recepción del objetivo
def go_pose(data):
    global pub, prev_pose, get_prev_pose

    # Se crea el goal con la ubicación recibida de la máquina de estados
    client = actionlib.SimpleActionClient('move_base', move_base_msgs.msg.MoveBaseAction)
    
    client.wait_for_server()    # Espera al servido

    goal = move_base_msgs.msg.MoveBaseGoal(data)
    # print(goal)
    
    # Envía el goal y espera el resultado
    client.send_goal(goal, feedback_cb=cb)
    client.wait_for_result()
    
    # Al acabar, envia la posición anterior a través del topic
    pub.publish(prev_pose)
    
    # Reinicia las variables
    prev_pose = Pose()
    get_prev_pose = False
    
# Suscriptor al topic donde se envian las posiciones objetivos
rospy.Subscriber("/go_pose", PoseStamped, go_pose)


# Main: bucle infinito en suspensión, solo se ejecuta código en el callback
if __name__ == "__main__":
    while True:
        pass