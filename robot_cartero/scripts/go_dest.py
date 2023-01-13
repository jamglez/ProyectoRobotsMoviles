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
pub_pose = rospy.Publisher("/move_base/goal", move_base_msgs.msg.MoveBaseActionGoal, queue_size=10)

# Flag para cuando se reciba la primera posición en el /action/feedback
get_prev_pose = False

# Posición de inicio del movimiento
prev_pose = Pose()
end = False

# Callback para el feedback del action_server: almacena el primer envío
def cb(data):
    global get_prev_pose, prev_pose
    if get_prev_pose == False:                      # Si no se ha registrado ninguna
            get_prev_pose = True                    # Se poner el falg a True
            prev_pose = data.feedback.base_position.pose     # Se almacena la posición


def cb_result(data):
    global end

    end = True

# Callback para la recepción del objetivo
def go_pose(data):
    global pub, prev_pose, get_prev_pose, pub_pose, end

    # Se crea el goal con la ubicación recibida de la máquina de estados
    client = actionlib.SimpleActionClient('move_base', move_base_msgs.msg.MoveBaseAction)
    
    client.wait_for_server()    # Espera al servido

    goal = move_base_msgs.msg.MoveBaseActionGoal()

    goal.goal.target_pose.header.frame_id = "map"
    goal.goal.target_pose.header.frame_id = "map"
    goal.goal.target_pose.pose.position.x = data.pose.position.x
    goal.goal.target_pose.pose.position.y = data.pose.position.y
    #La orientación es un quaternion. Tenemos que fijar alguno de sus componentes

    goal.goal.target_pose.pose.orientation.z = data.pose.orientation.z
    goal.goal.target_pose.pose.orientation.w = data.pose.orientation.w

    print(goal)
    
    # Envía el goal y espera el resultado
    pub_pose.publish(goal)

    while not end:
        pass

    # Al acabar, envia la posición anterior a través del topic
    pub.publish(prev_pose)
    
    # Reinicia las variables
    prev_pose = Pose()
    get_prev_pose = False
    end = False
    
# Suscriptor al topic donde se envian las posiciones objetivos
rospy.Subscriber("/go_pose", PoseStamped, go_pose)
rospy.Subscriber("move_base/feedback", move_base_msgs.msg.MoveBaseActionFeedback, cb)
rospy.Subscriber("move_base/result", move_base_msgs.msg.MoveBaseActionResult, cb_result)


# Main: bucle infinito en suspensión, solo se ejecuta código en el callback
if __name__ == "__main__":
    while True:
        pass