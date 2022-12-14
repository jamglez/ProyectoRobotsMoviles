#! /usr/bin/env python3
import rospy
import actionlib
from geometry_msgs.msg import PoseStamped
import move_base_msgs.msg

'''

get_state(): Esta función, cuando se le llama, devuelve un entero que indica en que estado (status) de la
acción que se está realizando.
○ 0 -> pendiente.
○ 1 -> activo.
○ 2 -> realizado.
○ 3 -> advertencia.
○ 4 -> error.

'''

def get_cb(msg):
    print(msg.base_position.pose)

def fibonacci_client():

    
    client = actionlib.SimpleActionClient('move_base', move_base_msgs.msg.MoveBaseAction)

    
    client.wait_for_server()
    
    desiredPose = PoseStamped()

    desiredPose.header.frame_id = "map"
    desiredPose.header.stamp = rospy.Time.now()
    
    desiredPose.pose.position.x = 4.8135480880737305#Convert distance and angle to waypoint from Polar to Cartesian co-ordinates then add current position of robot odometry 
    desiredPose.pose.position.y = 5.377022743225098
    desiredPose.pose.position.z = 0.0 #Assuming CurrPosZ is abslolute (eg barometer or GPS)
    desiredPose.pose.orientation.x = 0
    desiredPose.pose.orientation.y = 0
    desiredPose.pose.orientation.z = 0.4767427044570244
    desiredPose.pose.orientation.w = 0.8790428850442976    
    
    
    desiredPose2 = PoseStamped()

    desiredPose2.header.frame_id = "map"
    desiredPose2.header.stamp = rospy.Time.now()

    desiredPose2.pose.position.x = 7.04
    desiredPose2.pose.position.y = 2.18
        
    desiredPose2.pose.orientation.x = 0.0
    desiredPose2.pose.orientation.y = 0.0
    desiredPose2.pose.orientation.z = -0.707
    desiredPose2.pose.orientation.w = 0.707
    
    
    goal = move_base_msgs.msg.MoveBaseGoal(desiredPose)
    

    client.send_goal(goal, feedback_cb=get_cb)
    
    while client.get_state() != 3:
        b = 1

    print(client.get_state())
    return client.get_result() 



if __name__ == '__main__':
    rospy.init_node('fibonacci_client_py')
    result = fibonacci_client()
    print("Result:", result)
