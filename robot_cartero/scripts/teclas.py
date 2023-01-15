#!/usr/bin/env python3

from pynput import keyboard as kb
import rospy
from std_msgs.msg import String

rospy.init_node("teclas")
pub = rospy.Publisher("/teclas", String, queue_size=10)

def callback(tecla):
    s = String()
        
    if(str(tecla) == "'0'"):
        s.data = "0"
        
    elif(str(tecla) == "'1'"):
        s.data = "1"
           
    elif(str(tecla) == "'2'"):
        s.data = "2"

    pub.publish(s)


# Main
if __name__ == '__main__':    
    kb.Listener(callback).run()