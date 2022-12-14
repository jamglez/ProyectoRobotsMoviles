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
        
    elif(str(tecla) == "'a'"):
        s.data = "a"
        
    elif(str(tecla) == "'b'"):
        s.data = "b"
           
    elif(str(tecla) == "'c'"):
        s.data = "c"

    pub.publish(s)


# Main
if __name__ == '__main__':
    
    
    print("------- Start --------")
    kb.Listener(callback).run()