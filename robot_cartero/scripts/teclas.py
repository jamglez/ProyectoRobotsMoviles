#!/usr/bin/env python3

from pynput import keyboard as kb
import rospy
from std_msgs.msg import String

rospy.init_node("teclas")
pub = rospy.Publisher("/teclas", String, queue_size=10)

def callback(tecla):
    global pub
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

    elif(str(tecla) == "'d'"):
        s.data = "d"
    
    elif(str(tecla) == "'e'"):
        s.data = "e"

    elif(str(tecla) == "'f'"):
        s.data = "f"

    elif(str(tecla) == "'g'"):
        s.data = "g"

    elif(str(tecla) == "'h'"):
        s.data = "h"

    elif(str(tecla) == "'i'"):
        s.data = "i"

    elif(str(tecla) == "'j'"):
        s.data = "j"

    elif(str(tecla) == "'k'"):
        s.data = "k"

    elif(str(tecla) == "'l'"):
        s.data = "l"

    elif(str(tecla) == "'m'"):
        s.data = "m"

    elif(str(tecla) == "'n'"):
        s.data = "n"

    elif(str(tecla) == "'o'"):
        s.data = "o"

    elif(str(tecla) == "'p'"):
        s.data = "p"

    elif(str(tecla) == "'q'"):
        s.data = "q"

    elif(str(tecla) == "'r'"):
        s.data = "r"

    elif(str(tecla) == "'s'"):
        s.data = "s"

    elif(str(tecla) == "'t'"):
        s.data = "t"

    elif(str(tecla) == "'u'"):
        s.data = "u"

    elif(str(tecla) == "'v'"):
        s.data = "v"

    pub.publish(s)


# Main
if __name__ == '__main__':
    
    
    print("------- Start --------")
    kb.Listener(callback).run()