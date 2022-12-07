#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from std_msgs.msg import String


class arm_controller():

    def __init__(self) -> None:

        # Inicia el nodo
        rospy.init_node("arm_controller")

        # Suscripcion a topics para coger y dejar carta
        rospy.Subscriber("/arm_controller/pick",String,self.pick_callback)
        rospy.Subscriber("/arm_controller/place",String,self.pick_callback)

        # Publica en topics del control del brazo y pinza
        self._shoulder = rospy.Publisher("/arm_shoulder_pitch_joint/command",Float64,queue_size=10)
        self._elbow = rospy.Publisher("/arm_elbow_pitch_joint/command",Float64,queue_size=10) 
        self._wrist = rospy.Publisher("/arm_wrist_pitch_joint/command",Float64,queue_size=10)
        self._gripper = rospy.Publisher("/arm_gripper_prismatic_joint/command",Float64,queue_size=10)

    def extend_arm(self):
        self._shoulder.publish(-1)
        self._elbow.publish(0.8)
        self._wrist.publish(-0.5)

    def retract_arm(self):
        self._shoulder.publish(-1.5)
        self._elbow.publish(0)
        self._wrist.publish(0)

    def gripper_open(self):
        self._gripper.publish(0)

    def gripper_close(self):
        self._gripper.publish(1.2)

    def pick_callback(self):
        self.gripper_open()
        self.extend_arm()
        rospy.sleep(3)
        self.gripper_close()
        rospy.sleep(1.5)
        self.retract_arm()
        
    def place_callback(self):
        self.extend_arm()
        rospy.sleep(3)
        self.gripper_open()
        rospy.sleep(1)
        self.retract_arm()

if __name__ == "__main__":
    
    controller = arm_controller()

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        
        rate.sleep()

