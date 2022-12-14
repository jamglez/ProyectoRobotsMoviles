#! /usr/bin/env python3

from kobuki_msgs.msg import ButtonEvent, Led, Sound
import rospy
import time

rospy.init_node("node")

def listener(data):
    print(data.button)
    print(data.state)
    if data.button == 1:
        print("aaaaaaaaaaa")
    print("------------------------")
    print(" ")

rospy.Subscriber("/mobile_base/events/button", ButtonEvent, listener)
led2 = rospy.Publisher("/mobile_base/commands/led1", Led, queue_size=10)
sound = rospy.Publisher("/mobile_base/commands/sound", Sound, queue_size=10)

msg = Led()
msg_ = Sound()

a = False

msg_.value = msg_.ON
msg.value = msg.GREEN
sound.publish(msg_)
time.sleep(1)


print("neo")
while not rospy.is_shutdown():
    if a == False:
        sound.publish(msg_)
        a = True
    led2.publish(msg)
    pass