#!/usr/bin/env python3

import rospy
import cv2 as cv
import pytesseract as pt
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from imutils.video import FPS
from kobuki_msgs.msg import Sound
# import keras_ocr

class text_recognizer():
    
    def __init__(self) -> None:

        #Configuracion del reconocimiento texto de pytesseract
        self.__confg = r'--oem 3 --psm 7 outputbase digits'
        self.__count = 0
        self.__number = ""
        
        # Nodo ROS
        rospy.init_node("text_recognition")

        # Publisher en los topics:
        self.__sound = rospy.Publisher("/mobile_base/commands/sound", Sound, queue_size=10) #LED tambien?

        self.__text_pub = rospy.Publisher("/text_rec", String, queue_size=10)
        rospy.Subscriber("/text_rec_start", String, self.__start_cb)
        self.__rec = False
        

    # Comprueba el numero de mesa durante varios frames
    def __check_number(self,text):

        if self.__number == text and "DESTINO" in text:
                self.__count +=1
        else:
            self.__number = text
            self.__count = 1


    def __start_cb(self, data):
        self.__rec = data.data == "start"

    def webcam_test(self):
        
        fps = FPS().start()
        cap = cv.VideoCapture(0)
        cv.namedWindow('frame',cv.WINDOW_NORMAL)
        cv.resizeWindow('frame',640,360)
        
        scale_percent = 50

        while(True):
            
            if self.__rec:
                ret, frame_ = cap.read()
            
                if ret :
                    width = int(frame_.shape[1] * scale_percent / 100)
                    height = int(frame_.shape[0] * scale_percent / 100)
                    dim = (width, height)

                    frame = cv.resize(frame_, dim, interpolation = cv.INTER_AREA)

                    text = pt.image_to_string(frame)

                    self.__check_number(text)

                    if self.__count == 4:
                        s = String()
                        only_alpha = ""
                        for char in text:
                            
                            if ord(char) >= 65 and ord(char) <= 90:
                                only_alpha += char
                                
                            elif ord(char) >= 97 and ord(char) <= 122:
                                only_alpha += char
                        s.data = (only_alpha[-1].lower())
                        self.__rec = False
                        self.__text_pub.publish(s)
                        self.__count = 0

                    cv.imshow('frame', frame)
                    cv.waitKey(1)
            else:
                cv.destroyAllWindows()
        
        fps.stop()
        print(f"[INFO] elapsed time {round(fps.elapsed(), 2)}")
        print(f"[INFO] approx. FPS : {round(fps.fps(), 2)}")
         
        cap.release()
        cv.destroyAllWindows()


if __name__ == "__main__":

    recognizer = text_recognizer()

    rate = rospy.Rate(10)

    recognizer.webcam_test()