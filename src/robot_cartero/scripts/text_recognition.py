#!/usr/bin/env python3

import rospy
import cv2 as cv
import pytesseract as pt
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from imutils.video import FPS
from kobuki_msgs.msg import Sound
# import keras_ocr

class text_recognizer():
    
    def __init__(self) -> None:

        #Configuracion del reconocimiento texto de pytesseract
        self.__confg = r'--oem 3 --psm 7 outputbase digits'
        self.__count = 0
        self.__number = 0
        
        # Nodo ROS
        rospy.init_node("text_recognition")

        # Suscriber a los topics: 
        rospy.Subscriber('/camera/rgb/image_raw/compressed',Image,self.__camera_callback)
        self.__bridge = CvBridge()

        # Publisher en los topics:
        self.__sound = rospy.Publisher("/mobile_base/commands/sound", Sound, queue_size=10) #LED tambien?


    # Comprueba el numero de mesa durante varios frames
    def __check_number(self,text):
        for n in range(1,6):

            if str(n) in text:
                # print(n)
                if self.__number == n:
                        self.__count +=1
                else:
                    self.__number = n
                    self.__count = 1


    def webcam_test(self):
        
        fps = FPS().start()
        cap = cv.VideoCapture(0)
        cv.namedWindow('frame',cv.WINDOW_NORMAL)
        cv.resizeWindow('frame',1280,720)
        
        while(True):

            ret, frame = cap.read()
        
            if ret :

                data = pt.image_to_data(frame,config=self.__confg)
                text = pt.image_to_string(frame,config=self.__confg)

                frame = self.bounding_box_text(data,frame)

                self.__check_number(text)

                if self.__count == 3:
                    print(f'Mesa: {self.__number}')
                    self.__count = 0

                cv.imshow('frame', frame)
            
            fps.update()

            if cv.waitKey(1) & 0xFF == ord('q'):
                break
        
        fps.stop()
        print(f"[INFO] elapsed time {round(fps.elapsed(), 2)}")
        print(f"[INFO] approx. FPS : {round(fps.fps(), 2)}")
         
        cap.release()
        cv.destroyAllWindows()
        
    def __camera_callback(self,image):
        #Hacer que deje de detectar cuando no haga falta

        cv_image = self.__bridge.imgmsg_to_cv2(image, desired_encoding='passthrough')

        # data = pt.image_to_data(frame)
        # frame = self.bounding_box_text(data,frame)

        text = pt.image_to_string(cv_image,config=self.__confg)

        self.__check_number(text)

        if self.__count == 3:
            print(f'Mesa: {self.__number}')
            self.__sound.publish(1)
            self.__count = 0

        cv.imshow('frame', cv_image)
        # cv.waitKey(1)
            

    #Dibujar las bounding boxes
    def bounding_box_text(self,data,frame):
                        
        for z, a in enumerate(data.splitlines()):
            if z!= 0:
                a = a.split()
                if len(a) == 12:
                    x,y,width,height  = int(a[6]), int(a[7]),int(a[8]), int (a[9])
                    cv.rectangle(frame, (x,y), (x+width,y+height), (255,0,0), 1)
                    cv.putText(frame, a[11], (x,y+25), cv.FONT_HERSHEY_PLAIN,1,(0,0,255),2)

        return frame


if __name__ == "__main__":

    recognizer = text_recognizer()

    rate = rospy.Rate(10)

    recognizer.webcam_test()

    while not rospy.is_shutdown():
        rate.sleep()