#!/usr/bin/env python3

import rospy
import cv2 as cv
import pytesseract as pt
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from imutils.video import FPS
from kobuki_msgs.msg import Sound
import time
# import keras_ocr

class text_recognizer():
    
    def __init__(self) -> None:

        #Configuracion del reconocimiento texto de pytesseract
        self.__count = 0
        self.__number = ""
        
        # Nodo ROS
        rospy.init_node("text_recognition")

        self.__text_pub = rospy.Publisher("/text_rec", String, queue_size=10)
        rospy.Subscriber("/text_rec_start", String, self.__start_cb)
        self.__rec = False
        self.destroy = False
        self.first_rec= False
        

    # Comprueba el numero de mesa durante varios frames
    def __check_number(self,text):

        if self.__number == text and "DESTINO" in text:
                self.__count +=1
        else:
            self.__number = text
            self.__count = 1


    def __start_cb(self, data):
        self.__rec = data.data == "start"
        self.first_rec = True
        self.destroy = True

    # Aplica el algoritmo con capturas de la webcam
    def webcam_test(self):
        
        fps = FPS().start()
        cap = cv.VideoCapture(0)                    # Empieza la camara
        cv.namedWindow('frame',cv.WINDOW_NORMAL)    # Ventana
        cv.resizeWindow('frame',640,360)
        
        # Porcentaje de escala
        scale_percent = 50
        count = 0
        # Bucle infinito
        while(True):
            
            if self.__rec:
                ret, frame_ = cap.read()            # Lee un frame de la camara 

                # Si se leyó correctamente ...
                if ret :
                    # Reescalado del frame
                    width = int(frame_.shape[1] * scale_percent / 100)
                    height = int(frame_.shape[0] * scale_percent / 100)
                    dim = (width, height)

                    frame = cv.resize(frame_, dim, interpolation = cv.INTER_AREA)

                    if not self.first_rec:
                        count = 0
                        # Transforma la imagen a texto
                        text = pt.image_to_string(frame)

                        # Comprueba si se repitió el texto
                        self.__check_number(text)

                        # Si el texto se repitio mas que un umbral, construye y envíe el mensaje 
                        if self.__count == 2:
                            s = String()        # Mensaje
                            only_alpha = ""

                            # Obtiene solo el texto 
                            for char in text:
                                if ord(char) >= 65 and ord(char) <= 90:
                                    only_alpha += char
                                    
                                elif ord(char) >= 97 and ord(char) <= 122:
                                    only_alpha += char

                            # Convierte a minusucula solo la ultima letra
                            s.data = (only_alpha[-1].lower())

                            # Deja de detectar y reinicia la cuenta
                            self.__rec = False
                            self.__count = 0
                            
                            # Envía el mensaje
                            self.__text_pub.publish(s)
                    else:
                        count = count + 1
                        
                        if count == 20:
                            self.first_rec = False
                    

                    cv.imshow('frame', frame)
                    cv.waitKey(1)
            else:
                if self.destroy:
                    cv.destroyAllWindows()
                    self.destroy = False
        
        fps.stop()
        print(f"[INFO] elapsed time {round(fps.elapsed(), 2)}")
        print(f"[INFO] approx. FPS : {round(fps.fps(), 2)}")
         
        cap.release()
        cv.destroyAllWindows()


if __name__ == "__main__":

    recognizer = text_recognizer()

    rate = rospy.Rate(10)

    recognizer.webcam_test()