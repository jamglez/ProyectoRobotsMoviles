import cv2 as cv
import pytesseract as pt

image = cv.imread("imagenes/prueba.png")
img_rgb = cv.cvtColor(image,cv.COLOR_BGR2RGB)

print(pt.image_to_string(img_rgb))
