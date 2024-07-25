
# import os
# os.environ['TF_ENABLE_ONEDNN_OPTS'] = '0'

#Z oranını bularak mesafe ölçülebilir bunu yapacağız

import cv2
from cvzone.FaceDetectionModule import FaceDetector
import numpy as np
import serial 
import time


def ServoYaz(X,Y,Z):
    #pozlar = "200," + servoPos[0] + "," + servoPos[1] + "\n"
    pozlar = "{200," + str(int(X)) + "," + str(int(Y)) + str(int(Z)) + "}"
    print(pozlar)
    arduino.write(bytes(pozlar,"utf-8"))
    time.sleep(0.3)
    
 #  coming = arduino.readline()
 #  #coming = arduino.read_until(b"\n")
 #  coming = str(coming,'utf-8')
 #  if coming>"":print(coming)

###############################################################################

cap = cv2.VideoCapture(0)
ws, hs = 1280, 720
zz = 1468 #eni 1280 boyu 720 olan bir dikdörtgenin köşegen uzunluğu pisagordan 1468 olur
cap.set(3, ws)
cap.set(4, hs)

if not cap.isOpened():
    print("Camera couldn't Access!!!")
    exit()

arduino = serial.Serial("COM1",115200,timeout=0.1)

detector = FaceDetector()
servoPos = [90, 90, 90] # initial servo position
ServoYaz(90,90,90) #resetleyelim

while True:
    success, img = cap.read()
    img, bboxs = detector.findFaces(img, draw=False)
    print(bboxs)

    if bboxs:
        #get the coordinate
        fx, fy, fz= bboxs[0]["center"][0], bboxs[0]["center"][1], bboxs[0]["bbox"][2] #bbox tuple'ından kameraya olan uzaklığıma göre değişen z değerini aldım
        pos = [fx, fy, fz]
        #convert coordinat to servo degree
        servoX = np.interp(fx, [0, ws], [0, 180])
        servoY = np.interp(fy, [0, hs], [0, 180])
        servoZ = np.interp(fz, [0, zz], [0, 180]) #servoZ değişkeni tanımladım

        if servoX < 0:
            servoX = 0
        elif servoX > 180:
            servoX = 180
        if servoY < 0:
            servoY = 0
        elif servoY > 180:
            servoY = 180
        if servoZ < 0:      #servoZ kısıtlamalarını ekledim
            servoZ = 0
        elif servoZ > 180:
            servoZ = 180

        servoPos[0] = servoX
        servoPos[1] = servoY
        servoPos[2] = servoZ#servoPos listesine servoZ'i ekledim

        #eğer y değeri azalırsa x değeri artar
        cv2.circle(img, (fx, fy), 80, (0, 0, 255), 2)
        cv2.putText(img, str(pos), (fx+15, fy-15), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2 )
        cv2.line(img, (0, fy), (ws, fy), (0, 0, 0), 2)  # x line
        cv2.line(img, (fx, hs), (fx, 0), (0, 0, 0), 2)  # y line
        cv2.line(img, (fz, ws), (hs, fz), (0, 0, 0), 2) # experimental z line
        cv2.circle(img, (fx, fy), 15, (0, 0, 255), cv2.FILLED)
        cv2.putText(img, "TARGET LOCKED", (850, 50), cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 255), 3 )

    else:
        cv2.putText(img, "NO TARGET", (880, 50), cv2.FONT_HERSHEY_PLAIN, 3, (0, 0, 255), 3)
        cv2.circle(img, (640, 360), 80, (0, 0, 255), 2)
        cv2.circle(img, (640, 360), 15, (0, 0, 255), cv2.FILLED)
        cv2.line(img, (0, 360), (ws, 360), (0, 0, 0), 2)  # x line
        cv2.line(img, (640, hs), (640, 0), (0, 0, 0), 2)  # y line

    cv2.putText(img, f'Servo X: {int(servoPos[0])} deg', (50, 50), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)
    cv2.putText(img, f'Servo Y: {int(servoPos[1])} deg', (50, 100), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)
    cv2.putText(img, f'Servo Z: {int(servoPos[2])} deg', (50, 150), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)

    estimated_distance = 0 #cm birim

    match servoPos[2]:
        case degree if 70>=degree>65:
            estimated_distance = 25
            print(f"Estimated distance(approximately):{estimated_distance} cm")

        case degree if 65>=degree>60:
            estimated_distance = 35
            print(f"Estimated distance(approximately):{estimated_distance} cm")
        
        case degree if 60>=degree>55:
            estimated_distance = 45
            print(f"Estimated distance(approximately):{estimated_distance} cm")
        
        case degree if 55>=degree>50:
            estimated_distance = 55
            print(f"Estimated distance(approximately):{estimated_distance} cm")
        
        case degree if 50>=degree>45:
            estimated_distance = 65
            print(f"Estimated distance(approximately):{estimated_distance} cm")
        
        case degree if 45>=degree>40:
            estimated_distance = 75
            print(f"Estimated distance(approximately):{estimated_distance} cm")

        case degree if 40>=degree>35:
            estimated_distance = 85
            print(f"Estimated distance(approximately):{estimated_distance} cm")

        case degree if 35>=degree>30:
            estimated_distance = 95
            print(f"Estimated distance(approximately):{estimated_distance} cm")

        case degree if 30>=degree>25:
            estimated_distance = 105
            print(f"Estimated distance(approximately):{estimated_distance} cm")
        
        case degree if 25>=degree>21:
            estimated_distance = 115
            print(f"Estimated distance(approximately):{estimated_distance} cm")

        case degree if 20>=degree or degree>70:
            print("Distance limit is reached.")


    try:
        #servo_pinX.write(servoPos[0])
        #servo_pinY.write(servoPos[1])
        ServoYaz(servoPos[0],servoPos[1], servoPos[2])
        

        #time.sleep(1)
        
    except :
        pass

    cv2.imshow("Image", img)
    cv2.waitKey(1)
    


    