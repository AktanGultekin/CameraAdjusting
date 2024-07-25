import cv2
from cvzone.FaceDetectionModule import FaceDetector
import numpy as np
import serial
import time

"""
Camera Calibration Values: 
Kamera matrisi:
[[2.32281660e+05 0.00000000e+00 9.40382613e+02]
 [0.00000000e+00 2.32292939e+05 5.80784721e+02]
 [0.00000000e+00 0.00000000e+00 1.00000000e+00]]

Bozulma katsayıları:
[[-7.92369092e-02 -7.33690374e-02  2.25473785e-03  1.23448011e-03
  -3.10788749e-05]]

"""

def ServoYaz(X,Y):
    pozlar = "{200," + str(int(X)) + "," + str(int(Y)) + "}"
    print(pozlar)
    arduino.write(bytes(pozlar,"utf-8"))
    time.sleep(0.3)

cap = cv2.VideoCapture(0)
ws, hs = 1280, 720
cap.set(3, ws)
cap.set(4, hs)

if not cap.isOpened():
    print("Camera couldn't Access!!!")
    exit()

arduino = serial.Serial("COM1", 115200, timeout=0.1)
detector = FaceDetector()
servoPos = [90, 90] # initial servo position
ServoYaz(90,90) #reset

# Camera calibration parameters (example values)
focal_length = 42.75  # Example focal length in pixels. Calculated in https://www.omnicalculator.com/other/focal-length (not exact values)
real_face_height = 0.25  # Average face height in meters (16 cm). My head was 0.25 m

while True:
    success, img = cap.read()
    img, bboxs = detector.findFaces(img, draw=False)

    if bboxs:
        # Get the coordinates and size of the bounding box
        fx, fy = bboxs[0]["center"][0], bboxs[0]["center"][1]
        face_width_in_image = bboxs[0]["bbox"][2] - bboxs[0]["bbox"][0]
        face_height_in_image = bboxs[0]["bbox"][3] - bboxs[0]["bbox"][1]
        
        # Calculate the distance to the face
        distance = ((focal_length * real_face_height) / face_height_in_image)
        print("Distance: {:.3} m".format(distance))
        
        pos = [fx, fy]
        servoX = np.interp(fx, [0, ws], [0, 180])
        servoY = np.interp(fy, [0, hs], [0, 180])

        if servoX < 0:
            servoX = 0
        elif servoX > 180:
            servoX = 180
        if servoY < 0:
            servoY = 0
        elif servoY > 180:
            servoY = 180

        servoPos[0] = servoX
        servoPos[1] = servoY

        cv2.circle(img, (fx, fy), 80, (0, 0, 255), 2)
        cv2.putText(img, str(pos), (fx+15, fy-15), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2 )
        cv2.line(img, (0, fy), (ws, fy), (0, 0, 0), 2)  # x line
        cv2.line(img, (fx, hs), (fx, 0), (0, 0, 0), 2)  # y line
        cv2.circle(img, (fx, fy), 15, (0, 0, 255), cv2.FILLED)
        cv2.putText(img, "TARGET LOCKED", (850, 50), cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 255), 3 )
        cv2.putText(img, f'Distance: {distance:.2f} m', (850, 100), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 2)

    else:
        cv2.putText(img, "NO TARGET", (880, 50), cv2.FONT_HERSHEY_PLAIN, 3, (0, 0, 255), 3)
        cv2.circle(img, (640, 360), 80, (0, 0, 255), 2)
        cv2.circle(img, (640, 360), 15, (0, 0, 255), cv2.FILLED)
        cv2.line(img, (0, 360), (ws, 360), (0, 0, 0), 2)  # x line
        cv2.line(img, (640, hs), (640, 0), (0, 0, 0), 2)  # y line

    #cv2.putText(img, f'Servo X: {int(servoPos[0])} deg', (50, 50), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)
    #cv2.putText(img, f'Servo Y: {int(servoPos[1])} deg', (50, 100), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)

    try:
        ServoYaz(servoPos[0], servoPos[1])
    except:
        pass

    cv2.imshow("Image", img)
    cv2.waitKey(1)
