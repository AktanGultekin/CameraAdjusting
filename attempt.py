import cv2
from cvzone.FaceDetectionModule import FaceDetector
import numpy as np
import serial
import time

def ServoYaz(X, Y):
    pozlar = "{200," + str(int(X)) + "," + str(int(Y)) + "}"
    print(pozlar)
    arduino.write(bytes(pozlar, "utf-8"))
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
servoPos = [90, 90]  # initial servo position
ServoYaz(90, 90)  # reset

# Kamera özellikleri
focal_length_mm = 4.9  # mm cinsinden odak uzaklığı
sensor_height_mm = 4.0  # mm cinsinden sensör yüksekliği (1/4 inç sensör varsayımı)
sensor_height_pixels = 480  # Sensörün piksel cinsinden yüksekliği (640 x 480 çözünürlük)

# Gerçek odak uzaklığı piksel cinsinden hesaplama
focal_length_pixels = (focal_length_mm / sensor_height_mm) * sensor_height_pixels

real_face_height = 0.25  # Ortalama yüz yüksekliği, metre cinsinden

while True:
    success, img = cap.read()
    img, bboxs = detector.findFaces(img, draw=False)

    if bboxs:
        # Get the coordinates and size of the bounding box
        fx, fy = bboxs[0]["center"][0], bboxs[0]["center"][1]
        face_height_in_image = bboxs[0]["bbox"][3] - bboxs[0]["bbox"][1]

        # Ensure face_height_in_image is not zero to avoid division by zero error
        if face_height_in_image > 0:
            # Calculate the distance to the face
            distance = ((focal_length_pixels * real_face_height) / face_height_in_image)/4
            print("Distance: {:.3f} m".format(distance))

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
            cv2.putText(img, str(pos), (fx + 15, fy - 15), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)
            cv2.line(img, (0, fy), (ws, fy), (0, 0, 0), 2)  # x line
            cv2.line(img, (fx, hs), (fx, 0), (0, 0, 0), 2)  # y line
            cv2.circle(img, (fx, fy), 15, (0, 0, 255), cv2.FILLED)
            cv2.putText(img, "TARGET LOCKED", (850, 50), cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 255), 3)
            cv2.putText(img, f'Distance: {distance:.2f} m', (850, 100), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 2)
        else:
            cv2.putText(img, "Invalid Face Height", (850, 100), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 255), 2)

    else:
        cv2.putText(img, "NO TARGET", (880, 50), cv2.FONT_HERSHEY_PLAIN, 3, (0, 0, 255), 3)
        cv2.circle(img, (640, 360), 80, (0, 0, 255), 2)
        cv2.circle(img, (640, 360), 15, (0, 0, 255), cv2.FILLED)
        cv2.line(img, (0, 360), (ws, 360), (0, 0, 0), 2)  # x line
        cv2.line(img, (640, hs), (640, 0), (0, 0, 0), 2)  # y line

    """
    try:
        ServoYaz(servoPos[0], servoPos[1])
    except:
        pass
    """

    cv2.imshow("Image", img)
    cv2.waitKey(1)
