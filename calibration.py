import cv2
import numpy as np
import glob
import os

# Satranç tahtası boyutları (iç köşelerin sayısı)
chessboard_size = (8, 6)
square_size = 1.0

# Satranç tahtası köşe noktalarının 3D yerel uzayda koordinatları
objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2) * square_size

# 3D noktalar ve 2D görüntü noktaları
objpoints = []  # 3D noktalar (dünya koordinatları)
imgpoints = []  # 2D noktalar (görüntü düzleminde)

# Kalibrasyon görüntüleri klasörü
image_dir = "images"  # İndirilen satranç tahtası görüntülerinin bulunduğu klasör
images = glob.glob(os.path.join(image_dir, "*.jpg"))

for image_path in images:
    img = cv2.imread(image_path)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)
    if ret:
        objpoints.append(objp)
        imgpoints.append(corners)
        cv2.drawChessboardCorners(img, chessboard_size, corners, ret)
        cv2.imshow('Image', img)
        cv2.waitKey(500)

cv2.destroyAllWindows()

# Kamera kalibrasyonu
ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# Kalibrasyon parametrelerini kaydet
np.savez("calibration_data.npz", camera_matrix=camera_matrix, dist_coeffs=dist_coeffs)

print("Kamera matrisi:")
print(camera_matrix)
print("\nBozulma katsayıları:")
print(dist_coeffs)
