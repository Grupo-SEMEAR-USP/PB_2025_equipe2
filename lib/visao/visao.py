import cv2
import numpy as np

# Abre câmera
cap = cv2.VideoCapture(0)

if not cap.isOpened():

    print("Não foi possível abrir a webcam")
    exit()

while True:

    ret, frame = cap.read()

    if not ret:
        
        print("Não foi possível capturar frame")
