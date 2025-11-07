import cv2
import vlib
import navlib
import numpy as np
from scipy.optimize import curve_fit

cap = vlib.open_webcam(2)

def exp_model(y, a, b, c):
    return a * np.exp(b * y) + c

def calibrate_distance(y_pixels, real_distances):
    y_pixels = np.array(y_pixels, dtype= float)
    real_distances = np.array(real_distances, dtype= float)

    popt, _ = curve_fit(exp_model, y_pixels, real_distances, maxfev= 5000)
    return popt

y_pixels = []
real_distances = []

#params = navlib.calibrate_distance(y_pixels, real_distances)

while True:

    ret, frame = cap.read()

    if not ret:
                
        print("Não foi possível capturar frame")
        break

    frame = vlib.image_processing(frame)

    cv2.imshow('Linhas', frame)

    if cv2.waitKey(25) == ord('q'):
        break


cap.release()
cv2.destroyAllWindows()