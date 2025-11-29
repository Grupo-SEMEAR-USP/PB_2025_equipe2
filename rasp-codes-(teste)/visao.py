import cv2
import vlib
import time
import numpy as np
from scipy.optimize import curve_fit

cap = vlib.open_webcam(2)

def exp_model(y, a, b, c):
    return a * np.exp(b * y) + c

y_pixels = [20,61,102,253,348]
real_distances = [0.704,0.575,0.52,0.312,0.247]

def calibrate_distance(y_pixels, real_distances):
    y_pixels = np.array(y_pixels, dtype= float)
    real_distances = np.array(real_distances, dtype= float)

    popt, _ = curve_fit(exp_model, y_pixels, real_distances, maxfev= 5000)
    return popt

while True:

    ret, frame = cap.read()
    frame = cv2.rotate(frame, cv2.ROTATE_180)

    if not ret:
            
        print("Não foi possível capturar frame")
        break

    a, b, c = calibrate_distance(y_pixels, real_distances)
    #print(f"{a},{b},{c}")

    frame, h = vlib.image_processing(frame)
    print(f"{h}")

    cv2.imshow('Linhas', frame)

    time.sleep(0.1)

    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()