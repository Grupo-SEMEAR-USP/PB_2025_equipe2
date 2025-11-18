import cv2
import vlib

cap = vlib.open_webcam(2)

while True:

    ret, frame = cap.read()

    if not ret:
            
        print("Não foi possível capturar frame")
        break

    frame, area_left, area_right, closest_inter, min_angle = vlib.image_processing(frame)

    cv2.imshow('Linhas', frame)

    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()