import cv2
import vlib

cap = vlib.open_webcam(0)

while True:

    ret, frame = cap.read()
    #frame = cv2.rotate(frame, cv2.ROTATE_180)

    if not ret:
            
        print("Não foi possível capturar frame")
        break

    frame = vlib.image_processing(frame)

    cv2.imshow('Linhas', frame)

    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()