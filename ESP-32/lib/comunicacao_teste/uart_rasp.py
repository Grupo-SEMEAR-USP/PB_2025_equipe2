import serial
import time
import random

try:
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0)

except serial.SerialException as e:

    print(f"Erro ao abrir porta serial: {e}")
    exit()


def receive_data():

    buffer_size = 1024

    while True:

        try:
            if ser.in_waiting > 0:

                data = ser.read(buffer_size)

                data.split(sep = ';')
                
                left_rads = float(data[0])
                right_rads = float(data[1])

                print(f"Vel. motor esquerdo: {left_rads}, Vel. motor direito: {right_rads}")
            
            else:

                time.sleep(0.1)


        except (serial.SerialException) as e:
            
            print(f"Erro na comunicação serial: {e}")


def send_data(left_rads, right_rads):

    buffer_to_send = f"{left_rads};{right_rads}\n"

    ser.write(buffer_to_send.encode())
    ser.flush()

    time.sleep(0.1)


if __name__ == "__main__":

    print("Mandando dados para ESP...")
    
    num = 30

    while(True):

        send_data(num, num)
        #receive_data()
    
