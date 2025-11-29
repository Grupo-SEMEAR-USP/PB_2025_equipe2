import serial 
import threading
import time

SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200

class RobotHardwareInterface:
    def __init__(self):
        self.connection = None
        self.connected = False
        
        try:
            self.connection = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            time.sleep(2)  # Aguardar inicialização
            self.connected = True
            print("Interface de Hardware conectada ao ESP32.")
        except serial.SerialException as e:
            print(f"Erro Crítico: Não foi possível conectar ao hardware. {e}")
            self.connected = False
            return

        # Adicionar locks para thread safety
        self.lock = threading.Lock()
        self.current_left_wheel_velocity = 0.0
        self.current_right_wheel_velocity = 0.0

        self.is_running = True
        self.receiver_thread = threading.Thread(target=self._read_data_loop)
        self.receiver_thread.daemon = True
        self.receiver_thread.start()

    def _read_data_loop(self):
        buffer = ""
        while self.is_running and self.connected:
            if self.connection and self.connection.in_waiting > 0:
                try:
                    # Ler todos os dados disponíveis
                    data = self.connection.read(self.connection.in_waiting).decode('utf-8')
                    buffer += data
                    
                    # Processar linhas completas
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        
                        if line:
                            parts = line.split(';')
                            if len(parts) == 2:
                                try:
                                    left_vel = float(parts[0])
                                    right_vel = float(parts[1])
                                    
                                    # Atualizar com lock
                                    with self.lock:
                                        self.current_left_wheel_velocity = left_vel
                                        self.current_right_wheel_velocity = right_vel
                                    
                                    print(f"Dados ESP: L={left_vel:.3f}, R={right_vel:.3f}")
                                except ValueError as e:
                                    print(f"Erro convertendo valores: {e}")
                            else:
                                print(f"Formato inválido: {line}")
                                
                except (UnicodeDecodeError, ValueError, serial.SerialException) as e:
                    print(f"Erro na leitura serial: {e}")
                    self.connected = False
                    break
            
            time.sleep(0.01)

    def send_velocity_command(self, left_velocity, right_velocity):
        if not self.connected or not self.connection:
            print("Hardware não conectado, ignorando comando.")
            return
            
        # Formatar comando (usar ponto e vírgula como separador)
        command = f"{left_velocity:.3f};{right_velocity:.3f}\n"
        
        try:
            self.connection.write(command.encode('utf-8'))
            print(f"Comando enviado: {command.strip()}")
        except serial.SerialException as e:
            print(f"Erro ao enviar comando: {e}")
            self.connected = False

    def get_current_state(self):
        with self.lock:
            return (self.current_left_wheel_velocity, self.current_right_wheel_velocity)

    def is_connected(self):
        return self.connected

    def close(self):
        self.is_running = False
        self.connected = False
        if self.receiver_thread.is_alive():
            self.receiver_thread.join(timeout=1)
        if self.connection:
            self.connection.close()
        print("Interface de Hardware desconectada.")


