import serial
import time

class ArduinoCommunicator:
    def __init__(self, port, baudrate=115200, timeout=1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
        self.maxPosition = 0.7

    def connect(self):
        """시리얼 포트 연결을 초기화합니다."""
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            time.sleep(2)  # 아두이노 리셋 후 통신을 위한 충분한 시간 제공
            print(f"Connected to {self.port} at {self.baudrate} baudrate.")
        except serial.SerialException as e:
            print(f"Error connecting to {self.port}: {e}")
            raise

    def send_data(self, input1, input2):
        """입력 값을 아두이노로 전송합니다."""
        if self.ser is None:
            raise ValueError("Serial connection not established. Call connect() first.")
        
        input1, input2 = self.scaled(input1, input2)

        data = f"{input1},{input2}\n"
        self.ser.write(data.encode())
        print(f"Sent data: {data.strip()}")

    def scaled(self, input1, input2):
        scaledInput1 = (input1 + self.maxPosition) / (self.maxPosition * 2) * 1000 + 1000
        scaledInput2 = (input2 + self.maxPosition) / (self.maxPosition * 2) * 1000 + 1000
        return scaledInput1, scaledInput2

    def close(self):
        """시리얼 포트를 닫습니다."""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Serial connection closed.")

# 사용 예시
if __name__ == "__main__":
    port = 'COM3'  # Windows 예: 'COM3', Linux/Mac 예: '/dev/ttyUSB0'
    communicator = ArduinoCommunicator(port)

    try:
        communicator.connect()
        input1 = 0.5  # 예시 입력값
        input2 = -0.4  # 예시 입력값
        communicator.send_data(input1, input2)
    finally:
        communicator.close()
