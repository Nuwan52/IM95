import serial
import struct
import time
import threading


class LeadshineServo:
    def __init__(self, serial_conn, slave_id: int):
        self.serial = serial_conn
        self.slave_id = slave_id
        self.lock = threading.Lock()  # protect serial port

    def calculate_crc(self, data: bytes) -> int:
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc

    def build_frame(self, function_code: int, address: int, data: bytes = b'') -> bytes:
        frame = struct.pack('>BBH', self.slave_id, function_code, address) + data
        crc = self.calculate_crc(frame)
        frame += struct.pack('<H', crc)  # little endian
        return frame

    def send_receive(self, frame: bytes, expected_len: int = 100) -> bytes:
        with self.lock:
            self.serial.reset_input_buffer()
            self.serial.write(frame)
            time.sleep(0.02)
            response = self.serial.read(expected_len)
            return response

    def read_16bit_parameters(self, address: int, count: int = 1):
        data = struct.pack('>H', count)
        frame = self.build_frame(0x03, address, data)
        response = self.send_receive(frame)
        if len(response) >= 5 and response[0] == self.slave_id:
            byte_count = response[2]
            values = []
            for i in range(0, byte_count, 2):
                values.append(struct.unpack('>H', response[3 + i:5 + i])[0])
            return values
        return []

    def write_single_register(self, address: int, value: int):
        data = struct.pack('>H', value)
        frame = self.build_frame(0x06, address, data)
        response = self.send_receive(frame)
        return response

    def move_absolute(self, target_pos: int, speed: int):
        """Example: set speed then position register (adjust addresses per Leadshine manual)"""
        # Write speed register (example address 0x2102)
        self.write_single_register(0x2102, speed)
        # Write target position register (example address 0x2103)
        self.write_single_register(0x2103, target_pos)
        # Start motion command (example: 0x2100 = 1)
        self.write_single_register(0x2100, 1)


def feedback_loop(servos):
    """Continuously read parameters from both motors"""
    while True:
        for servo in servos:
            values = servo.read_16bit_parameters(0x6064, 1)  # position feedback
            if values:
                print(f"[ID {servo.slave_id}] Pos: {values[0]}")
        time.sleep(0.1)


def main():
    # One shared serial connection
    ser = serial.Serial(
        port='COM5',
        baudrate=38400,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_TWO,
        timeout=0.05
    )

    # Two motors on same RS485 line
    motor1 = LeadshineServo(ser, slave_id=1)
    motor2 = LeadshineServo(ser, slave_id=2)

    servos = [motor1, motor2]

    # Start feedback reader thread
    threading.Thread(target=feedback_loop, args=(servos,), daemon=True).start()

    # Define positions & speeds
    positions_motor1 = [1000, 2000, -1500, 500, 0]
    speeds_motor1 = [200, 250, 300, 200, 150]

    positions_motor2 = [500, 1500, -1000, 2000, 0]
    speeds_motor2 = [150, 180, 220, 200, 150]

    print("Starting motion sequence...")
    for i in range(len(positions_motor1)):
        print(f"\n--- Step {i+1} ---")
        motor1.move_absolute(positions_motor1[i], speeds_motor1[i])
        motor2.move_absolute(positions_motor2[i], speeds_motor2[i])

        # wait until motors reach positions (simplified)
        time.sleep(2)

    print("Sequence finished.")


if __name__ == "__main__":
    main()
