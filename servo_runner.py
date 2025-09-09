from logging import exception
import serial
import struct
import time
import threading
from typing import List


class LeadshineServo:
    axis_01_possition = 0
    axis_02_possition = 0
    axis_01_rpm = 0
    axis_02_rpm = 0
    axis_03_possition = 0
    axis_03_rpm = 0


    def __init__(self, port: str = 'COM5', baudrate: int = 38400, timeout: float = 0.00001):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial = None
        self.lock = threading.Lock()   # 🔒 Lock for thread safety

    def connect(self):
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_TWO,
                timeout=self.timeout
            )
            print(f"Connected to {self.port} at {self.baudrate} baud")
            return True
        except Exception as e:
            print(f"Failed to connect: {e}")
            return False

    def disconnect(self):
        with self.lock:   # lock even for closing
            if self.serial and self.serial.is_open:
                self.serial.close()
                print("Disconnected")

    def calculate_crc(self, data: bytes):
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc

    def build_frame(self, function_code: int, address: int, data: bytes = b'', device_id=1):
        frame = struct.pack('>BBH', device_id, function_code, address) + data
        crc = self.calculate_crc(frame)
        frame += struct.pack('<H', crc)  # Little endian CRC
        return frame

    def write_16bit_parameter(self, address: int, value: int, device_id=1):
        try:
            data = struct.pack('>H', value)
            frame = self.build_frame(0x06, address, data, device_id)
            response = self.send_receive(frame)

            if len(response) >= 8:
                return True
            return False
        except Exception as e:
            print(f"Error writing 16-bit parameter: {e}")
            return False

    def send_receive(self, frame: bytes) -> bytes:
        if not self.serial or not self.serial.is_open:
            raise Exception("Serial port not connected")

        with self.lock:   # 🔒 Protect all serial I/O
            try:
                self.serial.reset_input_buffer()
                # print(frame)
                self.serial.write(frame)
            except exception:
                print('something wrong with write')

            time.sleep(0.01)

            try:
                response = self.serial.read(100)
            except exception:
                print('something wrong with read')
                response = b''

        if response:
            # print(f"Received: {' '.join([f'{b:02X}' for b in response])}")
            return response
        else:
            raise Exception("No response received")

    def read_16bit_parameters(self, address: int, count: int = 1, device_id=1) -> List[int]:
        try:
            data = struct.pack('>H', count)
            frame = self.build_frame(0x03, address, data, device_id)
            response = self.send_receive(frame)

            if len(response) >= 5:
                byte_count = response[2]
                values = []
                for i in range(0, byte_count, 2):
                    value = struct.unpack('>H', response[3 + i:5 + i])[0]
                    values.append(value)
                return values
            return []
        except Exception as e:
            print(f"Error reading 16-bit parameters: {e}")
            return []

    def enable_servo(self, device_id) -> bool:
        print("Enabling servo motor...")
        return self.write_16bit_parameter(0x0409, 0x0083, device_id)

    def disable_servo(self, device_id) -> bool:
        print("Disabling servo motor...")
        return self.write_16bit_parameter(0x0409, 0x0000, device_id)

    def move_absolute_position(self, position: int, velocity: int = 600, device_id=1) -> bool:
        print(f"Moving to absolute position: {position} at velocity: {velocity}")
        try:
            # if not self.write_16bit_parameter(0x6200, 0x0001, device_id):
            #     print("Failed to set absolute position mode")
            #     return False
            # time.sleep(0.01)

            pos_high = (position >> 16) & 0xFFFF
            pos_low = position & 0xFFFF
            if not self.write_16bit_parameter(0x6201, pos_high, device_id):
                print("Failed to set position high word")
                return False


            if not self.write_16bit_parameter(0x6202, pos_low, device_id):
                print("Failed to set position low word")
                return False

            # if not self.write_16bit_parameter(0x6203, velocity, device_id):
            #     print("Failed to set velocity")
            #     return False
            # time.sleep(0.01)
            #
            # if not self.write_16bit_parameter(0x6204, 0x0032, device_id):
            #     print("Failed to set acceleration")
            #     return False
            # time.sleep(0.01)
            #
            # if not self.write_16bit_parameter(0x6205, 0x0032, device_id):
            #     print("Failed to set deceleration")
            #     return False
            # time.sleep(0.01)

            return True
        except Exception as e:
            print(f"Error in absolute position movement: {e}")
            return False

    def Tigger_motions(self, device_id=1):
        if not self.write_16bit_parameter(0x6002, 0x0010, device_id):
            print("Failed to trigger motion")
            return False
        time.sleep(0.1)
        return True


# try:
#     servo = LeadshineServo(port='COM5', baudrate=38400)
#     servo.connect()
#     servo.enable_servo(device_id=1)
#     servo.enable_servo(device_id=2)
#     time.sleep(0.1)
#
#     servo.move_absolute_position(0,4000,1)
#     servo.Tigger_motions(1)
#     servo.move_absolute_position(0, 4000, 2)
#     servo.Tigger_motions(2)
#
#
#     time.sleep(0.1)
#
#
#
#     while True:
#
#         values = servo.read_16bit_parameters(0x0B09, 1,device_id=1)
#         servo.axis_01_rpm = values[0]
#         print('Axis 01 RPM:', servo.axis_01_rpm)
#
#         time.sleep(0.1)
#
#         values = servo.read_16bit_parameters(0x0B09, 1, device_id=2)
#         servo.axis_02_rpm = values[0]
#         print('Axis 02 RPM:', servo.axis_02_rpm)
#
#         values = servo.read_16bit_parameters(0x0B1C, 2, device_id=1)
#         servo.axis_01_possition = values[0]
#         print('Axis 01 Possition:', servo.axis_01_possition)
#
#         time.sleep(0.1)
#
#         values = servo.read_16bit_parameters(0x0B1C, 2, device_id=2)
#         servo.axis_02_possition = values[0]
#         print('Axis 02 Possition:', servo.axis_02_possition)
#
#
#
# except KeyboardInterrupt:
#     servo.disable_servo(device_id=1)
#     servo.disable_servo(device_id=2)
#     servo.disconnect()
#     print('connection Disconnected')
# finally:
#     servo.disconnect()
#     print("Cleanup done. Exiting.")
