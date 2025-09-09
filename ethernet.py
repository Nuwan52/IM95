import serial
import struct
import time
import threading
import queue


class LeadshineServo:
    def __init__(self, port='COM5', baudrate=38400, timeout=0.2):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial = None
        self.running = False
        self.thread = None
        self.cmd_queue = queue.Queue()

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
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join()
        if self.serial and self.serial.is_open:
            self.serial.close()
            print("Disconnected")

    # ========== Low-level helpers ==========
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
        frame = struct.pack('>B B H', device_id, function_code, address) + data
        crc = self.calculate_crc(frame)
        frame += struct.pack('<H', crc)
        return frame

    def send_receive(self, frame: bytes, expected_len: int) -> bytes:
        self.serial.reset_input_buffer()
        self.serial.reset_output_buffer()
        self.serial.write(frame)
        print(f"Sent: {' '.join(f'{b:02X}' for b in frame)}")

        response = b''
        start = time.time()
        while len(response) < expected_len and (time.time() - start) < self.timeout:
            chunk = self.serial.read(expected_len - len(response))
            if chunk:
                response += chunk

        if len(response) < expected_len:
            raise Exception(f"Incomplete response (got {len(response)}/{expected_len} bytes)")

        print(f"Received: {' '.join(f'{b:02X}' for b in response)}")
        return response

    def write_16bit_parameter(self, address: int, value: int, device_id=1):
        try:
            data = struct.pack('>H', value)
            frame = self.build_frame(0x06, address, data, device_id)
            expected_len = 8
            self.send_receive(frame, expected_len)
            return True
        except Exception as e:
            print(f"Write error dev{device_id} @0x{address:04X}: {e}")
            return False

    def read_16bit_parameter(self, address: int, count=1, device_id=1):
        try:
            data = struct.pack('>H', count)
            frame = self.build_frame(0x03, address, data, device_id)
            expected_len = 5 + (count * 2)
            response = self.send_receive(frame, expected_len)
            byte_count = response[2]
            values = []
            for i in range(0, byte_count, 2):
                values.append(struct.unpack('>H', response[3 + i:5 + i])[0])
            return values
        except Exception as e:
            print(f"Read error dev{device_id} @0x{address:04X}: {e}")
            return []

    # ========== Servo control ==========
    def enable_servo(self, device_id):
        print(f"Enabling servo {device_id}")
        return self.write_16bit_parameter(0x0409, 0x0083, device_id)

    def disable_servo(self, device_id):
        print(f"Disabling servo {device_id}")
        # return self.write_16bit_parameter(0x0409, 0x0000, device_id)

    def queue_move_absolute(self, device_id: int, position: int, velocity: int = 600):
        """Queue a move (enable → write PR0 params → trigger → disable)"""
        self.cmd_queue.put(("move", device_id, position, velocity))

    def _do_move_absolute(self, device_id: int, position: int, velocity: int):
        print(f"Servo {device_id} -> Move {position} at {velocity}")

        # Enable first
        if not self.enable_servo(device_id):
            return

        try:
            # Step 1: set absolute mode
            time.sleep(0.05)
            if not self.write_16bit_parameter(0x6200, 0x0001, device_id):
                print(f"Warning: failed to set absolute mode on servo {device_id}")
            time.sleep(0.02)

            # Step 2: set position (32-bit split)
            pos_high = (position >> 16) & 0xFFFF
            pos_low = position & 0xFFFF
            if not self.write_16bit_parameter(0x6201, pos_high, device_id):
                print(f"Warning: failed to set pos_high on servo {device_id}")
            time.sleep(0.02)
            if not self.write_16bit_parameter(0x6202, pos_low, device_id):
                print(f"Warning: failed to set pos_low on servo {device_id}")
            time.sleep(0.02)

            # Step 3: velocity
            if not self.write_16bit_parameter(0x6203, velocity, device_id):
                print(f"Warning: failed to set velocity on servo {device_id}")
            time.sleep(0.02)

            # Step 4: acceleration
            self.write_16bit_parameter(0x6204, 0x0032, device_id)
            time.sleep(0.02)

            # Step 5: deceleration
            self.write_16bit_parameter(0x6205, 0x0032, device_id)
            time.sleep(0.02)

            # Step 6: trigger motion
            self.write_16bit_parameter(0x6002, 0x0010, device_id)
            time.sleep(0.1)  # allow motion to start

        finally:
            # Always disable at the end
            self.disable_servo(device_id)

    # ========== Worker thread ==========
    def _worker(self):
        while self.running:
            # 1. Process queued commands
            try:
                cmd = self.cmd_queue.get_nowait()
                if cmd[0] == "move":
                    _, device_id, pos, vel = cmd
                    self._do_move_absolute(device_id, pos, vel)
            except queue.Empty:
                pass

            # 2. Poll positions
            for dev in [1, 2]:  # extend to 6 later
                vals = self.read_16bit_parameter(0x0B09, 1, device_id=dev)
                if vals:
                    print(f"Servo {dev} position: {vals[0]}")
                time.sleep(0.1)

            time.sleep(0.2)

    def start(self):
        if not self.serial or not self.serial.is_open:
            raise Exception("Connect before starting worker")
        self.running = True
        self.thread = threading.Thread(target=self._worker, daemon=True)
        self.thread.start()



if __name__ == "__main__":
    servo = LeadshineServo('COM5', 38400)
    if servo.connect():
        servo.start()

        try:
            # Queue two moves (thread handles them safely)
            servo.queue_move_absolute(device_id=1, position=20000, velocity=1000)
            servo.queue_move_absolute(device_id=2, position=-15000, velocity=1200)

            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("Stopping...")
        finally:
            servo.disconnect()
