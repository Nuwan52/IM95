
import serial
import struct
import time
from typing import Union, List, Tuple


class LeadshineServo:
    def __init__(self, port: str = 'COM11', baudrate: int = 38400,
                 slave_id: int = 2, timeout: float = 0.01):
        self.port = port
        self.baudrate = baudrate
        self.slave_id = slave_id
        self.timeout = timeout
        self.serial = None

    def connect(self):
        """Establish serial connection"""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_TWO,  # 2 stop bits as specified
                timeout=self.timeout
            )
            print(f"Connected to {self.port} at {self.baudrate} baud")
            return True
        except Exception as e:
            print(f"Failed to connect: {e}")
            return False

    def disconnect(self):
        """Close serial connection"""
        if self.serial and self.serial.is_open:
            self.serial.close()
            print("Disconnected")

    def calculate_crc(self, data: bytes) -> int:
        """Calculate Modbus RTU CRC16"""
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc

    def build_frame(self, function_code: int, address: int, data: bytes = b'', id=1) -> bytes:
        """Build Modbus RTU frame with CRC"""
        frame = struct.pack('>BBH', id, function_code, address) + data
        crc = self.calculate_crc(frame)
        frame += struct.pack('<H', crc)  # Little endian CRC
        return frame

    def send_receive(self, frame: bytes) -> bytes:
        """Send frame and receive response"""
        if not self.serial or not self.serial.is_open:
            raise Exception("Serial port not connected")

        # Clear input buffer
        self.serial.flushInput()

        # Send frame
        self.serial.write(frame)
        print(f"Sent: {' '.join([f'{b:02X}' for b in frame])}")

        # Wait a bit for response
        time.sleep(0.1)

        # Read response
        response = self.serial.read(100)  # Read up to 100 bytes
        if response:
            print(f"Received: {' '.join([f'{b:02X}' for b in response])}")
            return response
        else:
            raise Exception("No response received")

    def write_16bit_parameter(self, address: int, value: int, id=1) -> bool:
        """
        Write 16-bit parameter (Function code 0x06)

        Args:
            address: Parameter address
            value: 16-bit value to write
        """
        try:
            data = struct.pack('>H', value)  # Big endian 16-bit value
            frame = self.build_frame(0x06, address, data , id)
            response = self.send_receive(frame)

            # Check if response is valid (should echo back the request)
            if len(response) >= 8:
                return True
            return False
        except Exception as e:
            print(f"Error writing 16-bit parameter: {e}")
            return False

    def write_32bit_parameter(self, address: int, value: int , id=1) -> bool:
        """
        Write 32-bit parameter (Function code 0x10)

        Args:
            address: Parameter address
            value: 32-bit value to write
        """
        try:
            # For function 0x10, we need: address, count, byte_count, data
            count = 2  # 2 registers for 32-bit
            byte_count = 4  # 4 bytes
            data = struct.pack('>BH', byte_count, count) + struct.pack('>I', value)
            frame = self.build_frame(0x10, address, data, id)
            response = self.send_receive(frame)

            if len(response) >= 8:
                return True
            return False
        except Exception as e:
            print(f"Error writing 32-bit parameter: {e}")
            return False

    def read_16bit_parameters(self, address: int, count: int = 1, id=1) -> List[int]:
        """
        Read 16-bit parameters (Function code 0x03)

        Args:
            address: Starting address
            count: Number of 16-bit registers to read
        """
        try:
            data = struct.pack('>H', count)
            frame = self.build_frame(0x03, address, data , id)
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

    def is_motion_complete(self, timeout: float = 10.0) -> bool:

        start_time = time.time()
        try:
            while time.time() - start_time < timeout:
                status = self.read_servo_status()
                if status < 0:
                    print("Failed to read servo status")
                    return False
                # Assuming bit or value in 0x0409 indicates motion completion
                # Adjust this condition based on actual datasheet information
                # For now, assuming status 0x0083 (servo enabled, no motion) indicates completion
                if status == 0x0083:
                    print("Motion completed")
                    return True
                time.sleep(0.1)  # Poll every 100ms
            print(f"Timeout waiting for motion completion after {timeout}s")
            return False
        except Exception as e:
            print(f"Error checking motion completion: {e}")
            return False

    def enable_servo(self , id) -> bool:
        """Enable the servo motor (your specific command: 01 06 04 09 00 83)"""
        print("Enabling servo motor...")
        return self.write_16bit_parameter(0x0409, 0x0083,id)

    def disable_servo(self ,id) -> bool:
        """Disable the servo motor"""
        print("Disabling servo motor...")
        return self.write_16bit_parameter(0x0409, 0x0000,id)

    def read_servo_status(self,id) -> int:
        """Read servo status from address 0x0409"""
        values = self.read_16bit_parameters(0x0409, 1,id)
        if values:
            return values[0]
        return -1

    def move_absolute_position(self, position: int, velocity: int = 600,id=1) -> bool:
        """
        Move to absolute position (based on your working 485 communication commands)
        Position in pulse units, velocity in appropriate units
        """
        print(f"Moving to absolute position: {position} at velocity: {velocity}")

        try:
            # Step 1: Set PR0 mode as absolute position (01 06 62 00 00 01 57 B2)
            if not self.write_16bit_parameter(0x6200, 0x0001,id):
                print("Failed to set absolute position mode")
                return False
            time.sleep(0.01)

            # Step 2: Set PR0 position (32-bit write)
            pos_high = (position >> 16) & 0xFFFF
            pos_low = position & 0xFFFF
            if not self.write_16bit_parameter(0x6201, pos_high,id):
                print("Failed to set position high word")
                return False
            time.sleep(0.01)
            if not self.write_16bit_parameter(0x6202, pos_low,id):
                print("Failed to set position low word")
                return False
            time.sleep(0.01)

            # Step 3: Set PR0 velocity (01 06 62 03 02 58 66 E8)
            if not self.write_16bit_parameter(0x6203, velocity,id):
                print("Failed to set velocity")
                return False
            time.sleep(0.01)

            # Step 4: Set PR0 acceleration (01 06 62 04 00 32 56 66)
            if not self.write_16bit_parameter(0x6204, 0x0032,id):
                print("Failed to set acceleration")
                return False
            time.sleep(0.01)

            # Step 5: Set PR0 deceleration (01 06 62 05 00 32 07 A6)
            if not self.write_16bit_parameter(0x6205, 0x0032,id):
                print("Failed to set deceleration")
                return False
            time.sleep(0.01)

            # Step 6: Trigger PR0 motion (01 06 60 02 00 10 37 C6)
            if not self.write_16bit_parameter(0x6002, 0x0010,id):
                print("Failed to trigger motion")
                return False
            time.sleep(0.1)

            # Step 7: Wait for motion to complete
            if not self.is_motion_complete():
                print("Motion did not complete successfully")
                return False

            # Step 8: Send stop/complete command (01 06 60 02 00 40 37 FA)
            if not self.write_16bit_parameter(0x6002, 0x0040,id):
                print("Failed to send stop command")
                return False

            print("✓ Absolute position movement command completed successfully!")
            return True

        except Exception as e:
            print(f"Error in absolute position movement: {e}")
            return False

    def move_relative_position(self, distance: int, velocity: int = 600) -> bool:
        """
        Move relative distance (based on your working commands)
        """
        print(f"Moving relative distance: {distance} at velocity: {velocity}")

        try:
            # Step 1: Set PR0 mode as relative position (01 06 62 00 00 41 56 42)
            if not self.write_16bit_parameter(0x6200, 0x0041):
                print("Failed to set relative position mode")
                return False
            time.sleep(0.01)

            # Step 2: Set relative distance (32-bit)
            dist_high = (distance >> 16) & 0xFFFF
            dist_low = distance & 0xFFFF
            if not self.write_16bit_parameter(0x6201, dist_high):
                return False
            time.sleep(0.01)
            if not self.write_16bit_parameter(0x6202, dist_low):
                return False
            time.sleep(0.01)

            # Step 3: Set velocity (matching your working format)
            if not self.write_16bit_parameter(0x6203, velocity):
                return False
            time.sleep(0.01)

            # Step 4: Set acceleration (00 32 format)
            if not self.write_16bit_parameter(0x6204, 0x0032):
                return False
            time.sleep(0.01)

            # Step 5: Set deceleration (00 32 format)
            if not self.write_16bit_parameter(0x6205, 0x0032):
                return False
            time.sleep(0.01)

            # Step 6: Trigger motion (00 10)
            if not self.write_16bit_parameter(0x6002, 0x0010):
                print("Failed to trigger motion")
                return False
            time.sleep(0.1)

            # Step 7: Wait for motion to complete
            if not self.is_motion_complete():
                print("Motion did not complete successfully")
                return False

            # Step 8: Send stop command (00 40)
            if not self.write_16bit_parameter(0x6002, 0x0040):
                print("Failed to send stop command")
                return False

            print("✓ Relative position movement command completed successfully!")
            return True

        except Exception as e:
            print(f"Error in relative position movement: {e}")
            return False

    def move_velocity_mode(self, velocity: int = 6000) -> bool:
        """
        Set velocity mode (continuous rotation)
        """
        print(f"Setting velocity mode with speed: {velocity} rpm")

        try:
            # Step 1: Set PR0 as velocity mode (01 06 62 00 00 02 17 B3)
            if not self.write_16bit_parameter(0x6200, 0x0002):
                print("Failed to set velocity mode")
                return False
            time.sleep(0.01)

            # Step 2: Set velocity (01 06 62 03 02 58 66 E8)
            if not self.write_16bit_parameter(0x6203, velocity):
                print("Failed to set velocity")
                return False
            time.sleep(0.01)

            # Step 3: Set acceleration
            if not self.write_16bit_parameter(0x6204, 0x3256):
                return False
            time.sleep(0.01)

            # Step 4: Set deceleration
            if not self.write_16bit_parameter(0x6205, 0x3207):
                return False
            time.sleep(0.01)

            # Step 5: Trigger motion
            if not self.write_16bit_parameter(0x6002, 0x0001):
                print("Failed to trigger motion")
                return False

            print("✓ Velocity mode started successfully!")
            return True

        except Exception as e:
            print(f"Error in velocity mode: {e}")
            return False

    def stop_motion(self) -> bool:
        """Stop current motion"""
        print("Stopping motor motion...")
        if self.write_16bit_parameter(0x6002, 0x0040):
            # Wait for motion to fully stop
            if self.is_motion_complete():
                print("✓ Motion stopped successfully!")
                return True
            else:
                print("Motion stop command sent but completion not confirmed")
                return False
        return False


def main():
    # Create servo instance
    servo = LeadshineServo(port='COM8', baudrate=38400, slave_id=1)

    try:

        if not servo.connect():
            return

        print("\n=== Leadshine Servo Control ===")


        # # Enable servo
        # if servo.enable_servo():
        #     print("✓ Servo enabled successfully!")
        # else:
        #     print("✗ Failed to enable servo")
        #     return
        #
        # # Read servo status
        # status = servo.read_servo_status()
        # if status >= 0:
        #     print(f"Servo status: 0x{status:04X}")
        #
        # # Example 1: Move to absolute position 200000 (600 velocity)
        # print("\n--- Testing Absolute Position Movement (200000) ---")
        # if servo.move_absolute_position(200000, 600):
        #     print("✓ First movement completed")
        # else:
        #     print("✗ First movement failed")
        #     servo.disable_servo()
        #     return
        #
        # # Example 2: Move to absolute position 100000 (1000 velocity)
        # print("\n--- Testing Absolute Position Movement (100000) ---")
        # if servo.move_absolute_position(100000, 1000):
        #     print("✓ Second movement completed")
        # else:
        #     print("✗ Second movement failed")
        #     servo.disable_servo()
        #     return
        #
        # # Example 3: Move to absolute position 500000 (3000 velocity)
        # print("\n--- Testing Absolute Position Movement (500000) ---")
        # if servo.move_absolute_position(500000, 3000):
        #     print("✓ Third movement completed")
        # else:
        #     print("✗ Third movement failed")
        #     servo.disable_servo()
        #     return
        #
        # # Stop motion
        # servo.stop_motion()

        # Read some parameters

        while True:
            print("\nReading parameters... 1")
            time.sleep(1)
            values = servo.read_16bit_parameters(0x0B17, 2 ,1)  # Read 3 registers starting from 0x0409
            for i, value in enumerate(values):
                print(f"Register 0x{0x0409 + i:04X}: {value} (0x{value:04X})")

            print("\nReading parameters... 2")
            time.sleep(1)
            values = servo.read_16bit_parameters(0x0B17, 2, 2)  # Read 3 registers starting from 0x0409
            for i, value in enumerate(values):
                print(f"Register 0x{0x0409 + i:04X}: {value} (0x{value:04X})")

    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        servo.disconnect()


if __name__ == "__main__":
    main()