import serial
import time

def calc_crc(data: bytes) -> bytes:
    """Calculate Modbus RTU CRC16 (returns 2 bytes: low, high)."""
    crc = 0xFFFF
    for pos in data:
        crc ^= pos
        for _ in range(8):
            if (crc & 0x0001) != 0:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return crc.to_bytes(2, byteorder="little")  # LSB first

def send_modbus_commands(port="COM5", baudrate=38400):
    ser = serial.Serial(
        port=port,
        baudrate=baudrate,
        bytesize=8,
        parity="N",
        stopbits=1,
        timeout=0.1
    )

    # Commands without slave ID and without CRC (start from function code 06)
    base_commands = [
        "06 62 00 00 41",   # Set PR0 mode as absolute position
        "06 62 01 00 03",   # Set PR0 position high bit
        "06 62 02 0D 40",   # Set PR0 position low bit
        "06 62 03 02 58",   # Set PR0 velocity
        "06 62 04 00 32",   # Set PR0 acceleration
        "06 62 05 00 32",   # Set PR0 deceleration
        "06 60 02 00 10",   # Trigger PR0 motion
        "06 60 02 00 40"    # Another trigger
    ]

    # Loop over slave IDs 1, 2, 3
    for slave_id in range(1, 4):
        print(f"\n=== Sending commands to slave ID {slave_id:02X} ===")

        for cmd in base_commands:
            # Prepend slave ID
            data = bytes([slave_id]) + bytes.fromhex(cmd)
            # Append CRC
            frame = data + calc_crc(data)

            print(f"Sending: {frame.hex(' ').upper()}")
            ser.write(frame)

            # Read response (if any)
            response = ser.read(256)
            if response:
                print("Received:", response.hex(" ").upper())
            else:
                print("No response")

            time.sleep(0.1)  # delay between commands

    ser.close()

if __name__ == "__main__":
    send_modbus_commands()
