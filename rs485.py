import time

def rs485_task(socketio,servo):

    while True:
        # print("RS485 thread is running...")

        socketio.sleep(0.1)  # non-blocking sleep

        try:
            values = servo.read_16bit_parameters(0x0B09, 1, device_id=1)
            servo.axis_01_rpm = values[0]
            print('Axis 01 RPM:', servo.axis_01_rpm)

            time.sleep(0.1)

            values = servo.read_16bit_parameters(0x0B09, 1, device_id=2)
            servo.axis_02_rpm = values[0]
            print('Axis 02 RPM:', servo.axis_02_rpm)

            values = servo.read_16bit_parameters(0x0B1C, 2, device_id=1)
            servo.axis_01_possition = values[0]
            print('Axis 01 Possition:', servo.axis_01_possition)

            time.sleep(0.1)

            values = servo.read_16bit_parameters(0x0B1C, 2, device_id=2)
            servo.axis_02_possition = values[0]
            print('Axis 02 Possition:', servo.axis_02_possition)

            time.sleep(0.1)

            values = servo.read_16bit_parameters(0x0B09, 1, device_id=3)
            servo.axis_03_rpm = values[0]
            print('Axis 03 RPM:', servo.axis_03_rpm)

            time.sleep(0.1)

            values = servo.read_16bit_parameters(0x0B1C, 2, device_id=3)
            servo.axis_03_possition = values[0]
            print('Axis 03 Possition:', servo.axis_03_possition)


        except Exception as e:
            print("something wrong")






