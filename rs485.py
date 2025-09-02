import time

def rs485_task(socketio):

    while True:
        print("RS485 thread is running...")

        # Example: send a message to UI
        socketio.emit('rs485_update', {'status': 'RS485 OK'})

        socketio.sleep(0.1)  # non-blocking sleep
