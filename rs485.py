import time

def rs485_task(socketio):
    """
    Example RS485 thread.
    Replace the loop with your real RS485 read/write logic.
    """
    while True:
        # TODO: replace this with actual RS485 read/write code
        print("RS485 thread is running...")

        # Example: send a message to UI
        socketio.emit('rs485_update', {'status': 'RS485 OK'})

        socketio.sleep(0.1)  # non-blocking sleep
