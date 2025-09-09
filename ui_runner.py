from flask import Flask, render_template
from flask_socketio import SocketIO
from rs485 import rs485_task
from servo_runner import LeadshineServo
import time
app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret!'
socketio = SocketIO(app, async_mode="threading")

# -------------------
# Counter thread
# -------------------
counters = [0, 0, 0, 0, 0, 0]
increments = [1, 2, 3, 5, 10, 20]

def counter_thread():
    while True:
        for i in range(len(counters)):
            counters[i] += increments[i]
        socketio.emit('update_counters', {'values': counters})
        socketio.sleep(0.01)

# -------------------
# Flask routes
# -------------------
@app.route('/')
def index():
    return render_template('index.html')

@socketio.on('button_clicked_01')
def handle_button_click(data):
    servo.move_absolute_position(1000000, 4000, 1)
    servo.Tigger_motions(1)

    servo.move_absolute_position(1000000, 4000, 2)
    servo.Tigger_motions(2)

    servo.move_absolute_position(1000000, 4000, 3)
    servo.Tigger_motions(3)
    print("Button clicked from webpage!")

@socketio.on('button_clicked_02')
def handle_button_click(data):
    servo.move_absolute_position(0, 4000, 1)
    servo.Tigger_motions(1)

    servo.move_absolute_position(0, 4000, 2)
    servo.Tigger_motions(2)

    servo.move_absolute_position(0, 4000, 3)
    servo.Tigger_motions(3)
    print("Button clicked from webpage!")


# -------------------
# Start app
# -------------------
if __name__ == '__main__':
    # start background threads

    try:
        servo = LeadshineServo(port='COM5', baudrate=38400)
        servo.connect()
        print('this is the twst')
        servo.enable_servo(device_id=1)
        servo.enable_servo(device_id=2)
        servo.enable_servo(device_id=3)

        # socketio.start_background_task(counter_thread)
        socketio.start_background_task(rs485_task, socketio,servo)

        socketio.run(app, debug=True, use_reloader=False, allow_unsafe_werkzeug=True)

    except KeyboardInterrupt:
        servo.disable_servo(device_id=1)
        servo.disable_servo(device_id=2)
        servo.disable_servo(device_id=3)
        servo.disconnect()
        print('connection Disconnected')
    finally:
        servo.disable_servo(device_id=1)
        servo.disable_servo(device_id=2)
        servo.disable_servo(device_id=3)
        servo.disconnect()
        print("Cleanup done. Exiting.")


