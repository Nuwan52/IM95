from flask import Flask, render_template
from flask_socketio import SocketIO
from rs485 import rs485_task

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

@socketio.on('button_clicked')
def handle_button_click(data):
    print("Button clicked from webpage!")
    socketio.emit('message', {'msg': 'Python function executed!'})

# -------------------
# Start app
# -------------------
if __name__ == '__main__':
    # start background threads
    socketio.start_background_task(counter_thread)
    socketio.start_background_task(rs485_task, socketio)

    socketio.run(app, debug=True, allow_unsafe_werkzeug=True)
