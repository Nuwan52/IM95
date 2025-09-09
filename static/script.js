var socket = io();

// Update counters
socket.on('update_counters', function(data) {
    for (let i = 0; i < data.values.length; i++) {
        document.getElementById("counter" + (i+1)).innerText = data.values[i];
    }
});

// Receive RS485 updates
socket.on('rs485_update', function(data) {
    console.log("RS485:", data.status);
});

// Button click
document.getElementById("f").addEventListener("click", function() {
    socket.emit('button_clicked_01', {data: 'clicked'});
});

document.getElementById("b").addEventListener("click", function() {
    socket.emit('button_clicked_02', {data: 'clicked'});
});

// Show message from Python
socket.on('message', function(data) {
    alert(data.msg);
});
