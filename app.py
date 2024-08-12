import paho.mqtt.client as mqtt
from flask import Flask, render_template
from flask_socketio import SocketIO

app = Flask(__name__)
socketio = SocketIO(app)

# Dictionary untuk menyimpan data dari setiap topik
topic_data = {}

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))

    for i in range(1, 4):
        node = f'/node{i}'
        for sensor_type in ['temperature', 'pressure', 'humidity', 'co', 'ldr']:
            topic = f'{node}/sensor/{sensor_type}'
            client.subscribe(topic)
            topic_data[topic] = []  # Inisialisasi list untuk menyimpan data

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    topic_parts = msg.topic.split('/')
    node_number_str = [s for s in topic_parts if s.startswith('node')][0][4:]

    try:
        node_number = int(node_number_str)
    except ValueError:
        print(f"Invalid node number: {node_number_str}")
        return

    # Handle data sensor
    data = float(msg.payload.decode("utf-8"))
    topic_data[msg.topic].append(data)  # Simpan data ke dalam list
    socketio.emit('mqtt_message', {'topic': msg.topic, 'message': data})
    send_realtime_data()  # Update data when a new message is received
# Fungsi untuk menghitung rata-rata dari semua data
def calculate_overall_average(sensor_type):
    overall_data = []
    for topic, data_list in topic_data.items():
        if topic.endswith(sensor_type) and data_list:
            overall_data.extend(data_list)
    if overall_data:
        overall_average = sum(overall_data) / len(overall_data)
        return overall_average
    return None

# ...

@app.route("/")
def main():
    overall_averages = {}  # Initialize overall_averages
    return render_template('homepage5.html', overall_averages=overall_averages)

# ...

# Fungsi untuk mengirim data secara real-time ke klien web
def send_realtime_data():
    overall_averages = {}
    sensor_types = ['temperature', 'pressure', 'humidity', 'co', 'ldr']
    for sensor_type in sensor_types:
        overall_averages[sensor_type] = calculate_overall_average(sensor_type)
    socketio.emit('update_data', overall_averages)

if __name__ == "__main__":
    mqttc = mqtt.Client()
    mqttc.on_connect = on_connect
    mqttc.on_message = on_message
    mqttc.connect("192.168.5.1", 1883)
    mqttc.loop_start()

    socketio.run(app, host='0.0.0.0', port=5430, debug=True)
