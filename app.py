from flask import Flask, render_template, jsonify
import paho.mqtt.client as mqtt

app = Flask(__name__)

# MQTT setup
mqttBroker = "mikedaiot.cloud.shiftr.io"
mqttUser = "mikedaiot"  # Optional
mqttPassword = "MHL9Od957yL7o9tP"  # Optional

sensor_data = {
    "humidity": 0,
    "temperature": 0,
    "light_level": 0
}

def on_connect(client, userdata, flags, rc):
    client.subscribe("home/sensors")

def on_message(client, userdata, msg):
    data = msg.payload.decode().split(',')
    sensor_data["humidity"] = float(data[0])
    sensor_data["temperature"] = float(data[1])
    sensor_data["light_level"] = int(data[2])

mqttClient = mqtt.Client()
mqttClient.username_pw_set(mqttUser, mqttPassword)  # Optional
mqttClient.on_connect = on_connect
mqttClient.on_message = on_message
mqttClient.connect(mqttBroker)
mqttClient.loop_start()

@app.route('/')
def index():
    return render_template('index.html', f_thanh_data=sensor_data)

@app.route('/sensor_data')
def get_sensor_data():
    return jsonify(sensor_data)

@app.route('/control/<device>/<action>')
def control_device(device, action):
    topic = "home/control"
    mqttClient.publish(topic, f"{device}_{action}")
    return jsonify({"status": "success"})

if __name__ == '__main__':
    app.run(debug=True)
