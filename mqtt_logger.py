import paho.mqtt.client as mqtt
import csv
from datetime import datetime
import os

# MQTT configuration
MQTT_BROKER = "e6f813fd27a044869bfce95a6a9b7973.s1.eu.hivemq.cloud"
MQTT_PORT = 8883
MQTT_USERNAME = "WitzealWebApp"
MQTT_PASSWORD = "WzSDH_1234"
MQTT_CLIENT_ID = "PythonMQTTClient"

# Global variable for current CSV file name
current_date = None
current_csv_file = None

# Initialize CSV file with headers if it doesn't exist
def initialize_csv():
    global current_date, current_csv_file
    today = datetime.now().strftime("%d_%m_%Y")
    if current_date != today:
        current_date = today
        current_csv_file = f"WZ_SDH_LOG_{current_date}.csv"
        if not os.path.exists(current_csv_file):
            with open(current_csv_file, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(["Date", "Time", "Topic", "Message", "Client ID"])

# Callback for connection success
def on_connect(client, userdata, flags, rc, props=None):
    if rc == 0:
        print("Connected to MQTT broker successfully.")
        client.subscribe("#")  # Subscribe to all topics
    else:
        print(f"Connection failed with code {rc}.")

# Callback for connection loss
def on_disconnect(client, userdata, rc, properties=None):
    print("Disconnected from MQTT broker. Attempting to reconnect...")
    try:
        client.reconnect()
    except Exception as e:
        print(f"Reconnection failed: {e}")

# Callback for received messages
def on_message(client, userdata, msg):
    initialize_csv()  # Ensure CSV file is up-to-date
    date = datetime.now().strftime("%Y-%m-%d")
    time = datetime.now().strftime("%H:%M:%S")
    topic = msg.topic
    message = msg.payload.decode("utf-8")
    client_id = MQTT_CLIENT_ID

    print(f"Message received: {message} on topic: {topic}")

    # Write message to CSV
    with open(current_csv_file, mode='a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow([date, time, topic, message, client_id])

# Initialize CSV file
initialize_csv()

# Initialize MQTT client
client = mqtt.Client(client_id=MQTT_CLIENT_ID, protocol=mqtt.MQTTv5)
client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
client.tls_set()  # Enable SSL/TLS

# Assign callbacks
client.on_connect = on_connect
client.on_disconnect = on_disconnect
client.on_message = on_message

# Connect to the broker
print("Connecting to MQTT broker...")
client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)

# Start the network loop
try:
    client.loop_forever()
except KeyboardInterrupt:
    print("Disconnected from MQTT broker.")
    client.disconnect()
