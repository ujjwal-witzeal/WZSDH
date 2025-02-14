import paho.mqtt.client as mqtt
import time
import threading

# MQTT Configuration
MQTT_URL = "e6f813fd27a044869bfce95a6a9b7973.s1.eu.hivemq.cloud"
MQTT_PORT = 8883
MQTT_USERNAME = "WitzealWebApp"
MQTT_PASSWORD = "WzSDH_1234"
TOPIC_SETUP = "setup"
TOPIC_HOME = "home"

# Constants
ATTENDANCE_MESSAGE = "ALL-ATTENDENCE"
ONLINE_STATUS = "-ST_ON00"
HOME_ACK = "-SC_home"
ACK_MESSAGE = "-S_ACK"

# Client-specific configuration
CLIENT_ID = "WZ_WEB_APP"
STATUS_MESSAGE = f"{CLIENT_ID}{ONLINE_STATUS}"
RELAY_ON = "WZ_4CR_E868E7DFF589-R401"
RELAY_OFF = "WZ_4CR_E868E7DFF589-R400"

# Store active users and ACK states
active_users = {}
ack_threads = {}  # Threads for each user's ACK resend logic


# MQTT Callbacks
def on_connect(client, userdata, flags, rc, properties):
    if rc == 0:
        print("Connection Successful")
        client.subscribe(TOPIC_SETUP)
        client.subscribe(TOPIC_HOME)
        client.publish(TOPIC_SETUP, ATTENDANCE_MESSAGE)
    else:
        print(f"Connection failed with code {rc}")


def on_message(client, userdata, msg):
    topic = msg.topic
    payload = msg.payload.decode()

    if topic == TOPIC_SETUP:
        handle_setup_message(client, payload)
    elif topic == TOPIC_HOME:
        handle_home_message(client, payload)


def handle_setup_message(client, payload):
    global active_users, ack_threads

    if ATTENDANCE_MESSAGE in payload:
        # Respond with status
        client.publish(TOPIC_SETUP, STATUS_MESSAGE)

    elif ONLINE_STATUS in payload:
        # Extract user ID from the payload
        user_id = payload.split(ONLINE_STATUS)[0]

        # Skip sending `-SC_home` to itself
        if user_id == CLIENT_ID:
            return

        # Register active user and send HOME_ACK
        if user_id not in active_users:
            active_users[user_id] = time.time()
            ack_threads[user_id] = threading.Event()  # Event to stop resend loop

            home_ack_message = f"{user_id}{HOME_ACK}"
            client.publish(TOPIC_SETUP, home_ack_message)
            start_ack_resend(client, user_id, home_ack_message)

    elif ACK_MESSAGE in payload:
        # Stop resending -SC_home when acknowledgment is received
        user_id = payload.split(ACK_MESSAGE)[0]
        if user_id in ack_threads:
            ack_threads[user_id].set()  # Stop the resend loop


def handle_home_message(client, payload):
    if "-S101" in payload:  # Motion detected
        client.publish(TOPIC_HOME, RELAY_ON)
    elif "-S100" in payload:  # No motion detected
        client.publish(TOPIC_HOME, RELAY_OFF)


# Helper Functions
def start_ack_resend(client, user_id, home_ack_message):
    """Start a thread to resend the -SC_home message until acknowledgment is received."""
    def resend():
        while not ack_threads[user_id].is_set():
            client.publish(TOPIC_SETUP, home_ack_message)
            time.sleep(15)  # Resend every 15 seconds
        # Clean up once acknowledgment is received
        ack_threads.pop(user_id, None)

    thread = threading.Thread(target=resend, daemon=True)
    thread.start()


# MQTT Client Setup
client = mqtt.Client(client_id=CLIENT_ID, protocol=mqtt.MQTTv5)  # Use MQTT version 5
client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
client.tls_set()  # Enable secure connection

# Attach callbacks
client.on_connect = on_connect
client.on_message = on_message

try:
    # Connect to broker
    client.connect(MQTT_URL, MQTT_PORT, 60)
    # Start the client loop
    client.loop_forever()
except KeyboardInterrupt:
    print("Disconnecting from MQTT Broker...")
    client.disconnect()
except Exception as e:
    print(f"Error: {e}")
