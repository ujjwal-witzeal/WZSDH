#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <EEPROM.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#define MOTION_PIN D7
#define STATUS_PIN D6
#define RESET_PIN D5  // Reset button pin

#define EEPROM_SIZE 512
#define SSID_ADDR 30
#define PASSWORD_ADDR (SSID_ADDR + 32)
#define MQTT_USERNAME_ADDR (PASSWORD_ADDR + 32)
#define MQTT_PASSWORD_ADDR (MQTT_USERNAME_ADDR + 32)
#define MQTT_SERVER_ADDR (MQTT_PASSWORD_ADDR + 32)
#define TOPIC_CONFIG_ADDR (MQTT_SERVER_ADDR + 64)  // Address for storing configTopic in EEPROM
#define TIMEOUT_ADDR (TOPIC_CONFIG_ADDR + 4)
#define RESET_HOLD_TIME 5000  // 5 seconds hold time for reset

static unsigned long resetButtonTimer = 0;
static bool resetButtonPressed = false;

char apSSID[32];
char apPassword[32] = "12345678";  // Default AP password
char mqttUsername[32];
char mqttPassword[32];
char mqttServer[64];

WiFiClientSecure espClient;
PubSubClient client(espClient);
ESP8266WebServer server(80);

int wifi_counter = 0;
bool configCompleted = false;
bool topicConfigCompleted = false;
String MacAddress = "";
String Topic = "";
String configTopic = "setup"; // Default topic if first boot
String Room = "";
int motionTimeout = 60000;   // 15 seconds timeout for motion detection
unsigned long lastMotionTimestamp = 0;
bool timeoutElapsed = false; // Tracks if the timeout period has elapsed
String lastMessage = "-S101";     // Tracks the last message sent
char softApSSID[32];
void handleConfigRequest();
bool connectToWiFiWithRetry(const char* ssid, const char* password);
void saveCredentialsToEEPROM(const char* ssid, const char* password, const char* cluster);
void loadCredentialsFromEEPROM();
void connectToMQTT();
void checkResetButton();

void setup() {
    Serial.begin(115200);
    EEPROM.begin(EEPROM_SIZE);

    pinMode(RESET_PIN, INPUT_PULLUP);
    pinMode(MOTION_PIN, INPUT);
    pinMode(STATUS_PIN, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(RESET_PIN), checkResetButton, FALLING);
    // Load config from EEPROM
    configCompleted = EEPROM.read(0) == 1;
    topicConfigCompleted = EEPROM.read(1) == 1;  // Flag for topic configuration
    blink_led(100,10);
    MacAddress = WiFi.macAddress();
    MacAddress.replace(":", "");
    sprintf(softApSSID, "WZ_MDR_%s", MacAddress.c_str());
    processMacAddress(MacAddress, mqttPassword, sizeof(mqttPassword));
    if (!configCompleted) 
    {
        blink_led(50,10);
        WiFi.softAP(softApSSID, apPassword);
        server.on("/config", handleConfigRequest);
        server.begin();
        Serial.println();
        Serial.println("Access point started: " + String(softApSSID));
        Serial.println("Password for MQTT: "+ String(mqttPassword));
        motionTimeout = 60000;

    } 
    else 
    {
        loadCredentialsFromEEPROM();
        // Load the topic from EEPROM if not the first boot
        if (topicConfigCompleted) 
        {
            
            EEPROM.get(TIMEOUT_ADDR, motionTimeout);
            motionTimeout = motionTimeout/1000;
            EEPROM.get(TOPIC_CONFIG_ADDR, configTopic);
            Serial.println("Motion Timeout: " + String(motionTimeout));
        }

        Serial.print("Loaded configTopic: ");
        Serial.println(configTopic);
        Room = configTopic;

        if (connectToWiFiWithRetry(apSSID, apPassword)) 
        {
            connectToMQTT();
        } 
        else 
        {
            Serial.println("Wi-Fi connection failed, entering AP mode for reconfiguration...");
            WiFi.softAP(softApSSID, apPassword);
            server.begin();
        }
    }
}

void loop() {
    sendKeepAlive();
    checkResetButton();
    if (!configCompleted) 
    {
        server.handleClient();
    } 
    else 
    {
      if (WiFi.status() != WL_CONNECTED) 
      {
        connectToWiFiWithRetry(apSSID, apPassword);
      }
      if (!client.connected()) 
      {
          connectToMQTT();
      }
      client.loop();
    }
    checkMotionTimeout();
    yield();
    checkResetButton();
    digitalWrite(STATUS_PIN, digitalRead(MOTION_PIN));
    ESP.wdtFeed();
}

void blink_led(int del, int times)
{
  for(int i = 0; i < times; i++)
  {
    digitalWrite(STATUS_PIN, HIGH);
    delay(del);
    digitalWrite(STATUS_PIN, LOW);
    delay(del);
  }
}

void handleConfigRequest() 
{
    if (server.hasArg("SSID") && server.hasArg("Password") && server.hasArg("Cluster")) {
        String ssid = server.arg("SSID");
        String password = server.arg("Password");
        String cluster = server.arg("Cluster");
        Serial.println("Received configuration:");
        Serial.println("SSID: " + ssid);
        Serial.println("Password: " + password);
        Serial.println("Cluster: " + cluster);

        // Try to connect to the provided network
        if (connectToWiFiWithRetry(ssid.c_str(), password.c_str())) {
            server.send(200, "text/plain", "Connected successfully");
            eraseEeprom();
            WiFi.softAPdisconnect(true);
            saveCredentialsToEEPROM(ssid.c_str(), password.c_str(), cluster.c_str());
            configCompleted = true;
            EEPROM.write(0, 1);  // Mark config as completed
            EEPROM.commit();
            ESP.restart();
        } else {
            server.send(500, "text/plain", "Failed to connect to Wi-Fi. Please try again.");
            Serial.println("Failed to connect to Wi-Fi.");
            eraseEeprom();
        }
    } else {
        server.send(400, "text/plain", "Invalid Request");
        Serial.println("Invalid configuration request received.");
    }
}
bool connectToWiFiWithRetry(const char* ssid, const char* password) 
{
    int attempt = 0;
    unsigned long softAPStartTime = 0;
    while (true)  // Infinite retry loop
    {
        // Step 1: Try connecting to Wi-Fi for up to 20 attempts
        WiFi.begin(ssid, password);
        Serial.println("Attempting to connect to Wi-Fi...");

        while (WiFi.status() != WL_CONNECTED && attempt < 20) 
        {
            blink_led(200, 5); // Indicate ongoing connection attempts
            Serial.println("Connection attempt " + String(attempt + 1) + "...");
            attempt++;
            delay(500); // Wait between attempts
        }

        // Check if connection was successful
        if (WiFi.status() == WL_CONNECTED)
        {
            Serial.println("Successfully connected to Wi-Fi!");
            Serial.println("IP Address: " + WiFi.localIP().toString());
            blink_led(500, 2); // Indicate success
            return true;
        }

        // Step 2: Switch to SoftAP mode if connection fails
        Serial.println("Failed to connect to Wi-Fi after 20 attempts.");
        Serial.println("Switching to SoftAP mode for 5 minutes...");
        WiFi.softAP(softApSSID, "12345678");  // Start SoftAP mode
        server.on("/config", handleConfigRequest);  // Handle configuration requests
        server.begin();
        softAPStartTime = millis();  // Record the time when SoftAP started

        // Step 3: Run SoftAP for 5 minutes and listen for user configuration
        while (millis() - softAPStartTime < 300000)  // 5 minutes timeout
        {
            blink_led(50,10);
            if (configCompleted) 
            {
              server.handleClient();
            } 
            else 
            {
              break;
            }
        }

        // Step 4: Turn off SoftAP mode and retry connection if no data received
        WiFi.softAPdisconnect(true);  // Disconnect SoftAP
        Serial.println("SoftAP timed out or no data received. Retrying Wi-Fi connection...");
        attempt = 0;  // Reset connection attempt counter
    }
    return false;  // This line should never be reached
}
void eraseEeprom()
{
      Serial.println("Erasing previous config");
        for (int i = 0; i < EEPROM_SIZE; i++) 
        {
          EEPROM.write(i, 0);
        }
        EEPROM.commit();
        delay(1000);
        //ESP.restart();
}
void saveCredentialsToEEPROM(const char* ssid, const char* password, const char* cluster) {
    strncpy(apSSID, ssid, sizeof(apSSID) - 1);
    EEPROM.put(SSID_ADDR, apSSID);

    strncpy(apPassword, password, sizeof(apPassword) - 1);
    EEPROM.put(PASSWORD_ADDR, apPassword);

    strncpy(mqttServer, cluster, sizeof(mqttServer) - 1);
    EEPROM.put(MQTT_SERVER_ADDR, mqttServer);

    sprintf(mqttUsername, "WZ_MDR_%s", MacAddress.c_str());
    EEPROM.put(MQTT_USERNAME_ADDR, mqttUsername);
    EEPROM.put(MQTT_PASSWORD_ADDR, mqttPassword);

    EEPROM.commit();
    Serial.println("Credentials saved to EEPROM.");
    blink_led(50, 20);
}

void processMacAddress(const String& MacAddress, char* mqttPassword, size_t maxLength) {
    bool firstAlphabetFound = false; // Flag to track if the first alphabet is found
    size_t index = 0; // To track position in mqttPassword

    // Convert the entire string to uppercase first
    String uppercaseMac = MacAddress;
    uppercaseMac.toUpperCase();

    // Iterate through the MacAddress string
    for (size_t i = 0; i < uppercaseMac.length() && index < maxLength - 1; i++) {
        char currentChar = uppercaseMac[i];

        if (!firstAlphabetFound && isalpha(currentChar)) {
            // If this is the first alphabet, convert it to lowercase
            mqttPassword[index++] = (char)tolower(currentChar);
            firstAlphabetFound = true; // Mark that we've found the first alphabet
        } else {
            // Append the character as is (already in uppercase)
            mqttPassword[index++] = currentChar;
        }
    }

    // Null-terminate the resulting char array
    mqttPassword[index] = '\0';
}

void loadCredentialsFromEEPROM() {
    EEPROM.get(SSID_ADDR, apSSID);
    EEPROM.get(PASSWORD_ADDR, apPassword);
    EEPROM.get(MQTT_SERVER_ADDR, mqttServer);
    EEPROM.get(MQTT_USERNAME_ADDR, mqttUsername);
    EEPROM.get(MQTT_PASSWORD_ADDR, mqttPassword);
    EEPROM.get(TIMEOUT_ADDR, motionTimeout);
    Serial.println("Loaded credentials from EEPROM:");
    Serial.println("SSID: " + String(apSSID));
    Serial.println("Password: " + String(apPassword));
    Serial.println("MQTT Server: " + String(mqttServer));
    Serial.println("MQTT Username: " + String(mqttUsername));
    Serial.println("MQTT Password: " + String(mqttPassword));
    blink_led(1000, 2);
}

void callback(char* topic, byte* payload, unsigned int length) {
    Serial.println(topic);  
    Topic = topic;
    // Parse messages only if it's from the configured topic
    if (Topic == "setup") 
    {
        String msg = "";
        for (int i = 0; i < length; i++) {
            msg += (char)payload[i];
        }
        if(msg == "ALL-ATTENDENCE")
        {
          char msg[64];
          sprintf(msg, "%s-ST_ON00", mqttUsername);
          Serial.println(msg);
          client.publish("setup", msg);
        }
        if (msg.indexOf("WZ") != -1 && msg.indexOf("MDR") != -1 && msg.indexOf(MacAddress) != -1) {
            int hyphenIndex = msg.indexOf('-');
            if (hyphenIndex != -1) {
                String receivedTopic = msg.substring(hyphenIndex + 1);

                if (receivedTopic.startsWith("SC_")) {
                    String newConfigTopic = receivedTopic.substring(3);  // Get part after "SC_"
                    if (newConfigTopic != Room) { // Only update if the topic is different
                        Room = newConfigTopic;  // Update room based on the new topic
                        configTopic = Room;

                        // Save the new topic to EEPROM
                        EEPROM.put(TOPIC_CONFIG_ADDR, configTopic);
                        EEPROM.write(1, 1); // Set the topicConfigCompleted flag
                        EEPROM.commit();

                        topicConfigCompleted = true;
                        Serial.println("New topic received and saved to EEPROM: " + configTopic);

                        // Acknowledge receipt of the new configuration only once
                        String RoomAck = String(mqttUsername) + "-S_ACK";
                        client.publish("setup", RoomAck.c_str()); // Publish acknowledgment

                        // Unsubscribe from the old room topic if it exists
                        if (!Room.isEmpty()) {
                            client.unsubscribe(Room.c_str());
                        }

                        // Subscribe to the new room topic
                        client.subscribe(Room.c_str());
                    }
                }
            }
        }
    } 
    else if (String(topic) == Room && topicConfigCompleted) 
    {
        // Only parse messages in the configured room
        String msg = "";
        for (int i = 0; i < length; i++) {
            msg += (char)payload[i];
        }
        
        // Additional processing if needed for the configured room
        Serial.println("Message received in the configured room: " + msg);
        additionalSetup(msg);
    }
}

void connectToMQTT() {
    espClient.setInsecure();
    client.setServer(mqttServer, 8883);  // Using port 8883 for secure connections
    client.setCallback(callback);

    while (!client.connected()&& WiFi.status() == WL_CONNECTED) 
    {
        Serial.println("Attempting MQTT connection...");
        blink_led(100, 1);
        if (client.connect(mqttUsername, mqttUsername, mqttPassword)) 
        {
            Serial.println("connected");
            char msg[64];
            sprintf(msg, "%s-ST_ON00", mqttUsername);
            Serial.println(msg);
            client.publish("setup", msg);
            attachInterrupt(digitalPinToInterrupt(MOTION_PIN), sensMotion, RISING);
            // Subscribe to setup topic and the configured room topic
            client.subscribe("setup");
            if (topicConfigCompleted) 
            {
                client.subscribe(Room.c_str());
                sendUpdate(lastMessage);            
            }
        } else {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            blink_led(750, 3);
        }
    }
}

ICACHE_RAM_ATTR void checkResetButton() 
{
  while(digitalRead(RESET_PIN) == LOW)
    if (digitalRead(RESET_PIN) == LOW) 
    {
        if (!resetButtonPressed) 
        {
            resetButtonPressed = true;
            resetButtonTimer = millis();
            Serial.println("Reset Pressed");
        } 
        else if (millis() - resetButtonTimer >= RESET_HOLD_TIME) 
        {
            Serial.println("Reset button pressed for 5 seconds. Erasing EEPROM...");
            for (int i = 0; i < EEPROM_SIZE; i++) 
            {
                EEPROM.write(i, 0);
            }
            EEPROM.commit();
            blink_led(50,50);
            ESP.restart();
        }
    } 
    else 
    {
        resetButtonPressed = false;
    }
}
ICACHE_RAM_ATTR void sensMotion() 
{
  bool currentState = digitalRead(MOTION_PIN);
  lastMotionTimestamp = millis(); // Reset the timeout clock
  timeoutElapsed = false;         // Reset timeout flag
  //String motionStr = String(mqttUsername) + "-S301";
  //client.publish(Room.c_str(), motionStr.c_str()); 
  Serial.print("Motion detected ");
  Serial.println(millis()/1000);
  if (currentState == HIGH && topicConfigCompleted) 
  {
    if (lastMessage != "-S101") 
    {
      sendUpdate("-S101"); // Send S101 only if the last message was not S101
    }
  }
}

void sendUpdate(String message) 
{
  String motionStr = String(mqttUsername) + message;
  client.publish(Room.c_str(), motionStr.c_str()); 
  lastMessage = message; // Track the last message sent
}

void checkMotionTimeout() 
{
  unsigned long currentTime = millis();
  if (!timeoutElapsed && (currentTime - lastMotionTimestamp) > motionTimeout) 
  {
    if (lastMessage != "-S100") 
    {
      sendUpdate("-S100"); // Send S100 only if the last message was not S100
    }
    timeoutElapsed = true;          // Mark the timeout as elapsed
    lastMotionTimestamp = currentTime; // Reset the timeout clock
  }
}
void sendKeepAlive()
{
  if(millis()%300000 == 0)
  {
    sendUpdate("-K200");
  }
}
void additionalSetup(String command)
{
  if (command.indexOf("WZ") != -1 && command.indexOf("MDR") != -1 && command.indexOf(MacAddress) != -1) 
  {
    int hyphenIndex = command.indexOf('-');
    if (hyphenIndex != -1) 
    {
      String value = command.substring(hyphenIndex + 1);
      if (value.startsWith("D")) 
      {
        motionTimeout = ((value.substring(1)).toInt())*1000;
        Serial.print("New motion timeout is ");
        Serial.println(value.substring(1));
        EEPROM.put(TIMEOUT_ADDR, motionTimeout*1000);
        EEPROM.commit();
      }
      if (value == "S100")
      {
        lastMotionTimestamp = millis()-motionTimeout; // Reset the timeout clock
        timeoutElapsed = true;
      }
      if (value == "RESET")
      {
        eraseEeprom();
        ESP.restart();
      }
    }
  }
  else if(command.indexOf("WZ") != -1 && command.indexOf("4CR") != -1 || command.indexOf("1CR") != -1)
  {
    int hyphenIndex = command.indexOf('-');
    if (hyphenIndex != -1) 
    {

      String value = command.substring(hyphenIndex + 1);
      if (value == "R100" || value == "R200" || value == "R300" || value == "R400" || value == "R110" || value == "R210" || value == "R310" || value == "R410")
      {
        Serial.println("App override, timer reset");
        lastMessage = "S100";
        lastMotionTimestamp = millis()-motionTimeout; // Reset the timeout clock
        timeoutElapsed = true;
      }
    }
  }
}