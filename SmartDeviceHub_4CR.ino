#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <EEPROM.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>

#define STATUS_PIN D8   // Status LED pin  
#define RELAY_PIN_1 D1  // Relay 1 control pin
#define RELAY_PIN_2 D2  // Relay 2 control pin
#define RELAY_PIN_3 D3  // Relay 3 control pin
#define RELAY_PIN_4 D4  // Relay 4 control pin
#define SWITCH_PIN_1 D5  // Switch 1 control pin
#define SWITCH_PIN_2 D6  // Switch 2 control pin
#define SWITCH_PIN_3 D7  // Switch 3 control pin
#define SWITCH_PIN_4 D0  // Switch 4 control pin

#define EEPROM_SIZE 512
#define SSID_ADDR 30
#define PASSWORD_ADDR (SSID_ADDR + 32)
#define MQTT_USERNAME_ADDR (PASSWORD_ADDR + 32)
#define MQTT_PASSWORD_ADDR (MQTT_USERNAME_ADDR + 32)
#define MQTT_SERVER_ADDR (MQTT_PASSWORD_ADDR + 32)
#define TOPIC_CONFIG_ADDR (MQTT_SERVER_ADDR + 64)  // Address for storing configTopic in EEPROM

bool relayOverride1 = false; // Tracks if the relay state is overridden internally
bool switchState1 = HIGH;     // Current state of the switch
bool relayOverride2 = false; // Tracks if the relay state is overridden internally
bool switchState2 = HIGH;     // Current state of the switch
bool relayOverride3 = false; // Tracks if the relay state is overridden internally
bool switchState3 = HIGH;     // Current state of the switch
bool relayOverride4 = false; // Tracks if the relay state is overridden internally
bool switchState4 = HIGH;     // Current state of the switch

char apSSID[32];
char apPassword[32] = "12345678";  // Default AP password
char mqttUsername[32];
char mqttPassword[32];
char mqttServer[64];
char softApSSID[32];

volatile unsigned long lastToggleTime = 0;
volatile uint8_t toggleCount = 0;
bool lastState = HIGH;
const unsigned long detectionWindow = 5000; // 3 seconds
const uint8_t toggleCountRequired = 7;
WiFiClientSecure espClient;
PubSubClient client(espClient);
ESP8266WebServer server(80);

bool configCompleted = false;
bool topicConfigCompleted = false;
String MacAddress = "";
String Topic = "";
String configTopic = "setup"; // Default topic if first boot
String Room = "";
String overrideStatus = "";
void handleConfigRequest();
bool connectToWiFiWithRetry(const char* ssid, const char* password);
void saveCredentialsToEEPROM(const char* ssid, const char* password, const char* cluster);
void loadCredentialsFromEEPROM();
void connectToMQTT();
void toggleRelay(String command);
void blink_led(int del, int times);

void setup() {
    Serial.begin(115200);
    pinMode(SWITCH_PIN_1, INPUT_PULLUP);
    pinMode(SWITCH_PIN_2, INPUT_PULLUP);
    pinMode(SWITCH_PIN_3, INPUT_PULLUP);
    pinMode(SWITCH_PIN_4, INPUT);
    attachInterrupt(digitalPinToInterrupt(SWITCH_PIN_4), handleToggle, CHANGE);
    pinMode(RELAY_PIN_1, OUTPUT);
    pinMode(RELAY_PIN_2, OUTPUT);
    pinMode(RELAY_PIN_3, OUTPUT);
    pinMode(RELAY_PIN_4, OUTPUT);
    pinMode(STATUS_PIN, OUTPUT);
    EEPROM.begin(EEPROM_SIZE);
    trackSwitchState();
    configCompleted = EEPROM.read(0) == 1;
    topicConfigCompleted = EEPROM.read(1) == 1;  // Flag for topic configuration
    blink_led(100,10);
    MacAddress = WiFi.macAddress();
    MacAddress.replace(":", "");
    sprintf(softApSSID, "WZ_4CR_%s", MacAddress.c_str());
    processMacAddress(MacAddress, mqttPassword, sizeof(mqttPassword));
    if (!configCompleted) 
    {
        trackSwitchState();
        blink_led(50,10);
        WiFi.softAP(softApSSID, apPassword);
        server.on("/config", handleConfigRequest);
        server.begin();
        Serial.println();
        Serial.println("Access point started: " + String(softApSSID));
        Serial.println("Password for MQTT: "+ String(mqttPassword));

    } 
    else 
    {
        trackSwitchState();
        loadCredentialsFromEEPROM();
        // Load the topic from EEPROM if not the first boot
        if (topicConfigCompleted) 
        {
            EEPROM.get(TOPIC_CONFIG_ADDR, configTopic);
        }

        Serial.print("Loaded configTopic: ");
        Serial.println(configTopic);
        Room = configTopic;

        if (connectToWiFiWithRetry(apSSID, apPassword)) 
        {
            trackSwitchState();
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

void loop() 
{
    int overrideFlag = 0;
    sendKeepAlive();
    ESP.wdtFeed();
    bool currentSwitchState1 = digitalRead(SWITCH_PIN_1);
    bool currentSwitchState2 = digitalRead(SWITCH_PIN_2);
    bool currentSwitchState3 = digitalRead(SWITCH_PIN_3);
    bool currentSwitchState4 = digitalRead(SWITCH_PIN_4);
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
    if (toggleCount >= toggleCountRequired) 
    {
      // Call the reset function and reset the counter
      reset_Device();
      toggleCount = 0; // Reset toggle counter
    }
    if (millis() - lastToggleTime > detectionWindow) 
    {
      toggleCount = 0;
    }

    if (currentSwitchState1 != switchState1) 
    {
      switchState1 = currentSwitchState1;      // Update stored state
      relayOverride1 = false;                 // Disable override
      digitalWrite(RELAY_PIN_1, !switchState1); // Relay follows the switch
      if(currentSwitchState1 == 1)
        overrideStatus = String(mqttUsername) + "-R110";
      else
        overrideStatus = String(mqttUsername) + "-R111";
      overrideFlag = 1;
    }
    // Example: Internal logic to override relay state
    if (!relayOverride1) 
    {
      relayOverride1 = true;            // Enable override
      digitalWrite(RELAY_PIN_1, switchState1); // Set relay state
    }

    if (currentSwitchState2 != switchState2) 
    {
      switchState2 = currentSwitchState2;      // Update stored state
      relayOverride2 = false;                 // Disable override
      digitalWrite(RELAY_PIN_2, !switchState2); // Relay follows the switch
      if(currentSwitchState2 == 1)
        overrideStatus = String(mqttUsername) + "-R210";
      else
        overrideStatus = String(mqttUsername) + "-R211";
      overrideFlag = 1;
    }
    // Example: Internal logic to override relay state
    if (!relayOverride2) 
    {
      relayOverride2 = true;            // Enable override
      digitalWrite(RELAY_PIN_2, switchState2); // Set relay state
    }

    if (currentSwitchState3 != switchState3) 
    {
      switchState3 = currentSwitchState3;      // Update stored state
      relayOverride3 = false;                 // Disable override
      digitalWrite(RELAY_PIN_3, !switchState3); // Relay follows the switch
      if(currentSwitchState3 == 1)
        overrideStatus = String(mqttUsername) + "-R310";
      else
        overrideStatus = String(mqttUsername) + "-R311";
      overrideFlag = 1;
    }
    // Example: Internal logic to override relay state
    if (!relayOverride3) 
    {
      relayOverride3 = true;            // Enable override
      digitalWrite(RELAY_PIN_3, switchState3); // Set relay state
    }

    if (currentSwitchState4 != switchState4) 
    {
      switchState4 = currentSwitchState4;      // Update stored state
      relayOverride4 = false;                 // Disable override
      digitalWrite(RELAY_PIN_4, !switchState4); // Relay follows the switch
      if(currentSwitchState4 == 1)
        overrideStatus = String(mqttUsername) + "-R410";
      else
        overrideStatus = String(mqttUsername) + "-R411";
      overrideFlag = 1;
    }
    // Example: Internal logic to override relay state
    if (!relayOverride4) 
    {
      relayOverride4 = true;            // Enable override
      digitalWrite(RELAY_PIN_4, switchState4); // Set relay state
    }
    if (overrideFlag == 1)
    {
      client.publish(Room.c_str(), overrideStatus.c_str());
    }
    yield();
}
ICACHE_RAM_ATTR void handleToggle() 
{
  bool currentState = digitalRead(SWITCH_PIN_4);
  Serial.println(currentState);
  // Ensure it's a valid state change (debouncing)
  if (currentState != lastState) {
    lastToggleTime = millis(); // Update last toggle time
    toggleCount++;             // Increment toggle count
    lastState = currentState;  // Update last state
  }
}
void blink_led(int del, int times)
{
  for(int i = 0; i < times; i++)
  {
    digitalWrite(STATUS_PIN, HIGH);
    delay(del);
    digitalWrite(STATUS_PIN, LOW);
    delay(del);
    ESP.wdtFeed();
    trackSwitchState();
  }
}

void handleConfigRequest() 
{
    if (server.hasArg("SSID") && server.hasArg("Password") && server.hasArg("Cluster")) {
        String ssid = server.arg("SSID");
        String password = server.arg("Password");
        String cluster = server.arg("Cluster");
        ESP.wdtFeed();
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
        
        trackSwitchState();
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
        while (millis() - softAPStartTime < 300000 && WiFi.status() != WL_CONNECTED)  // 5 minutes timeout
        {
            trackSwitchState();
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
void trackSwitchState()
{
    digitalWrite(RELAY_PIN_1, digitalRead(SWITCH_PIN_1));
    digitalWrite(RELAY_PIN_2, digitalRead(SWITCH_PIN_2));
    digitalWrite(RELAY_PIN_3, digitalRead(SWITCH_PIN_3));
    digitalWrite(RELAY_PIN_4, digitalRead(SWITCH_PIN_4));
    yield();
}
void eraseEeprom()
{
  Serial.println("Erasing previous config");
  for (int i = 0; i < EEPROM_SIZE; i++) 
  {
    EEPROM.write(i, 0);
    trackSwitchState();
  }
  EEPROM.commit();
  delay(1000);
  blink_led(50, 50);
        //ESP.restart();
}
void saveCredentialsToEEPROM(const char* ssid, const char* password, const char* cluster) {
    strncpy(apSSID, ssid, sizeof(apSSID) - 1);
    EEPROM.put(SSID_ADDR, apSSID);

    strncpy(apPassword, password, sizeof(apPassword) - 1);
    EEPROM.put(PASSWORD_ADDR, apPassword);

    strncpy(mqttServer, cluster, sizeof(mqttServer) - 1);
    EEPROM.put(MQTT_SERVER_ADDR, mqttServer);

    sprintf(mqttUsername, "WZ_4CR_%s", MacAddress.c_str());
    EEPROM.put(MQTT_USERNAME_ADDR, mqttUsername);
    EEPROM.put(MQTT_PASSWORD_ADDR, mqttPassword);

    EEPROM.commit();
    Serial.println("Credentials saved to EEPROM.");
    blink_led(50, 20);
}
// Function to process MacAddress and return the modified string as a char array
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

    Serial.println("Loaded credentials from EEPROM:");
    Serial.println("SSID: " + String(apSSID));
    Serial.println("Password: " + String(apPassword));
    Serial.println("MQTT Server: " + String(mqttServer));
    Serial.println("MQTT Username: " + String(mqttUsername));
    Serial.println("MQTT Password: " + String(mqttPassword));
}

void callback(char* topic, byte* payload, unsigned int length) {
    Serial.println(topic);  
    Topic = topic;

    // Parse messages only if it's from the configured topic
    if (Topic == "setup") {
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
        if (msg.indexOf("WZ") != -1 && msg.indexOf("4CR") != -1 && msg.indexOf(MacAddress) != -1) {
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
    } else if (String(topic) == Room && topicConfigCompleted) {
        // Only parse messages in the configured room
        String msg = "";
        for (int i = 0; i < length; i++) {
            msg += (char)payload[i];
        }
        
        // Additional processing if needed for the configured room
        Serial.println("Message received in the configured room: " + msg);
        toggleRelay(msg);
    }
}
void connectToMQTT() {
    espClient.setInsecure();
    client.setServer(mqttServer, 8883);  // Using port 8883 for secure connections
    client.setCallback(callback);
    while (!client.connected() && WiFi.status() == WL_CONNECTED) {
        trackSwitchState();
        Serial.println("Attempting MQTT connection...");
        if (client.connect(mqttUsername, mqttUsername, mqttPassword)) {
            Serial.println("connected");
            char msg[64];
            sprintf(msg, "%s-ST_ON00", mqttUsername);
            Serial.println(msg);
            client.publish("setup", msg);
            digitalWrite(STATUS_PIN, HIGH);
            // Subscribe to setup topic and the configured room topic
            client.subscribe("setup");
            if (topicConfigCompleted) {
                client.subscribe(Room.c_str());
            }
        } else {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            delay(2000);
        }
    }
}

void reset_Device() 
{         
 for (int i = 0; i < EEPROM_SIZE; i++) 
 {
    //trackSwitchState();
    EEPROM.write(i, 0);
  }
  EEPROM.commit();
  blink_led(200,20);
  ESP.restart();
}
void sendKeepAlive()
{
  if(millis()%300000 == 0)
  {
    String keepAlive = String(mqttUsername) + "-K200";
    client.publish(Room.c_str(), keepAlive.c_str()); 
  }
}
void toggleRelay(String command) {
    if (command == String(mqttUsername) + "-R101") {
        
        Serial.println("Relay 1 ON");
        digitalWrite(RELAY_PIN_1, LOW);
    } else if (command == String(mqttUsername) + "-R100") {
        
        Serial.println("Relay 1 OFF");
        digitalWrite(RELAY_PIN_1, HIGH);
    }
    if (command == String(mqttUsername) + "-R201") {
        
        Serial.println("Relay 2 ON");
        digitalWrite(RELAY_PIN_2, LOW);
    } else if (command == String(mqttUsername) + "-R200") {
        Serial.println("Relay 2 OFF");
        digitalWrite(RELAY_PIN_2, HIGH);
    }
    if (command == String(mqttUsername) + "-R301") {
        
        Serial.println("Relay 3 ON");
        digitalWrite(RELAY_PIN_3, LOW);
    } else if (command == String(mqttUsername) + "-R300") {
        Serial.println("Relay 3 OFF");
        digitalWrite(RELAY_PIN_3, HIGH);
    }
    if (command == String(mqttUsername) + "-R401") {
        
        Serial.println("Relay 4 ON");
        digitalWrite(RELAY_PIN_4, LOW);
    } else if (command == String(mqttUsername) + "-R400") {
        Serial.println("Relay 4 OFF");
        digitalWrite(RELAY_PIN_4, HIGH);
    }
    if (command == String(mqttUsername) + "-RESET")
      {
        eraseEeprom();
        reset_Device();
        ESP.restart();
      }

}
