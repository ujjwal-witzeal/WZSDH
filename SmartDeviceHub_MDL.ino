//To be run on a WEMOS D1 mini board

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <EEPROM.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include <FastLED.h>
#define MOTION_PIN A0 // Motion data input in 0-3.3v P-P
#define STATUS_PIN D6 // LED status connection pin
#define RESET_PIN D7  // Reset button pin
#define LIGHT_PIN D1  // PWM pin for standard white LED, typically connected to a MOSFET for brightness control
#define RGB_PIN D5    // Neopixel connection pin for RGB and colour based mood lighting

#define EEPROM_SIZE 512
#define SSID_ADDR 30
#define PASSWORD_ADDR (SSID_ADDR + 32)
#define MQTT_USERNAME_ADDR (PASSWORD_ADDR + 32)
#define MQTT_PASSWORD_ADDR (MQTT_USERNAME_ADDR + 32)
#define MQTT_SERVER_ADDR (MQTT_PASSWORD_ADDR + 32)
#define TOPIC_CONFIG_ADDR (MQTT_SERVER_ADDR + 64)  // Address for storing configTopic in EEPROM
#define BRIGHT_ADDR (480)
#define R_ADDR (476)
#define G_ADDR (472)
#define B_ADDR (468)
#define AUTO_ADDR (464)
#define TIMEOUT_ADDR (500)
#define SENS_ADDR (504)
#define RESET_HOLD_TIME 5000  // 5 seconds hold time for reset

static unsigned long resetButtonTimer = 0;
static bool resetButtonPressed = false;
bool mqtt_Connected = false;          

char apSSID[32];                       // To store user provided SSID
char apPassword[32] = "12345678";      // Default AP password
char mqttUsername[32];                 // To store MQTT username for login, typically in format of WZ_MDL_<MAC>
char mqttPassword[32];                 // To store mqtt login password, typical format <MAC>, with first alphabet in lowercase and all other in uppercase
char mqttServer[64];                   // To store cluster for connecting to MQTT
char softApSSID[32];                   // Self wifi SSID for configuration, typically same as mqttUsername
CRGB leds[8];                          // WS2812B string length

WiFiClientSecure espClient;
PubSubClient client(espClient);
ESP8266WebServer server(80);

int wifi_counter = 0;                  // WiFi connection attempt counter
bool configCompleted = false;          // Connectivity config flag from HTTP request
bool topicConfigCompleted = false;     // Topic config flag from MQTT flag
String MacAddress = "";                // Device Identifier
String Topic = "";                     // New topic storing temp variable
String configTopic = "setup";          // Default topic if first boot
String Room = "";                      // Actual topic where all communication will happen
int motionTimeout = 60;                // 60 seconds timeout for motion detection
int brightness = 99;                   // Default brightness of 240V led to be PWM controlled
int r,g,b = 99;                        // Default RGB value for WS2812B ring
unsigned long lastMotionTimestamp = 0; // Last time a motion event was detected in milliseconds of CPU clock
bool timeoutElapsed = false;           // Tracks if the timeout period has elapsed
String lastMessage = "-S100";          // Tracks the last message sent
int sensitivity_motion = 10;           // Default sensitivity to motion
const int bufferSize = 25;             // Size of the moving average buffer
int readings[bufferSize];              // Array to store readings
int Index = 0;                         // Current Index in the buffer
int total = 0;                         // Total of all values in the buffer
int movingAverage = 0;                 // Calculated moving average
const int neutralValue = 600;          // Neutral ADC value for moving average calculation
int auto_flag = 1;                     // Light auto on-off flag
int k = 0;                             // Random LED position tracker

void handleConfigRequest();
bool connectToWiFiWithRetry(const char* ssid, const char* password);
void saveCredentialsToEEPROM(const char* ssid, const char* password, const char* cluster);
void loadCredentialsFromEEPROM();
void connectToMQTT();
void checkResetButton();

void setup() 
{
    Serial.begin(115200);
    EEPROM.begin(EEPROM_SIZE);
    pinMode(RESET_PIN, INPUT_PULLUP);
    pinMode(MOTION_PIN, INPUT);
    pinMode(STATUS_PIN, OUTPUT);
    pinMode(LIGHT_PIN, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(RESET_PIN), checkResetButton, FALLING);
    FastLED.addLeds<NEOPIXEL, RGB_PIN>(leds, 8);
    for(int i = 0; i < 8; i++) 
    {
      leds[i] = CRGB::White;
      FastLED.show();
      delay(100);
    }
    delay(200);
    for(int i = 7; i >= 0; i--) 
    {
      leds[i] = CRGB::Black;
      FastLED.show();
      delay(100);
    }
    for (int i = 0; i < bufferSize; i++) 
    {
      readings[i] = 0;  // Start with no deflection
    }
    // Load config from EEPROM
    configCompleted = EEPROM.read(0) == 1;
    topicConfigCompleted = EEPROM.read(1) == 1;  // Flag for topic configuration
    blink_led(100,10);
    MacAddress = WiFi.macAddress();
    MacAddress.replace(":", "");
    sprintf(softApSSID, "WZ_MDL_%s", MacAddress.c_str());
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
        motionTimeout = 60;
        auto_flag = 1;
        r = 99;
        g = 99;
        b = 99;
        brightness = 99;
        auto_flag = 1;
        sensitivity_motion = 10;
        EEPROM.put(TIMEOUT_ADDR, motionTimeout);
        EEPROM.put(SENS_ADDR, sensitivity_motion);
        EEPROM.put(BRIGHT_ADDR, brightness);
        EEPROM.put(R_ADDR, r);
        EEPROM.put(G_ADDR, g);
        EEPROM.put(B_ADDR, b);
        EEPROM.put(AUTO_ADDR, auto_flag);

    } 
    else 
    {
        loadCredentialsFromEEPROM();
        // Load the topic from EEPROM if not the first boot
        EEPROM.get(TIMEOUT_ADDR, motionTimeout);
        EEPROM.get(TOPIC_CONFIG_ADDR, configTopic);
        EEPROM.get(SENS_ADDR, sensitivity_motion);
        EEPROM.get(BRIGHT_ADDR, brightness);
        EEPROM.get(R_ADDR, r);
        EEPROM.get(G_ADDR, g);
        EEPROM.get(B_ADDR, b);
        EEPROM.get(AUTO_ADDR, auto_flag);
        if(r == 0 && b == 0 && g == 0 && sensitivity_motion == 0)
        {
          motionTimeout = 60;
          auto_flag = 1;
          r = 99;
          g = 99;
          b = 99;
          brightness = 99;
          auto_flag = 1;
          sensitivity_motion = 10;
        }
        Serial.println("Motion Timeout: " + String(motionTimeout));
        Serial.println("Sensitivity: " + String(sensitivity_motion));
        Serial.println("Brightness: " + String(brightness));
        Serial.println("R: " + String(r));
        Serial.println("G: " + String(g));
        Serial.println("B: " + String(b));
        Serial.println("Auto flag: " + String(auto_flag));
        Serial.print("Loaded configTopic: ");
        Serial.println(configTopic);
        Room = configTopic;
        if(Room == "")
        {
          Room = "setup";
        }

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

void loop() 
{
  if (mqtt_Connected == 1)
  {
    sendKeepAlive();
    if(configCompleted == 1)
    {
      sensMotion();
    }
  }
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
  ESP.wdtFeed();
}

void blink_led(int del, int times)
{
  
  for(int i = 0; i < times; i++)
  {
    if(k > 7)
    {
      k = 0;
    }
    for(int j = 0; j< 8; j++)
    {
      digitalWrite(STATUS_PIN, HIGH);
      leds[j] = CRGB::Black;
      FastLED.show();
    }
    digitalWrite(STATUS_PIN, HIGH);
    leds[k] = CRGB::Blue;
    FastLED.show();
    delay(del);
    digitalWrite(STATUS_PIN, LOW);
    leds[k] = CRGB::Black;
    FastLED.show();
    delay(del);
    k++;
  }
  
}

void handleConfigRequest() 
{
  if (server.hasArg("SSID") && server.hasArg("Password") && server.hasArg("Cluster")) 
  {
    String ssid = server.arg("SSID");
    String password = server.arg("Password");
    String cluster = server.arg("Cluster");
    Serial.println("Received configuration:");
    Serial.println("SSID: " + ssid);
    Serial.println("Password: " + password);
    Serial.println("Cluster: " + cluster);

    // Try to connect to the provided network
    if (connectToWiFiWithRetry(ssid.c_str(), password.c_str())) 
    {
      server.send(200, "text/plain", "Connected successfully");
      eraseEeprom();
      WiFi.softAPdisconnect(true);
      saveCredentialsToEEPROM(ssid.c_str(), password.c_str(), cluster.c_str());
      configCompleted = true;
      EEPROM.write(0, 1);  // Mark config as completed
      EEPROM.commit();
      ESP.restart();
    } 
    else 
    {
      server.send(500, "text/plain", "Failed to connect to Wi-Fi. Please try again.");
      Serial.println("Failed to connect to Wi-Fi.");
      eraseEeprom();
    }
  } 
  else 
  {
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
    sensMotion();
    checkMotionTimeout();
    while (WiFi.status() != WL_CONNECTED && attempt < 20) 
    {
        Serial.println("Connection attempt " + String(attempt + 1) + "...");
        for(int i = 0; i < 8; i++) 
        {
          leds[i] = CRGB::Black;
          FastLED.show();
        }
        delay(500);
        for(int i = 0; i < 8; i++) 
        {
          leds[i] = CRGB::DarkGreen;
          leds[7-i] = CRGB::Black;
          FastLED.show();
        }
        delay(500);
        for(int i = 0; i < 8; i++) 
        {
          leds[i] = CRGB::Black;
          FastLED.show();
        }
        attempt++;
        mqtt_Connected = false;
    }

    // Check if connection was successful
    if (WiFi.status() == WL_CONNECTED)
    {
        Serial.println("Successfully connected to Wi-Fi!");
        Serial.println("IP Address: " + WiFi.localIP().toString());
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
    while ((millis() - softAPStartTime) < 300000)  // 5 minutes timeout
    {
      sensMotion();
      checkMotionTimeout();
      if (WiFi.status() != WL_CONNECTED && configCompleted ) 
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
  for (int i = 0; i < 450; i++) 
  {
    EEPROM.write(i, 0);
  }
  EEPROM.commit();
  delay(1000);
  //ESP.restart();
}
void saveCredentialsToEEPROM(const char* ssid, const char* password, const char* cluster) 
{
  strncpy(apSSID, ssid, sizeof(apSSID) - 1);
  EEPROM.put(SSID_ADDR, apSSID);

  strncpy(apPassword, password, sizeof(apPassword) - 1);
  EEPROM.put(PASSWORD_ADDR, apPassword);

  strncpy(mqttServer, cluster, sizeof(mqttServer) - 1);
  EEPROM.put(MQTT_SERVER_ADDR, mqttServer);

  sprintf(mqttUsername, "WZ_MDL_%s", MacAddress.c_str());
  EEPROM.put(MQTT_USERNAME_ADDR, mqttUsername);
  EEPROM.put(MQTT_PASSWORD_ADDR, mqttPassword);

  EEPROM.commit();
  Serial.println("Credentials saved to EEPROM.");
  blink_led(50, 20);
}

void processMacAddress(const String& MacAddress, char* mqttPassword, size_t maxLength) 
{
  bool firstAlphabetFound = false; // Flag to track if the first alphabet is found
  size_t index = 0; // To track position in mqttPassword

  // Convert the entire string to uppercase first
  String uppercaseMac = MacAddress;
  uppercaseMac.toUpperCase();

  // Iterate through the MacAddress string
  for (size_t i = 0; i < uppercaseMac.length() && index < maxLength - 1; i++) 
  {
    char currentChar = uppercaseMac[i];
    if (!firstAlphabetFound && isalpha(currentChar)) 
    {
      // If this is the first alphabet, convert it to lowercase
      mqttPassword[index++] = (char)tolower(currentChar);
      firstAlphabetFound = true; // Mark that we've found the first alphabet
    } 
    else 
    {
      // Append the character as is (already in uppercase)
      mqttPassword[index++] = currentChar;
    }
  }

  // Null-terminate the resulting char array
  mqttPassword[index] = '\0';
}

void loadCredentialsFromEEPROM() 
{
  EEPROM.get(SSID_ADDR, apSSID);
  EEPROM.get(PASSWORD_ADDR, apPassword);
  EEPROM.get(MQTT_SERVER_ADDR, mqttServer);
  EEPROM.get(MQTT_USERNAME_ADDR, mqttUsername);
  EEPROM.get(MQTT_PASSWORD_ADDR, mqttPassword);
  EEPROM.get(TIMEOUT_ADDR, motionTimeout);
  EEPROM.get(SENS_ADDR, sensitivity_motion);

  Serial.println("Loaded credentials from EEPROM:");
  Serial.println("SSID: " + String(apSSID));
  Serial.println("Password: " + String(apPassword));
  Serial.println("MQTT Server: " + String(mqttServer));
  Serial.println("MQTT Username: " + String(mqttUsername));
  Serial.println("MQTT Password: " + String(mqttPassword));
  blink_led(1000, 2);
}

void callback(char* topic, byte* payload, unsigned int length) 
{
  Serial.println(topic);  
  Topic = topic;
  // Parse messages only if it's from the configured topic
  if (Topic == "setup") 
  {
    String msg = "";
    for (int i = 0; i < length; i++) 
    {
      msg += (char)payload[i];
    }
    if(msg == "ALL-ATTENDENCE")
    {
      char msg[64];
      sprintf(msg, "%s-ST_ON00", mqttUsername);
      Serial.println(msg);
      client.publish("setup", msg);
    }
    if (msg.indexOf("WZ") != -1 && msg.indexOf("MDL") != -1 && msg.indexOf(MacAddress) != -1) 
    {
      int hyphenIndex = msg.indexOf('-');
      if (hyphenIndex != -1) 
      {
        String receivedTopic = msg.substring(hyphenIndex + 1);

        if (receivedTopic.startsWith("SC_")) 
        {
          String newConfigTopic = receivedTopic.substring(3);  // Get part after "SC_"
          if (newConfigTopic != Room) 
          { // Only update if the topic is different
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
            delay(500);
            for(int i = 0;i<50;i++)
            {
              sensMotion();
              checkMotionTimeout();
            }
            // Unsubscribe from the old room topic if it exists
            if (!Room.isEmpty()) 
            {
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
      for (int i = 0; i < length; i++) 
      {
          msg += (char)payload[i];
      }
      
      // Additional processing if needed for the configured room
      Serial.println("Message received in the configured room: " + msg);
      additionalSetup(msg);
  }
}

void connectToMQTT() 
{
  int attempt_mqtt = 0;
  espClient.setInsecure();
  client.setServer(mqttServer, 8883);  // Using port 8883 for secure connections
  client.setCallback(callback);

  while (!client.connected() && WiFi.status() == WL_CONNECTED) 
  {
      Serial.println("Attempting MQTT connection...");
      if (client.connect(mqttUsername, mqttUsername, mqttPassword) && WiFi.status() == WL_CONNECTED) 
      {
          attempt_mqtt = 0;
          for(int i = 0; i < 8; i++) 
          {
            leds[i] = CRGB::Black;
            FastLED.show();
          }
          for(int i = 0; i < 8; i++) 
          {
            leds[i] = CRGB::DarkOrange;
            FastLED.show();
          }
          delay(500);
          for(int i = 0; i < 8; i++) 
          {
            leds[i] = CRGB::Black;
            FastLED.show();
            delay(125);
          }
          Serial.println("connected");
          char msg[64];
          sprintf(msg, "%s-ST_ON00", mqttUsername);
          Serial.println(msg);
          client.publish("setup", msg);
          //attachInterrupt(digitalPinToInterrupt(MOTION_PIN), sensMotion, RISING);
          if(WiFi.status() != WL_CONNECTED)
          {
            break;
          }
          mqtt_Connected = true;
          // Subscribe to setup topic and the configured room topic
          client.subscribe("setup");
          if (topicConfigCompleted) 
          {
              client.subscribe(Room.c_str());
              mqtt_Connected = true;
              sendUpdate(lastMessage);            
          }
      } 
      else 
      {
          mqtt_Connected = false;
          Serial.print("failed, rc=");
          Serial.print(client.state());
          if(client.state() < 0 )
          {
            sensMotion();
            checkMotionTimeout();
            attempt_mqtt++;
          }
          if(attempt_mqtt > 5)
          {
            ESP.restart();
          }
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
void sensMotion()                       //---------- DO NOT TOUCH ----------- Works with some beyond this world logic, should not work in any logical sense, pure magic
{
  ESP.wdtFeed();
  total -= readings[Index];
  noInterrupts();
  readings[Index] = abs(analogRead(MOTION_PIN) - neutralValue);
  //Serial.print("1000, ");
  //Serial.print("0, ");
  //Serial.println(analogRead(MOTION_PIN));
  total += readings[Index];
  Index = (Index + 1) % bufferSize;
  movingAverage = total / bufferSize;
  bool currentState = (movingAverage>sensitivity_motion*10);
  //String motionStr = String(mqttUsername) + "-S301";
  //client.publish(Room.c_str(), motionStr.c_str()); 
  if (currentState == true) 
  {
    lastMotionTimestamp = millis(); // Reset the timeout clock
    if(auto_flag == 1)
    {
      analogWrite(LIGHT_PIN, map(brightness,0,99,0,255));
      for(int i = 0; i < 8; i++) 
      {
        leds[i] = CRGB(map(r,0,99,0,255),map(g,0,99,0,255),map(b,0,99,0,255));
        FastLED.show();
      }
    }
    //digitalWrite(STATUS_PIN, HIGH);
    timeoutElapsed = false;         // Reset timeout flag
  }
  if (currentState == true && topicConfigCompleted && mqtt_Connected == 1) 
  {
    if (lastMessage != "-S101") 
    {
      if(auto_flag == 1)
      {
        sendUpdate("-R111");
      }
      digitalWrite(STATUS_PIN, HIGH);
      sendUpdate("-S101"); // Send S101 only if the last message was not S101
    }
  }
  delay(5);
  interrupts();
  ESP.wdtFeed();
}

void sendUpdate(String message) 
{
  String motionStr = String(mqttUsername) + message;
  client.publish(Room.c_str(), motionStr.c_str()); 
  lastMessage = message; // Track the last message sent
}

void checkMotionTimeout() 
{
  ESP.wdtFeed();
  unsigned long currentTime = millis();
  if (!timeoutElapsed && (currentTime - lastMotionTimestamp) > motionTimeout*1000) 
  {
    if(auto_flag == 1)
    {
      analogWrite(LIGHT_PIN, brightness*0);
      for(int i = 0; i < 8; i++) 
      {
        leds[i] = CRGB::Black;
        FastLED.show();
      }
    }
    if (lastMessage != "-S100" && mqtt_Connected == 1) 
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
  ESP.wdtFeed();
}
void additionalSetup(String command)
{
  ESP.wdtFeed();
  if (command.indexOf("WZ") != -1 && command.indexOf("MDL") != -1 && command.indexOf(MacAddress) != -1) 
  {
    int hyphenIndex = command.indexOf('-');
    if (hyphenIndex != -1) 
    {
      String value = command.substring(hyphenIndex + 1);
      if (value.startsWith("D")) 
      {
        motionTimeout = ((value.substring(1)).toInt());
        Serial.print("New motion timeout is ");
        Serial.println(motionTimeout);
        EEPROM.put(TIMEOUT_ADDR, motionTimeout);
        EEPROM.commit();
      }
      if (value.startsWith("B")) 
      {
        brightness = ((value.substring(1)).toInt());
        analogWrite(LIGHT_PIN, map(brightness,0,99,0,255));
        for(int i = 0; i < 8; i++) 
        {
          leds[i] = CRGB(map(brightness,0,99,0,255),map(brightness,0,99,0,255),map(brightness,0,99,0,255));
          FastLED.show();
        }
        Serial.print("New brightness is ");
        Serial.println(brightness);
        EEPROM.put(BRIGHT_ADDR, brightness);
        
        EEPROM.commit();
      }
      if (value.startsWith("C")) 
      {
        r = ((value.substring(1)).toInt());
        b = r%100;
        r = r/100;
        g = r%100;
        r = r/100;
        EEPROM.put(R_ADDR, r);
        EEPROM.put(G_ADDR, g);
        EEPROM.put(B_ADDR, b);
        EEPROM.commit();
        Serial.print("R :");
        Serial.print(r);
        Serial.print(" G :");
        Serial.print(g);
        Serial.print(" B :");
        Serial.println(b);
        analogWrite(LIGHT_PIN, 0*10);
        for(int i = 0; i < 8; i++) 
        {
          leds[i] = CRGB(map(r,0,99,0,255),map(g,0,99,0,255),map(b,0,99,0,255));
          FastLED.show();
        }
      }
      if (value == "R101")
      {
        analogWrite(LIGHT_PIN, map(brightness,0,99,0,255));
        for(int i = 0; i < 8; i++) 
        {
          leds[i] = CRGB(map(brightness,0,99,0,255),map(brightness,0,99,0,255),map(brightness,0,99,0,255));
          FastLED.show();
        }
      }
      if (value == "R100")
      {
        analogWrite(LIGHT_PIN, brightness*0);
        for(int i = 0; i < 8; i++) 
        {
          leds[i] = CRGB::Black;
          FastLED.show();
        }
        lastMessage = "S100";
        lastMotionTimestamp = millis()-motionTimeout*1000; // Reset the timeout clock
        timeoutElapsed = true;
      }
      if (value == "A101")
      {
        auto_flag = 1;
        EEPROM.put(AUTO_ADDR, auto_flag);
        EEPROM.commit();
      }
      if (value == "A100")
      {
        auto_flag = 0;
        EEPROM.put(AUTO_ADDR, auto_flag);
        EEPROM.commit();
      }
      if (value == "SN03")
      {
        sensitivity_motion = 10;
        Serial.print("New sensitivity is ");
        Serial.println(sensitivity_motion);
        EEPROM.put(SENS_ADDR, sensitivity_motion);
        EEPROM.commit();
      }
      if (value == "SN02")
      {
        sensitivity_motion = 20;
        Serial.print("New sensitivity is ");
        Serial.println(sensitivity_motion);
        EEPROM.put(SENS_ADDR, sensitivity_motion);
        EEPROM.commit();
      }
      if (value == "SN01")
      {
        sensitivity_motion = 30;
        Serial.print("New sensitivity is ");
        Serial.println(sensitivity_motion);
        EEPROM.put(SENS_ADDR, sensitivity_motion);
        EEPROM.commit();
      }
      if (value == "S100")
      {
        lastMotionTimestamp = millis()-motionTimeout*1000; // Reset the timeout clock
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
        lastMotionTimestamp = millis()-motionTimeout*1000; // Reset the timeout clock
        timeoutElapsed = true;
      }
    }
  }
}