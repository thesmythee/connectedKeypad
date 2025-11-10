#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include "KeypadSerial.h"
#include "MSGprotocol.h"

// ------------------------------------
// 1a. WIFI SIDE CONFIGURATION
// ------------------------------------

// Wi-Fi Credentials
const char* WIFI_SSID = "Your IOT SSID";
const char* WIFI_PASSWORD = "Your IOT Password";

// MQTT Broker Details
const char* MQTT_SERVER = "192.168.XXX.XXX";
const int MQTT_PORT = 1883;
const char* MQTT_USER = "mqtt_user";     // Optional, set to "" if not needed
const char* MQTT_PASSWORD = "mqtt_password"; // Optional, set to "" if not needed
const char* MQTT_CLIENT_ID = "ECP_AlarmPanel";

// Alarm Configuration (MUST match your Home Assistant configuration)
// Home Assistant mqtt manual alarm default topics follow this pattern:
// State topic: <base_topic>/state
// Command topic: <base_topic>/command
const char* ALARM_BASE_TOPIC = "homeassistant/alarm_control_panel";
const char* ALARM_COMMAND_TOPIC = "homeassistant/alarm_control_panel/command";  // Send key sequences and alarm states to Home Assistant
const char* ALARM_STATE_TOPIC = "homeassistant/alarm_control_panel/state";      // Receive F7 messages and alarm states from Home Assistant

String ALARM_CODE = "1234"; // The code required to disarm the alarm. Set to "0000" to disable keypad managed alarm states


// ------------------------------------
// 1b. KEYPAD SIDE CONFIGURATION
// ------------------------------------

#define PRINT_BUF_SIZE   (128)
static char pBuf[PRINT_BUF_SIZE];  // sprintf buffer

static const uint32_t KP_POLL_PERIOD   = 1100;  // how often to poll keypad (ms)
static const uint32_t KP_F7_PERIOD     = 4000;  // how often to send F7 status message (ms)
static const uint32_t MIN_TX_GAP       =   50;  // allow at least this many ms between transmits to keypads
static const uint32_t READ_KEY_DELAY   =   40;  // delay between keypad poll response and keypad read
static const uint32_t KEYPRESS_TIMEOUT = 5000;  // timeout for adding keys to sequence

KeypadSerial kpSerial;     // keypadSerial class
MSGprotocol  msgProtocol;  // protocol class for converting msgs to/from USB serial

uint32_t kpF7time;        // global, last time F7 message sent
uint32_t kpPollTime;      // global, last time keypad was polled
uint32_t lastSendTime;    // global, last time message sent to keypad
uint32_t keyPressTime[8]; // global, last time keypress received from keypad

bool     keyPadRead;     // if true, in keypad read mode
uint8_t  keyPad;         // next keypad to read
uint8_t  numKeyPads;     // number of keypads that responded to poll

// ------------------------------------
// 2. STATE MANAGEMENT
// ------------------------------------

String currentAlarmState = "disarmed"; // Initial state
String keypadcode[8] = {"","","","","","","",""}; // initial state
char hex2char(uint8_t hex);
void codeCommands(uint8_t activeKeypad);

#define DISARMED_F7  "F7 p=1 t=0 n=0 x=0 f=0 r=1 s=0 a=0 w=0 d=0 i=0"
#define ARMSTAY_F7   "F7 p=1 t=0 r=0 s=1 a=0"
#define ARMAWAY_F7   "F7 p=1 t=0 r=0 s=0 a=1"
#define ARMNIGHT_F7  "F7 p=1 t=0 r=0 s=1 a=0 n=1"
#define ARMVACAY_F7  "F7 p=1 t=0 r=0 s=0 a=1 i=1"
#define ARMBYPASS_F7 "F7 p=1 t=0 r=0 s=0 a=1 d=1"
#define PENDING_F7   "F7 p=1 t=5"
#define TRIGGERED_F7 "F7 p=1 t=7 w=1"

// ------------------------------------
// 3. UTILITY & CONNECTION SETUP
// ------------------------------------

WiFiClient espClient;
PubSubClient client(espClient);
uint32_t lastMsg = 0;
bool firstF7 = 1;

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else {  // U_FS
      type = "filesystem";
    }

    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

// Function to reconnect to MQTT
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");

    if (client.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD)) {
      Serial.println("connected");
      // Once connected, resubscribe and publish initial state
      client.subscribe(ALARM_STATE_TOPIC);
      Serial.print("Subscribed to: ");
      Serial.println(ALARM_STATE_TOPIC);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" trying again in 5 seconds");
      delay(5000); // Wait 5 seconds before retrying
    }
  }
}

// ------------------------------------
// 4. MQTT CALLBACK (Command Handler)
// ------------------------------------

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  
  // Convert the payload to a String for easier comparison
  String command = "";
  for (unsigned int i = 0; i < length; i++) {
    command += (char)payload[i];
  }
  Serial.println(command);

  // Check if the command came from the correct topic
  if (strcmp(topic, ALARM_STATE_TOPIC) == 0) {

    // STATE: ARM_HOME
    if (command == "armed_home") {
      currentAlarmState = "armed_home";
      Serial.println("Alarm successfully ARMED_HOME.");
      msgProtocol.parseRecv(ARMSTAY_F7, strlen(ARMSTAY_F7));
    } 
    // STATE: ARM_AWAY
    else if (command == "armed_away") {
      currentAlarmState = "armed_away";
      Serial.println("Alarm successfully ARMED_AWAY.");
      msgProtocol.parseRecv(ARMAWAY_F7, strlen(ARMAWAY_F7));
    } 
    // STATE: ARM_NIGHT
    else if (command == "armed_night") {
      currentAlarmState = "armed_night";
      Serial.println("Alarm successfully ARMED_NIGHT.");
      msgProtocol.parseRecv(ARMNIGHT_F7, strlen(ARMNIGHT_F7));
    } 
    // STATE: ARM_VACATION
    else if (command == "armed_vacation") {
      currentAlarmState = "armed_vacation";
      Serial.println("Alarm successfully ARMED_VACATION.");
      msgProtocol.parseRecv(ARMVACAY_F7, strlen(ARMVACAY_F7));
    }  
    // STATE: ARM_CUSTOM_BYPASS
    else if (command == "armed_custom_bypass") {
      currentAlarmState = "armed_custom_bypass";
      Serial.println("Alarm successfully ARMED_CUSTOM_BYPASS.");
      msgProtocol.parseRecv(ARMBYPASS_F7, strlen(ARMBYPASS_F7));
    } 
    // STATE: PENDING
    else if (command == "pending") {
      currentAlarmState = "pending";
      Serial.println("Alarm PENDING.");
      msgProtocol.parseRecv(PENDING_F7, strlen(PENDING_F7));
    } 
    // STATE: TRIGGERED
    else if (command == "triggered") {
      currentAlarmState = "triggered";
      Serial.println("Alarm TRIGGERED.");
      msgProtocol.parseRecv(TRIGGERED_F7, strlen(TRIGGERED_F7));
    } 
    // STATE: DISARM
    else if (command == "disarmed") {
      currentAlarmState = "disarmed";
      Serial.println("Alarm successfully DISARMED.");
      msgProtocol.parseRecv(DISARMED_F7, strlen(DISARMED_F7));
    } 
    // STATE: New F7 message
    else if (command.startsWith("F7")) {
      Serial.print("Attempting to parse F7 from MQTT... ");
      if(msgProtocol.parseRecv((const char *)payload,(const uint8_t)length)){
        Serial.println("Success");
        kpF7time = 0;  // always update keypad as soon as new F7 message arrives
      }
      else Serial.println("Failed");
    }
    // Handle other states if needed (ARM_NIGHT, etc.)
    else {
      Serial.print("Unknown command: ");
      Serial.println(command);
      
      //Parse other messages including F7
      //uint8_t mqttMsgSize = 0;
      //const char * mqttMsg = piSerial.getMsg(&mqttMsgSize);

      uint8_t msgType = msgProtocol.parseRecv(command.c_str(), static_cast<uint8_t>(command.length()));

      if (msgType == 0xF7)
      {
          kpF7time = 0;  // always update keypad as soon as new F7 message arrives
      }
      return; // Do not publish state change if command is unknown
    }
  }
}

// ------------------------------------
// 5. ARDUINO SETUP & LOOP
// ------------------------------------

void setup() {
  Serial.begin(115200);
  Serial.println("Booting");

  setup_wifi();

  client.setServer(MQTT_SERVER, MQTT_PORT);
  client.setCallback(callback);

  // Note: We don't reconnect here. It happens at the start of loop().

  kpSerial.init();        // init class
  msgProtocol.init();     // init class

  uint32_t ms = millis();

  kpF7time = ms;
  kpPollTime = ms;
  lastSendTime = ms;
  for(uint8_t i = 0; i<8; i++) keyPressTime[i] = 0;

  keyPadRead = false;
  keyPad = 0;
  numKeyPads = 0;
  
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  ArduinoOTA.handle();

  //long now = millis();
  uint32_t ms = millis();  // milliseconds since start of run
  
  // Publish the current state periodically (every 1 minute) as a heartbeat
  // if (ms - lastMsg > 60000) { 
  //   lastMsg = ms;
  //   publishState();
  // }

  uint8_t k = 0;

  //if (kpSerial.read(&k, 0)) // if we have unhandled chars from keypad, consume them
  //{
  //    sprintf(pBuf, "WARN: unhandled keypad char %02x\n", k);
  //    Serial.println(pBuf);
  //}

  if (keyPadRead)  // we are in keypad read mode
  {
    if (ms - kpPollTime > READ_KEY_DELAY)  // after waiting the appropriate time after polling, read the keypad data
    {
      // read the next keypad, evaluate commands, send key messages to MQTT
      kpPollTime = ms;
      uint8_t msgType = kpSerial.requestData(keyPad);
      lastSendTime = millis();
      if (msgType == KEYS_MESG)       // if true, key presses were returned for this keypad
      {
        uint8_t activeKeypad = kpSerial.getAddr(keyPad);
        for(uint8_t i = 0; i<kpSerial.getKeyCount(); i++){
          keypadcode[activeKeypad-16] += hex2char(*(kpSerial.getKeys()+i));
        }
        client.publish(ALARM_COMMAND_TOPIC, msgProtocol.keyMsg(pBuf, PRINT_BUF_SIZE,
            kpSerial.getAddr(keyPad), kpSerial.getKeyCount(), kpSerial.getKeys(), msgType), true); // Retain flag set to true
        Serial.println(msgProtocol.keyMsg(pBuf, PRINT_BUF_SIZE,
            kpSerial.getAddr(keyPad), kpSerial.getKeyCount(), kpSerial.getKeys(), msgType));
        keyPressTime[activeKeypad-16] = ms;
        if(ALARM_CODE != "0000") codeCommands(activeKeypad);
      }
      else if (msgType != NO_MESG)  // we received some other type of message
      {
        client.publish(ALARM_COMMAND_TOPIC, msgProtocol.keyMsg(pBuf, PRINT_BUF_SIZE, 
            kpSerial.getAddr(keyPad), kpSerial.getRecvMsgLen(), kpSerial.getRecvMsg(), msgType), true); // Retain flag set to true
        Serial.println(msgProtocol.keyMsg(pBuf, PRINT_BUF_SIZE, 
            kpSerial.getAddr(keyPad), kpSerial.getRecvMsgLen(), kpSerial.getRecvMsg(), msgType));
      }
      if (++keyPad >= numKeyPads)  // this was the last keypad with data
      {
        keyPad = numKeyPads = 0;
        keyPadRead = false;  // end keypad read mode
      }
      else
      {
        // still in keyPadRead mode
      }
    }
  }

  else // not in a keypad read cycle, check if time to poll keypad, send F7 msg, etc.
  {
    if (ms - lastSendTime > MIN_TX_GAP)  // min time gap between any type of msg pushed to keypads
    {
      // there is an implied priority scheme here.  Once the min time between transmits 
      // expires, look for the next thing to do in this order:
      //   1. push out a recv'd F7 msg
      //   2. poll the keypad (so key presses are responsive)
      //   3. push out a periodic F7 msg (no new F7 msg from RPi, just time to send one)
      //   4. sample the system voltage levels
            
      if (kpF7time == 0) // just received an F7 message from RPi, push it out
      {
        kpF7time = ms;
        kpSerial.write(msgProtocol.getF7(), msgProtocol.getF7size());
        lastSendTime = millis();
        // Serial.println("Sent NEW F7");
        msgProtocol.toneCheck();  // don't repeat single, double, or triple beeps
      }

      else if (ms - kpPollTime > KP_POLL_PERIOD)  // time to poll keypad
      {
        // Serial.println("Polling partyline");
        kpPollTime = ms;
        if (kpSerial.poll())
        {
          keyPadRead = true;
          keyPad = 0;  // start with first keypad that responded
          numKeyPads = kpSerial.getNumKeypads();
        }
        lastSendTime = millis();
      }

      else if (ms - kpF7time > KP_F7_PERIOD)  // time to send periodic F7 msg
      {
        kpF7time = ms;
        kpSerial.write(msgProtocol.getF7(), msgProtocol.getF7size());
        lastSendTime = millis();
      }
    }
    // Check for keypress timeout
    for(uint8_t i=0; i<8; i++){
      if (keyPressTime[i]){
        if (ms - keyPressTime[i] > KEYPRESS_TIMEOUT){
          client.publish(ALARM_COMMAND_TOPIC, keypadcode[i].c_str());
          Serial.print("Keypad press timeout "); 
          Serial.print(i+16);
          Serial.print(" code: ");
          Serial.println(keypadcode[i]);
          keypadcode[i] = "";
          keyPressTime[i] = 0;
        }
      }
    }
  }
}

char hex2char(uint8_t hex){
  switch(hex){
    case 0x00:
      return '0';
    case 0x01:
      return '1';
    case 0x02:
      return '2';
    case 0x03:
      return '3';
    case 0x04:
      return '4';
    case 0x05:
      return '5';
    case 0x06:
      return '6';
    case 0x07:
      return '7';
    case 0x08:
      return '8';
    case 0x09:
      return '9';
    case 0x0a:
      return '*';
    case 0x0b:
      return '#';
    case 0x0c:
      return 'a';
    case 0x0d:
      return 'b';
    case 0x0e:
      return 'c';
    case 0x1c:
      return 'A';
    case 0x1d:
      return 'B';
    case 0x1e:
      return 'C';
    case 0x1f:
      return 'D';
  }
  return ' ';
}

void codeCommands(uint8_t activeKeypad){
  Serial.println(keypadcode[activeKeypad-16].c_str());
  Serial.println(ALARM_CODE+"1");
  if(keypadcode[activeKeypad-16] == ALARM_CODE + "1"){  // CODE TO DISARM
    client.publish(ALARM_COMMAND_TOPIC, "DISARM");
    keypadcode[activeKeypad-16] = "";
    keyPressTime[activeKeypad-16] = 0;
  }
  else if(keypadcode[activeKeypad-16] == ALARM_CODE + "3"){  // CODE TO ARM_HOME
    client.publish(ALARM_COMMAND_TOPIC, "ARM_HOME");
    keypadcode[activeKeypad-16] = "";
    keyPressTime[activeKeypad-16] = 0;
  }
  else if(keypadcode[activeKeypad-16] == ALARM_CODE + "2"){  // CODE TO ARM_AWAY
    client.publish(ALARM_COMMAND_TOPIC, "ARM_AWAY");
    keypadcode[activeKeypad-16] = "";
    keyPressTime[activeKeypad-16] = 0;
  }
  else if(keypadcode[activeKeypad-16] == ALARM_CODE + "7"){  // CODE TO ARM_NIGHT
    client.publish(ALARM_COMMAND_TOPIC, "ARM_NIGHT");
    keypadcode[activeKeypad-16] = "";
    keyPressTime[activeKeypad-16] = 0;
  }
  else if(keypadcode[activeKeypad-16] == ALARM_CODE + "4"){  // CODE TO ARM_VACATION
    client.publish(ALARM_COMMAND_TOPIC, "ARM_VACATION");
    keypadcode[activeKeypad-16] = "";
    keyPressTime[activeKeypad-16] = 0;
  }
  else if(keypadcode[activeKeypad-16] == ALARM_CODE + "6"){  // CODE TO ARM_CUSTOM_BYPASS
    client.publish(ALARM_COMMAND_TOPIC, "ARM_CUSTOM_BYPASS");
    keypadcode[activeKeypad-16] = "";
    keyPressTime[activeKeypad-16] = 0;
  }
}
