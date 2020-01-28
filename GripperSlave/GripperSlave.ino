
#define ACTUATOR_ID "g0"

#define TR2_AN_SSID "TR2_AN_111222333"
#define TR2_AN_PASS "MATHI78741"

#define PI 3.1415926535897932384626433832795
#define TAU (PI * 2)

#include <Wire.h>
#include "Gripper.h"
#include "Esp8266.h"

#define CMD_SET_MODE 0x10
#define CMD_SET_POS 0x11
#define CMD_RESET_POS 0x12
#define CMD_ROTATE 0x13
#define CMD_RETURN_STATUS 0x14
#define CMD_STOP_RELEASE 0x15
#define CMD_STOP_EMERGENCY 0x16

#define MODE_SERVO 0x10
#define MODE_BACKDRIVE 0x11
#define MODE_ROTATE 0x12

int btnPin = 5;

HardwareSerial Serial1((uint32_t)0, (uint32_t)1);
Esp8266 esp8266(&Serial1);

Gripper gripper;

int mode = MODE_ROTATE;
bool stopSoft = false;
bool stopEmergency = false;

volatile bool flagGripper = false;
volatile bool gripperOpen = true;

void request (uint8_t packet[16]) {
  uint8_t msgId = packet[0];
  uint8_t msgCmd = packet[3];

  if (msgCmd == CMD_SET_MODE) {
    uint8_t msgMode = packet[4];
    mode = msgMode;
  } else if (packet[3] == CMD_SET_POS) {
    int param = packet[4] + packet[5] * 256;
    if (param >= 1) {
      gripper.open();
    } else {
      gripper.close();
    }
  } else if (packet[3] == CMD_RESET_POS) {

  } else if (packet[3] == CMD_ROTATE) {
  
  } else if (packet[3] == CMD_STOP_RELEASE) {
    stopEmergency = false;
  } else if (packet[3] == CMD_STOP_EMERGENCY) {
    stopEmergency = true;
  }
}
  
void setup () {
  Serial.begin(115200);
  Serial.println("Starting...");
  
  pinMode(btnPin, INPUT_PULLDOWN);
  
  gripper.setUp();
  gripper.stop();

  esp8266.ssid = TR2_AN_SSID;
  esp8266.pass = TR2_AN_PASS;
  esp8266.setDebugSerial(&Serial);
  esp8266.setTimeout(600);
  esp8266.begin();
  esp8266.configure();
}


void parseCommand(char* cmd) {
  int bufIdx = 0;
  char buf[16];
  uint8_t packet[16];
  int packetIdx = 0;

  // no command
  if (strstr(cmd, "nc;")) {
    return;
  }

  for (int i = 0; i < strlen(cmd); i++) {
    char c = cmd[i];
    if (c != ',') {
      buf[bufIdx] = c;
      bufIdx++;
    } else {
      buf[bufIdx] = '\0';
      bufIdx = 0;
      packet[packetIdx] = atoi(buf);
      packetIdx++;
    }
  }

  request(packet);
}

long lastRead = 0;
void loop () {
  gripper.step();
  
  esp8266.step(ACTUATOR_ID, gripper.getState());
  parseCommand(esp8266.getLastCommand());
  esp8266.clearCmd();
  
  //5, 4, 3, 2
  int btnPressed = digitalRead(btnPin);
  if (btnPressed && (millis() - lastRead) > 250) {
    gripper.toggle();
    lastRead = millis();
  }
} 
