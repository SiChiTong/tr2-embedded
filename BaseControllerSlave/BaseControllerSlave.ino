
#define ACTUATOR_ID "b0"

#define TR2_AN_SSID "TR2_AN_111222333"
#define TR2_AN_PASS "MATHI78741"

#define PI 3.1415926535897932384626433832795
#define TAU (PI * 2)

#include "Motor.h"
#include "Esp8266.h"

#define CMD_SET_MODE 0x10
#define CMD_SET_POS 0x11
#define CMD_RESET_POS 0x12
#define CMD_ROTATE 0x13
#define CMD_RETURN_STATUS 0x14
#define CMD_STOP 0x15
#define CMD_STOP_EMERGENCY 0x16

#define MODE_SERVO 0x10
#define MODE_BACKDRIVE 0x11
#define MODE_ROTATE 0x12

#define RES_OK 0x20
#define RES_ERR 0x21
#define RES_OK_POS 0x22
#define RES_OK_TRQ 0x23

#define ERR_NONE 0x00
#define ERR_CHECKSUM 0x01 // checksum in msg doesn't fit
#define ERR_CMD_BOUNDS 0x02 // cmd doesn't match known
#define ERR_PARAM_BOUNDS 0x03 // param doesn't make sense
#define ERR_LENGTH 0x04 // msg length not equal to stated
#define ERR_NO_RESPONSE 0x05
#define ERR_I2C_BUS 0x06
#define ERR_OTHER 0xFF

int MotorLP1 = 3;
int MotorLP2 = 4;
int MotorLP3 = 2;
int MotorLE1 = A0;
int MotorLE2 = A1;
int MotorRP1 = 5;
int MotorRP2 = 9;
int MotorRP3 = 6;
int MotorRE1 = A2;
int MotorRE2 = A3;
int S0P1 = A6;

long vel_timeout = 0;
int vel_x = 0;
int vel_y = 0;

HardwareSerial Serial1((uint32_t)0, (uint32_t)1);
Esp8266 esp8266(&Serial1);
Motor motorLeft;
Motor motorRight;

int mode = MODE_ROTATE;
bool stopSoft = false;
bool stopEmergency = false;

volatile uint8_t msgId = 0;
volatile int msgErr = ERR_NONE;
volatile bool respondWithStatus = false;
volatile bool processingResponse = false;

void request (uint8_t packet[16]) {
  msgId = packet[0];

  if (packet[3] == CMD_SET_MODE) {
    stopSoft = false;
    mode = packet[4];
  } else if (packet[3] == CMD_SET_POS) {

  } else if (packet[3] == CMD_RESET_POS) {

  } else if (packet[3] == CMD_ROTATE) {
    stopSoft = false;
    int offsetBinary = 128;
    int motorLeftStep = packet[4] - 100;
    int motorRightStep = packet[5] - 100;
    int motorDuration = packet[5] + packet[6] * 256;

    if (motorDuration > 1000) {
      motorDuration = 1000;
    }

    motorLeft.prepareCommand(motorLeftStep, 250);
    motorRight.prepareCommand(motorRightStep, 250);
  } else if (packet[3] == CMD_RETURN_STATUS) {
    respondWithStatus = true;
  } else if (packet[3] == CMD_STOP) {
    stopSoft = true;
  } else if (packet[3] == CMD_STOP_EMERGENCY) {
    stopEmergency = true;
  } else {
    stopSoft = true;
    msgErr = ERR_CMD_BOUNDS;
  }
}

volatile int motorLeftPos = 0;
volatile int motorRightPos = 0;

void MotorLEncChange() {
  if (digitalRead(MotorLE2) == HIGH) {
    motorLeftPos++;
  } else {
    motorLeftPos--;
  }
}
void MotorREncChange() {
  if (digitalRead(MotorRE2) == HIGH) {
    motorRightPos++;
  } else {
    motorRightPos--;
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(S0P1, INPUT);
  pinMode(MotorLE1, INPUT);
  pinMode(MotorLE2, INPUT);
  pinMode(MotorRE1, INPUT);
  pinMode(MotorRE2, INPUT);

  attachInterrupt(MotorLE1, MotorLEncChange, RISING);
  attachInterrupt(MotorRE1, MotorREncChange, RISING);

  motorLeft = Motor(1, MotorLP1, MotorLP2, MotorLP3);
  motorRight = Motor(2, MotorRP1, MotorRP2, MotorRP3);

  motorLeft.setUp();
  motorRight.setUp();

  Serial.println ("Beginning esp8266 config...");
  
  esp8266.ssid = TR2_AN_SSID;
  esp8266.pass = TR2_AN_PASS;
  esp8266.setDebugSerial(&Serial);
  esp8266.setTimeout(600);
  esp8266.begin();
  esp8266.configure();

  delay(2000);
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

void modeRotate() {
  if (motorLeft.isFlagged()) {
    motorLeft.executePreparedCommand();
  } else {
    motorLeft.stop();
  }

  if (motorRight.isFlagged()) {
    motorRight.executePreparedCommand();
  } else {
    motorRight.stop();
  }
}

long lastS0Update = 0;
long s0UpdateDelay = 15000;

void loop() {
  esp8266.step(ACTUATOR_ID, 0);
  parseCommand(esp8266.getLastCommand());

  if (millis() - lastS0Update > s0UpdateDelay) {
    float R1 = 30000.0;
    float R2 = 7500.0;
    int vSensor = analogRead(S0P1);
    float vAnalog = (vSensor * 3.3) / 1024.0;
    float vBattery = vAnalog / (R2 / (R1 + R2));
  
    esp8266.resetTimeout();
    esp8266.step("s0", vBattery);
    lastS0Update = millis();
  }

  if (stopSoft == true) {
    motorLeft.stop();
    motorRight.stop();
    delay(50);
    return;
  } else if (stopEmergency == true) {
    motorLeft.stop();
    motorRight.stop();
    delay(500);
    return;
  }

  modeRotate();
}
