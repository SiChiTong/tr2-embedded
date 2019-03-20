
#define ESP_DEBUG 0
#define ACTUATOR_ID "a3"

#define PI 3.1415926535897932384626433832795
#define TAU (PI * 2)

#include <EEPROM.h>
#include <SoftwareSerial.h>
#include "Motor.h"
#include "Esp8266.h"
#include "ems22a.h"
#include "PID.h"
#include "fsr.h"

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

// EEPROM Addresses
int EAI2CAddress = 0;
int EAEncoderLap = 1;
int EAFsr1MinThreshold1 = 2;
int EAFsr1MinThreshold2 = 3;
int EAFsr1MaxThreshold1 = 4;
int EAFsr1MaxThreshold2 = 5;
int EAFsr2MinThreshold1 = 6;
int EAFsr2MinThreshold2 = 7;
int EAFsr2MaxThreshold1 = 8;
int EAFsr2MaxThreshold2 = 9;
int EAEncoderOffset1 = 10;
int EAEncoderOffset2 = 11;
int EAMotorP1 = 12;
int EAMotorP2 = 13;
int EAMotorP3 = 14;
int EAEmsP1 = 15;
int EAEmsP2 = 16;
int EAEmsP3 = 17;
int EAFsrDifMinTorque1 = 18;
int EAFsrDifMinTorque2 = 19;
int EAFsrDifMaxTorque1 = 20;
int EAFsrDifMaxTorque2 = 21;

// Set in EEPROM at time of set-up
int I2CBusAddress = 0;
int encoderLap = 0;
int encoderOffset = 0;
int fsr1MinThreshold = 0;
int fsr1MaxThreshold = 0;
int fsr2MinThreshold = 0;
int fsr2MaxThreshold = 0;
int fsr1MaxTorque = 0;
int fsr2MaxTorque = 0;

// Pins for components, can be overwritten in EEPROM
// EEPROM integration is in case of arduino-level pin failure
// while maintaining a single code-base for actuators
int MotorP1 = 5;
int MotorP2 = 6;
int MotorP3 = 7;
int EmsP1 = 8;
int EmsP2 = 9;
int EmsP3 = 10;
int Fsr1P1 = A6;
int Fsr2P1 = A7;

double pidPos, pidOut, pidGoal = 0;
double pidThreshold = 0;//0.003;
PID pid = PID(&pidPos, &pidOut, &pidGoal, 8.5, 7.0, 0.2, DIRECT);

SoftwareSerial ss(2, 3);
Esp8266 esp8266(&Serial);
Motor motor;
Ems22a ems22a;
FSR fsr1;
FSR fsr2;

int mode = MODE_ROTATE;
bool stopSoft = false;
bool stopEmergency = false;

long offsetBinary16 = 32768;
int fsrDifMin = -1023;
int fsrDifMax = 1023;

double formatAngle (double x) {
  if (x < -TAU) {
    x += TAU;
    return formatAngle(x);
  } else if (x > TAU) {
    x -= TAU;
    return formatAngle(x);
  } else {
    return x;
  }
}

volatile uint8_t msgId = 0;
volatile int msgErr = ERR_NONE;
volatile bool respondWithStatus = false;
volatile bool processingResponse = false;

void request (uint8_t packet[16]) {
  msgId = packet[0];

  if (packet[3] == CMD_SET_MODE) {
    stopSoft = false;
    mode = packet[4];

    if (mode == MODE_SERVO) {
      pidGoal = (double)ems22a.getAngleRadians();
    }
  } else if (packet[3] == CMD_SET_POS) {
    stopSoft = false;
    int param = packet[4] + packet[5] * 256;
    double pos = param / 65535.0 * TAU;
    pidGoal = formatAngle(pos);
  } else if (packet[3] == CMD_RESET_POS) {
    bool noOffset = true;
    int encoderLap = 0;
    int emsRead = ems22a.readPosition(noOffset);

    ems22a.setOffset(emsRead);
    ems22a.setLap(0);

    pidGoal = 0;
    pidOut = 0;
    pidPos = 0;
    pid.clear();

    EEPROM.write(EAEncoderLap, encoderLap);
    EEPROM.write(EAEncoderOffset1, emsRead % 256);
    EEPROM.write(EAEncoderOffset2, floor(emsRead / 256));
  } else if (packet[3] == CMD_ROTATE) {
    stopSoft = false;
    int offsetBinary = 128;
    int motorStep = packet[4] - offsetBinary;
    int motorDuration = packet[5] + packet[6] * 256;
    motor.prepareCommand(motorStep, motorDuration);
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

void setup() {
  Serial.begin(115200);

  I2CBusAddress = EEPROM.read(EAI2CAddress);
  encoderLap = EEPROM.read(EAEncoderLap);
  encoderOffset = EEPROM.read(EAEncoderOffset1) + EEPROM.read(EAEncoderOffset2) * 256;
  fsr1MinThreshold = EEPROM.read(EAFsr1MinThreshold1) + EEPROM.read(EAFsr1MinThreshold2) * 256;
  fsr1MaxThreshold = EEPROM.read(EAFsr1MaxThreshold1) + EEPROM.read(EAFsr1MaxThreshold2) * 256;
  fsr2MinThreshold = EEPROM.read(EAFsr2MinThreshold1) + EEPROM.read(EAFsr2MinThreshold2) * 256;
  fsr2MaxThreshold = EEPROM.read(EAFsr2MaxThreshold1) + EEPROM.read(EAFsr2MaxThreshold2) * 256;
  fsrDifMin = EEPROM.read(EAFsrDifMinTorque1) + (EEPROM.read(EAFsrDifMinTorque2) * 256) - offsetBinary16;
  fsrDifMax = EEPROM.read(EAFsrDifMaxTorque1) + (EEPROM.read(EAFsrDifMaxTorque2) * 256) - offsetBinary16;

  int mp1 = EEPROM.read(EAMotorP1);
  int mp2 = EEPROM.read(EAMotorP2);
  int mp3 = EEPROM.read(EAMotorP3);
  if (mp1 >= 2 && mp1 <= 13) MotorP1 = mp1;
  if (mp2 >= 2 && mp2 <= 13) MotorP2 = mp2;
  if (mp3 >= 2 && mp3 <= 13) MotorP3 = mp3;
  motor = Motor(1, MotorP1, MotorP2, MotorP3);

  int ep1 = EEPROM.read(EAEmsP1);
  int ep2 = EEPROM.read(EAEmsP2);
  int ep3 = EEPROM.read(EAEmsP3);
  if (ep1 >= 2 && ep1 <= 13) EmsP1 = ep1;
  if (ep2 >= 2 && ep2 <= 13) EmsP2 = ep2;
  if (ep3 >= 2 && ep3 <= 13) EmsP3 = ep3;
  ems22a = Ems22a(0, EmsP1, 0, EmsP2, 0, EmsP3);

  fsr1 = FSR(A6);
  fsr2 = FSR(A7);

  motor.setUp();

  ems22a.setUp();
  ems22a.setOffset(encoderOffset);
  ems22a.setLap(encoderLap);
  ems22a.step();

  fsr1.setUp();
  fsr1.setThreshold(fsr1MinThreshold, fsr1MaxThreshold);
  fsr1.setFilter(FSR_FILTER_THRESHOLD_AVG);

  fsr2.setUp();
  fsr2.setThreshold(fsr2MinThreshold, fsr2MaxThreshold);
  fsr2.setFilter(FSR_FILTER_THRESHOLD_AVG);

  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-1, 1);

  pidGoal = (double)ems22a.getAngleRadians();

  esp8266.ssid = "TR2_AN_123132321";
  esp8266.pass = "ALLEN65802";
#if ESP_DEBUG == 1
  esp8266.setDebugSerial(&ss);
#endif
  esp8266.setTimeout(600);
  esp8266.begin();
  esp8266.configure();

  delay(2000);
}

int getMotorValue() {
  int fsr1Read = fsr1.getRead();
  int fsr2Read = fsr2.getRead();
  int fsrDif = fsr1Read - fsr2Read;

  int motorValue = 0;
  float motorCoef = 10.0;
  motorValue = floor(fsrDif * motorCoef);

  int maxValue = 100;
  if (motorValue > maxValue) motorValue = maxValue;
  else if (motorValue < -maxValue) motorValue = -maxValue;

  return motorValue;
}

const int prevMotorN = 3;
int prevMotor[prevMotorN];
void modeBackdrive() {
  int motorValue = getMotorValue();

  for (int i = prevMotorN - 1; i > 0; i--) {
    prevMotor[i] = prevMotor[i - 1];
  }
  prevMotor[0] = motorValue;

  int avgMotor = 0;
  for (int i = 0; i < prevMotorN; i++) {
    avgMotor += prevMotor[i];
  }
  avgMotor = floor(avgMotor / prevMotorN);

  motor.step(motorValue);
}

void modeServo() {
  pidPos = (double)ems22a.getAngleRadians();

  // ignore if within 0.001 rad of goal -- good 'nuff
  if (abs(pidPos - pidGoal) >= pidThreshold) {
    double d = formatAngle(pidPos) - formatAngle(pidGoal);
    if (d > PI) {
      pidGoal = formatAngle(pidGoal);
      pidGoal += TAU;
    } else if (d < -PI) {
      pidPos = formatAngle(pidPos);
      pidPos += TAU;
    } else {
      pidGoal = formatAngle(pidGoal);
      pidPos = formatAngle(pidPos);
    }

    pid.Compute();
    motor.step(pidOut * 100.0);
  } else {
    //pid.clear();
  }
}

void modeRotate() {
  motor.executePreparedCommand();
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

void loop() {
  ems22a.step();
  fsr1.step();
  fsr2.step();

  esp8266.step(ACTUATOR_ID, ems22a.getAngleRadians());
  parseCommand(esp8266.getLastCommand());

  if (stopSoft == true) {
    motor.stop();
    delay(50);
    return;
  } else if (stopEmergency == true) {
    motor.stop();
    delay(500);
    return;
  }

  if (mode == MODE_BACKDRIVE) {
    modeBackdrive();
  } else if (mode == MODE_SERVO) {
    modeServo();
  } else if (mode == MODE_ROTATE) {
    modeRotate();
  }

  delay(10);
}
