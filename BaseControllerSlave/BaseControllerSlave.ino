#include <Wire.h>
#include "Motor.h"
#include "Interrupt.h"
#include "Encoder.h"

#define CMD_SET_MODE 0x10
#define CMD_SET_POS 0x11
#define CMD_RESET_POS 0x12
#define CMD_ROTATE 0x13
#define CMD_RETURN_STATUS 0x14
#define CMD_SET_FREQUENCY 0x15

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
#define ERR_OTHER 0xFF

// front-left, front-right, etc -- from robot's perspective
// Motor::Motor(int id, int pinEnable, int pinDrive1, int pinDrive2);
Motor motorL(6, 4, 6, 7);
Motor motorR(7, 5, 8, 9);
Encoder encoderL(6, 2, 22);
Encoder encoderR(7, 3, 23);

union ArrayToInteger {
  byte array[2];
  uint16_t integer;
};

int pins[] = {32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47};

int lpulses = 0;
int rpulses = 0;
volatile uint8_t msgId = 0;
volatile int msgErr = ERR_NONE;
volatile bool respondWithStatus = false;

void requestEvent() {
  if (msgErr == ERR_NONE) {
    int packet[8];
    packet[0] = msgId; 
    packet[1] = 4;
    packet[2] = RES_OK;

    if (respondWithStatus == true) {
      respondWithStatus = false;
    }

    int checksum = 0;
    for (int i = 0; i < packet[1] - 1; i++) {
      checksum += packet[i];
    }
    packet[packet[1] - 1] = floor(checksum % 256);

    for (int i = 0; i < packet[1]; i++) {
      Wire.write(packet[i]);
    }
  } else {
    int packet[8];
    packet[0] = msgId;
    packet[1] = 5;
    packet[2] = RES_ERR;
    packet[3] = msgErr;

    int checksum = 0;
    for (int i = 0; i < packet[1] - 1; i++) {
      checksum += packet[i];
    }
    packet[4] = floor(checksum % 256);

    for (int i = 0; i < packet[1]; i++) {
      Wire.write(packet[i]);
    }
  }
  
  msgErr = ERR_NONE;
}

bool checkSum (int packet[8]) {
  int len = packet[1];
  int checksum = 0;
  for (int i = 0; i < len - 1; i++) {
    checksum += packet[i];
  }
  return (floor(checksum % 256) == packet[len - 1]);
}

void receiveEvent(int howMany) {
  int i = 0;
  int packet[8];
  while (Wire.available()) {
    if (i < 16) {
      packet[i] = Wire.read();
    } else {
      Wire.read();
    }
    i++;
  }

  msgId = packet[0];
  
  if (checkSum(packet) == false) {
    Serial.println("checksum error");
    msgErr = ERR_CHECKSUM;
    Serial.println(msgErr);
  } else if (packet[2] == CMD_SET_MODE) {
    
  } else if (packet[2] == CMD_SET_POS) {
    int param = packet[3] + packet[4] * 256;
    double pos = param / 65535.0 * PI * 2.0;
  } else if (packet[2] == CMD_RESET_POS) {
    
  } else if (packet[2] == CMD_ROTATE) {
    int offsetBinary = 100;
    int ml = packet[3] - offsetBinary;
    int mr = packet[4] - offsetBinary;
    int dur = packet[5] + packet[6] * 256;
    motorL.prepareCommand(ml, dur);
    motorR.prepareCommand(mr, dur);
    
  } else if (packet[2] == CMD_RETURN_STATUS) {
    respondWithStatus = true;
  } else if (packet[2] == CMD_SET_FREQUENCY) {
    
  } else {
    Serial.println("Command 0x");
    Serial.print(packet[2], HEX);
    Serial.println(" out of bounds");
    msgErr = ERR_CMD_BOUNDS;
  }
}

void setup() {
  Wire.begin(0x70);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);
  Serial.begin(115200);
  Serial.println("Ready");

  motorL.setUp();
  motorR.setUp();
  encoderL.setUp(interruptL);
  encoderR.setUp(interruptR);
}

void loop() {
  // these execute if a command has been flagged/prepared
  motorL.executePreparedCommand();
  motorR.executePreparedCommand();

  lpulses = interruptL_pulses;
  rpulses = interruptR_pulses;
}
