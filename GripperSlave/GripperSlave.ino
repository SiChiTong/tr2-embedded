#define PI 3.1415926535897932384626433832795
#define TAU (PI * 2)

#include <SoftwareSerial.h>
#include <Wire.h>
#include "Gripper.h"

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

SoftwareSerial ss(2, 3);

Gripper gripper;

bool stopSoft = false;
bool stopEmergency = false;

int BusAddress = 0x30;

volatile uint8_t msgId = 0;
volatile int msgErr = ERR_NONE;
volatile bool respondWithStatus = false;
volatile bool processingResponse = false;

volatile bool flagGripper = false;
volatile bool gripperOpen = true;

void response() {
  if (msgErr == ERR_NONE) {
    uint8_t packet[16];
    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = msgId;
    packet[3] = 4;
    packet[4] = RES_OK;

    if (respondWithStatus == true) {
      float r = 0;
      if (gripperOpen == true) {
        r = TAU;
      } else {
        r = 0;
      }
      
      long x = r / TAU * 65535.0;
      packet[4] = RES_OK_POS;
      packet[5] = floor(x % 256);
      packet[6] = floor(x / 256);

      packet[3] = 6;

      respondWithStatus = false;
    }

    int checksum = 0;
    for (int i = 2; i < packet[3] + 1; i++) {
      checksum += packet[i];
    }
    packet[packet[3] + 1] = floor(checksum % 256);
    Serial.write(packet, packet[3] + 2);
  } else {
    uint8_t packet[16];
    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = msgId;
    packet[3] = 5;
    packet[4] = RES_ERR;
    packet[5] = msgErr;

    int checksum = 0;
    for (int i = 2; i < packet[3] + 1; i++) {
      checksum += packet[i];
    }
    packet[4] = floor(checksum % 256);
    Serial.write(packet, packet[3] + 2);
  }
  
  msgErr = ERR_NONE;
}

bool checkSum (uint8_t packet[16]) {
  int len = packet[2];
  int checksum = 0;
  for (int i = 0; i < len - 1; i++) {
    checksum += packet[i];
  }
  return (floor(checksum % 256) == packet[len - 1]);
}

void request (uint8_t packet[16]) {
  msgId = packet[0];
  
  if (checkSum(packet) == false) {
    msgErr = ERR_CHECKSUM;
  } else if (packet[3] == CMD_SET_MODE) {
    
  } else if (packet[3] == CMD_SET_POS) {
    int param = packet[4] + packet[5] * 256;
    double pos = param / 65535.0 * PI * 2.0;
    flagGripper = true;
    if (pos < 0) {
      gripperOpen = false;
    } else {
      gripperOpen = true;
    }
  } else if (packet[3] == CMD_RESET_POS) {
    
  } else if (packet[3] == CMD_ROTATE) {
    
  } else if (packet[3] == CMD_RETURN_STATUS) {
    respondWithStatus = true;
  } else if (packet[3] == CMD_STOP) {
    stopSoft = true;
  } else if (packet[3] == CMD_STOP_EMERGENCY) {
    stopEmergency = true;
  } else {
    msgErr = ERR_CMD_BOUNDS;
  }
}

static uint8_t startByte = 0xFF;
volatile bool packetStart = false;
volatile uint8_t startByteCount = 0;
volatile uint8_t packetIndex = 0;
volatile uint8_t packet[16];

static bool packetCheckSum () {
  uint8_t len = packet[2];
  int checksum = 0;
  for (int i = 0; i < len - 1; i++) {
    checksum += packet[i];
  }
  return (floor(checksum % 256) == packet[len - 1]);
}

static void handleRxEvent (uint8_t c) {
  if (packetStart == false) {
    if (c == startByte) {
      startByteCount++;
      if (startByteCount >= 2) {
        packetStart = true;
        packetIndex = 0;
      }
    } else {
      startByteCount = 0;
      packetStart = false;
    }
  } else {
    packet[packetIndex] = c;
    if (packetIndex < 2) {
      packetIndex++;
    } else if (packetIndex + 1 >= packet[2]) {
      if (packet[1] == BusAddress) {
        if (packetCheckSum()) {
          processingResponse = true;
          request(packet);
          response();
          processingResponse = false;
        } else {
          processingResponse = true;
          msgErr = ERR_CHECKSUM;
          response();
          processingResponse = false;
        }
      }
      packetStart = false;
      startByteCount = 0;
      packetIndex = 0;
    } else {
      packetIndex++;
    }
  }
}

void setup () {
  Serial.begin(38400);
  ss.begin(1000000ul);
  gripper.setUp(&ss);
  //gripper.setUp(&Serial);
}

void loop () {
  while (Serial.available()) {
    handleRxEvent(Serial.read());
  }
  
  if (flagGripper == true) {
    flagGripper = false;
    if (gripperOpen == true) {
      gripper.open();
      delay(50);
    } else {
      gripper.close();
      delay(50);
    }
  }

  delay(50);
} 
