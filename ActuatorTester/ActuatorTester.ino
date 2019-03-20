#include <EEPROM.h>
#include "Motor.h"
#include "ems22a.h"
#include "fsr.h"

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

Motor motor;
Ems22a ems22a;
FSR fsr1;
FSR fsr2;

long offsetBinary16 = 32768;
int fsrDifMin = -1023;
int fsrDifMax = 1023;

int recoveryDif = 0;

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

void setup() {
  Serial.begin(115200);

  //bodge();

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
  fsr1.setUp();
  fsr2.setUp();

  fsr1.setThreshold(fsr1MinThreshold, fsr1MaxThreshold);
  fsr2.setThreshold(fsr2MinThreshold, fsr2MaxThreshold);

  fsr1.setFilter(FSR_FILTER_THRESHOLD_AVG);
  fsr2.setFilter(FSR_FILTER_THRESHOLD_AVG);

  ems22a.setOffset(encoderOffset);
  ems22a.setLap(encoderLap);
  
  ems22a.step();
}

void loop() {
  motor.step(-100);
  fsr1.step();
  fsr2.step();
  
  Serial.print(ems22a.readPosition());
  Serial.print(" // ");
  Serial.print(fsr1.getRead());
  Serial.print(" // ");
  Serial.println(fsr2.getRead());
  delay(500);
}
