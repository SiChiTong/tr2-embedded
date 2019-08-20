#include <EEPROM.h>
#include "Motor.h"
#include "ems22a.h"

// EEPROM Addresses
int EAI2CAddress = 0;
int EAEncoderLap = 1;
int EAEncoderOffset1 = 10;
int EAEncoderOffset2 = 11;
int EAMotorP1 = 12;
int EAMotorP2 = 13;
int EAMotorP3 = 14;
int EAEmsP1 = 15;
int EAEmsP2 = 16;
int EAEmsP3 = 17;

// Set in EEPROM at time of set-up
int I2CBusAddress = 0;
int encoderLap = 0;
int encoderOffset = 0;

// Pins for components, can be overwritten in EEPROM
// EEPROM integration is in case of arduino-level pin failure
// while maintaining a single code-base for actuators
int MotorP1 = 9;
int MotorP2 = 11;
int MotorP3 = 10;
int EmsP1 = 12;
int EmsP2 = 6;
int EmsP3 = 5;

Motor motor;
Ems22a ems22a;
Ems22a ems22a_pos;

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

  I2CBusAddress = EEPROM.read(EAI2CAddress);
  encoderLap = EEPROM.read(EAEncoderLap);
  encoderOffset = EEPROM.read(EAEncoderOffset1) + EEPROM.read(EAEncoderOffset2) * 256;
  
  int mp1 = EEPROM.read(EAMotorP1);
  int mp2 = EEPROM.read(EAMotorP2);
  int mp3 = EEPROM.read(EAMotorP3);
  if (mp1 >= 2 && mp1 <= 13) MotorP1 = mp1;
  if (mp2 >= 2 && mp2 <= 13) MotorP2 = mp2;
  if (mp3 >= 2 && mp3 <= 13) MotorP3 = mp3;
  motor = Motor(1, MotorP1, MotorP2, MotorP3);
  //motor = Motor(1, 3, 5, 4);

  int ep1 = EEPROM.read(EAEmsP1);
  int ep2 = EEPROM.read(EAEmsP2);
  int ep3 = EEPROM.read(EAEmsP3);
  if (ep1 >= 2 && ep1 <= 13) EmsP1 = ep1;
  if (ep2 >= 2 && ep2 <= 13) EmsP2 = ep2;
  if (ep3 >= 2 && ep3 <= 13) EmsP3 = ep3;
  ems22a = Ems22a(0, 12, 0, 6, 0, 5);
  ems22a_pos = Ems22a(0, 4, 0, 3, 0, 2);

  motor.setUp();
  ems22a.setUp();
  ems22a_pos.setUp();
  
  ems22a.setOffset(encoderOffset);
  ems22a.setLap(encoderLap);
  ems22a_pos.setOffset(encoderOffset);
  ems22a_pos.setLap(encoderLap);
  
  ems22a.step();
  ems22a_pos.step();

  Serial.println("hello world");
  bool TestMode = false;
}
bool TestMode;
void Test(){
  int e_pos = ems22a_pos.readPosition();
  int e = ems22a.readPosition();
 Serial.print("ACT_ENC_POS: ");
  Serial.print(e_pos);
  Serial.print(", TRQ_ENC_POS: ");
  Serial.print(e);
  Serial.println( "   TestMode Activated");
  delay(1000);TestMode = true;
    delay(50);
    int hold = ems22a_pos.readPosition();
    motor.step(100);
    delay(10); motor.step(100);delay(10);motor.step(100);
    delay(250);
    motor.step(0);
    if (hold < ems22a_pos.readPosition()){
      Serial.println("Motor Correct");
      Serial.print("ACT_ENC_POS: ");
  Serial.print(ems22a_pos.readPosition());
  Serial.print(", TRQ_ENC_POS: ");
  Serial.print(ems22a.readPosition());
    }
    if (hold > ems22a_pos.readPosition()){
      Serial.println("Motor Incorrect");
      Serial.print("ACT_ENC_POS: ");
      Serial.print(ems22a_pos.readPosition());
      Serial.print(", TRQ_ENC_POS: ");
      Serial.print(ems22a.readPosition());
      motor.flipDrivePins();

    }
  
  
  }
  

  void TestOff(){TestMode = false;
  { Serial.println("TestMode Deactivated");}}





void loop() {
  int encMin = 600;
  int encMax = 640;
  
  int dzMin = encMin + 10;
  int dzMax = encMax - 10;

  if (encMin > encMax) {
    encMax += 1020;
  }

  if (dzMin > dzMax) {
    dzMax += 1020;
  }

  int azLow = dzMin - encMin;
  int azHigh = encMax - dzMax;
  
  int e_pos = ems22a_pos.readPosition();
  
  int e = ems22a.readPosition();
  int r = e;

  if (encMin > 610 && r < encMin - 15) {
    r += 1020;
  }
  
  if (r < dzMin) r -= dzMin;
  else if (r > dzMax) r -= dzMax;
  else r = 0;
  
  float frictionCoef = 3.0;

  float encToDistRatio = 1.5 / 9.0;
  float springDistFromAxis = 47.5; // milimeters
  float springRate = 17.5; // Newtons
  
  float dist = r * encToDistRatio; // millimeters
  float force = springRate * dist * frictionCoef; // Newtons
  float torque = force * springDistFromAxis / 1000.0; // Newton-meters

  int m = 0;

  if (r > 0) m = r * 1.0 / azHigh * 100.0;
  else if (r < 0) m = r * 1.0 / azLow * 100.0;

  if (m > 100) m = 100;
  if (m < -100) m = -100;
  
  motor.step(m);

  while (Serial.available()) {
    char x = Serial.read();
    if(x == 64) {
    Test();}
    if(x == 35) {
    TestOff();}
    }
   
  if (TestMode == true){
  
  /*Serial.print("ACT_ENC_POS: ");
  Serial.print(e_pos);
  Serial.print(", TRQ_ENC_POS: ");
  Serial.print(e);
  Serial.print(", MTR: ");
  Serial.println(m);
  /*Serial.print(ems22a.readPosition());
  Serial.print(" // ");
  Serial.print(fsr1.getRead());
  Serial.print(" // ");
  Serial.println(fsr2.getRead());*/
  delay(50);}}
