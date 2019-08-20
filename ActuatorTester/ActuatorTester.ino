#include <EEPROM.h>
#include "Motor.h"
#include "ems22a.h"

// Set in EEPROM at time of set-up
int I2CBusAddress = 0;
int encoderLap = 0;
int encoderOffset = 0;
int encMid = 0;

// Pins for components, can be overwritten in EEPROM
// EEPROM integration is in case of arduino-level pin failure
// while maintaining a single code-base for actuators
int MotorP1 = 9;
int MotorP2 = 10;
int MotorP3 = 11;
int EmsP1 = 4;
int EmsP2 = 3;
int EmsP3 = 2;
int EmsTP1 = 12;
int EmsTP2 = 6;
int EmsTP3 = 5;

Motor motor;
Ems22a ems22a;
Ems22a ems22aT;

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
  Serial.println("Starting...");
  
  motor = Motor(0, MotorP1, MotorP2, MotorP3);
  motor.stop();
  delay(50);
  
  ems22a = Ems22a(0, EmsP1, 0, EmsP2, 0, EmsP3);
  ems22aT = Ems22a(0, EmsTP1, 0, EmsTP2, 0, EmsTP3);

  motor.setUp();

  ems22a.setUp();
  ems22a.step();
  
  ems22aT.setUp();
  ems22aT.setOffset(encMid);
  ems22aT.step();

  //config();

  Serial.println("Ready...");
}

void config () {
  Serial.println("Center torque sensors on actuator in 5...");
  delay(1000);
  Serial.println("4...");
  delay(1000);
  Serial.println("3...");
  delay(1000);
  Serial.println("2...");
  delay(1000);
  Serial.println("1...");
  delay(1000);

  int read1 = ems22aT.readPosition(true);
  delay(50);
  int read2 = ems22aT.readPosition(true);
  delay(50);
  int read3 = ems22aT.readPosition(true);
  delay(50);

  encMid = floor((read1 + read2 + read3) / 3.0);

  Serial.print("Encoder mid position set to ");
  Serial.println(encMid);
  ems22aT.setOffset(encMid);
  
  motor.step(0);
  delay(500);

  Serial.println("Configuring motor directionality...");

  delay(2000);
  
  int e0 = ems22a.readPosition(true);
  float enc_0 = PI * 2.0 * e0 / 1020.0;
  Serial.print("e0: ");
  Serial.print(e0);
  delay(50);
  
  motor.step(100);
  delay(1500);
  motor.step(0);
  delay(500);
  
  int er = ems22a.readPosition(true);
  float enc_r = PI * 2.0 * er / 1020.0;
  Serial.print(", er: ");
  Serial.print(er);
  delay(50);
  
  motor.step(-100);
  delay(3000);
  motor.step(0);
  delay(500);

  int el = ems22a.readPosition(true);
  float enc_l = PI * 2.0 * el / 1020.0;
  Serial.print(", el: ");
  Serial.print(el);
  delay(50);
  
  motor.step(100);
  delay(1500);
  motor.step(0);
  delay(500);
  
  float d_r = atan2(sin(enc_r-enc_0), cos(enc_r-enc_0));
  float d_l = atan2(sin(enc_l-enc_0), cos(enc_l-enc_0));

  Serial.print(", dr: ");
  Serial.print(d_r);
  Serial.print(", dl: ");
  Serial.println(d_l);

  if (d_r > 0.25 && d_l < 0.25) {
    int p2 = MotorP2;
    MotorP2 = MotorP3;
    MotorP3 = p2;
    motor = Motor(1, MotorP1, MotorP2, MotorP3);
  
    Serial.println("Config completed...");
  } else if (d_r < 0.25 && d_l > 0.25) {
    Serial.println("Config completed....");
  } else {
    Serial.println("Could not determine motor directionality. Retrying...");
    config();
  }
}

void loop() {
  int e_pos = ems22a.readPosition();
  int e_trq = ems22aT.readPosition();
  if (e_trq > 510) e_trq = e_trq - 1020;
  float t_dif = e_trq * 0.0980;
  if (t_dif < 0) t_dif = e_trq * 0.0925;
  float a_pos = (e_pos / 1020.0 * PI * 2.0) + (t_dif / 360.0 * PI * 2);
  
  Serial.print(ems22a.readPosition());
  Serial.print(", ");
  Serial.print(ems22aT.readPosition());
  Serial.print(", ");
  Serial.print(t_dif);
  Serial.print(", ");
  Serial.print(a_pos);
  Serial.print(", ");

  int m = t_dif * -35.0;

  if (m > 100) {
    m = 100;
  } else if (m < -100) {
    m = -100;
  }

  Serial.println(m);
  
  motor.step(m);
  delay(50);
}
