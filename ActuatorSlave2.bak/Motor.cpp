#include <Arduino.h>
#include "Motor.h"

Motor::Motor() {

}

Motor::Motor(int id, int pinEnable, int pinDrive1, int pinDrive2) {
  // motorSpeed set to a percentage of max voltage to pins
  this->motorSpeed = 0;
  this->id = id;
  this->pinEnable = pinEnable;
  this->pinDrive1 = pinDrive1;
  this->pinDrive2 = pinDrive2;
}

Motor::~Motor() {
  this->stop();
}

void Motor::setUp() {
  pinMode(pinEnable, OUTPUT);
  pinMode(pinDrive1, OUTPUT);
  pinMode(pinDrive2, OUTPUT);

  setPinSpeed();
}

void Motor::setPinSpeed() {
  if (motorSpeed > maxSpeed) {
    motorSpeed = maxSpeed;
  }
  
  int pinStrength = int(255.0 * (motorSpeed / 100.0));
  analogWrite(pinEnable, pinStrength);
}

void Motor::forward(int speed = 100) {
  // given speed should be a percentage of total speed so that we can tell it to start slowly
  this->motorSpeed = speed;
  if (speed < minSpeed) {
    analogWrite(pinEnable, LOW);
    digitalWrite(pinDrive1, LOW);
    digitalWrite(pinDrive2, LOW);
  } else {
    setPinSpeed();
    digitalWrite(pinDrive1, HIGH);
    digitalWrite(pinDrive2, LOW);
  }
}

void Motor::backward(int speed = 100) {
  this->motorSpeed = speed;
  if (speed < minSpeed) {
    analogWrite(pinEnable, LOW);
    digitalWrite(pinDrive1, LOW);
    digitalWrite(pinDrive2, LOW);
  } else {
    setPinSpeed();
    digitalWrite(pinDrive1, LOW);
    digitalWrite(pinDrive2, HIGH);
  }
}

void Motor::step(int speed = 100) {
  if (speed > 0) {
    forward(abs(speed));
  } else if (speed < 0) {
    backward(abs(speed));
  } else {
    stop();
  }
}

void Motor::stop() {
  this->motorSpeed = 0;
  analogWrite(pinEnable, LOW);
  digitalWrite(pinDrive1, LOW);
  digitalWrite(pinDrive2, LOW);
}

bool Motor::isFlagged() {
  return flagExecute;
}

void Motor::clearPreparedCommand() {
  flagExecute = false;
  flagExecuteSpeed = 0;
  flagExecuteDuration = 0;
  flagExecuteExpiration = millis() - 2000;
}

void Motor::prepareCommand(int motorSpeed, int duration) {
  flagExecute = true;
  flagExecuteSpeed = motorSpeed;
  flagExecuteDuration = duration;
  flagExecuteExpiration = millis() + duration;

  if (flagExecuteExpiration > millis() + 1000) {
    flagExecuteExpiration = millis() + 1000;
  }

  lastPreparedCommand[0] = motorSpeed;
  lastPreparedCommand[1] = duration;
}

void Motor::executePreparedCommand() {
  if (flagExecute == true && millis() < flagExecuteExpiration) {
    this->step(flagExecuteSpeed);
  } else if (millis() > flagExecuteExpiration) {
    this->step(0);
    flagExecute = false;
  } else {
    this->step(0);
  }
}

void Motor::getLastMotorCommand(int *cmd) {
  cmd[0] = lastPreparedCommand[0];
  cmd[1] = lastPreparedCommand[1];
}
