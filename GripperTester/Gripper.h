#ifndef GRIPPER_H
#define GRIPPER_H

#define GRIPPER_CLOSED 0
#define GRIPPER_OPEN 1

#include "AX12A.h"

class Gripper {
  private:
    HardwareSerial* ser;
    unsigned int _directionPin = 10u;
    unsigned long _baudRate = 1000000ul;
    unsigned int _id = 1;
    int _speed = 1023;
    int _maxTorque = 1023;
    int _state = GRIPPER_OPEN;

  public:
    Gripper(HardwareSerial* serial) {
      ser = serial;
    }

    void setUp() {
      ser->begin(38400);
      ax12a.begin(_baudRate, _directionPin, ser);
      ax12a.torqueStatus(_id, true);
      ax12a.setEndless(_id, ON);
      ax12a.setMaxTorque(_id, _maxTorque);
    }

    int getState() {
      return _state;
    }

    int read() {
      return ax12a.readPosition(_id);
    }

    void toggle() {
      if (_state == GRIPPER_OPEN) {
        close();
      } else {
        open();
      }
    }

    void open() {
      ax12a.ledStatus(_id, OFF);
      ax12a.turn(_id, RIGHT, _speed);
      //ax12a.move(_id, 1023);
      _state = GRIPPER_OPEN;
      delay(20);
    }

    void close() {
      ax12a.ledStatus(_id, ON);
      ax12a.turn(_id, LEFT, _speed);
      //ax12a.move(_id, 725);
      _state = GRIPPER_CLOSED;
      delay(20);
    }

    
};

#endif
