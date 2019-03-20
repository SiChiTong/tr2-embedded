#ifndef GRIPPER_H
#define GRIPPER_H

#include "AX12A.h"

class Gripper {
  private:
    unsigned int _directionPin = 10u;
    unsigned long _baudRate = 1000000ul;
    unsigned int _id = 1u;
    int _speed = 100;

  public:
    Gripper() { };
    ~Gripper() { };

    void setUp(HardwareSerial *ser) {
      ax12a.begin(_baudRate, _directionPin, ser);
      //ax12a.setEndless(_id, ON);
    }

    void setUp(SoftwareSerial *ser) {
      ax12a.begin(_baudRate, _directionPin, ser);
      //ax12a.setEndless(_id, ON);
    }

    int read() {
      return ax12a.readPosition(_id);
    }

    void open() {
      ax12a.ledStatus(_id, OFF);
      //ax12a.turn(_id, RIGHT, _speed);
      ax12a.move(_id, 400);
    }

    void close() {
      ax12a.ledStatus(_id, ON);
      //ax12a.turn(_id, LEFT, _speed);
      ax12a.move(_id, 725);
    }

    
};

#endif
