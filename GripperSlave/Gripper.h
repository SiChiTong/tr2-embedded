#ifndef GRIPPER_H
#define GRIPPER_H

#define GRIPPER_CLOSED 0
#define GRIPPER_OPEN 1

#include "Motor.h"

class Gripper {
  private:
    Motor _motor = Motor(1, 13, 11, 12);
    int _state = GRIPPER_OPEN;

  public:
    Gripper() {
    }

    void setUp() {
      _motor.setUp();
    }

    int getState() {
      return _state;
    }

    void stop() {
      _motor.stop();
      delay(20);
    }

    void step() {
      _motor.executePreparedCommand();
    }

    void toggle() {
      if (_state == GRIPPER_OPEN) {
        close();
      } else {
        open();
      }
    }

    void open() {
      _motor.prepareCommand(100, 10000);
      _state = GRIPPER_OPEN;
    }

    void close() {
      _motor.prepareCommand(-100, 10000);
      _state = GRIPPER_CLOSED;
    }

    
};

#endif
