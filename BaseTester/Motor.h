#ifndef MOTOR_H
#define MOTOR_H

class Motor {
  private:
    int pinEnable;
    int pinDrive1;
    int pinDrive2;
    int encoderTickCount;
    int previousEncoderTickCount;
    int getPreviousEncoderTickCount();
    bool flagExecute = false;
    int flagExecuteSpeed = 0;
    int flagExecuteDuration = 0;
    int lastPreparedCommand[2] = {0, 0};
    int motorDirection = 0;
    unsigned long flagExecuteExpiration = millis();
    void setPinSpeed();
    int minSpeed = 18;
  
  public:
    int id;
    int motorSpeed;
    bool isMovingForward = false;
    bool isMovingBackward = false;
    void setUp();
    void forward(int);
    void backward(int);
    void step(int);
    void stop();
    void changeMotorDirection();
    int getEncoderTickCount();
    int getEncoderTickCountDelta();
    void resetEncoderTickCount();
    void incrementEncoderTickCount();
    void decrementEncoderTickCount();
    void getLastMotorCommand(int *cmd);
    bool isFlagged();
    void prepareCommand(int, int);
    void executePreparedCommand();
    void clearPreparedCommand();
    Motor();
    Motor(int, int, int, int);
    ~Motor();
};

#endif
