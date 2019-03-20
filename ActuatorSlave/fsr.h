#ifndef FSR_H
#define FSR_H

#define FSR_MIN_THRESHOLD 0
#define FSR_MAX_THRESHOLD 1023
#define FSR_FILTER_NONE 1
#define FSR_FILTER_THRESHOLD 2
#define FSR_FILTER_THRESHOLD_AVG 3

class FSR {
  private:
    int PIN_INPUT;
    int minThreshold = FSR_MIN_THRESHOLD;
    int maxThreshold = FSR_MAX_THRESHOLD;
    int mode = FSR_FILTER_THRESHOLD;
    int maxRead = 1010;
    static const int prevReadN = 3;
    int prevRead[prevReadN];
    void recordReading();
    float getPrevReadAvg();
    int applyThreshold(int);
    void recordReading(int);
    int read();
    int _readCount = 0;
  
  public:
    void setUp();
    void setThreshold(int, int);
    void setFilter(int);
    void setFilterMode(int);
    bool isMaxRead();
    void setMaxRead(int);
    int getMaxRead();
    int getRead(bool noFilter = false);
    bool flagMaxTorque = false;
    void step();
    FSR();
    FSR(int);
    ~FSR();
};

#endif
