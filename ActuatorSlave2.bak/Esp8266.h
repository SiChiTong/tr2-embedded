
#define ESP_RES_OK "OK"
#define ESP_RES_ERROR "ERROR"
#define ESP_REQ_RST "AT+RST\r\n"
#define ESP_REQ_HTTP "GET %s HTTP/1.1\r\nHost: %s\r\nConnection: close\r\n\r\n"

unsigned long lastStep = millis();

class Esp8266 {
  private:
    int id, minId = 0;
    int maxId = 4;
    bool tcpOpen = false;
    bool networkOpen = false;
    int resIdx = 0;
    char route[64];
    char res[128];
    char req[128];
    char lastReqCmd[64];
    char debugBuffer[128];

    unsigned long commandSentOn = millis();
    unsigned long commandTimeout = 1500;
    HardwareSerial* ser;
    HardwareSerial* serDebug;
    bool useDebugSerial = false;

  public:
    char* host = "192.168.4.1";
    char* port = "80";
    char* ssid;
    char* pass;

    long dataLastReceived = 0;

    bool flagActuatorConfig = false;
    char* actuatorCfg = "\0";
    
    Esp8266 (HardwareSerial* serial) {
      ser = serial;
    }

    void setDebugSerial(HardwareSerial* serial) {
      serDebug = serial;
      useDebugSerial = true;
    }
    void begin () {
      ser->begin(115200);
      if (useDebugSerial == true) {
        serDebug->begin(115200);
      }
    }

    void configure() {
      delay(1000);
      flush();
      tcpOpen = false;
      reset();
      setMode();
      joinWifi();
      enableMultipleConnections();
    }

    int getId () {
      return id;
    }

    void setId () {
      if (id >= maxId) {
        id = minId;
      } else {
        id++;
      }
    }

    void printDebug (char* str) {
      if (useDebugSerial == true) {
        serDebug->print(str);
      }
    }

    void setTimeout(unsigned long t) {
      commandTimeout = t;
    }

    void getConfig (char* actuatorId) {
      if (networkOpen == false) {
        joinWifi();
        return;
      }
      
      sprintf(route, "/cfg/%s", actuatorId);
      printDebug("\r\n");
      lastStep = millis();
      tcpStart();
      httpGET(route);
      tcpEnd();
      sprintf(debugBuffer, "%s\r\n", lastReqCmd);
      printDebug(debugBuffer);
    }
    
    void step (char* actuatorId, float encoderAngle) {
      static char state[8];
      dtostrf(encoderAngle, 6, 4, state);

      long dlr = millis() - dataLastReceived;
      if (dlr > 3000) {
        clear();
        configure();
        delay(2000);
      } else if (dlr > 1000) {
        clear();
      }

      if (flagActuatorConfig == false) {
        sprintf(route, "/cmd/%s?s=%s", actuatorId, state);
      } else {
        sprintf(route, "/cmd/%s?s=%s&cfg=%s", actuatorId, state, actuatorCfg);
      }

      read();
      if (millis() - lastStep > 300) {
        printDebug("\r\n");
        lastStep = millis();
      
        if (networkOpen == false) {
          joinWifi();
          return;
        }
      
        tcpStart();
        httpGET(route);
        tcpEnd();
        sprintf(debugBuffer, "%s\r\n", lastReqCmd);
        printDebug(debugBuffer);
        flagActuatorConfig = false;
      }
    }

    void clear() {
      tcpOpen = false;
    }

    bool available() {
      return tcpOpen;
    }

    char* getLastCommand () {
      return lastReqCmd;
    }

    void flush () {
      while (ser->available()) {
        ser->read();
      }
    }

    void send (char* cmd) {
      sprintf(debugBuffer, "REQ -> %s", cmd);
      printDebug(debugBuffer);
      ser->print(cmd);

      commandSentOn = millis();
    }

    char* read (char eol = '\n') {
      bool resComplete = false;
      
      char c = ' ';
      while (!resComplete) {
        if (ser->available()) {
          c = (char)ser->read();
        } else {
          resComplete = true;
          return "";
        }

        if (resIdx < 126) {
          res[resIdx] = c;
          resIdx++;
        } else {
          resComplete = true;
        }

        if (c == eol) {
          resComplete = true;
        }
      }

      res[resIdx] = '\0';
      resIdx++;
        
      if (resIdx > 1) {
        if (strstr(res, "0,CLOSED")) {
          tcpOpen = false;
        } else if (strstr(res, "0,CONNECT")) {
          tcpOpen = true;
        }

        if (strstr(res, "WIFI CONNECTED")) {
          networkOpen = false;
        } else if (strstr(res, "WIFI DISCONNECT") || strstr(res, "no ip")) {
          networkOpen = false;
        }
        
        sprintf(debugBuffer, "RES <- %s", res);
        printDebug(debugBuffer);
      }

      resIdx = 0;
      return res;
    }

    void reset () {
      send(ESP_REQ_RST);

      bool resComplete = false;
      while (resComplete == false) {
        char* res = read();
        if (strstr(res, ESP_RES_OK)) {
          resComplete = true;
        }

        if (millis() - commandSentOn > 5000) { // 10 sec delay
          reset();
          resComplete = true;
        }
      }

      delay(3000);
      flush();
    }

    void setMode() {
      char* cmd = "AT+CWMODE=1\r\n";
      send(cmd);

      bool resComplete = false;
      while (resComplete == false) {
        char* res = read();
        if (strstr(res, ESP_RES_OK)) {
          resComplete = true;
        }
        
        if (millis() - commandSentOn > commandTimeout) {
          setMode();
          resComplete = true;
        }
      }
    }

    void joinWifi() {
      sprintf(req, "AT+CWJAP=\"%s\",\"%s\"\r\n", ssid, pass);
      send(req);

      bool resComplete = false;
      while (resComplete == false) {
        char* res = read();
        if (strstr(res, ESP_RES_OK)) {
          resComplete = true;
          networkOpen = true;
        }

        if (millis() - commandSentOn > 10000) { // 10 sec delay
          reset();
          joinWifi();
          resComplete = true;
        }
      }
    }

    void enableMultipleConnections () {
      char* cmd = "AT+CIPMUX=1\r\n";
      send(cmd);

      bool resComplete = false;
      while (resComplete == false) {
        char* res = read();
        if (strstr(res, ESP_RES_OK)) {
          resComplete = true;
        }
      }
    }

    void tcpStart() {
      sprintf(req, "AT+CIPSTART=0,\"TCP\",\"%s\",%s\r\n", host, port);
      
      if (networkOpen == false) {
        joinWifi();
      }
        
      if (tcpOpen == false) {
        send(req);
      } else {
        return;
      }

      bool resComplete = false;
      while (resComplete == false) {
        char* res = read();
        if (strstr(res, ESP_RES_OK)) {
          resComplete = true;
        } else if (strstr(res, ESP_RES_ERROR)) {
          resComplete = true;
        }

        if (millis() - commandSentOn > commandTimeout) {
          resComplete = true;
        }
      }
    }

    void tcpEnd() {
      sprintf(req, "AT+CIPCLOSE=0\r\n");

      if (networkOpen == false) {
        return;
      }
      
      if (tcpOpen == true) {
        send(req);
      } else {
        return;
      }

      bool resComplete = false;
      while (resComplete == false) {
        char* res = read();
        if (strstr(res, ESP_RES_OK)) {
          resComplete = true;
        } else if (strstr(res, ESP_RES_ERROR)) {
          resComplete = true;
        }

        if (millis() - commandSentOn > commandTimeout) {
          resComplete = true;
        }
      }
    }

    void prepareSend(int len) {
      sprintf(req, "AT+CIPSEND=0,%i\r\n", len);

      if (networkOpen == false) {
        return;
      }
      
      if (tcpOpen == true) {
        send(req);
      } else {
        return;
      }

      bool resComplete = false;
      while (resComplete == false) {
        char* res = read();
        if (strstr(res, ESP_RES_OK)) {
          resComplete = true;
        } else if (strstr(res, ESP_RES_ERROR)) {
          resComplete = true;
        }

        if (tcpOpen == false) {
          resComplete = true;
          return;
        }

        if (millis() - commandSentOn > commandTimeout) {
          break;
        }
      }
    }

    void clearCmd() {
      for (int i = 0; i < 64; i++) {
        lastReqCmd[i] = '\0';
      }
    }

    void httpGET (char* route) {
      if (networkOpen == false) {
        return;
      }
      
      sprintf(req, ESP_REQ_HTTP, route, host);
      prepareSend(strlen(req));

      sprintf(req, ESP_REQ_HTTP, route, host);
      if (tcpOpen == true) {
        send(req);
      } else {
        lastReqCmd[0] = '\0';
        return;
      }
      
      bool resComplete = false;
      while (resComplete == false) {
        char* res = read();
        if (strstr(res, "cmd:") || strstr(res, "cfg:")) {
          dataLastReceived = millis();
          int j = 0;
          bool foundColon = false;
          bool foundSemiColon = false;
          for (int i = 0; i < 64; i++) {
            if (foundColon == false) {
              if (res[i] == ':') {
                foundColon = true;
              }
            } else if (foundSemiColon == false) {
              if (res[i] == ':') {
                lastReqCmd[0] = '\0';
                j = 0;
              } else {
                lastReqCmd[j++] = res[i];
              }
              if (res[i] == ';') {
                foundSemiColon = true;
              }
            }
          }
          
          lastReqCmd[j++] = '\0';
          resComplete = true;
        } else if (strstr(res, "SEND FAIL")) {
          lastStep = 0;
          resComplete = true;
        } else if (strstr(res, ESP_RES_ERROR)) {
          lastStep = 0;
          resComplete = true;
        }

        if (tcpOpen == false) {
          resComplete = true;
          return;
        }

        if (millis() - commandSentOn > commandTimeout) {
          break;
        }
      }
    }
}; 
