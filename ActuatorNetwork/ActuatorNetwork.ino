#include <WiFi.h>

const char *ssid = "TR2_AN_123132321";
const char *password = "ALLEN65802";

WiFiServer server(80, 16);

int clientLength = 0;
WiFiClient clients[16];
WiFiClient _client = NULL;

String indexHtml;

int numRoutes = 0;
String actuatorNames[32];
String routeNames[32];
String commands[32];
long commandsTS[32];
bool commandsReceived[32];
String states[32];
long statesTS[32];

void setup () {
  Serial.begin(115200);

  WiFi.softAP(ssid, password, 1, 16);
  IPAddress myIP = WiFi.softAPIP();

  addRoute("Arm Actuator 0", "/cmd/a0", "nc;");
  addRoute("Arm Actuator 1", "/cmd/a1", "nc;");
  addRoute("Arm Actuator 2", "/cmd/a2", "nc;");
  addRoute("Arm Actuator 3", "/cmd/a3", "nc;");
  addRoute("Arm Actuator 4", "/cmd/a4", "nc;");
  addRoute("Gripper Actuator 0", "/cmd/g0", "nc;");
  addRoute("Head Actuator Pan", "/cmd/h0", "nc;");
  addRoute("Head Actuator Tilt", "/cmd/h1", "nc;");

  server.begin();
}

void loop(){
  parseSerial();
  if (server.hasClient()) {
    _client = server.available();
  }

  if (_client.connected()) {
    handleClient(_client);
  } else if (_client != NULL) {
    _client = NULL;
    _client.stop();
  }
}

// -> "a0:0,0,0,19,128,2,0,;"
void parseSerialString(char *serialString) {
  String str = String(serialString);
  String actuatorId = "";
  String command = "";

  bool setCommand = false;
  bool returnState = false;
  for (int i = 0; i < str.length(); i++) {
    char c = str.charAt(i);

    if (c == ':') {
      setCommand = true;
    } else if (c == '?') {
      returnState = true;
    } else if (setCommand == false && returnState == false) {
      actuatorId += c;
    } else if (setCommand == true) {
      command += c;
    }
  }

  String route = "/cmd/" + actuatorId;
  
  if (setCommand == true) {
    editRouteCommand(route, command);
  }

  if (returnState == true) {
    Serial.print("+TR2:");
    Serial.print(actuatorId);
    Serial.print("=");
    Serial.print(getActuatorState(route));
    Serial.print(";\r\n");
  }
}

char serialString[64];
static int serialIdx = 0;
char lastChar = NULL;
void parseSerial () {
  while (Serial.available()) {
    char c = Serial.read();
    if (lastChar == '\r' && c == '\n') {
      parseSerialString(serialString);
      serialIdx = 0;
      serialString[serialIdx] = NULL;
    } else {
      serialString[serialIdx] = c;
      serialIdx++;
      serialString[serialIdx] = '\0';
    }

    lastChar = c;
  }
}

String currentLine = "";
String content = "index";
String contentType = "text/plain";

void handleClient (WiFiClient client) {
  while (client.available()) {
    char c = client.read();
    if (c == '\n') {
      // Custom routes
      for (int i = 0; i < numRoutes; i++) {
        if (currentLine.indexOf("GET " + routeNames[i]) >= 0) {
          String s = midString(currentLine, "s=", " HTTP");
          setActuatorState(routeNames[i], s);

          if (commandsReceived[i] == true) {
            content = "nc;";
          } else {
            content = getRouteCommand(routeNames[i]);
            commandsReceived[i] = true;
          }
          contentType = "text/plain";
        }
      }
      
      if (currentLine.indexOf("POST /cmd?") >= 0) {
        String cmd = midString(currentLine, "c=", "&a=");
        String act = midString(currentLine, "a=", " HTTP");
        editRouteCommand("/cmd/" + act, cmd);
        content = "OK;";
      }
      
      if (currentLine.indexOf("GET / ") >= 0) {
        setIndexHtml();
        content = indexHtml;
        contentType = "text/html";
      }
    
      if (currentLine.length() == 0) {
        client.println("HTTP/1.1 200 OK");
        client.println("Content-type: " + contentType);
        client.println("Content-length: " + String(content.length()));
        client.println("Connection: close");
        client.println();
  
        client.print(content);
        client.println();
      } else {
        currentLine = "";
      }
    } else if (c != '\r') {  // if you got anything else but a carriage return character,
      currentLine += c;      // add it to the end of the currentLine
    }
  
  }
}

String midString(String str, String start, String finish){
  int locStart = str.indexOf(start);
  if (locStart==-1) return "";
  locStart += start.length();
  int locFinish = str.indexOf(finish, locStart);
  if (locFinish==-1) return "";
  return str.substring(locStart, locFinish);
}

void setActuatorState (String routeName, String state) {
  for (int i = 0; i < numRoutes; i++) {
    if (routeNames[i] == routeName) {
      states[i] = state;
      statesTS[i] = millis();
    }
  }
}

String getActuatorState (String routeName) {
  String s = "";
  for (int i = 0; i < numRoutes; i++) {
    if (routeNames[i] == routeName) {
      String stateTS = String((millis() - statesTS[i]) / 1000.0);
      s = states[i] + "," + stateTS;
    }
  }
  return s;
}

void addRoute (String actuatorName, String routeName, String cmd) {
  actuatorNames[numRoutes] =  actuatorName;
  routeNames[numRoutes] =  routeName;
  commands[numRoutes] = cmd;
  commandsReceived[numRoutes] = false;
  commandsTS[numRoutes] = millis();
  numRoutes += 1;
}

String getRouteCommand (String routeName) {
  String cmd = "";
  for (int i = 0; i < numRoutes; i++) {
    if (routeNames[i] == routeName) {
      cmd = commands[i];
    }
  }
  return cmd;
}

void editRouteCommand (String routeName, String cmd) {
  for (int i = 0; i < numRoutes; i++) {
    if (routeNames[i] == routeName) {
      commands[i] = cmd;
      commandsReceived[i] = false;
      commandsTS[i] = millis();
    }
  }
}

void setIndexHtml () {
  String styleTABLE = "border:1px solid #ccc;border-collapse:collapse;";
  String styleTR = "border:1px solid #ccc;";
  String styleTH = "border:1px solid #ccc;padding:5px;";
  String styleTD = "border:1px solid #ccc;padding:5px;";

  indexHtml = "";
  indexHtml += "<script>";
  indexHtml += "function handleFormSubmit () {";
  indexHtml += "  var command = document.getElementById('command').value;";
  indexHtml += "  var actuatorid = document.getElementById('actuatorid').value;";
  indexHtml += "  var xhr = new XMLHttpRequest();";
  indexHtml += "  xhr.onreadystatechange = function () {";
  indexHtml += "      if (xhr.readyState == 4 && xhr.status == 200) {";
  indexHtml += "          window.location.reload();";
  indexHtml += "      }";
  indexHtml += "  };";
  indexHtml += "  xhr.open(\"POST\", \"/cmd?c=\"+command+\"&a=\"+actuatorid, true);";
  indexHtml += "  xhr.send();";
  indexHtml += "}";
  indexHtml += "</script>";
  
  indexHtml += "<h2>TR2 Actuator Network State</h2>";
  indexHtml += "<p>This is the server in your TR2 that routes serial commands ";
  indexHtml += "from your robot's onboard computer to its various wireless actuators. ";
  indexHtml += "Actuators set their state using the \"s\" paramter in the url query by ";
  indexHtml += "setting it equal to the actuator's angle in radians when visiting the route. Actuators ";
  indexHtml += "request the latest commands from the main onboard computer by parsing the ";
  indexHtml += "response body using their respective route below.</p>";
  
  indexHtml += "<p>As an example, " + actuatorNames[0] + " requests it's latest command to be executed by visiting ";
  indexHtml += "<i>" + routeNames[0] + "</i>. It can simultaneously update the actuator's state to the main onboard computer ";
  indexHtml += "with the url <i>" + routeNames[0] + "?s=3.1415</i>, given a state of 3.1415 radians.</p>";
  
  indexHtml += "<table style=\"" + styleTABLE + "\">";
  indexHtml += "<tr style=\"" + styleTR + "\">";
  indexHtml += "<th style=\"" + styleTH + "\">Actuator</th>";
  indexHtml += "<th style=\"" + styleTH + "\">Route</th>";
  indexHtml += "<th style=\"" + styleTH + "\">Last Command</th>";
  indexHtml += "<th style=\"" + styleTH + "\">Last Command Updated (sec ago)</th>";
  indexHtml += "<th style=\"" + styleTH + "\">Last State</th>";
  indexHtml += "<th style=\"" + styleTH + "\">Last State Updated (sec ago)</th>";
  indexHtml += "</tr>";
  for (int i = 0; i < numRoutes; i++) {
    String commandTS = String((millis() - commandsTS[i]) / 1000.0);
    String stateTS = String((millis() - statesTS[i]) / 1000.0);
    
    indexHtml += "<tr style=\"" + styleTR + "\">";
    indexHtml += "<td style=\"" + styleTD + "\">" + actuatorNames[i] + "</td>";
    indexHtml += "<td style=\"" + styleTD + "\">" + routeNames[i] + "</td>";
    indexHtml += "<td style=\"" + styleTD + "\">" + commands[i] + "</td>";
    indexHtml += "<td style=\"" + styleTD + "\">" + commandTS + "</td>";
    indexHtml += "<td style=\"" + styleTD + "\">" + states[i] + "</td>";
    indexHtml += "<td style=\"" + styleTD + "\">" + stateTS + "</td>";
    indexHtml += "</tr>";
  }
  indexHtml += "</table>";
  
  indexHtml += "<div style=\"padding:5px;border-top:1px solid #ccc;margin-top:25px;\">";
  indexHtml += "<h3>Update command</h3>";
  indexHtml += "<span>Actuator ID: </span>";
  indexHtml += "<input id=\"actuatorid\" type=\"text\" name=\"actuatorid\" value=\"\"><br>";
  indexHtml += "<span>New Command: </span>";
  indexHtml += "<input id=\"command\" type=\"text\" name=\"command\" value=\"\"><br>";
  indexHtml += "<input style=\"margin-top:15px\" type=\"button\" value=\"Send Command\" onclick=\"handleFormSubmit();\">";
  indexHtml += "</div>";
}
