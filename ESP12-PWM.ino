/*  
 *   For a LED lamp with an ESP-12 chip.
 *   Most code by Thomas Friberg
 *   Updated 27/05/2017
 */

// Import ESP8266 libraries
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

//Sensor details
const int devices=2; //Number of LED devices
const unsigned long devID[devices+1] = {76552232,32257373,43435522}; //Name of sensor - last one is button
const unsigned long devType[devices+1] = {37,37,31};
int defaultFade[devices] = {15,15}; //Miliseconds between fade intervals - Think about putting this in EEPROM
const int devPin[devices+1] = {13,12,14}; //LED pin number 12 for LED2, 13 for LED1. 2 for ESP-01

// WiFi parameters
const char* ssid = "ThomasWifi"; //Enter your WiFi network name here in the quotation marks - Will need to be in EEPROM
const char* password = "vanillamoon576"; //Enter your WiFi pasword here in the quotation marks - Will need to be in EEPROM

//Server details
unsigned int localPort = 5007;  //UDP send port
const char* ipAdd = "192.168.0.100"; //Server address
byte packetBuffer[256]; //buffer for incoming packets

//Sensor variables specific to LEDs
int ledPinState[devices] = {0,0}; //Default boot state of LEDs and last setPoint of the pin between 0 and 100
int ledSetPoint[devices] = {0,0}; //Target for dimming
int brightness[devices] = {100,100}; //last 'on' setpoint for 0-100 scale brightness
static const unsigned int PWMTable[101] = {0,1,2,3,5,6,7,8,9,10,12,13,14,16,18,20,21,24,26,28,31,33,36,39,42,45,49,52,56,60,64,68,72,77,82,87,92,98,103,109,115,121,128,135,142,149,156,164,172,180,188,197,206,215,225,235,245,255,266,276,288,299,311,323,336,348,361,375,388,402,417,432,447,462,478,494,510,527,544,562,580,598,617,636,655,675,696,716,737,759,781,803,826,849,872,896,921,946,971,997,1023}; //0 to 100 values for brightnes
int fadeSpeed[devices] = {defaultFade[0],defaultFade[1]}; //Time between fade intervals - 20ms between change in brightness
String data = "";
int timerCount[devices] = {0,0};
int timerSetpoint[devices] = {0,0};
boolean dimFlag[devices] = {true,true};
boolean timerPrimer=false;
unsigned long lastFadeTime=0;

//Button related
bool lastButtonState=0;
bool buttonState=0;
long buttonTriggerTime=millis();
long currentTime=millis();
bool primer[4]={0,0,0,0};


WiFiUDP Udp; //Instance to send packets


//--------------------------------------------------------------

void setup()
{
  SetupLines();
}

//--------------------------------------------------------------


void loop()
{
  //Check timer state
  CheckTimer();
  
  data=ParseUdpPacket(); //Code for receiving UDP messages
  if (data!="") {
    ProcessMessage(data);//Conditionals for switching based on LED signal
  }
  
  FadeLEDs(); //Fading script

  CheckButton();

  if (WiFi.status() != WL_CONNECTED) {
    ConnectWifi();
  }

}

//--------------------------------------------------------------


void SetupLines() {
  //Set pins and turn off LED
  for (int i=0;i<devices;i++){
    pinMode(devPin[i], OUTPUT); //Set as output
    digitalWrite(devPin[i], 0); //Turn off LED while connecting
  }
  //button setup
  pinMode(devPin[devices],INPUT_PULLUP);
  
  // Start //Serial port monitoring
  Serial.begin(115200);
  
  ConnectWifi();
  
  digitalWrite(devPin[0], HIGH); //Turn off LED while connecting
  delay(10); //A flash of light to confirm that the lamp is ready to take commands
  digitalWrite(devPin[0], LOW); //Turn off LED while connecting
  
}

void ConnectWifi() {
  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("WiFi connected with IP: ");
  // Print the IP address
  Serial.println(WiFi.localIP());

  //Open the UDP monitoring port
  Udp.begin(localPort);
  Serial.print("Udp server started at port: ");
  Serial.println(localPort);

  //Register on the network with the server after verifying connect
  
  delay(2000); //Time clearance to ensure registration
  for (int i=0;i<devices+1;i++){
    SendUdpValue(0,devID[i],devType[i]); //Register LED on server
  }
}


String ParseUdpPacket() {
  int noBytes = Udp.parsePacket();
  String udpData = "";
  if ( noBytes ) {
    Serial.print("---Packet of ");
    Serial.print(noBytes);
    Serial.print(" characters received from ");
    Serial.print(Udp.remoteIP());
    Serial.print(":");
    Serial.println(Udp.remotePort());
    // We've received a packet, read the data from it
    Udp.read(packetBuffer,noBytes); // read the packet into the buffer

    // display the packet contents in HEX
    for (int i=1;i<=noBytes;i++) {
      udpData = udpData + char(packetBuffer[i - 1]);
    } // end for
    Serial.println("Data reads: " + udpData);
  } // end if
  return udpData;
}

void ProcessMessage(String dataIn) {
  unsigned long dID;
  unsigned long msg;
  int comma=dataIn.indexOf(",",0);
  byte arg1;
  byte arg2;
  unsigned int arg3;
  byte numArgs;
  int i;
  int count=0;
  String tempStr;
  
  for(i=0; dataIn[i]; i++) {
    count += (dataIn[i] == ',');
  }

  if (count==1) { //Only process if valid message
    tempStr=dataIn.substring(0, comma);
    if(IsNumeric(tempStr)) {
      dID=atoi(tempStr.c_str()); //Break down the message in to it's parts  
      Serial.println("DevID reads: " + String(dID));
    }
    else{
      dID=0;
      Serial.println("DevID is non-numeric");
    }
    tempStr=dataIn.substring(comma+1);
    if(IsNumeric(tempStr)) {
      msg=atoi(tempStr.c_str());
      Serial.println("Message reads: " + String(msg));
    }
    else {
      dID=0;
      msg=0;
      Serial.println("Message is non-numeric");
    }
  }
  else {
    Serial.println("Message not properly delimited");
    dID=0;
    msg=0;
  }

  for(int i=0;i<devices;i++) {
    if (devID[i]==dID) { //Only do this set of commands if there is a message for an LED device
      //Code for splitting message in to arguments ---------------------------------
      if (msg>=65536) { //for 4 bit message
        numArgs=3;
        for (int j=0;j<8;j++){
          bitWrite(arg1,j,bitRead(msg,j));
        }
        Serial.println("Arg1 = " + String(arg1));
        for (int j=8;j<16;j++){
          bitWrite(arg2,j-8,bitRead(msg,j));
        }
        Serial.println("Arg2 = " + String(arg2));
        for (int j=16;j<32;j++){
          bitWrite(arg3,j-16,bitRead(msg,j));
        }
        Serial.println("Arg3 = " + String(arg3));
      }
      else if (msg>=256) {
        numArgs=2;
        for (int j=0;j<8;j++){
          bitWrite(arg1,j,bitRead(msg,j));
        }
        Serial.println("Arg1 = " + String(arg1));
        for (int j=8;j<16;j++){
          bitWrite(arg2,j-8,bitRead(msg,j));
        }
        Serial.println("Arg2 = " + String(arg2));
      }
      else if (msg<=255) {
        numArgs=1;
        arg1=(byte)msg;
        Serial.println("Arg1 = " + String(arg1));
      }
      else {
        numArgs=0;
        Serial.println("Invalid msg recieved: " + msg);
      }
      // Actual functions below here -------------------------------------------
      if (numArgs==4 && arg1==230) { //Fade
        ledSetPoint[i]=arg2;
        fadeSpeed[i]=arg3/1000; //arg is in tenths of seconds for full fade. split this over 100 increments.
        Serial.println("Custom fade increment speed of " + String(fadeSpeed[i]) + " miliseconds per increment trigged");
      }
      else {
        fadeSpeed[i]=defaultFade[i];
      }
      if (numArgs==4 && arg1==231) { //Timer
        timerSetpoint[i]=arg2;
        timerCount[i]=arg3;
      }
      else {
        timerCount[i]=0;
      }
      // 232 for flicker
      // 233 strobe even duty
      // 234 strobe uneven duty
      if (numArgs==4 && arg1==235) { //newFadeSetting
        defaultFade[i]=arg3;
        fadeSpeed[i]=defaultFade[i];
        Serial.println("New fade speed setting");
      }
      // Dim and instant below here -------------------------------
      if (numArgs==1) {
        if (arg1<=109) {
          dimFlag[i]=false;
        }
        else if (arg1>=110 && arg1<=219) {
          dimFlag[i]=true;
        }
        // Main functions
        if (arg1==0 || arg1==110) { //Set off
          ledSetPoint[i]=0;
          Serial.println("Set off");
        }
        else if (arg1>=1 && arg1<=100) { //level set
          ledSetPoint[i]=arg1;
          brightness[i]=arg1;
          Serial.println("Instant set");
        }
        else if (arg1>=111 && arg1<=210) { // dim level set
          ledSetPoint[i]=arg1-110;
          brightness[i]=arg1-110;
          Serial.println("Dim set");
        }
        else if (arg1==101 || arg1==211) { //On to last
          ledSetPoint[i]=brightness[i];
          Serial.println("On to last");
        }
        else if (arg1==102 || arg1==212) { //Toggle to last
          if (ledSetPoint[i]==0) {
            ledSetPoint[i]=brightness[i];
          }
          else {
            ledSetPoint[i]=0;
          }
          Serial.println("Toggle to last");
        }
        else if (arg1==103 || arg1==213) { //Toggle to 100%
          if (ledSetPoint[i]==0) {
            ledSetPoint[i]=100;
          }
          else {
            ledSetPoint[i]=0;
          }
          Serial.println("Toggle to 100");
        }
        else if (arg1==104 || arg1==214) {  //Default press
          Serial.print("Default press triggered: ");
          if (ledSetPoint[i]>0 && ledPinState[i]==ledSetPoint[i]) {
            ledSetPoint[i]=0;
            Serial.println("Turning off");
          }
          else if (ledPinState[i]>0 && ledPinState[i]!=ledSetPoint[i]){
            ledSetPoint[i]=ledPinState[i];
            brightness[i]=ledPinState[i];
            Serial.println("Holding");
          }
          else {
            ledSetPoint[i]=brightness[i];
            Serial.println("Turning on");
          }
        }
        else if (arg1==220) {
          if (ledPinState[i]>0 && ledPinState[i]!=ledSetPoint[i]){
            ledSetPoint[i]=ledPinState[i];
            brightness[i]=ledPinState[i];
          }
          Serial.println("Hold triggered");
        }
      }
      else if (numArgs==2) {
      //2 byte messages WIP
        if (arg1==221 || arg1==222) {
          dimFlag[i]=false;
          Serial.print("2 byte instant ");
        }
        else {
          dimFlag[i]=true;
          Serial.print("2 byte dim ");
        }
        if (arg1==221 || arg1==223) { //Instant increment
          if (ledSetPoint[i]+arg2<=100) {
            ledSetPoint[i]=ledSetPoint[i]+arg2;
            Serial.println("increment by " + String(arg2));
          }
        }
        else if (arg1==222 || arg1==224) { //Instant decrement
          if (ledSetPoint[i]-arg2>=0) {
            ledSetPoint[i]=ledSetPoint[i]-arg2;
            Serial.println("decriment by " + String(arg2));
          }
        }
      }
    }
  }
}


void FadeLEDs() {
  if(millis() != lastFadeTime) {
    for (int i=0;i<devices;i++) {
      if ((!dimFlag[i]) && (ledPinState[i]!=ledSetPoint[i])) { //do this if instant command
        ledPinState[i] = ledSetPoint[i];
        analogWrite(devPin[i], PWMTable[ledPinState[i]]);
      }
      else if (dimFlag[i] && (millis() % fadeSpeed[i] == 0)) { //do this if dim command
        if ((ledPinState[i] < ledSetPoint[i])) {
          ledPinState[i] = ledPinState[i] + 1;
          Serial.println("LED state is now set to " + String(ledPinState[i]));
          analogWrite(devPin[i], PWMTable[ledPinState[i]]);
        }
        else if (ledPinState[i] > ledSetPoint[i]) {
          ledPinState[i] = ledPinState[i] - 1;
          Serial.println("LED state is now set to " + String(ledPinState[i]));
          analogWrite(devPin[i], PWMTable[ledPinState[i]]);
        }
      }
    }
    lastFadeTime=millis();
  }
}

void CheckTimer() {
  String dat;
  if(millis() % 1000 == 5) {
    timerPrimer=true;
  }
  if((millis() % 1000 == 7) && timerPrimer) {
    timerPrimer=false;
    for(int i=0;i<devices;i++) {
      if(timerCount[i]==0) {
        //Do nothing
      }
      else if (timerCount[i]>1) {
        timerCount[i]=timerCount[i]-1;
        Serial.println("Timer value reduced to " + String(timerCount[i]) + "");
      }
      else { //when timerCounter is 1
        timerCount[i]=timerCount[i]-1;
        dat=String(devID[i])+','+String(timerSetpoint[i]);
        ProcessMessage(dat);
      }
    }
  }
}

void CheckButton() {
  buttonState=(!digitalRead(devPin[devices]));
  currentTime=millis();
  if (buttonState!=lastButtonState) {
    if (buttonState && currentTime-buttonTriggerTime>300) {
      SendUdpValue(1,devID[devices],1);
      buttonTriggerTime=currentTime;
      primer[0]=1;
      primer[1]=1;
      primer[2]=1;
      primer[3]=1;
    }
    else if (!buttonState) {
      primer[0]=0;
      primer[1]=0;
      primer[2]=0;
      primer[3]=0;
    }
  }
  lastButtonState=buttonState;
  if (primer[0] && (currentTime-buttonTriggerTime>600)) {
    SendUdpValue(1,devID[devices],2);
    primer[0]=0;
  }
  else if (primer[1] && (currentTime-buttonTriggerTime>1500)) {
    SendUdpValue(1,devID[devices],3);
    primer[1]=0;
  }
  else if (primer[2] && (currentTime-buttonTriggerTime>4000)) {
    SendUdpValue(1,devID[devices],4);
    primer[2]=0;
  }
  else if (primer[3] && (currentTime-buttonTriggerTime>8000)) {
    SendUdpValue(0,devID[devices],devType[devices]);  //Register
    primer[3]=0;
  }
}

boolean IsNumeric(String str) {  //Checks that a string is numerical
  unsigned int stringLength = str.length();
  if (stringLength == 0) {
    return false;
  }
  boolean seenDecimal = false;
  for(unsigned int i = 0; i < stringLength; ++i) {
      if (isDigit(str.charAt(i))) {
        continue;
      }
      return false;
  }
  return true;
}

void SendUdpValue(byte type, unsigned long devID, unsigned long value) {
  //Print GPIO state in Serial
  Serial.print("-Value sent via UDP: ");
  Serial.println(String(type) + "," + String(devID) + "," + String(value));

  // send a message, to the IP address and port
  Udp.beginPacket(ipAdd,localPort);
  Udp.print(String(type));
  Udp.write(",");
  Udp.print(String(devID));
  Udp.write(",");
  Udp.print(String(value)); //This is the value to be sent
  Udp.endPacket();
}
