/*  
 *   For a LED lamp with an ESP-12 chip.
 *   Most code by Thomas Friberg
 *   Updated 21/04/2017
 */

// Import ESP8266 libraries
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

//Sensor details
const int devices=2; //Number of LED devices
const unsigned long devID[devices+1] = {323267777,246332767,623414555}; //Name of sensor - last one is button
const unsigned long devType[devices+1] = {5,5,1};
const int defaultFade = 15; //Miliseconds between fade intervals - Think about putting this in EEPROM
const int devPin[devices+1] = {12,13,14}; //LED pin number 12 for LED2, 13 for LED1. 2 for ESP-01

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
int fadeSpeed[devices] = {defaultFade,defaultFade}; //Time between fade intervals - 20ms between change in brightness
String data = "";
int timerCount[devices] = {0,0};
int timerSetpoint[devices] = {0,0};

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
  delay(20); //A flash of light to confirm that the lamp is ready to take commands
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

  dID=atoi(dataIn.substring(0, comma).c_str()); //Break down the message in to it's parts  
  //Serial.println("DevID reads: " + String(dID));
  msg=atoi(dataIn.substring(comma+1).c_str());
  //Serial.println("Message reads: " + String(msg));

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
      else {
        numArgs=1;
        arg1=(byte)msg;
        Serial.println("Arg1 = " + String(arg1));
      }
      // Actual functions below here -------------------------------------------
      if (numArgs==4 && arg1==240) { //Fade
        ledSetPoint[i]=arg2;
        fadeSpeed[i]=arg3/1000; //arg is in tenths of seconds for full fade. split this over 100 increments.
        Serial.println("Custom fade increment speed of " + String(fadeSpeed[i]) + " miliseconds per increment trigged");
      }
      else {
        fadeSpeed[i]=defaultFade;
      }
      if (numArgs==4 && arg1==242) { //Timer
        timerSetpoint[i]=arg2;
        timerCount[i]=arg3;
      }
      else {
        timerCount[i]=0;
      }
      if (numArgs==1 && arg1>=0 && arg1<=100) { //Instant level set
        ledPinState[i]=arg1;
        ledSetPoint[i]=arg1;
        analogWrite(devPin[i], PWMTable[arg1]);
        Serial.println("Instant set");
      }
      else if (numArgs==1 && arg1>=101 && arg1<=202) { //Regular dimming set
        ledSetPoint[i]=arg1-101;
      }
      else if (numArgs==1 && arg1==203) {  //onToLast brightness
        ledSetPoint[i]=brightness[i];
      }
      else if (numArgs==1 && arg1==211) {  //Default press
        Serial.println("Default press triggered");
        if (ledSetPoint[i]>0 && ledPinState[i]==ledSetPoint[i]) {
          ledSetPoint[i]=0;
        }
        else if (ledSetPoint[i]>0 && ledPinState[i]!=ledSetPoint[i]){
          ledSetPoint[i]=ledPinState[i];
          brightness[i]=ledPinState[i];
        }
        else {
          ledSetPoint[i]=brightness[i];
        }
      }
      else if (numArgs==1 && arg1==206) {  //Toggle full on/off
        Serial.println("Toggle full on/off triggered");
        if (ledSetPoint[i]>0) {
          ledSetPoint[i]=0;
        }
        else {
          ledSetPoint[i]=100;
        }
      }
      else if (numArgs==1 && arg1==210) {  //Hold
        Serial.println("Hold triggered");
        if (ledSetPoint[i]>0 && ledPinState[i]!=ledSetPoint[i]){
          ledSetPoint[i]=ledPinState[i];
          brightness[i]=ledPinState[i];
        }
      }
    }
  }
}


void FadeLEDs() {
  for(int i=0;i<devices;i++) {
    if ((millis() % fadeSpeed[i] == 0) && (ledPinState[i] < ledSetPoint[i])) {
      ledPinState[i] = ledPinState[i] + 1;
      analogWrite(devPin[i], PWMTable[ledPinState[i]]);
      //Serial.println("LED state is now set to " + String(ledPinState));
      delay(1);
    }
    else if ((millis() % fadeSpeed[i] == 0) && (ledPinState[i] > ledSetPoint[i])) {
      ledPinState[i] = ledPinState[i] - 1;
      analogWrite(devPin[i], PWMTable[ledPinState[i]]);
      //Serial.println("LED state is now set to " + String(ledPinState));
      delay(1);
    }
  }
}

void CheckTimer() {
  String dat;
  for(int i=0;i<devices;i++) {
    if(millis() % 1000 == 0) {
      if(timerCount[i]==0) {
        //Do nothing
      }
      else if (timerCount[i]>1) {
        timerCount[i]=timerCount[i]-1;
        //Serial.println("Timer value reduced to " + String(timerCount) + "");
      }
      else {
        timerCount[i]=timerCount[i]-1;
        dat=String(devID[i])+','+String(timerSetpoint[i]);
        ProcessMessage(dat);
      }
      delay(1);
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

void SendUdpValue(byte type, unsigned long devID, unsigned long value) {
  //Print GPIO state in //Serial
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
