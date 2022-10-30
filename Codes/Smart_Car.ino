// Libraries importing
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

// Motor pins declaration
int motor_left = 15;
int motor_right = 12;

const int enableleft = 13;
const int enableright = 14;

// Motor control variables
const int rollTreshold_R = 10;
const int rollTreshold_L = -10;
const int pitchTreshold = -10;
bool calibrationEnd = false;

// WI-FI network login credentials
#ifndef STASSID
#define STASSID "CAR_AP"
#define STAPSK  "SmartCar"
#endif

unsigned int localPort = 2390;      // local port to listen on

// buffers for receiving and sending data
char packetBuffer[UDP_TX_PACKET_MAX_SIZE + 1]; //buffer to hold incoming packet,

//Motor control variables declaration
int x, y, z;
int velPitch, velRollR, velRollL;

WiFiUDP Udp;

void setup() {
  
  // Serial connection setup
  Serial.begin(9600);
  
  // WI-FI connection setup
  WiFi.mode(WIFI_STA);
  WiFi.begin(STASSID, STAPSK);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(500);
  }
  Serial.print("Connected! IP address: ");
  Serial.println(WiFi.localIP());
    
  // UDP connection setup
  Serial.printf("UDP server on port %d\n", localPort);
  Udp.begin(localPort);

  // Motor control pins initialization function call
  initializeMotorPins();

}

void loop() {
  
  // if there's data available, read a packet
    int packetSize = Udp.parsePacket();
    if (packetSize) {
  
  // read the packet into packetBufffer
    int n = Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
    packetBuffer[n] = 0;
    
  // Print on serial monitor contents of the received packet
    Serial.println(packetBuffer);
  }

  // Packet values char-to-int conversion
    char * token = strtok(packetBuffer,"/");
    int i=0;
    int xyz[3];
    while( token != 0 ) {      
        xyz[i]= atoi(token);
        i++;
        token = strtok(0, "/");
   }
     x = xyz[0];
     y = xyz[1];
     z = xyz[2];
     velPitch = map(y, 0, 35, 0, 1023);
     if (z > 0){
     velRollR = map(z, 0, 35, 0, 1023);
     } else {   
          velRollL = map(z, 0, -35, 0, 1023);
    }
 
  // Motor control function call
    if ((x==0 && y==0 && z==0)||(calibrationEnd == true)) {
    calibrationEnd = true;
    digitalWrite(2, LOW);
    motorControl(x, y, z, velPitch, velRollR, velRollL);
  }
  else if (calibrationEnd == false){
    blink_LED();
    }
}


// Motor control pins initialization function 
  void initializeMotorPins() 
  {
    pinMode(motor_left, OUTPUT);
    pinMode(motor_right, OUTPUT);
    pinMode(enableleft, OUTPUT); 
    pinMode(enableright, OUTPUT);
    pinMode(2, OUTPUT); 
 
  }

//Motor control function 
void motorControl(int x, int y, int z, int velPitch, int velRollR, int velRollL)
{
  if (z < rollTreshold_R && z > rollTreshold_L && y > pitchTreshold){
      motor_stop();
    }
  else if (y > pitchTreshold && (z > rollTreshold_R)) {
      turn_right(velRollR);
  }
  else if (y > pitchTreshold && (z < rollTreshold_L)) {
      turn_left(velRollL);
  }
  else if (z < rollTreshold_R && z > rollTreshold_L && y < pitchTreshold) {
      drive_forward(velPitch);
    }
     
  
}

void motor_stop(){
analogWrite(enableleft, 0);
analogWrite(enableright, 0);
  
digitalWrite(motor_left, LOW);
digitalWrite(motor_right, LOW);
    Serial.print("Motor stop\n");

}

void drive_forward(int velPitch){
  analogWrite(enableleft, -velPitch);
  analogWrite(enableright, -velPitch);

  digitalWrite(motor_left, HIGH);
  digitalWrite(motor_right, HIGH);
      Serial.print("Drive forward\n");

}

void turn_left(int velRollL){
  analogWrite(enableleft, 0);
  analogWrite(enableright, velRollL);
  
  digitalWrite(motor_left, LOW);
  digitalWrite(motor_right, HIGH);
      Serial.print("Turn left\n");

}

void turn_right(int velRollR){
  analogWrite(enableleft, velRollR);
  analogWrite(enableright, 0);

  digitalWrite(motor_left, HIGH);
  digitalWrite(motor_right, LOW);
      Serial.print("Turn right\n");

}

void blink_LED(){
  digitalWrite(2, HIGH);
  delay(50);
  digitalWrite(2, LOW);
  delay(50);
  }

