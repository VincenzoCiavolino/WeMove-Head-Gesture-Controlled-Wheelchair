// Libraries importing
#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <MPU6050.h>
#include "Wire.h"
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

WiFiUDP Udp;
MPU6050 mpu;

// MPU6050 processor variables
bool dmpReady = false;       // set true if DMP (Digital Motion Processor) init was successful
uint8_t mpuIntStatus;        // holds actual interrupt status byte from MPU
uint8_t devStatus;           // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;         // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;          // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];      // FIFO storage buffer

int i = 0;

// Orientation/motion vars
Quaternion q;                           // [w, x, y, z]         quaternion container
VectorInt16 aa;                         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;                     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;                    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;                    // [x, y, z]            gravity vector
float euler[3];                         // [psi, theta, phi]    Euler angle container
float ypr[3];                           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

// OUR VARIABLES
float angles_sum[3] = {0.0, 0.0, 0.0};         // [x, y, z]

//WI-FI network login credentials
const char *ssid = "CAR_AP";
const char *password = "SmartCar";
unsigned int localPort = 2390;

void setup() {
  Wire.begin();             //I2C connection initialization
  Serial.begin(9600);       //Serial connection initialization

  // MPU6050 Gyroscope-Accelerometer sensor initialization
  while (!Serial); // wait for Leonardo enumeration, others continue immediately
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  Serial.println(F("Initializing DMP..."));

  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1788 factory default for my test chip

  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(digitalPinToInterrupt(15), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  // UDP connection initialization
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  Udp.begin(localPort);
}

void loop() {

  // Sensor data reading
  if (!dmpReady) return;
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();
  // Serial.println(fifoCount);
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
  } else if (mpuIntStatus & 0x02) {
    while (fifoCount < packetSize) {
      fifoCount = mpu.getFIFOCount();
    }
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    int x = 0;
    int y = 0;
    int z = 0;

    // Initial calibration
    if (!(i < 1999)) {
      x = ypr[0] * 180 / M_PI - angles_sum[0];
      y = ypr[1] * 180 / M_PI - angles_sum[1];
      z = ypr[2] * 180 / M_PI - angles_sum[2];
    } else {
      x = ypr[0] * 180 / M_PI;
      y = ypr[1] * 180 / M_PI;
      z = ypr[2] * 180 / M_PI;
    }

    // Values print for debug
   // Serial.print(x); Serial.print("/"); Serial.print(y); Serial.print("/"); Serial.println(z); Serial.print("\n"); Serial.println(i);

    // UDP packet creation and transmission
    Udp.beginPacket("192.168.4.2", localPort);
    Udp.print(x); Udp.print("/"); Udp.print(y); Udp.print("/"); Udp.print(z);
    Udp.endPacket();

  }
  // Initial mean values computation
  if (i < 2000) {
    if (i >= 1990) {
      angles_sum[0] += ypr[0] * 18 / M_PI;
      angles_sum[1] += ypr[1] * 18 / M_PI;
      angles_sum[2] += ypr[2] * 18 / M_PI;
    }
    i++;
  }
  mpu.resetFIFO();     //FIFO buffer reset to avoid overflow
}


// Interrupt function
ICACHE_RAM_ATTR void dmpDataReady()
{
  mpuInterrupt = true;
}
