/* ESPA8B1
  Wireless Servo Control, with ESP as Access Point

  Usage: 
    Connect phone or laptop to "ESP_XXXX" wireless network, where XXXX is the ID of the robot
    Go to 192.168.4.1. 
    A webpage with four buttons should appear. Click them to move the robot.

  Installation: 
    In Arduino, go to Tools > ESP8266 Sketch Data Upload to upload the files from ./data to the ESP
    Then, in Arduino, compile and upload sketch to the ESP

  Requirements:
    Arduino support for ESP8266 board
      In Arduino, add URL to Files > Preferences > Additional Board Managers URL.
      See https://learn.sparkfun.com/tutorials/esp8266-thing-hookup-guide/installing-the-esp8266-arduino-addon

    Websockets library
      To install, Sketch > Include Library > Manage Libraries... > Websockets > Install
      https://github.com/Links2004/arduinoWebSockets
    
    ESP8266FS tool
      To install, create "tools" folder in Arduino, download, and unzip. See 
      https://github.com/esp8266/Arduino/blob/master/doc/filesystem.md#uploading-files-to-file-system

  Hardware: 
  * NodeMCU Amica DevKit Board (ESP8266 chip)
  * Motorshield for NodeMCU 
  * 2 continuous rotation servos plugged into motorshield pins D1, D2
  * Ultra-thin power bank 
  * Paper chassis

*/

#include <Arduino.h>

#include <Hash.h>
#include <FS.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <WebSocketsServer.h>
#include <ESP8266mDNS.h>

#include <Servo.h>
#include <Wire.h> 
#include <VL53L0X.h>

#include "debug.h"
#include "file.h"
#include "server.h"

#define    MPU9250_ADDRESS            0x68
#define    MAG_ADDRESS                0x0C

#define    GYRO_FULL_SCALE_250_DPS    0x00  
#define    GYRO_FULL_SCALE_500_DPS    0x08
#define    GYRO_FULL_SCALE_1000_DPS   0x10
#define    GYRO_FULL_SCALE_2000_DPS   0x18

#define    ACC_FULL_SCALE_2_G        0x00  
#define    ACC_FULL_SCALE_4_G        0x08
#define    ACC_FULL_SCALE_8_G        0x10
#define    ACC_FULL_SCALE_16_G       0x18

#define SDA_PORT 14
#define SCL_PORT 12

#define SerialDebug true 

VL53L0X sensor;
VL53L0X sensor2;

int16_t accelCount[3];

float magCalibration[3] = {0, 0, 0}, magbias[3] = {0, 0, 0};  // Factory mag calibration and mag bias

const int SERVO_LEFT = D1;
const int SERVO_RIGHT = D2;
Servo servo_left;
Servo servo_right;
int servo_left_ctr = 90;
int servo_right_ctr = 90;


// WiFi AP parameters
char ap_ssid[13];
char* ap_password = "";

// WiFi STA parameters
char* sta_ssid = 
  "...";
char* sta_password = 
  "...";

char* mDNS_name = "paperbot";

String html;
String css;


// This function read Nbytes bytes from I2C device at address Address. 
// Put read bytes starting at register Register in the Data array. 
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();
  
  // Read Nbytes
  Wire.requestFrom(Address, Nbytes); 
  uint8_t index=0;
  while (Wire.available())
    Data[index++]=Wire.read();
}


// Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}

//initializes for mag calibration 
void initAK8963(float * destination) 
{
    // First extract the factory calibration for each magnetometer axis
  uint8_t rawData[3];  // x/y/z gyro calibration data stored here
  I2CwriteByte(MPU9250_ADDRESS,0x0A,0x00); // Power down magnetometer  
  delay(10);
  I2CwriteByte(MPU9250_ADDRESS,0x0A,0x0F); // Enter Fuse ROM access mode
  delay(10);
  I2Cread(MAG_ADDRESS, 0x10, 3, &rawData[0]);  // Read the x-, y-, and z-axis calibration values
  destination[0] =  (float)(rawData[0] - 128)/256. + 1.;   // Return x-axis sensitivity adjustment values, etc.
  destination[1] =  (float)(rawData[1] - 128)/256. + 1.;  
  destination[2] =  (float)(rawData[2] - 128)/256. + 1.; 
  I2CwriteByte(MPU9250_ADDRESS, 0x0A, 0x00); // Power down magnetometer  
  delay(10);
  // Configure the magnetometer for continuous read and highest resolution
  // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
  // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
  //I2CwriteByte(MPU9250_ADDRESS, 0x0A, Mscale << 4 | Mmode); // Set magnetometer data resolution and sample ODR

  // Set by pass mode for the magnetometers
  I2CwriteByte(MPU9250_ADDRESS,0x37,0x02);
  
  // Request first magnetometer single measurement
  I2CwriteByte(MAG_ADDRESS,0x0A,0x01);
  
  delay(10);
}

//Reads in acceleration data and puts it into destination array 
void readAccelData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  //readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
  I2Cread(MAG_ADDRESS, 0x3B, 6, &rawData[0]); 
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;  
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ; 
}


void setup() {

    setupPins();

    // Arduino initializations
    Wire.begin(SDA_PORT,SCL_PORT);

    sprintf(ap_ssid, "ESP_%08X", ESP.getChipId());

    for(uint8_t t = 4; t > 0; t--) {
        Serial.printf("[SETUP] BOOT WAIT %d...\n", t);
        Serial.flush();
        LED_ON;
        delay(500);
        LED_OFF;
        delay(500);
    }
    LED_ON;
    //setupSTA(sta_ssid, sta_password);
    setupAP(ap_ssid, ap_password);
    LED_OFF;

    setupFile();
    html = loadFile("/controls.html");
    css = loadFile("/style.css");
    registerPage("/", "text/html", html);
    registerPage("/style.css", "text/css", css);

    setupHTTP();
    setupWS(webSocketEvent);
    //setupMDNS(mDNS_name);

    stop();

    // Initialize device for active mode read of magnetometer
    initAK8963(magCalibration); 
    Serial.println("AK8963 initialized for active data mode....");
    if(SerialDebug) {
      //  Serial.println("Calibration values: ");
      Serial.print("X-Axis sensitivity adjustment value "); Serial.println(magCalibration[0], 2);
      Serial.print("Y-Axis sensitivity adjustment value "); Serial.println(magCalibration[1], 2);
      Serial.print("Z-Axis sensitivity adjustment value "); Serial.println(magCalibration[2], 2);
    }
}

void test_pmw() {
  int pmw = 0; 
  while(pmw < 270){
    drive(0, pmw); 
    pmw+=10; 
    delay(10000); 
  }
}

void run_mpu(float* mag_values, float* accel_values) {
  // _____________________
  // :::  Magnetometer ::: 

  // Request first magnetometer single measurement
  I2CwriteByte(MAG_ADDRESS,0x0A,0x01);
  
  // Read register Status 1 and wait for the DRDY: Data Ready
  
  uint8_t ST1;
  do
  {
    I2Cread(MAG_ADDRESS,0x02,1,&ST1);
  }
  while (!(ST1&0x01));

  // Read magnetometer data  
  uint8_t Mag[7];  
  // readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]) same 
  I2Cread(MAG_ADDRESS,0x03,7,Mag);

  readAccelData(accelCount);

  // Create 16 bits values from 8 bits data
  
  // Magnetometer
  int16_t d_mx=(Mag[1]<<8 | Mag[0]);
  int16_t d_my=(Mag[3]<<8 | Mag[2]);
  int16_t d_mz=(Mag[5]<<8 | Mag[4]);

  magbias[0] = +112.;  // User environmental x-axis correction in milliGauss, should be automatically calculated
  magbias[1] = +178.;  // User environmental x-axis correction in milliGauss
  magbias[2] = -135.; 

  //16 bit resolution 
  float mRes = 10.*4912./32760.0;

  float mx = (float)d_mx*mRes*magCalibration[0] - magbias[0];  // get actual magnetometer value, this depends on scale being set
  float my = (float)d_my*mRes*magCalibration[1] - magbias[1];  
  float mz = (float)d_mz*mRes*magCalibration[2] - magbias[2];

  float heading = atan2(mx, my);

  // Once you have your heading, you must then add your 'Declination Angle',
  // which is the 'Error' of the magnetic field in your location. Mine is 0.0404 
  // Find yours here: http://www.magnetic-declination.com/
  
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  float declinationAngle = 0.0404;
  heading += declinationAngle;

  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;

  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;

  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180/PI; 

  mag_values[0] = mx; 
  mag_values[1] = my; 
  mag_values[2] = heading; 

  // _____________________
  // :::  Accelerometer ::: 

  float aRes = 2.0/32768.0; //2 GS (00) 
  // Now we'll calculate the accleration value into actual g's
  float ax = (float)accelCount[0]*aRes; // - accelBias[0];  // get actual g value, this depends on scale being set
  float ay = (float)accelCount[1]*aRes; // - accelBias[1];   
  float az = (float)accelCount[2]*aRes; // - accelBias[2]; 

   accel_values[0] = ax; 
   accel_values[1] = ay; 

//  Serial.print("\rHeading:\t");
//  Serial.print(heading);
//  Serial.print(" Radians   \t");
//  Serial.print(headingDegrees);
//  Serial.println(" Degrees   \t");
//
//  Serial.print ("Magnetometer readings:"); 
//  Serial.print ("\tMx:");
//  Serial.print (mx); 
//  Serial.print ("\tMy:");
//  Serial.print (my);
//  Serial.print ("\tMz:");
//  Serial.print (mz);  
//  Serial.println ("\t");
}

void run_laser(float* laser_values) { 
  Serial.print("Lidar 1 range(mm): ");
  Serial.print(sensor.readRangeSingleMillimeters());
  laser_values[0] = sensor.readRangeSingleMillimeters();
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  
  Serial.print("  Lidar 2 range(mm): ");
  Serial.println(sensor2.readRangeSingleMillimeters());
  laser_values[1] = sensor2.readRangeSingleMillimeters();
  if (sensor.timeoutOccurred()) { Serial.println(" TIMEOUT"); }
  
}
void loop() {
    wsLoop();
    httpLoop();

    float mag[3] = {0,0,0}; 
    float accel[3]= {0,0,0}; 
    run_mpu(mag, accel);

    float laser[2] = {0,0}; 
    run_laser(laser); 

//    Serial.println(mag[0]); 
//    Serial.println(mag[1]); 
//    Serial.println(mag[2]); 
//
//    Serial.println(accel[0]); 
//    Serial.println(accel[1]); 
//    Serial.println(accel[2]); 

//    Serial.println(laser[0]); 
//    Serial.println(laser[1]); 
    
}


//
// Movement Functions //
//

void drive(int left, int right) {
  servo_left.write(left);
  Serial.println(left);
  servo_right.write(right);
}

void stop() {
  DEBUG("stop");
  drive(servo_left_ctr, servo_right_ctr);
  LED_OFF;
}

void forward() {
  
  DEBUG("forward");
  drive(0, 180);
}

void backward() {
  DEBUG("backward");
  drive(180, 0);
}

void left() {
  DEBUG("left");
  drive(180, 180);
}

void right() {
  DEBUG("right");
  drive(0, 0);
}



//
// Setup //
//

void setupPins() {
    // setup Serial, LEDs and Motors
    Serial.begin(115200);
    DEBUG("Started serial.");

    pinMode(LED_PIN, OUTPUT);    //Pin D0 is LED
    LED_OFF;                     //Turn off LED
    DEBUG("Setup LED pin.");

    servo_left.attach(SERVO_LEFT);
    servo_right.attach(SERVO_RIGHT);
    DEBUG("Setup motor pins");
    

    //set up lasers 
    pinMode(D3, OUTPUT);
  pinMode(D4, OUTPUT);
  digitalWrite(D7, LOW);
  digitalWrite(D8, LOW);

  delay(500);
  Wire.begin(SDA_PORT,SCL_PORT);

  digitalWrite(D3, HIGH);
  delay(150);
  Serial.println("00");
  
  sensor.init(true);
  Serial.println("01");
  delay(100);
  sensor.setAddress((uint8_t)22);

  digitalWrite(D4, HIGH);
  delay(150);
  sensor2.init(true);
  Serial.println("03");
  delay(100);
  sensor2.setAddress((uint8_t)25);
  Serial.println("04");

  Serial.println("addresses set");
  
  Serial.println ("I2C scanner. Scanning ...");
  byte count = 0;

  for (byte i = 1; i < 120; i++)
  {

    Wire.beginTransmission (i);
    if (Wire.endTransmission () == 0)
    {
      Serial.print ("Found address: ");
      Serial.print (i, DEC);
      Serial.print (" (0x");
      Serial.print (i, HEX);
      Serial.println (")");
      count++;
      delay (1);  // maybe unneeded?
    } // end of good response
  } // end of for loop
  Serial.println ("Done.");
  Serial.print ("Found ");
  Serial.print (count, DEC);
  Serial.println (" device(s).");

  delay(500);
}

void webSocketEvent(uint8_t id, WStype_t type, uint8_t * payload, size_t length) {

    switch(type) {
        case WStype_DISCONNECTED:
            DEBUG("Web socket disconnected, id = ", id);
            break;
        case WStype_CONNECTED: 
        {
            // IPAddress ip = webSocket.remoteIP(id);
            // Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", id, ip[0], ip[1], ip[2], ip[3], payload);
            DEBUG("Web socket connected, id = ", id);

            // send message to client
            wsSend(id, "Connected to ");
            wsSend(id, ap_ssid);
            break;
        }
        case WStype_BIN:
            DEBUG("On connection #", id)
            DEBUG("  got binary of length ", length);
            for (int i = 0; i < length; i++)
              DEBUG("    char : ", payload[i]);

            if (payload[0] == '~') 
              drive(180-payload[1], payload[2]);

        case WStype_TEXT:
            DEBUG("On connection #", id)
            DEBUG("  got text: ", (char *)payload);

            if (payload[0] == '#') {
                if(payload[1] == 'C') {
                  LED_ON;
                  wsSend(id, "Hello world!");
                }
                else if(payload[1] == 'F') 
                  forward();
                else if(payload[1] == 'B') 
                  backward();
                else if(payload[1] == 'L') 
                  left();
                else if(payload[1] == 'R') 
                  right();
                else if(payload[1] == 'U') {
                  if(payload[2] == 'L') 
                    servo_left_ctr -= 1;
                  else if(payload[2] == 'R') 
                    servo_right_ctr += 1;
                  char tx[20] = "Zero @ (xxx, xxx)";
                  sprintf(tx, "Zero @ (%3d, %3d)", servo_left_ctr, servo_right_ctr);
                  wsSend(id, tx);
                }
                else if(payload[1] == 'D') {
                  if(payload[2] == 'L') 
                    servo_left_ctr += 1;
                  else if(payload[2] == 'R') 
                    servo_right_ctr -= 1;
                  char tx[20] = "Zero @ (xxx, xxx)";
                  sprintf(tx, "Zero @ (%3d, %3d)", servo_left_ctr, servo_right_ctr);
                  wsSend(id, tx);
                }
                else 
                  stop();
            }

            break;
    }
}
