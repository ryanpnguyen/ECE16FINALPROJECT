#include <AltSoftSerial.h>

#include <FilterDerivative.h>
#include <FilterOnePole.h>
#include <Filters.h>
#include <FilterTwoPole.h>
#include <FloatDefine.h>
#include <RunningStatistics.h>

float filterFrequency = 2.0;
FilterOnePole lowpassFilter (LOWPASS, filterFrequency);

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

#define NUMFLAKES 10
#define XPOS 0
#define YPOS 1
#define DELTAY 2


#define LOGO16_GLCD_HEIGHT 16
#define LOGO16_GLCD_WIDTH  16

String incomingByte;

static const unsigned char PROGMEM logo16_glcd_bmp[] =
{ B00000000, B11000000,
  B00000001, B11000000,
  B00000001, B11000000,
  B00000011, B11100000,
  B11110011, B11100000,
  B11111110, B11111000,
  B01111110, B11111111,
  B00110011, B10011111,
  B00011111, B11111100,
  B00001101, B01110000,
  B00011011, B10100000,
  B00111111, B11100000,
  B00111111, B11110000,
  B01111100, B11110000,
  B01110000, B01110000,
  B00000000, B00110000
};

#if (SSD1306_LCDHEIGHT != 32)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

AltSoftSerial BTserial;

const int MPU_addr=0x68;  // I2C address of the MPU-6050, can be changed to 0x69
MPU6050 IMU(MPU_addr);
const int interruptPin = 2;
volatile bool ipinReady = false;
int16_t ax, ay, az, tp, gx, gy, gz;

unsigned long samplePeriod = 40000; // 25 Hz for the BLE limitations
unsigned long startTime = 0;
unsigned long volatile elapsedTime = 0;
unsigned long volatile currentTime = 0;
unsigned long volatile lastTime = 0;
unsigned long volatile currentTimehr = 0;
unsigned long volatile lastTimehr = 0;
bool newRead = false;
bool sending = false;

int button = 4;
int count = 2;
int value = 0;
int reset = 0;
String c;
double numtest = 56.22;
char buf[10];
int stepcount = 0;
int reading = 0;

float hr = 0;
float LPF_hr = 0;
float hr_dif = 0;
int BPM = 0;
float hr_thresh = 347;

float derivative = 0;
float logg;
float der_log;
float lastLPF = 0;
int BLE_BPM = 0;

int redPin = 5;
int greenPin = 6;
int bluePin = 7;
bool rgbon = true; 
String status_led = "OFF";
/* 
 *  Function to check the interrupt pin to see if there is data available in the MPU's buffer
 */
void interruptPinISR() {
  ipinReady = true;
}

void setColor(int red, int green, int blue)
{
  #ifdef COMMON_ANODE
    red = 255 - red;
    green = 255 - green;
    blue = 255 - blue;
  #endif
  analogWrite(redPin, red);
  analogWrite(greenPin, green);
  analogWrite(bluePin, blue);  
}

void detect_hr(){
  hr = analogRead(A1);
//  lowpassFilter.input( hr);
//  LPF_hr = lowpassFilter.output();
  currentTimehr = millis();
  derivative = hr - lastLPF; //derivative

  lowpassFilter.input( derivative );
  LPF_hr = lowpassFilter.output();
  
//  Serial.println(LPF_hr);
//  Serial.print(" ");
//  Serial.println(350);
  if (LPF_hr > 5){
    hr_dif = (currentTimehr - lastTimehr)/1000.00; // Converts millis to seconds
//
//    Serial.print("HR DIFF: ");
//    Serial.println(hr_dif);
    BPM = 60 / (hr_dif); //Ex. Time between beats = 1. 60/1 = 60 BPM
    
//    Serial.print("Heart Rate: ");
//    Serial.println(BPM);
    lastTimehr = currentTimehr; 
  }
  lastLPF = hr;
  delay(25);
}

/* 
 *  Function to read a single sample of IMU data
 *  NOTE: ONLY READ WHAT YOU INTEND TO USE IN YOUR ALGORITHM!
 */
void readIMU() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);                    // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  
  //Accelerometer (3 Axis)
  ax=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  ay=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  az=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  
  //Temperature
  tp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  
  //Gyroscope (3 Axis)
  gx=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  gy=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  gz=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

}

/* 
 *  Function to poll to see if data is available to read
 *  'samplePeriod' defines how frequently we poll... default set to 25Hz
 */
void pollData() {

  if (ipinReady && !newRead) {
  
    currentTime = micros();
    if (currentTime - lastTime >= samplePeriod) {
      elapsedTime = (currentTime - startTime)/1e3;
      lastTime = currentTime;

      readIMU();
      newRead = true;
    }
  }  
}

/* 
 *  Function to send data to the Python processing
 *  NOTE: MODIFY ACCORDING TO YOUR ALGORITHM!
 */
void sendData() {
  // Displays the raw value and the time it was sampled
        dtostrf(currentTime, 9, 0, buf);
        BTserial.write(buf);
        BTserial.write(" ");
//        Serial.print(buf);
//        Serial.print(" ");
        
        itoa(gz, buf, 10);
        BTserial.write(buf);   //gz
        BTserial.write(" ");
//        Serial.print(buf);
//        Serial.print(" ");

        itoa(ax, buf, 10);
        BTserial.write(buf);   //ax
        BTserial.write(" ");

        itoa(az, buf, 10);
        BTserial.write(buf);   //az
        BTserial.write(" ");

        itoa(stepcount, buf, 10);
        BTserial.write(buf);   //stepcount
        BTserial.write(" ");

//        itoa(hr, buf, 10);
//        BTserial.write(buf);   //HR
//        BTserial.write(" ");

        itoa(BLE_BPM, buf, 10);
        BTserial.write(buf);   //BPM
        BTserial.write(" ");
        BTserial.write("\n");
              
}

/* 
 *  Function to do the usual Arduino setup
 */
void setup(){
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.display();
  display.clearDisplay();

  // Intialize the IMU and the DMP ont he IMU
  IMU.initialize();
  IMU.dmpInitialize();
  IMU.setDMPEnabled(true);
  
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(MPU_addr);   // PWR_MGMT_1 register
  Wire.write(0);          // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.display();
  delay(500);
  display.clearDisplay();
  
  Serial.begin(9600);
  BTserial.begin(9600);
  Serial.println("BTserial started");
  BTserial.write("AT+ADTY0");
  delay(500);
  BTserial.write("AT+RENEW");
  delay(500);
   
  pinMode(button, INPUT);
  pinMode(4, INPUT_PULLUP);

  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);  
  setColor(0, 255, 0);  // green
//  pinMode(A1, INPUT);
  
  // Create an interrupt for pin2, which is connected to the INT pin of the MPU6050
  pinMode(interruptPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPin), interruptPinISR, RISING);

  // Start time of the Arduino (for elapsed time)
  startTime = micros();
}

/* 
 *  Function to loop continuously: poll ==> send ==> read
 *  NOTE: MODIFY TO SUIT YOUR ALGORITHM!
 */
void loop(){
  // no longer using an ISR for the sake of AltSoftSerial
  value = digitalRead(button);
  while (value == 1) {
    value = digitalRead(button);
//        Serial.print("Count: ");
//        Serial.println(count);

//    if ( (count % 2) == 0) {     //Executes once
//      reading = 1;
//      if (reading == 1)
//        sending = true;
//      if (reading == 0)
//        sending = false;
//    }
  pollData();
  detect_hr();
  
  sending = true;
  if (newRead && sending) {
    sendData();
    newRead = false;
  }

  if (BTserial.available() > 0) { 
    String dataFromPython =  BTserial.readStringUntil('\n'); // I assume data points are separated by commas, but anything is fine
    Serial.print("Received: ");
    Serial.println(dataFromPython);
    if ( dataFromPython == "1"){ //STEP DETECTION
      stepcount = stepcount + 1;
    }
//    if ( dataFromPython == "2"){ //LED DETECTION
//        if (!rgbon) { // if not on, turn on 
//          rgbon = true;
//          setColor(0, 255, 0);  // green
//      }
//      else{ // turn it off
//          rgbon = false;
//          setColor(255, 0, 0);  // red
//      }
//    }
  }

    if(ax > 8000 && az < -8000){
        if (!rgbon) { // if not on, turn on 
          rgbon = true;
          setColor(0, 255, 0);  // green
          status_led = "OFF";
          delay(400);
      }
      else{ // turn it off
          rgbon = false;
          setColor(255, 0, 0);  // red
          status_led = "ON";     
          delay(400); 
      }
    }
    if(BPM > 45 && BPM < 130){
      BLE_BPM = BPM;
    }

    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(10, 0);
    display.clearDisplay();
    display.print("Step Count: ");
    display.println(stepcount);
    display.print("Heart Rate: ");
    display.println(BLE_BPM);
    display.print("LED Status: ");
    display.println(status_led);
        
//    Serial.print("Recieved: ");
    Serial.println(c);
    display.display();
    delay(10);
  }
  
value = digitalRead(button);
Serial.println(value);
Serial.print("Count: ");
Serial.print(count);
    if (value == 0) {
      while (value == 0) {
        value = digitalRead(button);
      }
      count = count + 1;
      if( (count % 2) != 0){
        BTserial.write("1");
        delay(500);      
        BTserial.write("AT");
        delay(500);
        BTserial.write("AT+ADTY3");
        delay(500);
        BTserial.write("AT+RESET");
        delay(1000);
        BTserial.write("AT+SLEEP");
        delay(1000);
        while( value == 1){
          value = digitalRead(button);
        }
      }
      else if ( (count % 2) == 0){
        //WAKE
        BTserial.write("AT+00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000");
        delay(500);
        Serial.println("Wake Command Sent!!!");
        BTserial.write("0");
        delay(500);
        BTserial.write("AT+ADTY0");
        delay(500);
        BTserial.write("AT+RENEW");
        delay(1000);
      }
    }
}
