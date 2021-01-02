//created with assitance from SparkFun MPL library
//created with assitance from Arduino project hub forum
// https://create.arduino.cc/projecthub/Nicholas_N/how-to-use-the-accelerometer-gyroscope-gy-521-6dfc19


#include <Wire.h> //for the pressure sensor
#include "SparkFunMPL3115A2.h" //for the pressure sensor
#include <LiquidCrystal.h>  //for the lcd
#include <SPI.h> //for the sd card adapter
#include <SD.h> //for the sd card adapter
#include <math.h> //need math functions to perform computation on gyro sensor


//since they both us i2c, u can connect them to the same bus - do some more research on how i2c works
//choose a temp reading to stick with

File data_recording; //making a file object
MPL3115A2 PressureSensor; //making instance of pressure sensor component 
int Contrast=75;
LiquidCrystal lcd(10, 9, 5, 8, 3, 2);  //making instance of liquid crystal display using the following pins as to communicate with it
//                12  11    4
const int MPU=0x68; //I2C address of the gyro sensor



int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ; //16-bit integers
int AcXcal,AcYcal,AcZcal,GyXcal,GyYcal,GyZcal,tcal; //calibration variables
double t,tx,tf,pitch,roll;


int on_off_pot_pin = A0;//0 means tht we want the system to be off, 1023 means tht it is on (highest value)
//int gyro_pin = 7;
//int barometer_pin = 6;
int measurement_num = 1;
float accel = 0.0;
float alt = 0.0;
float pressure = 0.0;
float temp = 0.0;

void init_serial() {
   //init the sd card adapter
  SD.begin(4); //init the sd card
  data_recording = SD.open("HKN", FILE_WRITE);
}

  
void end_serial() {
    //close the file you are writing to
    data_recording.close();
  }

void setup()
 {
    Wire.beginTransmission(MPU); //begin transmission to gy
    Wire.write(0x6B); // PWR_MGMT_1 register
    Wire.write(0); // wake it up
    Wire.endTransmission(true); //ends transmission to gy

//    Serial.begin(9600);
    Wire.begin();     
    //SD.begin(4); //move this here?
    analogWrite(6,Contrast); //set the initial contrast level
    lcd.begin(16, 2); //start the display
//    pinMode(gyro_pin, OUTPUT); //make the 7th pin output (power for  gyro)
//    pinMode(barometer_pin, OUTPUT); //make the 6th pin output (power for  barometer)
  
  } 


void getAngle(int Ax,int Ay,int Az) 
{
    double x = Ax;
    double y = Ay;
    double z = Az;

    pitch = atan(x/sqrt((y*y) + (z*z))); //pitch calculation
    roll = atan(y/sqrt((x*x) + (z*z))); //roll calculation

    //converting radians into degrees
    pitch = pitch * (180.0/3.14);
    roll = roll * (180.0/3.14) ;
}



void read_gyro() {
    Wire.beginTransmission(MPU); //begin transmission to I2C slave device
    Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false); //restarts transmission to gy
    Wire.requestFrom(MPU,14,true); //request 14 registers in total  
    //Acceleration data correction
    AcXcal = -950;
    AcYcal = -300;
    AcZcal = 0;

    //Temperature correction
    tcal = -1600;

    //Gyro correction
    GyXcal = 480;
    GyYcal = 170;
    GyZcal = 210;


 //read accelerometer data
    AcX=Wire.read()<<8|Wire.read(); 
    AcY=Wire.read()<<8|Wire.read(); 
    AcZ=Wire.read()<<8|Wire.read(); 
  
    //read temperature data 
    Tmp=Wire.read()<<8|Wire.read(); 
  
    //read gyroscope data
    GyX=Wire.read()<<8|Wire.read(); 
    GyY=Wire.read()<<8|Wire.read();
    GyZ=Wire.read()<<8|Wire.read(); 

    //temperature calculation
    tx = Tmp + tcal;
    t = tx/340 + 36.53; //equation for temperature in degrees C from datasheet
    tf = (t * 9/5) + 32; //fahrenheit

    //get pitch/roll
    getAngle(AcX,AcY,AcZ);
  

    data_recording.println("Gyro Reading:");
    data_recording.println("Pitch is " + String(pitch));
    data_recording.println("Roll is " + String(roll));
    data_recording.println("X Acceleration is " + String(AcX + AcXcal));
    data_recording.println("Y Acceleration is " + String(AcY + AcYcal));
    data_recording.println("Z Acceleration is " + String(AcY + AcYcal));
    data_recording.println("Temp in F is " + String(tf));
    data_recording.println("X Orientation is " + String(GyX + GyXcal));
    data_recording.println("Y Orientation is " + String(GyY + GyYcal));
    data_recording.println("Z Orientation is " + String(GyZ + GyZcal));
    
  }

void read_baro() {
//  digitalWrite(gyro_pin, LOW);
//  digitalWrite(barometer_pin, HIGH);  
  PressureSensor.begin();
  PressureSensor.setModeBarometer(); //bring online the barometer
  PressureSensor.setOversampleRate(7); // set sampling rate to recommended amount -128 
  PressureSensor.enableEventFlags(); // Enable all three pressure and temp event flags 
    
  //might have top alterante btwn barometer and altitidue modes
  pressure = PressureSensor.readPressure(); //reading the pressure 
//  temp = PressureSensor.readTempF();//reading the temp
  
  PressureSensor.setModeAltimeter(); //bring online altimeter
  alt = PressureSensor.readAltitude(); //read altitidue 

  data_recording.println("Baro Reading:");
  data_recording.println("Pressure is " + String(pressure));
//  data_recording.println("Temp is " + String(temp));
  data_recording.println("Altitude is " + String(alt));
  }

void loop()
 {
   while (analogRead(on_off_pot_pin) != 0) {
       lcd.print("TURN ON SYSTEM"); ; //do nothing until user turns system on
       delay(1000);
       lcd.clear();
    }

   init_serial(); //OPEN FILE WHEN SYSTEM IS ON
   while (analogRead(on_off_pot_pin) == 0){
     data_recording.println("Measurement Number " + String(measurement_num));
     lcd.print("Reading gyro");
     read_gyro();
//     data_recording.println(analogRead(on_off_pot_pin));
     delay(2000); //2 second delay between reading the sensors     lcd.clear();
     lcd.clear();
     
     lcd.print("Reading baro");
     read_baro();
//     data_recording.println(analogRead(on_off_pot_pin));
     data_recording.println(); //put a newline at the end of each set of measurements
     ++measurement_num; //iterate measurement
     delay(2000); //2 second delay between reading the sensors     lcd.clear();
     lcd.clear();
   }

   end_serial(); //CLOSE FILE WHEN SYSTEM IS DONE
   lcd.print("SYSTEM FINISHED");
   delay(2000);
   lcd.clear();
   
   
 }
