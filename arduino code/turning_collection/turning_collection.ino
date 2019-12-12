#include <Zumo32U4.h>
#include <Wire.h>
#include <L3G.h>

Zumo32U4Motors motors;
Zumo32U4Encoders encoders;
Zumo32U4ButtonA buttonA;

L3G gyro;

unsigned long startTime = 0;
unsigned long timeRunning = 3000000/2;
unsigned long fudgeTime = 400000;
float spd = 170;
float start_spd = 140;

int countsRight;
int countsLeft;
int voltage;
unsigned long elapsedTime;

void setup() {
  
  // waiting for button
  
  buttonA.waitForButton();
  delay(1000);
  startTime = micros();

  Serial.println("time, omega x, omega y, omega z, voltage");

   // other setup
  Serial.begin(115200);
  

  // gyro initialization
  Wire.begin();

  gyro.init();
  gyro.enableDefault();

  

  motors.setSpeeds(-start_spd,int(.95*start_spd));
  delay(800);

 
  

}

void loop() {

  gyro.read();
  
  elapsedTime = micros() - startTime;

  if(elapsedTime < timeRunning){
    motors.setSpeeds(-spd,int(.95*spd));
  }
 
  else motors.setSpeeds(0,0);

  if(elapsedTime < (timeRunning + fudgeTime)){

    voltage = readBatteryMillivolts();

    Serial.print(elapsedTime);
    Serial.print(',');
    Serial.print(gyro.g.x);
    Serial.print(',');
    Serial.print(gyro.g.y);
    Serial.print(',');
    Serial.print(gyro.g.z);
    Serial.print(',');
    Serial.print(voltage);
    Serial.println();

  }

}
