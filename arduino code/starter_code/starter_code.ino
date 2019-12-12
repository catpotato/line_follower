#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4LineSensors lineSensors;
Zumo32U4Motors motors;
Zumo32U4ButtonA buttonA;
Zumo32U4LCD lcd;
Zumo32U4ProximitySensors proxSensors;

//number of sensors on the Zumo.
//the robot has 5 sensors, but we will only use 3.
#define NUM_SENSORS 3
//array of sensor values
unsigned int lineSensorValues[NUM_SENSORS];

//a variable to hold the integer value from the distance sensor.
//this variable goes from about 2 to about 6. 6 is very close, 2 is far.
int proxDist = 0;

//routine for calibrating the sensors
//this function was written by Ben Schmidel from Pololu, copied here.
//this allows you to calibrate the sensors right when zumo powers up.
//The calibration sets the threshold value for the sensors to distinguish 
//between the track (dark) and surroundings (light).
//INSTRUCTIONS:  During calibration mode, slide the zumo laterally across 
//the track so that all of the line sensors are alternately exposed to a 
//a light surface and a dark surface.  
void calibrateLineSensors()
{
  // To indicate we are in calibration mode, turn on the yellow LED
  // and print "Line cal" on the LCD.
  ledYellow(1);
  lcd.clear();
  lcd.print(F("Line cal"));

  for (uint16_t i = 0; i < 400; i++)
  {
    lcd.gotoXY(0, 1);
    lcd.print(i);
    lineSensors.calibrate();
  }

  ledYellow(0);
  lcd.clear();
}


void setup()
{
  Serial.begin(115200);
  //initialize 3 line sensors... middle, and two far outside.
  lineSensors.initThreeSensors();
  //initialize only the front proximity sensor
  proxSensors.initFrontSensor();

  // Wait for button A to be pressed and released.
  lcd.clear();
  lcd.print(F("Press A"));
  lcd.gotoXY(0, 1);
  lcd.print(F("to calib"));
  buttonA.waitForButton();

  //this will loop until you move the zumo across the line several times by hand.
  calibrateLineSensors();
}

void loop()
{

  //update line sensor array reading
  lineSensors.readCalibrated(lineSensorValues);
  //read proxSensor to see if there's an obstacle.
  proxSensors.read();

  // Our "error" is how far we are away from the center of the
  // line. This is the integer version in counts
  int error = readLineAAB(lineSensorValues);
  //this is a rough conversion from counts to meters. we have
  //a useable detection range of about 1 cm either way from the line.
  //your controller must be pretty good to avoid losing the line!
  float lineerror_m = error*0.01/700;//this scaling is approximate. YMMV.
  //this pulls a value from the proximity sensor, which we read above.
  int proxCounts = proxSensors.countsFrontWithLeftLeds();
  //print the error from the line and the 
  Serial.print(lineerror_m, 10);
  Serial.print("\t");
  Serial.println(proxCounts);

  //short delay to allow the analog to digital converters to settle.
  delayMicroseconds(1000);


}


//prof. Brown's function to read a fat (3.5") line and turn into an "error"
int readLineAAB(unsigned int vals[5]){
  //function returns integer counts representing error from line center
  //values 0, 2, 4 are the actual 3 sensors
  int pos = vals[1]+vals[0]-vals[2]-1000;
  return pos;
}
