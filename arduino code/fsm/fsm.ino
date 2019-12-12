#include <Zumo32U4.h>
#include <Wire.h>
#include <L3G.h>

Zumo32U4ButtonA buttonA;
Zumo32U4ButtonB buttonB;
Zumo32U4ButtonC buttonC;

Zumo32U4LCD lcd;
Zumo32U4Motors motors;
Zumo32U4Encoders encoders;
Zumo32U4LineSensors lineSensors;
Zumo32U4ProximitySensors proxSensors;

L3G gyro;

bool A, B, C, D, E, F, G, H, I, J, K, L, M, N;
bool STOPPED, FOLLOW_LINE, TURN_90_CW, TURN_90_CCW, MOVE_CL;
bool obj_detected, finished, ang_reached, pos_reached, c_button, SP0, last_leg, prev_pos_reached, b_button;

float zumo_width = 0.086;
float box_width = 0.120;

int moves, motor_speed;

// other things
float bumo_voltage, gyro_noise;

// different gyro noise for the angle, not sure why this works
float gyro_noise_angle = 180.552 - 22.95;

// timing
unsigned long prev_time = 0;
unsigned long current_time = 0;
double dt = 0;

// desired signals 
float r_ang = 0;
float r_spd = 0;
float r_pos = 0;
float r_yaw = 0;

// raw signals 
float spd = 0;
float ang = 0;
float pos = 0;
float yaw = 0;

// y signals
float y_ang = 0;
float y_spd = 0;
float y_pos = 0;
float y_yaw = 0;

float prev_pos = 0;

// noise in sensor
float gyro_noise_ang = 180.552 - 22.95;

// error signals

// k values
float k_p_pos = 26.959208108763857;
float k_d_pos = 2.246600675730321;
float k_sum_pos = 0.1;
// float k_sum_pos = 1;

float k_sum_ang = 5;
float k_d_ang = 0.00304407866625909;
float k_i_ang = 0;
float k_p_ang = 0.0273967079963318;

float k_p_spd = 4.16873686460883;
float k_i_spd = 66.6997898337413;
float k_sum_spd = 1;

float k_p_yaw = 1.045823900452918;
float k_i_yaw = 26.145597511322936;
float k_d_yaw = 0;
float k_sum_yaw = 1;

float k_p_line = 22.636093592411786;
float k_i_line = 0;
float k_d_line = 0.754536453080393;
float k_sum_line = 0.5;

// raw signals
float e_pos = 0;
float e_ang = 0;
float e_spd = 0;
float e_yaw = 0;
float e_line = 0;

// integral of error 
float integral_pos = 0;
float integral_ang = 0;
float integral_spd = 0;
float integral_yaw= 0;

// previous error
float prev_e_pos = 0;
float prev_e_ang = 0;
float prev_e_spd = 0;
float prev_e_yaw = 0;
float prev_e_line = 0;

// derivative of error
float de_pos = 0;
float de_ang = 0;
float de_spd = 0;
float de_yaw = 0;
float de_line = 0;

// outputs
float u_spd;
float u_ang;
float u_pos;
float u_yaw;

// other control things
float delta = 0;
float d = 0.0470;
float u = 0.3;
float start_pos = 0;

// conversion
float counts_to_dps = 8.75/1000.0;
float counts_to_rps = counts_to_dps*PI/180.0;
float counts_to_m = 0.000191002136569940*.95;
double microsToS = 1/1000000.0;


// line following variables
#define NUM_SENSORS 3
unsigned int lineSensorValues[NUM_SENSORS];

void setup() {

  lcd.clear();
  lcd.print(F("Press A"));
  lcd.gotoXY(0, 1);
  lcd.print(F("to init"));

  buttonA.waitForButton();

  ledRed(1); 

  // control variables initialize
  
  SP0 = true;
  ang_reached = false;
  pos_reached = false;
  finished = false;
  obj_detected = false;
  last_leg = false;

  moves = 0;

  // find bumo voltage

  float v_tot = 0;
  
  for(int i = 0; i < 10; i++){
    v_tot += readBatteryMillivolts();
    
  }

  v_tot /= 10.0;

  bumo_voltage = v_tot/1000.0;

  Serial.println("voltage: " + String(bumo_voltage));

  // start gyro
  Wire.begin();
  gyro.init();
  gyro.enableDefault();

  // find offset value 
  float calib_tot = 0;
  float factor = 200.0;

  delay(200);
  
  for(int i = 0; i < factor; i++){
    gyro.read();
    calib_tot += gyro.g.z;
    delay(10);
    
  }

  gyro_noise = calib_tot/factor;
  Serial.println("gyro noise: " + String(gyro_noise));

  // initialize variables
  

  ledRed(0);
  
  // calibrate the sensors

  lcd.clear();
  lcd.print(F("Press B"));
  lcd.gotoXY(0, 1);
  lcd.print(F("for line"));

  buttonB.waitForButton();
  ledRed(1);
  
   Serial.begin(115200);
  //initialize 3 line sensors... middle, and two far outside.
  lineSensors.initThreeSensors();
  //initialize only the front proximity sensor
  proxSensors.initFrontSensor();

  //this will loop until you move the zumo across the line several times by hand.
  calibrateLineSensors();

  ledRed(0);

  // don't panic
  lcd.clear();
  lcd.print(F(" Don't"));
  lcd.gotoXY(0, 1);
  lcd.print(F(" Panic!"));


}

void loop() {

  // block 1

  c_button = buttonC.isPressed();
  b_button = buttonB.isPressed();

  // calculate time
  current_time = micros();
  dt = double(current_time - prev_time)*microsToS;
  prev_time = current_time;
  
  // position
  pos = float(encoders.getCountsRight())*counts_to_m;

  // speed
  double dp = pos - prev_pos;
  spd = dp/dt;
  prev_pos = pos;

  // proximity
  int proxCounts = proxSensors.countsFrontWithLeftLeds();
  delayMicroseconds(1000);

  // calculate y 
  y_spd = spd;
  y_ang = ang;
  y_pos = pos - start_pos;

  // block 2

  A = SP0;
  B = STOPPED&&(!c_button);
  C = STOPPED&&c_button;
  D = FOLLOW_LINE&&(!obj_detected)&&(!finished);
  E = FOLLOW_LINE&&(obj_detected);
  F = FOLLOW_LINE&&finished;
  G = TURN_90_CW&&(!ang_reached);
  H = TURN_90_CW&&ang_reached&&(!last_leg);
  I = MOVE_CL&&(!pos_reached);
  J = MOVE_CL&&(pos_reached)&&(!last_leg);
  K = TURN_90_CCW&&ang_reached;
  L = TURN_90_CCW&&(!ang_reached);
  M = MOVE_CL&&(pos_reached)&&(last_leg);
  N = TURN_90_CW&&ang_reached&&last_leg;

  // for testing FSM purposes
  

  // block 3
  STOPPED = A||B||F;
  FOLLOW_LINE = C||D||N;
  TURN_90_CW = E||G||M;
  TURN_90_CCW = J||L;
  MOVE_CL = H||I||K;
  
  // block 4

  

  if(moves >= 3){

    last_leg = true;
    
  }

  // if you go into following the line, reset the position measuremnet

  if(C || N){

    start_pos = pos;
    
  }

  

  // update velocity controller r when entering follow_line stage
  if(C){

    r_spd = u;
    
  }

  // update r_ang and r_pos durring transions

  if(E){

    r_ang = 90;
    
  }

  if(J){

    r_ang = -90;
    
  }

  // reset r ang and r_pos when the states are exited

  if(H || (K && moves==2)){

    r_pos = (box_width + zumo_width)/2;
    
  }

  if(K&&moves==1){

    r_pos = box_width + zumo_width;
    
  }

  //Serial.println("STOPPED: " + String(STOPPED) + ", FOLLOW_LINE: " + String(FOLLOW_LINE) + ", TURN_90_CW:" + String(TURN_90_CW) + ", TURN_90_CCW: " + String(TURN_90_CCW) + ", MOVE_CL: " + String(MOVE_CL));


  // increase moves each time position is reached

  if(H||K){

    moves++;
    
  }

  // angle controller

  if(TURN_90_CW || TURN_90_CCW){

    gyro.read();
    
    float raw_out = gyro.g.z - gyro_noise_ang;
    float v = raw_out*counts_to_dps;

    e_ang = r_ang - ang;
    ang = ang + v*dt;
    de_ang = e_ang - prev_e_ang;
    integral_ang = integral_ang + e_ang*dt;
    
  }

  if(FOLLOW_LINE){
    
    
    // LINE ERROR

    // calculate error for line sensors
  
    lineSensors.readCalibrated(lineSensorValues);
    proxSensors.read();
  
    int error = readLineAAB(lineSensorValues);
    float lineerror_m = error*0.01/700;//this scaling is approximate. YMMV.
    e_line = lineerror_m;
    
//    Serial.print(lineerror_m, 10);
//    Serial.print("\t");
//    Serial.println(proxCounts);

    // control the error

    de_line = e_line - prev_e_line;
    delta = k_sum_line*(k_d_line*de_line + k_p_line*e_line);
    r_yaw = delta*u/d;

 
    // set previous 
    prev_e_line = e_line;

    // YAW ERROR

    // read the yaw
    gyro.read();
    float yaw = (gyro.g.z - gyro_noise)*counts_to_rps;

    // calculate error
    e_yaw = r_yaw - yaw;
    de_yaw = e_yaw - prev_e_yaw;
    integral_yaw = integral_yaw + e_yaw*dt;

    float battery_offset = 1.0;

    if (r_yaw < 0) battery_offset *= -1;
    // other direction

    battery_offset = 0;  

    u_yaw = k_sum_yaw*(k_p_yaw*e_yaw  + k_d_yaw*(de_yaw/dt) + k_i_yaw*integral_yaw) + battery_offset;

    // make sure motor isn't spinning too fast to clip the accelrometer
    if (u_yaw > 2.5) u_yaw = 2.5;
    if (u_yaw < -2.5) u_yaw = -2.5;

    float motor_speed_yaw = 400*(u_yaw/bumo_voltage);

    Serial.print("delta: ");
    Serial.print(delta, 10);
    Serial.print(", k_p_yaw*e_yaw: ");
    Serial.print(k_p_yaw*e_yaw , 10);
    Serial.print(", k_i_yaw*integral_yaw): ");
    Serial.print((k_i_yaw*integral_yaw) , 10);
    Serial.print(", r_yaw: ");
    Serial.print(r_yaw, 10);
    Serial.print(", yaw: ");
    Serial.print(yaw, 10);
    Serial.print(", motor_speed_yaw: ");
    Serial.println(motor_speed_yaw, 10);

    //motor_speed_yaw = 0;


    // previous
    prev_e_yaw = e_yaw;

    // VELOCITY ERROR
//
//    Serial.print("spd: ");
//    Serial.println(spd, 10);

    e_spd = r_spd - y_spd;
    integral_spd = integral_spd + e_spd*dt;

    u_spd = k_sum_spd*(k_i_spd*integral_spd + k_p_spd*e_spd);

    float motor_speed_vel = 400*u_spd/bumo_voltage;


    motors.setSpeeds(motor_speed_vel-motor_speed_yaw, int(1.02*motor_speed_vel) + int(.99*motor_speed_yaw));

//    Serial.print("motor_speed_vel: ");
//    Serial.print(motor_speed_vel, 10);
//    Serial.print(", motor_speed_yaw: ");
//    Serial.println(motor_speed_yaw, 10);

    //motors.setSpeeds(motor_speed_vel, int(1.02*motor_speed_vel) );

    

  }
  
  

  obj_detected = false;
  finished = false;
  ang_reached = false;
  pos_reached = false;

  SP0 = false;


}





// brown's funcs

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

//prof. Brown's function to read a fat (3.5") line and turn into an "error"
int readLineAAB(unsigned int vals[5]){
  //function returns integer counts representing error from line center
  //values 0, 2, 4 are the actual 3 sensors
  int pos = vals[1]+vals[0]-vals[2]-1000;
  return pos;
}
