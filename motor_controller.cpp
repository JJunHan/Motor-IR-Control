#include <PID_v1.h>
#include <EnableInterrupt.h>
#include <DualVNH5019MotorShield.h>
#include "motor_controller.h"
#include "SharpIR.h"

//IR Sensor 
SharpIR sensorFR(SharpIR::GP2Y0A21YK0F, A4); //Right side front sensor
SharpIR sensorFL(SharpIR::GP2Y0A21YK0F, A0); //Left side front sensor
SharpIR sensorLF(SharpIR::GP2Y0A21YK0F, A2); //Left side left front sensor
SharpIR sensorLB(SharpIR::GP2Y0A21YK0F, A3); //Left side left back sensor
SharpIR sensorMF(SharpIR::GP2Y0A21YK0F, A1); //middle front sensor
SharpIR sensorRF(SharpIR::GP2Y0A02YK0F, A5 ); //right side front sensor
float LF_D = 0.0;
float LB_D = 0.0;
float FR_D = 0.0;
float FL_D = 0.0;
float MF_D = 0.0;
float difference = 0.0;

constexpr int RIGHT_PULSE_PORT = 3; //input port A
constexpr int LEFT_PULSE_PORT = 11; //input port A
constexpr int normalSpeed = 380; //base speed
constexpr int slowSpeed = 300; //385
const int rotation_ticks[] = {48,109,177,244,319,380,454,528,602,676,745,820,1607,3284,4891}; //15,30,45,60,75,90,105,120,135,150,165,180,360,720,1080
const int distance_customp[] = {298,596}; //10cm,20cm
DualVNH5019MotorShield md;

//Motors

void startMotor() {
	md.setSpeeds(0, 0); //Make the motor coast
	md.setBrakes(0, 0);
}

void stopMotor() {
	md.setSpeeds(0, 0);
	md.setBrakes(400, 400);
}

//Interrupts

volatile int rightTick = 0; //because of interupts
volatile int leftTick = 0;
double RPML = 0; //current rpm for left motor
double RPMR = 0; //current rpm for right motor
unsigned long time1 = 0;
unsigned long time2 = 0;
unsigned long time_last1 = 0;
unsigned long time_last2 = 0;

void left_tick_counter() { //called upon rising edge
	leftTick++; 
	//time1 = (micros() - time_last1);
	//time_last1 = micros();
}

void right_tick_counter() { 
	rightTick++; 
	//time2 = (micros() - time_last2);
	//time_last2 = micros();
}

void reset_ticks() {
  rightTick = 0;
  leftTick = 0;
}

//initializing motor

void bootup_motor() {
	md.init();
	pinMode(RIGHT_PULSE_PORT, INPUT); //make encoder input
	pinMode(LEFT_PULSE_PORT, INPUT); //make encoder input
	enableInterrupt(RIGHT_PULSE_PORT, right_tick_counter, RISING); //call interupt on rising edge to up the tick counters
	enableInterrupt(LEFT_PULSE_PORT, left_tick_counter, RISING);
}

// turn off interupts

void shutdown_motor() {
	disableInterrupt(RIGHT_PULSE_PORT);
	disableInterrupt(LEFT_PULSE_PORT);
}

//PID

double consKp= 1.71, consKi= 0.05, consKd= 0.00; //1.9,0.025 , 0.01 //2.5 0.05/ 0.055
double Setpoint;
double Input;
double PID_Output;

//Specify the links and initial tuning parameters
PID myPID(&Input, &PID_Output, &Setpoint, consKp, consKi, consKd, DIRECT);

void on_PID() {
  myPID.SetMode(AUTOMATIC); //initialise
  myPID.SetOutputLimits(-120,120); //M1 is right, M2 is left
  myPID.SetSampleTime(5); //0.01 seconds
}
void move_ramp(){ //10cm move
  
}


void move_front_back(int cm,bool front_back, bool fast_slow) { //1124.5 sq waves per revolution of 6cm (high and low), so 562.25 per revolution of 6cm if using interupts
  bool flagt = true;
	reset_ticks(); //reset both pulse inputs from encoders
	startMotor(); //set to cruise
	int total_ticks = convert_cm_to_ticks(cm);
	int chosen_speed = fast_slow ? normalSpeed : slowSpeed;
  PID_Output = 0;
	while (rightTick <= total_ticks || leftTick <= total_ticks) { //keep adjusting the speed until reach number of total ticks for both.
		Input = rightTick;
    Setpoint = leftTick;
		myPID.Compute(); 
		if (front_back) {
      md.setSpeeds( chosen_speed + PID_Output,  -(chosen_speed - PID_Output)); 
    }
    else {
      md.setSpeeds(-(chosen_speed + PID_Output),  chosen_speed - PID_Output); 
    }
		//Serial.println(PID_Output);
	}
	stopMotor(); 
	delay(10);
}

void rotate_right_left(int degree_position, bool right_left ,bool fast_slow) { //position array by difference of 15 degrees. 0-11, 11 is 180, 5 is 90.
	reset_ticks();
	startMotor();
	int total_ticks = rotation_ticks[degree_position];
	int chosen_speed = fast_slow ? normalSpeed : slowSpeed;
  PID_Output = 0;
	while (rightTick <= total_ticks || leftTick <= total_ticks) { 
		Input = rightTick;
    Setpoint = leftTick;
		myPID.Compute(); 
		md.setSpeeds( right_left ? -(chosen_speed + PID_Output):(chosen_speed - PID_Output), right_left ? -(chosen_speed + PID_Output):(chosen_speed - PID_Output)); 
		//Serial.println(PID_Output);
	}

	stopMotor();
	delay(10);
}

void cali_left() {
  reset_ticks(); 
  startMotor();
  PID_Output = 0;
  LF_D = sensorLF.getDistance();
  LB_D = sensorLB.getDistance();
  
  difference = abs(LF_D - LB_D);
  while (difference >= 0.5) { //while it is greater than 0.5 cm difference, keep calibrating to match.
    Input = rightTick;
    Setpoint = leftTick;
    myPID.Compute();
    if (LF_D > LB_D) {//if left front is greater, means it is tilted right. right motor have to move left abit.
      md.setSpeeds((100 + PID_Output), 100 - PID_Output);
    }
    else { //back sensor is greater, means it is tilted left, left motor have to move right.
      md.setSpeeds(-(100 + PID_Output), -(100 - PID_Output));
    }
    //refresh data
    LF_D = sensorLF.getDistance();
    LB_D = sensorLB.getDistance();
    difference = abs(LF_D - LB_D);
  }
  stopMotor();
  delay(10);
}

void cali_front() {
  reset_ticks(); 
  startMotor();
  PID_Output = 0;
  FR_D = sensorFR.getDistance();
  FL_D = sensorFL.getDistance();
  //MF_D = sensorMF.getDistance();
  
  difference = abs(FR_D - FL_D);
  while (difference >= 0.5) { 
    Input = rightTick;
    Setpoint = leftTick;
    myPID.Compute(); 
    if (FR_D > FL_D) { 
      md.setSpeeds((100 + PID_Output), 100 - PID_Output); //turn right
    }

    else { 
      md.setSpeeds(-(100 + PID_Output), -(100 - PID_Output)); //turn left
    }
    //refresh data
    FR_D = sensorFR.getDistance();
    FL_D = sensorFL.getDistance();
    difference = abs(FR_D - FL_D);
  }
  stopMotor();
  delay(10);
}

void start_cali() { //send IR readings at 0 and 180 facing. then turn back to 180
  reset_ticks(); 
  startMotor();
  PID_Output = 0;
  cali_left(); //straighten first
  delay(10);
  get_all_IR(); //read obstacle from starting phase
  delay(10);
  rotate_right_left(11, true, true); //180, rotate right, fast speed
  delay(300);
  get_all_IR(); //read obstacle from starting phase
  delay(10);
  rotate_right_left(11, true, true); //180, rotate right, fast speed
  //stopMotor();
  delay(10);
}

int get_FR(){
  return round(sensorFR.getDistance());
}
int get_FL(){
  return round(sensorFL.getDistance());
}
int get_LF(){
  return round(sensorLF.getDistance());
}
int get_LB(){
  return round(sensorLB.getDistance());
}
int get_MF(){
  return round(sensorMF.getDistance());
}
int get_RF(){
  return round(sensorRF.getDistance());
}

void get_all_IR(){
  /*
  Serial.print("FR:");
  Serial.print(get_FR());

  Serial.print(":MF:");
  Serial.print(get_MF());
  
  Serial.print(":FL:");
  Serial.print(get_FL()); 
  
  Serial.print(":LF:");  
  Serial.print(get_LF());

  Serial.print(":LB:");
  Serial.print(get_LB());

  Serial.print(":RF:");  
  Serial.print(get_RF());
  Serial.println("#");
*/

  Serial.print("sensor,(");
  Serial.print(get_FR());

  Serial.print(": ");
  Serial.print(get_MF());
  
  Serial.print(": ");
  Serial.print(get_FL()); 
  
  Serial.print(": ");  
  Serial.print(get_LF());

  Serial.print(": ");
  Serial.print(get_LB());

  Serial.print(": ");  
  Serial.print(get_RF());
  Serial.println(")#");
  
  Serial.flush();
  delay(10);
}

int convert_degree_to_ticks(int degree) {
 //distance from center of the robot to the tip of the wheels is 9cm
  //circumference of distance to travel is 90/360 * 2pi(9) = 14.137cm
  //14.137* 29.828 = 421.6834 ticks to turn 90 degrees
  //421.6834/90 = 4.685 ticks per degree turned
  double singleDegree = 4.20;  //4.45 for 180 degree turn. //4.25 for 90 degree turn.
  return degree * singleDegree;
}

int convert_cm_to_ticks(int cm) {
  //Distance traveled = (ticks ÷ (CPR × gear ratio)) × circumference of wheel //CPR in this case is 48.
  //562.25/(6*pi) = 29.828
  //double singleCM = 28.5;
  double singleCM = 29.7; 
  return cm * singleCM;
}

double convert_to_rpm(int speed, bool R_or_L) {
  if (R_or_L) { //if true means Right motor conversion
    return 0.3454 * speed - 11.038;
  }

  else { //left motor conversion
    return 0.354 * speed - 12.729;
  }
}

int convert_to_speed(double rpm, bool R_or_L) {
  if (R_or_L) { //if true means Right motor conversion
    return 2.9942 * rpm +20.249;
  }

  else { //left motor conversion
    return 3.0506 * rpm + 17.72;
  }
}

double calc_rpm(unsigned long period){
  return (((60.0 / ((double)period/1000000.0))) / 562.25);
}
