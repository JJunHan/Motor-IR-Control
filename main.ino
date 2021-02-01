#include "motor_controller.h"
#include "SharpIR.h"

bool flag = true;
int counter = 0;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  bootup_motor(); 
  on_PID();
}
char rx = 0;

void loop() {
  //Serial.println("hello rpi");
  //delay(200);
  // put your main code here, to run repeatedly:
  if(Serial.available() > 0){ //check for input
    rx = Serial.read();
  }
  switch (rx){
    case 'w':
    move_front_back(10,true,true); //distance, front, fast
    break;
    
    case 's':
    move_front_back(10,false,true); //distance, back, slow
    break;

    case 'd':
    rotate_right_left(5, true,true); //90, right, fast
    break;

    case 'e':
    rotate_right_left(11, true, true); //180, right, slow 
    break;
    
    case 'a':
    rotate_right_left(5, false,true); //90, left, fast
    break;
    
    case 'q':
    rotate_right_left(11, false,true); //180, left, slow
    break;

    case 'x':
    cali_left();
    break;

    case 'c':
    start_cali();
    break;

    case '1':
    rotate_right_left(0, true,true); //0-11
    break;

    case '2':
    rotate_right_left(1, true,true); //0-11
    break;
    
  }    
}
