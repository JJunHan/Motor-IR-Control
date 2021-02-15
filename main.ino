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

  Serial.print(get_MF());
  Serial.print(":");
  Serial.print(get_FL());
  Serial.print(":");
  Serial.println(get_FR());
  delay(200);

  
  //Serial.println(get_FR());
  //delay(200);
  //Serial.print("Front left");
  //Serial.println(get_FL());
  
  //Serial.print("Front middle");
  //Serial.println(get_MF());
  
  //Serial.println(get_LF());
  //Serial.println(get_LB());
  //Serial.println(get_RF());
  
  if(Serial.available() > 0){ //check for input
    rx = Serial.read();
  }
  switch (rx){

    case 'f': //print all ir sensor result once.
    get_all_IR();
    break;
    
    case 'w':
    move_front_back(110,true,true); //distance, front, fast
    break;
    
    case 'z':
    move_front_back(30,true,true); //100cm test
    break;
    
    case 's':
    move_front_back(50,false,true); //distance, back, slow
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

    case 'v':
    cali_left(); //calibrate left
    break;

    case 'b':
    cali_front(); //calibrate front
    break;

    case 'c':
    start_cali();
    break;

    case '1': //15
    rotate_right_left(0, true,true); //0-11
    break;

    case '2': //30
    rotate_right_left(1, true,true); //0-11
    break;

    case '3': //45
    rotate_right_left(2, true,true); //0-11
    break;

    case '4': //60
    rotate_right_left(3, true,true); //0-11
    break;
    
    case '5': //75
    rotate_right_left(4, true,true); //0-11
    break;

    case '6': //90
    rotate_right_left(5, true,true); //0-11
    break;

    case '7': //105
    rotate_right_left(6, true,true); //0-11
    break;

    case '8': //120
    rotate_right_left(7, true,true); //0-11
    break;
    case '9': //135
    rotate_right_left(8, true,true); //0-11
    break;

    case '0': //150
    rotate_right_left(9, true,true); //0-11
    break;
    
    case 'p': //165
    rotate_right_left(10, true,true); //0-11
    break;

    case 'o': //180
    rotate_right_left(11, true,true); //0-11
    break;

    case 'i': //360
    rotate_right_left(12, true,true); //0-11
    break;

    case 'u': //360
    rotate_right_left(13, true,true); //0-11
    break;

    case 'y': //1080
    rotate_right_left(14, true,true); //0-11
    break;
  }    
}
