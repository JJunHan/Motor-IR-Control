#include "motor_controller.h"
#include "SharpIR.h"


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  bootup_motor(); 
  on_PID();
}

String rx = "";
char inputarray[64]; //max 64
char *tokenizer;
bool speed = false;
void loop() {
  //Serial.println("hello rpi");
  //delay(200);
  // put your main code here, to run repeatedly:
  if(Serial.available() > 0){ //check for input
    rx = Serial.readString(); //read it as a string
    rx.toCharArray(inputarray,64); 
    tokenizer = strtok(inputarray, "#");//use # as delimiter, return null when nothing else to read.
  }

  if(strcmp(tokenizer,"f")== 0){
    tokenizer = strtok(NULL,"#"); //reads next input sorted by delimiter
    move_front_back(10,true,speed); //front
    delay(50);
    get_all_IR(); 
  }

  else if(strcmp(tokenizer,"r")== 0){
    tokenizer = strtok(NULL,"#"); //reads next input sorted by delimiter
    rotate_right_left(5, true,speed); //90, right, fast
    delay(50);
    get_all_IR(); 
  }

  else if(strcmp(tokenizer,"v")== 0){
    tokenizer = strtok(NULL,"#"); //reads next input sorted by delimiter
    move_front_back(10, false, speed);  //back
    delay(50);
    get_all_IR(); 
  }

  else if(strcmp(tokenizer,"l")== 0){
    tokenizer = strtok(NULL,"#"); //reads next input sorted by delimiter
    rotate_right_left(5, false,speed); //90, left, fast
    delay(50);
    get_all_IR(); 
  }

  else if(strcmp(tokenizer,"A")== 0){
    tokenizer = strtok(NULL,"#"); //reads next input sorted by delimiter
    cali_left(); //calibrate left
  }

  else if(strcmp(tokenizer,"FRONTCALIBRATE")== 0){
    tokenizer = strtok(NULL,"#"); //reads next input sorted by delimiter
    cali_front(); //calibrate front
  }
  
  else if(strcmp(tokenizer,"INITIALCALIBRATE")== 0){
    tokenizer = strtok(NULL,"#"); //reads next input sorted by delimiter
    start_cali(); //start cali to send sensor readings
  }

  else if(strcmp(tokenizer,"SIDECALIBRATE")== 0){
    tokenizer = strtok(NULL,"#"); //reads next input sorted by delimiter
    cali_left(); //calibrate left
  }
  
  else if(strcmp(tokenizer,"BFOk")== 0){
    tokenizer = strtok(NULL,"#"); //reads next input sorted by delimiter
    speed = true; 
  }
  
  else if(strcmp(tokenizer,"BEOk")== 0){
    tokenizer = strtok(NULL,"#"); //reads next input sorted by delimiter
    speed = false; 
    delay(50);
    Serial.print("ALG|Map,");
    get_all_IR(); 
  }
  
  else if(strcmp(tokenizer,"w")== 0){
    tokenizer = strtok(NULL,"#"); //reads next input sorted by delimiter
    move_front_back(130,true,speed); //distance, front, fast
  }
} //end loop