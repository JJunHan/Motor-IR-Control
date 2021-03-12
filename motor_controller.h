//#pragma once
#ifndef motor_controller
#define motor_controller

void startMotor();
void stopMotor();
void left_tick_counter();
void right_tick_counter();
void reset_ticks();
void bootup_motor();
void shutdown_motor();
void on_PID();
void move_front_back(int cm, bool front_back, bool fast_slow);
int convert_cm_to_ticks(int cm);
int convert_degree_to_ticks(int degree);
double convert_to_rpm(int speed, bool R_or_L);
int convert_to_speed(double rpm, bool R_or_L);
double calc_rpm(unsigned long period);
void rotate_right_left(int degree_position, bool right_left ,bool fast_slow);
void cali_left();
void cali_front();
void start_cali();
int get_FR();
int get_FL();
int get_MF();
int get_LF();
int get_LB();
int get_RF();
void get_all_IR();
void avoid_obs();
void avoid_obs45();
void left_hug();
#endif //end
