/*
  THIS CODE IS MADE FOR ARDUINO MEGA using the PWM controlled by Timer 5 !
  Don't forget to change the counter if you use another board !
 
  LogArduino.h:
    1) header
    2) steering_setpoint
    3) steering_absolute_encoder
    4) steering_angle
    5) steering_delta
    6) throttle_voltage
    7) brake_setpoint
    8) brake_linear_encoder
    9) brake_position
    10) brake_delta
    11) brake_current
    12) brake_R_IS
    13) brake_L_IS
    14) brake_R_current
    15) brake_L_current
    16) brake_pwm
    17) brake_state
  Control.h:
    1) header
    2) steer
    3) throttle
    4) brake
 */

#include <ros.h>
#include <Wire.h>
#include <pkg_ta/Control.h>
#include <pkg_ta/LogArduino.h>
#define BAUD 500000

/********************** PIN CONFIGURATION *************************/
#define RPWM 44 // See the counter at void setup()
#define LPWM 45 // See the counter at void setup()
#define LIN_ENC A1
#define R_IS A2
#define L_IS A3
#define res_val 660.0f // Ohm (Value of The Resistor)
#define current_gain (8500.0f*5.0f/1023.0f/res_val);
/******************************************************************/

/**********************TIMING*************************/
int update_braking_period = 1000; // Microsecond, 1 kHz
int ros_period = 5; // Milisecond, 200 Hz
unsigned long update_braking_time = 0; // Microsecond
unsigned long ros_time = 0; // Milisecond
/****************************************************/

/************* BRAKING GLOBAL VARIABLE *************/
float braking_setpoint = 0; // cm
float braking_position = 0; // cm
float braking_delta_min_move = 0.025; // cm (must be less than stay)
float braking_delta_min_stay = 0.051; // cm (must be greater than move)
float braking_zero_offset = 0.0f;
bool braking_moving = false;
String braking_state = "STOP"; // STOP | PULL | RELEASE
#define max_brake 3.0
#define min_brake 0.0
/***************************************************/

ros::NodeHandle  nh;
pkg_ta::LogArduino pub_msg;
ros::Publisher pub("logging_brake", &pub_msg);

void receive_message(const pkg_ta::Control& sub_msg) {
  braking_setpoint = min(max(sub_msg.action_brake, min_brake), max_brake);
  pub_msg.brake_setpoint = braking_setpoint;
}

ros::Subscriber<pkg_ta::Control> sub("control_signal", &receive_message );

void setup() {
  // For Arduino Nano for pin D3 and D11
  // set timer 2 divisor to 8 for PWM frequency of  3921.16 Hz
  TCCR5B = TCCR5B & B11111000 | B00000010; 
  
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 0);
  braking_state = "STOP";

  nh.getHardware()-> setBaud(BAUD);
  nh.initNode();
  nh.advertise(pub);
  nh.subscribe(sub);

  pub_msg.header.frame_id = "/log_brake";
  pub_msg.steering_setpoint = 0.;
  pub_msg.steering_absolute_encoder = 0;
  pub_msg.steering_angle = 0.;
  pub_msg.steering_delta = 0.;
  pub_msg.throttle_voltage = 0.;
  pub_msg.brake_setpoint = braking_setpoint;
  //braking_state.toCharArray(pub_msg.brake_state, braking_state.length());
}

void loop() {
  if ((micros() - update_braking_time) >= update_braking_period){
    update_braking_time = micros();
    
    sensing_braking_position();
    sensing_braking_current();
    process_braking();
  }

  if ((millis() - ros_time) >= ros_period) { // Publish the pub_msg
    ros_time = millis();
    
    pub_msg.header.stamp = nh.now();
    pub.publish( &pub_msg);
  }
  nh.spinOnce();
}

void sensing_braking_position(){
  float lin_dist = analogRead(LIN_ENC);
  pub_msg.brake_linear_encoder = lin_dist;

  lin_dist = (lin_dist - braking_zero_offset) / 98.8;
  pub_msg.brake_position = lin_dist;
  braking_position = lin_dist;
}

void sensing_braking_current(){
  float current_read;
  
  current_read = analogRead(R_IS);
  pub_msg.brake_R_IS = current_read;
  current_read = current_read * current_gain ; // A
  pub_msg.brake_R_current = current_read;

  current_read = analogRead(L_IS);
  pub_msg.brake_L_IS = current_read;
  current_read = current_read * current_gain ; // A
  pub_msg.brake_L_current = current_read;
}

void process_braking(){
  int pwm = 0;
  
  float delta_brake = braking_setpoint - braking_position;
  pub_msg.brake_delta = delta_brake;
  
  if (!braking_moving && abs(delta_brake) >= braking_delta_min_stay){
    braking_moving = true;
  }
  else if (braking_moving && abs(delta_brake) <= braking_delta_min_move){
    braking_moving = false;
  }  
  
  if(braking_moving){
    pwm = 255;
    if (braking_setpoint == 0.0){ pwm = 150; }
    else if (braking_setpoint <= 0.25 * max_brake){ pwm = 100; }
    else if (braking_setpoint <= 0.375 * max_brake){ pwm = 150; }
    else if (braking_setpoint <= 0.60 * max_brake){ pwm = 155; }
    else if (braking_setpoint <= 0.75 * max_brake){ pwm = 165; }
    else if (braking_setpoint <= 0.80 * max_brake){ pwm = 175; }
    else if (braking_setpoint <= 0.85 * max_brake){ pwm = 200; }

    if(delta_brake >= 0){
      braking_state = "PULL";
      analogWrite(LPWM, 0);
      analogWrite(RPWM, pwm);
    }
    else{
      braking_state = "RELEASE";
      analogWrite(RPWM, 0);
      analogWrite(LPWM, pwm);
    }
  }
  else {
    braking_state = "STOP";
    pwm = 0;
    
    analogWrite(RPWM, 0);
    analogWrite(LPWM, 0);
  }
  
  pub_msg.brake_pwm = pwm;
  //braking_state.toCharArray(pub_msg.brake_state, braking_state.length()); // Ini bikin BUG
}
