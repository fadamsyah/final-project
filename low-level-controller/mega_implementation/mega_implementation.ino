/*
  ENA LOW --> Nyala
  ENA HIGH --> Mati
  DIR LOW --> Clockwise
  DIR HIGH --> Counter Clockwise

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
#include <Adafruit_MCP4725.h>
#define BAUD 500000

/********************** PIN CONFIGURATION *************************/
#define TRIGGER 22 // For communication purpose to the Arduino Nano
#define S1 23 // For communication purpose to the Arduino Nano
#define S2 24 // For communication purpose to the Arduino Nano
int ENCODER[ ] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11}; // Absolute Encoder Pin

/********************** PIN CONFIGURATION *************************/
#define RPWM 44 // See the counter at void setup()
#define LPWM 45 // See the counter at void setup()
#define LIN_ENC A1
#define R_IS A2
#define L_IS A3
#define THROTTLE_ENA A4
#define res_val 680.0f // Ohm (Value of The Resistor)
#define current_gain (8500.0f*4.95f/1023.0f/res_val);
/******************************************************************/

/**********************TIMING*************************/
int update_steering_period = 10; // Milisekon, 100 Hz
int update_braking_period = 1000; // Microsecond, 1 kHz
int ros_period = 20; // Milisekon, 50 Hz
unsigned long update_steering_time = 0; // Milisecond
unsigned long update_braking_time = 0; // Microsecond
unsigned long ros_time = 0; // Milisecond
/****************************************************/

/************* STEERING GLOBAL VARIABLE *************/
float steering_angle = 0; // Degree
float steering_setpoint = 0;
float steering_delta_min_move = 0.2; // degree (must be less than stay)
float steering_delta_min_stay = 0.8; // degree (must be greater than move)
bool steering_moving = false;
bool s1 = LOW;
bool s2 = LOW;
bool s1_prev = HIGH;
bool s2_prev = HIGH;
float angle = 0;
float delta_steer = 0;
String steering_state = "STOP";
#define max_steer 28.0f
#define min_steer -35.0f
#define steering_gradient 0.09053716f
#define steering_bias -42.12905785f
/***************************************************/

/************* BRAKING GLOBAL VARIABLE *************/
float braking_setpoint = 0; // cm
float braking_position = 0; // cm
float braking_delta_min_move = 0.025; // cm (must be less than stay)
float braking_delta_min_stay = 0.100; // cm (must be greater than move)
float braking_zero_offset = 0.0f;
bool braking_moving = false;
String braking_state = "STOP"; // STOP | PULL | RELEASE
#define max_brake 3.0f
#define min_brake 0.0f
#define breaking_gradient 0.01012f
#define breaking_bias 0.0f
//#define bf_coeff 0.75f
#define bf_coeff 0.f
/***************************************************/

/************* THROTTLE GLOBAL VARIABLE *************/
float throttle_setpoint = 0; // 0-1
float throttle_voltage = 0; // 0-4095
Adafruit_MCP4725 dac;
/***************************************************/

ros::NodeHandle  nh;
pkg_ta::LogArduino pub_msg;
ros::Publisher pub("logging_arduino", &pub_msg);

void receive_message(const pkg_ta::Control& sub_msg) {
  steering_setpoint = min(max(sub_msg.action_steer, min_steer), max_steer);
  pub_msg.steering_setpoint = steering_setpoint;

  braking_setpoint = min(max(sub_msg.action_brake, min_brake), max_brake);
  pub_msg.brake_setpoint = braking_setpoint;

  throttle_setpoint = min(max(sub_msg.action_throttle, 0), 1);
}

ros::Subscriber<pkg_ta::Control> sub("control_signal", &receive_message );

void setup() {
  // For Arduino Mega: Set PWM frequency for D44, D45 & D46
  // set timer 5 divisor to 8 for PWM frequency of 3921.16 Hz
  TCCR5B = TCCR5B & B11111000 | B00000010; 

  pinMode(TRIGGER, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  for (int i = 0; i < 10; i++) {
    pinMode(ENCODER[i], INPUT_PULLUP);
  }

  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 0);
  braking_state = "STOP";

  nh.getHardware()-> setBaud(BAUD);
  nh.initNode();
  nh.advertise(pub);
  nh.subscribe(sub);

  dac.begin(0x60); 
  dac.setVoltage(0, true);
  pinMode(THROTTLE_ENA, OUTPUT);

  pub_msg.header.frame_id = "/log_arduino_mega";
  pub_msg.steering_setpoint = steering_setpoint;
  pub_msg.brake_setpoint = braking_setpoint;
  //steering_state.toCharArray(pub_msg.steering_state, steering_state.length());
  //braking_state.toCharArray(pub_msg.brake_state, braking_state.length());
}

void loop() {
  if ((millis() - update_steering_time) >= update_steering_period) { // Steering angle control
    update_steering_time = millis();
    
    sensing_steering();
    process_steering();
    process_throttling();    
  }

  if ((micros() - update_braking_time) >= update_braking_period){
    update_braking_time = micros();
    
    sensing_braking_position();
    //sensing_braking_current(); // Kalau ga dipake, taruh di loop ros aja
    process_braking();
    
  }

  if ((millis() - ros_time) >= ros_period) { // Publish the pub_msg
    ros_time = millis();
    
    sensing_braking_current();
    pub_msg.header.stamp = nh.now();
    pub.publish( &pub_msg);
  }
  nh.spinOnce();
}

void sensing_steering() {
  angle = 0;
  for (int i = 0; i < 10; i++) {
    if (!digitalRead(ENCODER[i])){
      angle +=  pow(2, i);
    }
  }
  
  pub_msg.steering_absolute_encoder = int(angle + 0.5);
  
  //preprocessing
  if(angle<500) { angle += 1024;}
  angle -= 500;
  
  steering_angle = steering_gradient*angle + steering_bias; // convert encoder to angle (ackerman)
  pub_msg.steering_angle = steering_angle;
  
}

void process_steering() {
  delta_steer = steering_setpoint - steering_angle;

  pub_msg.steering_delta = delta_steer;

  if (!steering_moving && abs(delta_steer) >= steering_delta_min_stay){
    steering_moving = true;
  }
  else if (steering_moving && abs(delta_steer) <= steering_delta_min_move){
    steering_moving = false;
  }

  if(steering_moving){
    if (delta_steer >= 0) { // KANAN (CW)
      steering_state = "CW";
      s1 = HIGH;
      s2 = LOW;
    }
    else{ // KIRI (CCW)
      steering_state = "CCW";
      s1 = LOW;
      s2 = HIGH;
    }
  }
  else{
    steering_state = "STOP";
    s1 = LOW;
    s2 = LOW;
  }
  
  //if ( (s1 != s1_prev) || (s2 != s2_prev) ) {
  digitalWrite(S1, s1);
  digitalWrite(S2, s2);
  delayMicroseconds(10);
  digitalWrite(TRIGGER, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER, LOW);
  s1_prev = s1;
  s2_prev = s2;

  //steering_state.toCharArray(pub_msg.steering_state, steering_state.length());
}

void sensing_braking_position(){
  float lin_dist = analogRead(LIN_ENC);
  pub_msg.brake_linear_encoder = lin_dist;

  lin_dist = breaking_gradient*lin_dist + breaking_bias;
  pub_msg.brake_position = lin_dist;
  braking_position = bf_coeff*braking_position + (1.0f - bf_coeff)*lin_dist;
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
    if (braking_setpoint == 0.0){ pwm = 180; }
    else if (braking_setpoint <= 0.25 * max_brake){ pwm = 100; }
    else if (braking_setpoint <= 0.375 * max_brake){ pwm = 110; }
    else if (braking_setpoint <= 0.60 * max_brake){ pwm = 130; }
    else if (braking_setpoint <= 0.75 * max_brake){ pwm = 150; }
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
  //braking_state.toCharArray(pub_msg.brake_state, braking_state.length());
}

void process_throttling(){
  throttle_voltage = floor(throttle_setpoint * 0.6 * 4095); // Voltage max = 60% dari 4096
  
  // ENABLE
  if(throttle_voltage > 0){ //perlu cari deadband throttle
    digitalWrite(THROTTLE_ENA, HIGH);
  } else {
    digitalWrite(THROTTLE_ENA, LOW);
  }

  // SPEED COMMAND
  dac.setVoltage(throttle_voltage, false);
  
  pub_msg.throttle_voltage = throttle_voltage;
}
