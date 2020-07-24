/*
  ENA LOW --> Nyala
  ENA HIGH --> Mati
  DIR LOW --> Clockwise
  DIR HIGH --> Counter Clockwise

  LogArduino.h:
    1) header
    2) steering_setpoint
    3) steering_absolute_encoder
    4) steering_angle
    5) steering_delta
    6) throttle_voltage
    7) brake_setpoint
    8) brake_current
    9) brake_current_sensor
    10) brake_position
    11) brake_linear_encoder
    12) brake_pwm

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
#define S1 12 // For communication purpose to the Arduino Nano
#define S2 13 // For communication purpose to the Arduino Nano
#define TRIGGER 22 // For communication purpose to the Arduino Nano
int ENCODER[ ] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11}; // Absolute Encoder Pin

#define INA 42
#define INB 43
#define PWM 44 // See the counter at void setup()
#define LIN_ENC A1
#define CS A2
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
float steering_delta_min = 0.2;
bool s1 = LOW;
bool s2 = LOW;
bool s1_prev = HIGH;
bool s2_prev = HIGH;
float angle = 0;
float delta_steer = 0;
#define max_steer 28.0
#define min_steer -35.0
/***************************************************/

/************* BRAKING GLOBAL VARIABLE *************/
float braking_setpoint = 0; // cm
float braking_position = 0; // cm
float braking_current = 0; // Ampere
float braking_delta_min_move = 0.025; // cm (must be less than stay)
float braking_delta_min_stay = 0.051; // cm (must be greater than move)
float braking_zero_offset = 0.0f;
bool braking_moving = false;
int braking_sign = 1; // -1 or +1. To give the sign while calculating the measured current
#define max_brake 1.55
#define min_brake 0.0
/***************************************************/

ros::NodeHandle  nh;
pkg_ta::LogArduino pub_msg;
ros::Publisher pub("logging_arduino", &pub_msg);

void receive_message(const pkg_ta::Control& sub_msg) {
  steering_setpoint = min(max(sub_msg.steer, min_steer), max_steer);
  pub_msg.steering_setpoint = steering_setpoint;

  braking_setpoint = min(max(sub_msg.brake, min_brake), max_brake);
  pub_msg.brake_setpoint = braking_setpoint;
}

ros::Subscriber<pkg_ta::Control> sub("control_signal", &receive_message );

void setup() {
  // For Arduino Mega: Set PWM frequency for D44, D45 & D46
  // set timer 5 divisor to 8 for PWM frequency of 3921.16 Hz
  TCCR5B = TCCR5B & B11111000 | B00000010; 
  
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(TRIGGER, OUTPUT);
  for (int i = 0; i < 10; i++) {
    pinMode(ENCODER[i], INPUT_PULLUP);
  }

  pinMode(PWM, OUTPUT);
  pinMode(INA, OUTPUT);
  pinMode(INB, OUTPUT);
  digitalWrite(INA, LOW);
  digitalWrite(INB, LOW);

  nh.getHardware()-> setBaud(BAUD);
  nh.initNode();
  nh.advertise(pub);
  nh.subscribe(sub);

  pub_msg.header.frame_id = "/log_arduino_mega";
}

void loop() {
  if ((millis() - update_steering_time) >= update_steering_period) { // Steering angle control
    update_steering_time = millis();
    
    sensing_steering();
    process_steering();    
  }

  if ((micros() - update_braking_time) >= update_braking_period){
    update_braking_time = micros();
    
    sensing_braking();
    process_braking();
  }

  if ((millis() - ros_time) >= ros_period) { // Publish the pub_msg
    ros_time = millis();
    
    pub_msg.header.stamp = nh.now();
    pub.publish( &pub_msg);
    nh.spinOnce();
  }
}

void sensing_steering() {
  angle = 0;
  /*
  for (int i = 0; i < 10; i++) {
    angle += !digitalRead(ENCODER[i]) * pow(2, i);
  }*/
  /*
  angle += !digitalRead(ENCODER[0]);
  angle += !digitalRead(ENCODER[1])*2;
  angle += !digitalRead(ENCODER[2])*4;
  angle += !digitalRead(ENCODER[3])*8;
  angle += !digitalRead(ENCODER[4])*16;
  angle += !digitalRead(ENCODER[5])*32;
  angle += !digitalRead(ENCODER[6])*64;
  angle += !digitalRead(ENCODER[7])*128;
  angle += !digitalRead(ENCODER[8])*256;
  angle += !digitalRead(ENCODER[9])*512;
  */
  for (int i = 0; i < 10; i++) {
    if (!digitalRead(ENCODER[i])){
      angle +=  pow(2, i);
    }
  }
  
  pub_msg.steering_absolute_encoder = int(angle + 0.5);
  
  //preprocessing
  if(angle<500) { angle += 1024;}
  angle -= 500;
  
  steering_angle = 0.09053716*angle - 42.12905785; // convert encoder to angle (ackerman)
  pub_msg.steering_angle = steering_angle;
  
}

void process_steering() {
  delta_steer = steering_setpoint - steering_angle;

  pub_msg.steering_delta = delta_steer;

  if(abs(delta_steer) > steering_delta_min){
    if (delta_steer > 0) { // KANAN (CW)
      s1 = HIGH;
      s2 = LOW;
    }
    else if (delta_steer < 0) { // KIRI (CCW)
      s1 = LOW;
      s2 = HIGH;
    }
  } else {
    s1 = LOW;
    s2 = LOW;
  }
  
  if ( (s1 != s1_prev) || (s2 != s2_prev) ) {
    digitalWrite(S1, s1);
    digitalWrite(S2, s2);
    delayMicroseconds(10);
    digitalWrite(TRIGGER, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER, LOW);
    s1_prev = s1;
    s2_prev = s2;
  }
}

void sensing_braking(){
  float lin_dist = analogRead(LIN_ENC);
  pub_msg.brake_linear_encoder = int(lin_dist + 0.5);

  lin_dist = (lin_dist - braking_zero_offset) / 98.8;
  pub_msg.brake_position = lin_dist;
  braking_position = lin_dist;

  float current_read = analogRead(CS);
  pub_msg.brake_current_sensor = int(current_read + 0.5);

  current_read = braking_sign * current_read * 5 / 1023 / 0.13; //0.13 V per Ampere
  pub_msg.brake_current = current_read;
  braking_current = current_read;
}

void process_braking(){
  int pwm = 0;
  
  float delta_brake = braking_setpoint - braking_position;
  pub_msg.braking_delta = delta_brake;
  
  if (!braking_moving && abs(delta_brake) >= braking_delta_min_stay){
    braking_moving = true;
  }
  else if (braking_moving && abs(delta_brake) <= braking_delta_min_move){
    braking_moving = false;
  }  
  
  if(braking_moving){
    bool b1 = LOW;
    braking_sign = -1;
    if(delta_brake >= 0){
      b1 = HIGH;
      braking_sign = 1;
    }

    digitalWrite(INA, b1);
    digitalWrite(INB, !b1);

    pwm = 255;
    if (braking_setpoint == 0.0){ pwm = 200; }
    else if (braking_setpoint <= 0.25 * max_brake){ pwm = 100; } //0.3875cm
    else if (braking_setpoint <= 0.375 * max_brake){ pwm = 150; } //0.58125cm
    else if (braking_setpoint <= 0.60 * max_brake){ pwm = 155; } //0.93cm
    else if (braking_setpoint <= 0.75 * max_brake){ pwm = 165; } //1.1625cm
    else if (braking_setpoint <= 0.80 * max_brake){ pwm = 175; } //1.24cm
    else if (braking_setpoint <= 0.85 * max_brake){ pwm = 200; } //1.3175cm
    
    analogWrite(PWM, pwm);
  }
  else {
    digitalWrite(INA, LOW);
    digitalWrite(INB, LOW);

    braking_sign = 1;
    pwm = 0;
    analogWrite(PWM, pwm);
  }
  
  pub_msg.brake_pwm = pwm;
}
