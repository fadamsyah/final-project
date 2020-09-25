#include <ros.h>
#include <Wire.h>
#include <pkg_ta/LogBrakeCommand.h>
#include <pkg_ta/LogArduino.h>
#define BAUD 500000

/********************** PIN CONFIGURATION *************************/
#define RPWM 44
#define LPWM 45
#define LIN_ENC A1
#define R_IS A2
#define L_IS A3
#define res_val 680.0f // Ohm (Value of The Resistor)
#define current_gain (8500.0f*4.95f/1023.0f/res_val);
/******************************************************************/

/********************** TIMING *************************/
int update_braking_period = 1000; // Microsecond, 1 kHz
int ros_period = 5; // Milisekon, 200 Hz
unsigned long update_braking_time = 0; // Microsecond
unsigned long ros_time = 0; // Milisecond
/*******************************************************/

/************* BRAKING GLOBAL VARIABLE *************/
float braking_setpoint = 0; // cm
float braking_position = 0; // cm
float braking_delta_min_move = 0.025; // cm (must be less than stay)
float braking_delta_min_stay = 0.100; // cm (must be greater than move)
float braking_zero_offset = 0.0f;
bool braking_moving = false;
unsigned int pwm_ref = 0;
String braking_state = "STOP"; // STOP | PULL | RELEASE
#define max_brake 3.0f
#define min_brake 0.0f
#define breaking_gradient 0.01012f
#define breaking_bias 0.0f
#define bf_coeff 0.f
/***************************************************/

ros::NodeHandle  nh;
pkg_ta::LogArduino pub_msg;
ros::Publisher pub("logging_arduino", &pub_msg);

void receive_message(const pkg_ta::LogBrakeCommand& sub_msg) {
  braking_setpoint = min(max(sub_msg.action_brake, min_brake), max_brake);
  pub_msg.brake_setpoint = braking_setpoint;
  pwm_ref = min(max(sub_msg.pwm, 0), 255);
}

ros::Subscriber<pkg_ta::LogBrakeCommand> sub("control_signal", &receive_message );

void setup() {
  // For Arduino Mega: Set PWM frequency for D44, D45 & D46
  // set timer 5 divisor to 8 for PWM frequency of 3921.16 Hz
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

  pub_msg.header.frame_id = "/log_arduino_mega";
  pub_msg.brake_setpoint = braking_setpoint;
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
    else if (braking_setpoint <= 0.25 * max_brake){ pwm = 100; } // 0.75 cm
    else if (braking_setpoint <= 0.375 * max_brake){ pwm = 110; } // 1.125 cm
    else if (braking_setpoint <= 0.60 * max_brake){ pwm = 130; } // 1.8 cm
    else if (braking_setpoint <= 0.75 * max_brake){ pwm = 150; } // 2.25 cm
    else if (braking_setpoint <= 0.80 * max_brake){ pwm = 175; } // 2.4 cm
    else if (braking_setpoint <= 0.85 * max_brake){ pwm = 200; } // 2.55 cm

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
