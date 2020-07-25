/*
  ENA LOW --> Nyala
  ENA HIGH --> Mati
  DIR LOW --> Clockwise (KANAN)
  DIR HIGH --> Counter Clockwise (KIRI)
*/

#define ENA  5 // Enable pin
#define DIR  4 // Direction pin
#define PUL  3 // Pulse pin
#define S1  6 // Direction Pin
#define S2  7 // Direction Pin

int initial = 600; // Mikrosekon || The maximum timing of the stepper motor
float delay_micros = initial; // Mikrosekon || The instantaneous timing of the stepper motor
int limit_1 = 450; // Microsecond || The limit of the stepper's timing of the first acceleration
int limit_2 = 200; // Microsecond || The limit of the stepper's timing of the second acceleration
int limit_3 = 150; // Microsecond || The limit of the stepper's timing of the third acceleration
                   // i.e. the MINIMUM timing of the stepper motor
int increment_1 = -1; // The first acceleration of the stepper motor
                      // At the begining, the required torque is large because of the friction force acting to the tire
                      // Therefore, the acceleration must be small in the begining of the movement of the stepper motor
int increment_2 = -3; // The second acceleration of the stepper motor
float increment_3 = -0.5; // The third acceleration of the stepper motor
                          // When the timing is low, i.e. the movement of the stepper is extremely fast, the acceleration must be
                          // small. Otherwise, the stepper will fail to follow the reference current
int max_delay = 650; // If the delay between the two pulses is too long,
                     // the stepper will fail if the timing at that time is small.
                     // Therefore, the timing of the stepper must be reset to initial timing

unsigned long start_time = 0; 
unsigned long run_time = 0;

volatile bool move_ = LOW; // State of the stepper motor, STALL (LOW) or MOVING (HIGH)
volatile bool step_logic = HIGH; // The step logic (the driver needs sequence of high-low pulse to drive the stepper motor)
volatile bool dir = HIGH; // The direction of the rotation or movement
volatile bool prev_dir = LOW; // The previous direction of the rotation or movement

int now = 0;
bool s1,s2;

void setup() {
  Serial.begin(115200);
  Serial.println("BEGIN");
  
  pinMode(ENA, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(PUL, OUTPUT);
  pinMode(S1, INPUT);
  pinMode(S2, INPUT);
  
  digitalWrite(ENA, LOW); // Turn ON the stepper motor
  digitalWrite(DIR, HIGH); // Set the direction of the movement (for initialization)

  attachInterrupt(digitalPinToInterrupt(2), interrupt, RISING); // Set the interrupt pin to detect whether
                                                                // there is a signal from the Arduino Mega or not
  Serial.println("LOOP");
}

void loop() {
  now = micros() - start_time;
  
  if ( now > delay_micros) {
    if ( now > max_delay ) { 
      // If the delay between the two pulses is too long,
      // the timing will be reset to the initial timing  
      delay_micros = initial;
    } 

    start_time = micros();
    if (move_) { // If the stepper state is MOVING
      digitalWrite(PUL, step_logic);
      step_logic = !step_logic; // Update the step logic

      // Accelerating the stepper motor
      if (delay_micros > limit_3) {
        if (delay_micros > limit_1) {
          delay_micros += increment_1;
        }
        else if (delay_micros > limit_2) {
          delay_micros += increment_2;
        }
        else {
          delay_micros += increment_3;
        }
      }
    }
    
  }
}

void interrupt()
{
  s1 = digitalRead(S1);
  s2 = digitalRead(S2);

  move_ = HIGH;
  if (s1 && !s2){
    dir = LOW;
  }
  else if (!s1 && s2){
    dir = HIGH;
  }
  else{
    delay_micros = initial;
    move_ = LOW;
  }

//  if(move_){
//    digitalWrite(ENA, LOW);
//  } else {
//    digitalWrite(ENA, HIGH);
//  }
  
  // If the commanded direction is changed, update the direction and drive the stepper motor to the desired direction.
  // Note that the direction is updated ONLY when direction is changed.
  // Also, the timing is reset to the initial timing
  if (dir != prev_dir) {
    digitalWrite(DIR, dir);
    prev_dir = dir;
    // step_logic = dir; // I think, this line of code doesn't matter//
    delay_micros = initial;
  }
  Serial.println(dir);
  
  //Serial.print("DIR : ");
  //Serial.print(dir);
  //Serial.print("  |  move_ : ");
  //Serial,k.println(move_);
}
