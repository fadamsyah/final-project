/*
  ENA LOW --> Nyala
  ENA HIGH --> Mati
  DIR LOW --> Clockwise (KANAN)
  DIR HIGH --> Counter Clockwise (KIRI)
*/

#define PUL  4 // Pulse pin
#define DIR  3 // Direction pin
#define ENA  5 // Enable pin
#define S1  6 // Direction Pin
#define S2  7 // Direction Pin

#define initial  600 // Mikrosekon || The maximum timing of the stepper motor
#define max_delay  650 // If the delay between the two pulses is too long,
                       // the stepper will fail if the timing at that time is small.
                       // Therefore, the timing of the stepper must be reset to initial timing
#define N 5
const int limit[N] = {450, 200, 150, 125, 100}; // Microsecond || The limit of the stepper's timing of the first, second, & third acceleration
                                                     // i.e. the MINIMUM timing of the stepper motor
const float increment[N] = {-1.f, -3.f, -0.5f, -0.05f, -0.005f}; // At the begining, the required torque is large because of the friction force acting to the tire
                                                  // Therefore, the acceleration must be small in the begining of the movement of the stepper motor
                                                  // When the timing is low, i.e. the movement of the stepper is extremely fast, the acceleration must be
                                                  // small. Otherwise, the stepper will fail to follow the reference current                          

float delay_micros = initial; // Mikrosekon || The instantaneous timing of the stepper motor


unsigned long start_time = 0; 
unsigned long run_time = 0;

volatile bool move_ = LOW; // State of the stepper motor, STALL (LOW) or MOVING (HIGH)
volatile bool step_logic = HIGH; // The step logic (the driver needs sequence of high-low pulse to drive the stepper motor)
volatile bool dir = HIGH; // The direction of the rotation or movement
volatile bool prev_dir = dir; // The previous direction of the rotation or movement

int dt = 0;
bool s1,s2;

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(PUL, OUTPUT);
  pinMode(S1, INPUT);
  pinMode(S2, INPUT);
  
  digitalWrite(ENA, LOW); // Turn ON the stepper motor
  digitalWrite(DIR, dir); // Set the direction of the movement (for initialization)

  attachInterrupt(digitalPinToInterrupt(2), interrupt, RISING); // Set the interrupt pin to detect whether                                                               // there is a signal from the Arduino Mega or not
}

void loop() {
  dt = micros() - start_time;
  
  if ( dt > delay_micros) {
    start_time = micros();
    
    if ( dt > max_delay ) { 
      // If the delay between the two pulses is too long,
      // the timing will be reset to the initial timing  
      delay_micros = initial;
    } 

    if (move_) { // If the stepper state is MOVING
      digitalWrite(PUL, step_logic);
      step_logic = !step_logic; // Update the step logic

      // Accelerating the stepper motor
      for (int i=0; i < N; i++){
        if (delay_micros > limit[i]){
          delay_micros += increment[i];
          break;
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
  
  // If the commanded direction is changed, update the direction and drive the stepper motor to the desired direction.
  // Note that the direction is updated ONLY when direction is changed.
  // Also, the timing is reset to the initial timing
  if (dir != prev_dir) {
    digitalWrite(DIR, dir);
    prev_dir = dir;
    // step_logic = dir; // I think, this line of code doesn't matter//
    delay_micros = initial;
    delay(1); // Delay for 1 milisecond
  }
}
