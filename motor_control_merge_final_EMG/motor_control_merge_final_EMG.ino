#include <math.h> // For the sin function

//Ouput Constants
#define PWM 4  // Example pin, can be used for PWM
#define IN1 25  // Example pin, can be used for digital I/O
#define IN2 26  // Example pin, can be used for digital I/O
#define m1_encB 14  // Example pin, can be used for digital I/O and external interrupts
#define m1_encA 32  // Example pin, can be used for digital I/O and external interrupts
#define PWM_CHANNEL 4
#define PWM_RESOLUTION 8
#define PWM_FREQUENCY 4000

//current sensing
#define c_in 2 // Higher voltage for current sensor
#define c_out 36 // Lower voltage for current sensor

//input constants
#define SAMPLE_RATE 500
#define BAUD_RATE 115200
#define INPUT_PIN 27
#define CAR_SIZE 50

//Position of the motors UPDATE WITH THE IDLE POSITIONS FOR THE HAND
int pos1 = 2;
int pos2 = 0;
int pos3 = 0;
int pos4 = 0;
int pos5 = 0;
int pos6 = 0;
int dir = 0;

int setPoint = 0;

//Controller variables
long prev_T  = 0;

//PID Gains (could also be placed inside the void loop)
float Kp = 6; 
float Ki = .001; 
float Kd = .4; 

// Parameters for the sine wave
const float amplitude = 1000.0; // Maximum deviation from the midpoint
const float frequency = 1.2;  // Number of cycles per second
const float midpoint = 0.0;   // Center point of the sine wave


//Boxcar averager (for smooth EMG reading)
int bcsum_emg = 0;
int bcsum_current = 0;
int boxcar_emg[CAR_SIZE];
int boxcar_current[CAR_SIZE];
int bcn_emg = 0;
int bcn_current = 0;
float average_emg;
float average_current;
int counter = 0;
int counter_limit = 25;

//Arm state machine variables
bool engage_arm_clench = false;
float clench_threshold_value = 60; // right? 
bool clench_state = false;

int go = 0;

//avging constants for current sensing
const int bufferSize = 10; // Size of the moving average buffer, adjust as needed
float buffer[bufferSize];
int bufferIndex = 0;
float sum = 0;
float movingAverage = 0;

// Threshold value
float threshold = .1; // Set your threshold value here
bool thresholdTriggered = false;

// Delay for current sensing
int currentDelayCount = 0;
int currentLimitActive = 0;
int currentLimitTriggered = 0;

void setup() {
  pinMode(m1_encA, INPUT);
  pinMode(m1_encB, INPUT);

  pinMode(c_in, INPUT);
  pinMode(c_out, INPUT);

  pinMode(INPUT_PIN, INPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(PWM, OUTPUT);

  Serial.begin(115200);

  // Setup PWM
  ledcSetup(PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(PWM, PWM_CHANNEL);

  //moving average buffer instance
  for (int i = 0; i < bufferSize; i++) {
  buffer[i] = 0;
  }

  attachInterrupt(digitalPinToInterrupt(m1_encA),incrementEncoder, RISING);
}

void loop() {
  //output stuffs
  long currentTime = micros();

  float sensor_value = analogRead(INPUT_PIN);
  float EMG = abs(EMGFilter(sensor_value));
  float EMG_avg = update_boxcar(0, EMG);

  if(EMG_avg > clench_threshold_value){
    engage_arm_clench = 1;
  }
  else{
    engage_arm_clench = 0;
  }

  if (engage_arm_clench){
    setPoint = 1000;
  }
  else{
    setPoint = 0;
  }

  //------------------------------------

  //current sensing 
  float c_before = analogRead(c_in) * 3.3 / 4095;
  float c_after = analogRead(c_out) * 3.3 / 4095;
  float v_diff = c_before - c_after;

  // Update moving average
  //movingAverage = update_boxcar(v_diff);

  // Check threshold
  thresholdTriggered = (movingAverage > threshold);
  if (thresholdTriggered){
    //dir = 0;
  }

  motor(setPoint,pos1);
  
  delay(10);

  // Space for printing for debugging
  Serial.print(EMG);
  Serial.print(',');
  Serial.println(engage_arm_clench*50);
}

void Motor_pwr(int dir, int PWM_val, int PWM_pin, int in1, int in2) {
  
  ledcWrite(PWM_CHANNEL, PWM_val); // Use ledcWrite for ESP32 PWM

  // if ((dir == 1) && (thresholdTriggered) && (currentDelayCount > 400)){
  //   dir = 0;
  //   setPoint = pos1;
  //   currentLimitTriggered = 1;
  // }
  // if (pos1 < 25){
  //   currentLimitTriggered = 0;
  // }

  if(dir == 1) {
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in2,HIGH);
    digitalWrite(in1,LOW);
  }
  else if(dir == 0){
    digitalWrite(in2,HIGH);
    digitalWrite(in1,HIGH);
  }
}

float getSetPoint(long currentTime) {
  float timeInSeconds = ((float)currentTime) / 1.0e6;  // Convert microseconds to seconds
  float setPoint = midpoint + amplitude * sin(2.0 * M_PI * frequency * timeInSeconds);
  return setPoint;
}

void incrementEncoder() {
  pos1 = pos1 + dir;

  if (dir == 1){
    currentDelayCount++;
  }
  else{
    currentDelayCount = 0;
  }
}

void motor(int targ, int pos) {
  long current_T = micros();
  float delta_T = ((float)(current_T - prev_T))/1.0e6;
  prev_T = current_T;
  
  static float ep = 0;
  static float ei = 0;
  
  float err = targ - pos;
  float derivative = (err - ep) / delta_T;
  float integral = ei + (err * delta_T);
  float output = Kp * err + Ki * integral + Kd * derivative;

  float pwr = fabs(output) * 2;
  if (pwr > 1000) {
    pwr = 1000;
  }

  dir = 1;
  if(output < targ) {
    dir = -1;
  }

  Motor_pwr(dir, pwr, PWM, IN1, IN2);
  
  ep = err;
}

float EMGFilter(float input)
{
  float output = input;
  {
    static float z1, z2; // filter section state
    float x = output - 0.05159732*z1 - 0.36347401*z2;
    output = 0.01856301*x + 0.03712602*z1 + 0.01856301*z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2; // filter section state
    float x = output - -0.53945795*z1 - 0.39764934*z2;
    output = 1.00000000*x + -2.00000000*z1 + 1.00000000*z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2; // filter section state
    float x = output - 0.47319594*z1 - 0.70744137*z2;
    output = 1.00000000*x + 2.00000000*z1 + 1.00000000*z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2; // filter section state
    float x = output - -1.00211112*z1 - 0.74520226*z2;
    output = 1.00000000*x + -2.00000000*z1 + 1.00000000*z2;
    z2 = z1;
    z1 = x;
  }
  return output;
}

float update_boxcar(int emg_select, float filtered_in) {
  //updating boxcar array
  //emg_select determines which boxcar to use: 0 for emg
  if (emg_select == 0) {
    bcsum_emg -= boxcar_emg[bcn_emg];
    bcsum_emg += filtered_in;
    boxcar_emg[bcn_emg] = filtered_in;
    bcn_emg += 1;
    if (bcn_emg >= CAR_SIZE)
      bcn_emg = 0;
    average_emg = bcsum_emg / CAR_SIZE;
    return average_emg;
  }
  //emg_select determines which boxcar to use 1 for current
  else {
    bcsum_current -= boxcar_current[bcn_current];
    bcsum_current += filtered_in;
    boxcar_current[bcn_current] = filtered_in;
    bcn_current += 1;
    if (bcn_current >= CAR_SIZE)
      bcn_current = 0;
    average_current = bcsum_current / CAR_SIZE;
    return average_current;
  }
}
