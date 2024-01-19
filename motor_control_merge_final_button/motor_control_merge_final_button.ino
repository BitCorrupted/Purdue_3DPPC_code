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
#define c_in 35 // Higher voltage for current sensor
#define c_out 36 // Lower voltage for current sensor

//input constants
#define SAMPLE_RATE 500
#define BAUD_RATE 115200
#define INPUT_PIN 1
#define CAR_SIZE 50
#define button 27

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
int bcsum = 0;
int boxcar[CAR_SIZE];
int bcn = 0;
float average;
int LED = 16;
int counter = 0;
int counter_limit = 25;

//Arm state machine variables
bool engage_arm_clench = false;
float clench_threshold_value = 80; // right?
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

  pinMode(button, INPUT);

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

  attachInterrupt(digitalPinToInterrupt(m1_encA), incrementEncoder, RISING);
}

void loop() {
  //output stuffs
  long currentTime = micros();
  //int setPoint = (int) getSetPoint(currentTime);

  int buttonVal = digitalRead(button);
  //if ((buttonVal) && (!currentLimitTriggered)){
  if (buttonVal == HIGH) {
    setPoint = 1000;
  }
  else if (buttonVal == LOW) {
    setPoint = 0;
  }

  //Serial.print("sine: ");
  //Serial.print(setPoint);
  //Serial.print(" ");
  motor(setPoint, pos1);

  // input stuffs
  float sensor_value = analogRead(INPUT_PIN);
  update_boxcar(sensor_value);

  // Serial.println(average);

  delay(10);

  //XOR whether or not sensor reading is above threshold value, with saved clench_state.
  //If the state machine thinks its below the threshold (clench_state is false) and the value is above, set clench_state to be true. Rising edge, so engage arm.
  //If the machine thinks its above the threshold (clench_state is true) and the value is below, set clench_state to be true. Falling edge, so do nothing else.
  //If the machine is right about being on either edge of the threshold (clench_sate is in agreement with the threshold), do nothing.

  if ((average > clench_threshold_value)^clench_state) {
    clench_state = !clench_state;
    if (clench_state || buttonVal) {
      engage_arm_clench = !engage_arm_clench;
    }
  }

  digitalWrite(LED, clench_state);
  //-------------------------------------------------
  //current sensing
  float c_before = analogRead(c_in) * 3.3 / 4095;
  float c_after = analogRead(c_out) * 3.3 / 4095;
  float v_diff = c_before - c_after;

  // Update moving average
  sum -= buffer[bufferIndex];
  buffer[bufferIndex] = v_diff;
  sum += buffer[bufferIndex];
  bufferIndex = (bufferIndex + 1) % bufferSize;
  movingAverage = sum / bufferSize;

  // Check threshold
  thresholdTriggered = (movingAverage > threshold);
  if (thresholdTriggered) {
    //dir = 0;
  }

// Print values for debugging
  Serial.print("Moving Average: ");
  Serial.println(v_diff);
//  Serial.print(",");
//  Serial.print(movingAverage);
//  Serial.print(" Threshold Triggered: ");
//  Serial.print(",");
//  Serial.println(thresholdTriggered);
//
//  Serial.print(digitalRead(m1_encB));
//  Serial.print(",");
//  Serial.print(digitalRead(m1_encA));
//  Serial.print(",");
//  Serial.println(pos1);
//
//  Serial.print(thresholdTriggered*500);
//  Serial.print(",");
//  Serial.println(currentDelayCount);
//
//  Serial.print(c_before);
//  Serial.print(',');
//  Serial.println(c_after);
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

  if (dir == 1) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    //Serial.print("frwd");
  }
  else if (dir == -1) {
    digitalWrite(in2, HIGH);
    digitalWrite(in1, LOW);
    //Serial.print("back");
  }
  else if (dir == 0) {
    digitalWrite(in2, HIGH);
    digitalWrite(in1, HIGH);
    //Serial.print("stall");
  }
}

float getSetPoint(long currentTime) {
  float timeInSeconds = ((float)currentTime) / 1.0e6;  // Convert microseconds to seconds
  float setPoint = midpoint + amplitude * sin(2.0 * M_PI * frequency * timeInSeconds);
  return setPoint;
}

void incrementEncoder() {
  pos1 = pos1 + dir;

  if (dir == 1) {
    currentDelayCount++;
  }
  else {
    currentDelayCount = 0;
  }
}
void motor(int targ, int pos) {
  long current_T = micros();
  float delta_T = ((float)(current_T - prev_T)) / 1.0e6;
  prev_T = current_T;

  static float ep = 0;
  static float ei = 0;

  float err = targ - pos;
  float derivative = (err - ep) / delta_T;
  float integral = ei + (err * delta_T);
  float output = Kp * err + Ki * integral + Kd * derivative;

  //Serial.println(output);

  float pwr = fabs(output) * 2;
  if (pwr > 1000) {
    pwr = 1000;
  }

  dir = 1;
  if (output < targ) {
    dir = -1;
  }

  //Serial.print(" ");
  //Serial.print(dir);

  Motor_pwr(dir, pwr, PWM, IN1, IN2);

  ep = err;
}


float EMGFilter(float input)
{
  float output = input;
  {
    static float z1, z2; // filter section state
    float x = output - 0.05159732 * z1 - 0.36347401 * z2;
    output = 0.01856301 * x + 0.03712602 * z1 + 0.01856301 * z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2; // filter section state
    float x = output - -0.53945795 * z1 - 0.39764934 * z2;
    output = 1.00000000 * x + -2.00000000 * z1 + 1.00000000 * z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2; // filter section state
    float x = output - 0.47319594 * z1 - 0.70744137 * z2;
    output = 1.00000000 * x + 2.00000000 * z1 + 1.00000000 * z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2; // filter section state
    float x = output - -1.00211112 * z1 - 0.74520226 * z2;
    output = 1.00000000 * x + -2.00000000 * z1 + 1.00000000 * z2;
    z2 = z1;
    z1 = x;
  }
  return output;
}

void update_boxcar(float sensor_value) {
  //filter signal
  float filtered_in = abs(EMGFilter(sensor_value));
  //updating boxcar array
  bcsum -= boxcar[bcn];
  bcsum += filtered_in;
  boxcar[bcn] = filtered_in;
  bcn += 1;
  if (bcn >= CAR_SIZE)
    bcn = 0;
  average = bcsum / CAR_SIZE;
}
