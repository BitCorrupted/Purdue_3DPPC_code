/**
   ╔══════════════════════════════════════════════════════════╗
   ║ File Name: test_rig_code_V4.ino
   ║
   ║ Description:
   ║ Control code for the Evan Test Rig.
   ║
   ║ Version: 4
   ║
   ║ Last Modified By: Alex Khalitov
   ║
   ║ Last Modified On: 12/2/2023
   ║
   ║ Last Modified For: Fixed the EMG reading going negative. Now provides positive threshold.
   ║ Code is now stable and ready to ship in test rig, but is unstable for prosthetic use as it bounces
   ║ above and below threshold too readily.
   ╚══════════════════════════════════════════════════════════╝
*/

#include <SD.h>
#include <SPI.h>

//for SD Card
const int chipSelect = 10;

//general parameters
#define div_constant 10
#define SAMPLE_RATE 500
#define BAUD_RATE 9600
#define INPUT_PIN A0
#define POT_PIN A2
#define SWITCH A1 //will be used in digital mode
#define CAR_SIZE 20
#define LEDA 5
#define LEDB 4
#define LEDC 3
#define sample_delay 35 // milis delay in main loop


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
int clench_threshold_value = 8; // customize this threshold for each person
bool clench_state = false;


//Data Mode variables
bool is_testing;
bool switch_active;
int test_number = 0;
String base_name = "datalog.txt";

/**
   ╔════════════════════════════════════════════════════╗
   ║                             Setup Function
   ║
   ║ Setup function. Configures baud rate, pinmode, etc.
   ║
   ╚════════════════════════════════════════════════════╝
*/
void setup() {
  // Serial connection begin
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  /* Initializing LEDS */
  pinMode(LEDA, OUTPUT);
  pinMode(LEDB, OUTPUT);
  pinMode(LEDC, OUTPUT);
  pinMode(SWITCH, INPUT);

  //power led
  digitalWrite(LEDA, HIGH);
  
  /* Initializing sd Card */
  //Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1);
  }
  //Serial.println("card initialized.");
}

void loop() {
  //checking sensor value
  float sensor_value = abs(analogRead(INPUT_PIN));
  update_boxcar(sensor_value);

  //updating threshold
  update_threshold();

  //updating clench state
  if (abs(average) > clench_threshold_value) {
    clench_state = true;
    digitalWrite(LEDB, HIGH);
  }
  else {
    clench_state = false;
    digitalWrite(LEDB, LOW);
  }

  //checking switch value
  int switch_pressed = true; //for lack of a switch on the test rig, we are always recording.

  //checking current state
  if (switch_pressed && (is_testing == false)) {
    //start testing
    test_number++;
    is_testing = true;
    digitalWrite(LEDC, HIGH);
    File dataFile = SD.open(base_name, FILE_WRITE);
    if (dataFile) {
      dataFile.println("\nBegin test " + String(test_number) + " of session. Sample delay is " + String(sample_delay));
      dataFile.close();
    }
  }
  else if (!switch_pressed && (is_testing == true)) {
    //stop testing
    is_testing = false;
    digitalWrite(LEDC, LOW);
  }

  /*
     Writing to SD if testing
  */
  if (is_testing) {

    File dataFile = SD.open(base_name, FILE_WRITE);

    String dataString = String(average) + "," + "," + String(sensor_value) + "," + String(clench_state) + "," + String(clench_threshold_value);
    // if the file is available, write to it:
    if (dataFile) {
      dataFile.println(dataString);
      Serial.println(dataString);
      dataFile.close();
    }

  }
  delay(sample_delay);
  //Serial.println(average);
}

/**
   ╔════════════════════════════════════════════════════╗
   ║                                Update Threshold
   ║
   ║ Updates Threshold value based on user inputed Potentiometer Values
   ║                                                                         e
   ║
   ║ @return void (output is provided through global variable clench_threshold_value)
   ╚════════════════════════════════════════════════════╝
*/
void update_threshold(void) {
  int pot_val = analogRead(POT_PIN);
  clench_threshold_value = pot_val / div_constant;
}

/**
   ╔════════════════════════════════════════════════════╗
   ║                                EMG Boxcar Averager
   ║
   ║ Calculates the rolling average the EMG sensor values.
   ║
   ║ This function takes a sensor value as an input, and has an array boxcar, with
   ║ globals bcsum, bcn, average, and constant CAR_SIZE. boxcar is a FILO buffer;
   ║ When the function is called with a sensor_value, sensor_value is filtered, and the
   ║ magnitude is taken; the first-in value in boxcar is popped, the filtered sensor
   ║ value is inserted, and an average is taken of the entire boxcar array. Thus, the
   ║ average variable is a rolling average of the past 50 filtered abs sensor values.
   ║ https://en.wikipedia.org/wiki/Boxcar_averager
   ║
   ║ @param sensor_value: EMG sensor value
   ║
   ║ @return void (output is provided through global variable average)
   ╚════════════════════════════════════════════════════╝
*/
void update_boxcar(float sensor_value) {
  //filter signal
  float filtered_in = EMGFilter(sensor_value);
  if (filtered_in < 0) {
    filtered_in = -1 * filtered_in;
  }
  //updating boxcar array
  bcsum =  bcsum - boxcar[bcn];
  bcsum =  bcsum + filtered_in;
  boxcar[bcn] = filtered_in;
  bcn = bcn + 1;
  if (bcn >= CAR_SIZE) {
    bcn = 0;
  }
  average = bcsum / CAR_SIZE;
}

/**
   ╔════════════════════════════════════════════════════╗
   ║                                EMGFilter
   ║
   ║ Fourth-order software filter for EMG sensor value.
   ║
   ║ Band-Pass Butterworth IIR digital filter, generated using filter_gen.py.
   ║ Sampling rate: 500.0 Hz, frequency: [74.5, 149.5] Hz.
   ║ Filter is order 4, implemented as second-order sections (biquads).
   ║ Reference:
   ║ https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.butter.html
   ║ https://courses.ideate.cmu.edu/16-223/f2020/Arduino/FilterDemos/filter_gen.py
   ║
   ║ @param input: EMG sensor value
   ║
   ║ @return float output: Filtered sensor value.
   ╚════════════════════════════════════════════════════╝
*/

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
