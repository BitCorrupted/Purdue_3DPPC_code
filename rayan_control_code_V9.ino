/**
 * ╔══════════════════════════════════════════════════════════╗
 * ║ File Name: rayan_control_code_V9.ino                                                    
 * ║                                                                          
 * ║ Description:                                                            
 * ║ EMG control code, refactored for use with the ESP32.   
 * ║                                                                          
 * ║ Version: 9
 * ║ 
 * ║ Last Modified By: Alex Khalitov                                                  
 * ║                                                                         
 * ║ Last Modified On: 10/21/2023                                            
 * ║
 * ║ Last Modified For: Adding comments and function documentation, formatted by Ben Miller
 * ╚══════════════════════════════════════════════════════════╝
 */

 #include <SD.h>
 #include <SPI.h>

const int chipSelect = 10;
 
#define SAMPLE_RATE 500
#define BAUD_RATE 9600
#define INPUT_PIN A0
#define CAR_SIZE 50

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
float clench_threshold_value = 60; // customize this threshold for each person
bool clench_state = false;

/**
 * ╔════════════════════════════════════════════════════╗
 * ║                             Setup Function                          
 * ║
 * ║ Setup function. Configures baud rate, pinmode, etc.
 * ║                        
 * ╚════════════════════════════════════════════════════╝
 */
void setup() {
  // Serial connection begin
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }


  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1);
  }
  Serial.println("card initialized.");
}

void loop() {
  float sensor_value = analogRead(INPUT_PIN);
  update_boxcar(sensor_value);

  //used for debug; delete? when not in use


  delay(30);
  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  String dataString = String(average);
  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    // print to the serial port too:
    Serial.println(dataString);
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }
  digitalWrite(LED, clench_state);
}

/**
 * ╔════════════════════════════════════════════════════╗
 * ║                                EMG Boxcar Averager                          
 * ║                                                                         
 * ║ Calculates the rolling average the EMG sensor values.                           
 * ║                                                                         
 * ║ This function takes a sensor value as an input, and has an array boxcar, with                                    
 * ║ globals bcsum, bcn, average, and constant CAR_SIZE. boxcar is a FILO buffer; 
 * ║ When the function is called with a sensor_value, sensor_value is filtered, and the
 * ║ magnitude is taken; the first-in value in boxcar is popped, the filtered sensor
 * ║ value is inserted, and an average is taken of the entire boxcar array. Thus, the
 * ║ average variable is a rolling average of the past 50 filtered abs sensor values.
 * ║ https://en.wikipedia.org/wiki/Boxcar_averager
 * ║                                                                       
 * ║ @param sensor_value: EMG sensor value                                                                    
 * ║                                                                       
 * ║ @return void (output is provided through global variable average)                       
 * ╚════════════════════════════════════════════════════╝
 */
// Function to update the boxcar average
// 
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

/**
 * ╔════════════════════════════════════════════════════╗
 * ║                                EMGFilter                          
 * ║                                                                         
 * ║ Fourth-order software filter for EMG sensor value.                           
 * ║                                                                         
 * ║ Band-Pass Butterworth IIR digital filter, generated using filter_gen.py.                                   
 * ║ Sampling rate: 500.0 Hz, frequency: [74.5, 149.5] Hz.
 * ║ Filter is order 4, implemented as second-order sections (biquads).
 * ║ Reference: 
 * ║ https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.butter.html
 * ║ https://courses.ideate.cmu.edu/16-223/f2020/Arduino/FilterDemos/filter_gen.py
 * ║                                                                       
 * ║ @param input: EMG sensor value                                                                    
 * ║                                                                       
 * ║ @return float output: Filtered sensor value.                       
 * ╚════════════════════════════════════════════════════╝
 */
 
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
