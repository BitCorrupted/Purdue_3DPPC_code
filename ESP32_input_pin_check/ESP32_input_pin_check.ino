int pins[] = {0,2,4,5,12,13,14,15,16,17,18,19,21,22,23,25,26,27,32,33,34,35,36,37,38,39};
int beginning = 19;
int end = 20;

void setup() {
  // put your setup code here, to run once:

  for (int i = beginning; i < end; i++){
    pinMode(pins[i], INPUT);
  }

  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:

  for (int i = beginning; i < (end-1); i++){
    Serial.print(analogRead(pins[i]) * 3.3 / 4095);
    Serial.print(',');
  }
  Serial.println(analogRead(pins[end]) * 3.3 / 4095);

  delay(10);
}
