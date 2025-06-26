/*
  Read two potentiometers (A0, A1) and print values to Serial.
  Connect each pot’s wiper to the analog pin, the outer legs to +5 V and GND.
*/

const uint8_t POT_A0_PIN = A0;
const uint8_t POT_A1_PIN = A1;
float val0;
float val1;
float ref0;
float ref1;

void setup() {
  Serial.begin(9600);          // Open serial port at 9600 bps
  pinMode(POT_A0_PIN, INPUT);  // (optional) analog pins default to INPUT
  pinMode(POT_A1_PIN, INPUT);
  ref0 = analogRead(POT_A0_PIN);
  ref1 = analogRead(POT_A1_PIN);
}

void loop() {
  int potA0 = analogRead(POT_A0_PIN);  // 0–1023
  int potA1 = analogRead(POT_A1_PIN);
  val0 = abs(360*(potA0-ref0)/1023);
  val1 = abs(360*(potA1-ref1)/1023);

  // Send as tab-separated values: easy to paste into a spreadsheet
  Serial.print(val0);
  Serial.print('\t');
  Serial.println(val1);

  delay(10);  // ≈100 Hz update rate; adjust or remove as needed
}
