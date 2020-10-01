#define PIN_LED 7

void setup() {
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(115200); // Initialize serial port
  while (!Serial) {
    ; // wait for serial port to connect.
  }
}

void loop() {
    //digitalWrite(PIN_LED, 0);
    digitalWrite(PIN_LED, 1);    
}
