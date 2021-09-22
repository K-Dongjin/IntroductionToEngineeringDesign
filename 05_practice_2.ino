#define PIN_LED 7
unsigned int count, toggle;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(115200); // initialize seral port.
  while (!Serial) {
    ; // wait for serial port to connect.
  }
  count = toggle = 1;
  digitalWrite(PIN_LED, 0);
  delay(1000); // turn on LED while 1seconds.
}

void loop() {
  for (count = 1; count < 11; count++) {
    toggle = count % 2;
    toggle = toggle_state(toggle); // toggle LED value.
    digitalWrite(PIN_LED, toggle); // update LED status.
    delay(100); // wati for 100 milliseconds.
  }
  while(true) {
    digitalWrite(PIN_LED, 1); // infinite loop(turn off LED).
  }
}

int toggle_state(int toggle) {
  return toggle;
} 
