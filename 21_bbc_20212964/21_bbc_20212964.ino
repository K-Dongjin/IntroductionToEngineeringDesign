// Arduino pin assignment
#include <Servo.h>
#define PIN_IR A0
#define PIN_SERVO 10

Servo myservo;

#define _DIST_ALPHA 0.5 // EMA filter constant
#define _RAW_DIST_100 72 // The value when the actual distance is 100
#define _RAW_DIST_400 333 // The value when the actual distance is 400

// global variables
float dist_cali_ema, dist_cali_ema_bef, alpha; // for EMA filter
float raw_dist_100, raw_dist_400, dist_cali, dist_cali_bef, raw_dist; // for calibrated distance

void setup() {
// initialize serial port
  Serial.begin(57600);

// initialize USS related variables
  raw_dist_100 = _RAW_DIST_100;
  raw_dist_400 = _RAW_DIST_400;
  alpha = _DIST_ALPHA;
  dist_cali_ema_bef = 0;

  myservo.attach(PIN_SERVO); 
  myservo.writeMicroseconds(1410);
  delay(1000);
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

void loop() {
  raw_dist = ir_distance();
  if ((raw_dist < 50) || (raw_dist > 500)) {
    raw_dist = dist_cali_ema;
  }
  dist_cali = 100 + 300.0 / (raw_dist_400 - raw_dist_100) * (raw_dist - raw_dist_100);
  dist_cali_bef = dist_cali;
  dist_cali_ema = alpha * dist_cali + (1 - alpha) * dist_cali_ema_bef;
  dist_cali_ema_bef = dist_cali_ema;

  
  Serial.print("min: 10, max: 500, dist_cali: ");
  Serial.println(dist_cali);
  
  if (dist_cali_ema < 255) {
    myservo.writeMicroseconds(1310);
  }
  else {
    myservo.writeMicroseconds(1500);
  }
  delay(20);
}
