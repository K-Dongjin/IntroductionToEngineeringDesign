#include <Servo.h>

/////////////////////////////
// Configurable parameters //
/////////////////////////////
// Arduino pin assignment
#define PIN_LED 9
#define PIN_SERVO 10
#define PIN_IR A0

// Framework setting
#define _DIST_TARGET 270
#define _DIST_MIN 100
#define _DIST_MAX 410

// Distance sensor
#define _DIST_ALPHA 0.3  // ema 필터에 적용할 알파값

// Servo range
#define _DUTY_MIN 500  // 서보의 최소 각도값
#define _DUTY_NEU 1300  // 서보의 중간 각도값
#define _DUTY_MAX 2000  // 서보의 최대 각도값

// Servo speed control
#define _SERVO_ANGLE 30  // 서보 각도 설정
#define _SERVO_SPEED 100  // 서보의 속도 설정

// Event periods
#define _INTERVAL_DIST 20  // 센서의 거리측정 인터벌값
#define _INTERVAL_SERVO 20  // 서보 INTERVAL값 설정
#define _INTERVAL_SERIAL 100  // 시리얼 모니터/플로터의 인터벌값 설정

// PID parameters
#define _KP 2.5  // 비례 제어 값

#define a 72.70  // 실제 거리가 100일 때 센서가 읽는 값
#define b 344.20  // 실제 거리가 400일 때 센서가 읽는 값



//////////////////////
// global variables //
//////////////////////

// Servo instance
Servo myservo;

// Distance sensor
float dist_target;  // location to send the ball
float dist_raw, dist_ema;  // dist_raw: 적외선센서로 얻은 거리를 저장하는 변수
                           // dist_ema: 거리를 ema필터링을 한 값을 저장하는 변수
float alpha;  // ema의 알파값을 저장할 변수

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial;  // last_sampling_time_dist: 거리센서 측정 주기
                                                                                             // last_sampling_time_servo: 서보 위치 갱신 주기
                                                                                             // last_sampling_time_serial: 제어 상태 시리얼 출력 주기
bool event_dist, event_servo, event_serial;  // 각각의 주기에 도달했는지를 불리언 값으로 저장하는 변수

// Servo speed control
int duty_chg_per_interval;  // 한 주기당 변화할 서보 활동량을 정의
int duty_target, duty_curr;  // 목표위치, 서보에 입력할 위치

// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm;  // error_curr: 목표값 - 측정값
                                                             // error_prev: 이전의 error_curr
                                                             // control: 비례 제어 상수



void setup() {
  // initialize GPIO pins for LED and attach servo 
  pinMode(PIN_LED, OUTPUT);  // LED를 GPIO 9번 포트에 연결
  myservo.attach(PIN_SERVO);  // 서보 모터를 GPIO 10번 포트에 연결
  
  // initialize global variables
  alpha = _DIST_ALPHA;  // ema의 알파값 초기화
  dist_ema = 0;  // dist_ema 초기화
  duty_curr = _DUTY_NEU;  // duty_curr 초기화

  // move servo to neutral position
  myservo.writeMicroseconds(_DUTY_NEU);  // 서보 모터를 중간 위치에 지정
    
  // initialize serial port
  Serial.begin(115200);  // 시리얼 포트를 115200 연결
  
  // convert angle speed into duty change per interval.
  duty_chg_per_interval = (_DUTY_MAX - _DUTY_MIN) * (_SERVO_SPEED / 180.0) * (_INTERVAL_SERVO / 1000.0);  // 한 주기마다 이동할 양(180.0, 1000.0은 실수타입이기 때문에 나눗셈의 결과가 실수타입으로 리턴)
  
  // 이벤트 변수 초기화
  last_sampling_time_dist = 0;  // 마지막 거리 측정 시간 초기화
  last_sampling_time_servo = 0;  // 마지막 서보 업데이트 시간 초기화
  last_sampling_time_serial = 0;  // 마지막 출력 시간 초기화
  event_dist = event_servo = event_serial = false;  // 각 이벤트 변수 false로 초기화
}



void loop(){
  /////////////////////
  // Event generator //
  /////////////////////
  unsigned long time_curr = millis();
  // event 발생 조건 설정
  if(time_curr >= last_sampling_time_dist + _INTERVAL_DIST) {
    last_sampling_time_dist += _INTERVAL_DIST;
    event_dist = true;  // 거리 측정 주기에 도달했다는 이벤트 발생
  }
  if(time_curr >= last_sampling_time_servo + _INTERVAL_SERVO) {
    last_sampling_time_servo += _INTERVAL_SERVO;
    event_servo = true;  // 서보모터 제어 주기에 도달했다는 이벤트 발생
  }
  if(time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL) {
    last_sampling_time_serial += _INTERVAL_SERIAL;
    event_serial = true;  // 출력주기에 도달했다는 이벤트 발생
  }

  ////////////////////
  // Event handlers //
  ////////////////////
  // get a distance reading from the distance sensor
  if(event_dist) { 
    event_dist = false;
    dist_raw = ir_distance_filtered();  // dist_raw에 필터링된 측정값 저장
    if (dist_ema == 0){                  
      dist_ema = dist_raw;  // 초기 ema값 = 필터링된 측정값
    }
    else{
      dist_ema = alpha * dist_raw + (1 - alpha) * dist_ema;  // ema 구현
    }
  }

  // PID control logic
  error_curr = _DIST_TARGET - dist_ema;  // 오차 설정
  pterm = error_curr;
  control = _KP * pterm;

  // duty_target = f(duty_neutral, control)
  duty_target = _DUTY_NEU - control;  // 목적지에서의 거리에 비례하여 목표 각도 변경

  // keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
  if(duty_target < _DUTY_MIN){duty_target = _DUTY_MIN;}
  else if(duty_target > _DUTY_MAX){duty_target = _DUTY_MAX;}

  if(event_servo){
    event_servo = false;
    // adjust duty_curr toward duty_target by duty_chg_per_interval
    if(duty_target > duty_curr){
      duty_curr += duty_chg_per_interval;
      if(duty_curr > duty_target){duty_curr = duty_target;}
    }
    else{
      duty_curr -= duty_chg_per_interval;
      if(duty_curr < duty_target){duty_curr = duty_target;}
    }
    // update servo position
    myservo.writeMicroseconds(duty_curr);
  }

  if(event_serial){
    event_serial = false;
    Serial.print("error_curr:");
    Serial.print(error_curr);
    Serial.print("Min:0,Low:200,dist:");
    Serial.print(dist_raw);
    Serial.print(",pterm:");
    Serial.print(pterm);
    Serial.print(",duty_target:");
    Serial.print(duty_target);
    Serial.print(",duty_curr:");
    Serial.print(duty_curr);
    Serial.println(",High:310,Max:2000");
  }
}


float ir_distance(void){  // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

float ir_distance_filtered(void){  // return value unit: mm
  float val = ir_distance();
  return 100 + 300.0 / (b - a) * (val - a);
}
