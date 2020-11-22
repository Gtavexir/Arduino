#include <Servo.h>

/////////////////////////////
// Configurable parameters 값 설정하기!!!!//
/////////////////////////////

// Arduino pin assignment
#define PIN_LED 9
#define PIN_SERVO 10
#define PIN_IR A0

// Framework setting
#define _DIST_TARGET 255
#define _DIST_MIN 10
#define _DIST_MAX 410

// Distance sensor
#define _DIST_ALPHA 0.35  //ema 필터의 alpha 값을 0.0으로 설정

// Servo range
#define _DUTY_MIN 1210    //서보의 가동 최소 각도(0)
#define _DUTY_NEU 1410    //servo neutral position (90 degree)
#define _DUTY_MAX 1610    //서보의 최대 가동 각도(180º)

// Servo speed control
#define _SERVO_ANGLE 20   //레일플레이트가 사용자가 원하는 가동범위를 움직일때, 이를 움직이게 하는 서보모터의 가동범위
#define _SERVO_SPEED 30   //서보 속도를 30으로 설정

// Event periods
#define _INTERVAL_DIST 20   //Distance Sensing을 20(ms) 마다 실행한다.
#define _INTERVAL_SERVO 20 //서보를 20(ms)마다 조작하기
#define _INTERVAL_SERIAL 100 //시리얼 0.1초 마다 업데이트

// PID parameters
#define _KP 0.725       //비례상수 설정

// ir_filter
#define _INTERVAL_DIST 30  // DELAY_MICROS * samples_num^2 의 값이 최종 거리측정 인터벌임. 넉넉하게 30ms 잡음.
#define DELAY_MICROS  1500 // 필터에 넣을 샘플값을 측정하는 딜레이(고정값!)
float ema_dist=0.0;          // EMA 필터에 사용할 변수
float samples_num = 3;     // 스파이크 제거를 위한 부분필터에 샘플을 몇개 측정할 것인지. 3개로 충분함! 가능하면 수정하지 말 것.

//////////////////////
// global variables //
//////////////////////

// Servo instance
Servo myservo;
// Distance sensor
float dist_target; // location to send the ball 
float dist_raw, dist_ema;    //실제 거리측정값과 ema필터를 적용한 값을 저장할 변수

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; //거리, 서보, 시리얼에 대한 마지막 샘플링 시간을 나타내는 변수
bool event_dist, event_servo, event_serial; //거리센서, 서보, 시리얼 모니터의 발생을 지시하는 변수

// Servo speed control
int duty_chg_per_interval;    //주기동안 duty 변화량 변수
int duty_target, duty_curr=_DUTY_NEU;    //서보의 목표위치와 서보에 실제로 입력할 위치

// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm; //비례 제어를 위한 전에 측정한 오차, 새로 측정한 오차 값, 비례항, 적분항, 미분항 변수

void setup() {
// initialize GPIO pins for LED and attach servo 
  pinMode(PIN_LED, OUTPUT);
  myservo.attach(PIN_SERVO);
// initialize global variables
  last_sampling_time_dist = last_sampling_time_servo = last_sampling_time_serial = 0;
  event_dist = event_servo = event_serial = false;
  dist_target = _DIST_TARGET; //목표지점 변수 초기화


// move servo to neutral position
  myservo.writeMicroseconds(_DUTY_NEU); //서보를 중간으로 이동
// initialize serial port
  Serial.begin(57600);
// convert angle speed into duty change per interval.
  duty_chg_per_interval = int((_DUTY_MAX - _DUTY_MIN) * (_SERVO_SPEED /  _SERVO_ANGLE) * (float(_INTERVAL_SERVO) / 1000.0));   //서보의 각속도를 원하는 Angle로 나누어 interval을 설정한다.
}
  

void loop() {
/////////////////////
// Event generator //
/////////////////////
  unsigned long time_curr = millis();
  if(time_curr >= last_sampling_time_dist + _INTERVAL_DIST) {
        last_sampling_time_dist += _INTERVAL_DIST;
        event_dist = true;
  }
  if(time_curr >= last_sampling_time_servo + _INTERVAL_SERVO) {
        last_sampling_time_servo += _INTERVAL_SERVO;
        event_servo = true;
  }
  if(time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL) {
        last_sampling_time_serial += _INTERVAL_SERIAL;
        event_serial = true;
  }


////////////////////
// Event handlers //
////////////////////

  if(event_dist) {
    event_dist = false;
  // get a distance reading from the distance sensor
    dist_raw = ir_distance_filtered();  //적외선 선세로 측정한 값에 필터를 적용한 값
    
  // PID control logic
    error_curr = _DIST_TARGET - dist_raw; //목표값 에서 현재값을 뺀 값이 오차값
    pterm = error_curr; 
    control = _KP * (pterm);

  // duty_target = f(duty_neutral, control)
    if(control <= 0) duty_target = (float)(_DUTY_NEU-_DUTY_MIN)/(0-(225-410)) * control + (float)_DUTY_NEU;
    else duty_target = (float)(_DUTY_MAX-_DUTY_NEU)/((225-10)-0) * control + (float)_DUTY_NEU;
  // keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
    if (duty_target < _DUTY_MIN) {
      duty_target = _DUTY_MIN;  //duty_target < _DUTY_MIN 일 때 duty_target 를 _DUTY_MIN 로 고정
    } else if (duty_target > _DUTY_MAX) {
       duty_target = _DUTY_MAX; // duty_target > _DUTY_MAX 일 때 duty_target 를 _DUTY_MAX 로 고정
    }  //_DUTY_MIN, _DUTY_MAX] 로 서보의 가동범위를 고정하기 위한 최소한의 안전장치
  }
  
  if(event_servo) {
    event_servo = false;
    // adjust duty_curr toward duty_target by duty_chg_per_interval
    if(duty_target > duty_curr) {
      duty_curr += duty_chg_per_interval;
      if(duty_curr > duty_target) duty_curr = duty_target;
    }  
    else {
      duty_curr -= duty_chg_per_interval;
      if(duty_curr < duty_target) duty_curr = duty_target;
    }       //서보가 현재위치에서 목표위치에 도달할 때까지 duty_chg_per_interval값 마다 움직임(duty_curr에 duty_chg_per_interval값 더하고 빼줌)
    
    // update servo position
    myservo.writeMicroseconds(duty_curr);   //위에서 바뀐 현재위치 값을 갱신
  }
  
  if(event_serial) {
    event_serial = false;
    Serial.print("dist_ir:");
    Serial.print(dist_raw);
    Serial.print(",pterm:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",duty_target:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",duty_curr:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",Min:100,Low:200,dist_target:255,High:310,Max:410");
  }
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

float _dist_[] = {71, 130, 165, 205, 262, 327, 391};

float ir_distance_filtered(void){ // return value unit: mm
  float val = filtered_ir_distance();
  if(val <= _dist_[1]) return 50.0/(_dist_[1]-_dist_[0])*val + 100.0 - 50.0/(_dist_[1]-_dist_[0])*_dist_[0];
  else if(val <= _dist_[2]) return 50.0/(_dist_[2]-_dist_[1])*val + 150.0 - 50.0/(_dist_[2]-_dist_[1])*_dist_[1];
  else if(val <= _dist_[3]) return 50.0/(_dist_[3]-_dist_[2])*val + 200.0 - 50.0/(_dist_[3]-_dist_[2])*_dist_[2];
  else if(val <= _dist_[4]) return 50.0/(_dist_[4]-_dist_[3])*val + 250.0 - 50.0/(_dist_[4]-_dist_[3])*_dist_[3];
  else if(val <= _dist_[5]) return 50.0/(_dist_[5]-_dist_[4])*val + 300.0 - 50.0/(_dist_[5]-_dist_[4])*_dist_[4];
  else return 50.0/(_dist_[6]-_dist_[5])*val + 350.0 - 50.0/(_dist_[6]-_dist_[5])*_dist_[5];
  return val;
}

float under_noise_filter(void){ // 아래로 떨어지는 형태의 스파이크를 제거해주는 필터
  float currReading;
  float largestReading = 0.0;
  for (int i = 0; i < samples_num; i++) {
    currReading = ir_distance();
    if (currReading > largestReading) { largestReading = currReading; }
    // Delay a short time before taking another reading
    delayMicroseconds(DELAY_MICROS);
  }
  return largestReading;
}

float filtered_ir_distance(void){ // 아래로 떨어지는 형태의 스파이크를 제거 후, 위로 치솟는 스파이크를 제거하고 EMA필터를 적용함.
  // under_noise_filter를 통과한 값을 upper_nosie_filter에 넣어 최종 값이 나옴.
  float currReading;
  float lowestReading = 1024.0;
  for (int i = 0; i < samples_num; i++) {
    currReading = under_noise_filter();
    if (currReading < lowestReading) { lowestReading = currReading; }
  }
  // eam 필터 추가
  ema_dist = _DIST_ALPHA*lowestReading + (1.0-_DIST_ALPHA)*ema_dist;
  return ema_dist;
}
