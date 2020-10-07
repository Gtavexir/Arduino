// Arduino pin assignment
#define PIN_LED 9
#define PIN_TRIG 12
#define PIN_ECHO 13

// configurable parameters
#define SND_VEL 346.0 // sound velocity at 24 celsius degree (unit: m/s)
#define INTERVAL 25 // sampling interval (unit: ms)
#define _DIST_MIN 100 // minimum distance to be measured (unit: mm)
#define _DIST_MAX 300 // maximum distance to be measured (unit: mm)
#define _DIST_ALPHA 0.3 // EMA weight of new sample (range: 0 to 1). Setting this value to 1 effectively disables EMA filter.

// global variables
float timeout; // unit: us
float dist_min, dist_max, dist_raw, dist_ema, alpha; // unit: mm
unsigned long last_sampling_time; // unit: ms
float scale; // used for pulse duration to distance conversion

void setup() {
// initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  pinMode(PIN_TRIG,OUTPUT);
  digitalWrite(PIN_TRIG, LOW);
  pinMode(PIN_ECHO,INPUT);

// initialize USS related variables
  dist_min = _DIST_MIN; 
  dist_max = _DIST_MAX;
  alpha = _DIST_ALPHA;
  timeout = (INTERVAL / 2) * 1000.0; // precalculate pulseIn() timeout value. (unit: us)
  dist_raw = 0.0; // raw distance output from USS (unit: mm)
  scale = 0.001 * 0.5 * SND_VEL;

// initialize serial port
  Serial.begin(57600);

// initialize last sampling time
  last_sampling_time = 0;
}

void loop() {
// wait until next sampling time. 
// millis() returns the number of milliseconds since the program started. Will overflow after 50 days.
  if(millis() < last_sampling_time + INTERVAL) return;

// get a distance reading from the USS
  dist_raw = USS_measure(PIN_TRIG,PIN_ECHO);
  dist_ema = Median_filter(dist_raw);

// output the read value to the serial port
  Serial.print("Min:0,");
  Serial.print("raw:");
  Serial.print(dist_raw);
  Serial.print(",");
  Serial.print("ema:");
  Serial.print(dist_ema);
  Serial.print(map(dist_ema,0,400,100,500));
  Serial.print(",");
  Serial.println("Max:500");

// turn on the LED if the distance is between dist_min and dist_max
  if(dist_raw < dist_min || dist_raw > dist_max) {
    analogWrite(PIN_LED, 255);
  }
  else {
    analogWrite(PIN_LED, 0);
  }

// update last sampling time
  last_sampling_time += INTERVAL;
}

// get a distance reading from USS. return value is in millimeter.
float USS_measure(int TRIG, int ECHO)
{
  float reading;
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  reading = pulseIn(ECHO, HIGH, timeout) * scale; // unit: mm
  if(reading < dist_min || reading > dist_max) reading = 0.0; // return 0 when out of range.
  return reading;
}


int N = 3; //3, 10, 30 유지할 최근 샘플의 개수
float que[100]={0,}; //최근 N개의 샘플을 유지할 수 있도록 que 구현
int que_size = 0; //que의 크기
//중위수 구하는 필터 작성
float Median_filter(float value) {
  //샘플이 충분하지 않다면 측정 값 리턴
  if(que_size < N) {
    que[que_size] = value;
    que_size++;
    return value;
  }
  float arr[100]={0,};
  //중위수를 구하기 위해 정렬될 함수 생성 및 값 저장
  for(int i=0;i<N;i++) {
    arr[i] = que[i];
  }
  //버블sort 이용해서 정렬
  for(int i=0;i<N;i++) {
    for(int j=0;j<N-1;j++) {
      if(arr[j] > arr[j+1]) {
        float temp = arr[j];
        arr[j] = arr[j+1];
        arr[j+1] = temp;      
      }
    }
  }
  //최근 샘플 갱신
  for(int i=0;i<N-1;i++) {
    que[i] = que[i+1];
  }
  que[N-1] = value;

  //중위수 리턴[N/2]
  return que[(int)N/2];
}
