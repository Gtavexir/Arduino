#define LED 7

bool plus = true;
int d = 0;
int p, delay_t; 

void setup() {
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
  delay(2000); // 촬영용
//  set_period(100);
//  set_period(1000);
  set_period(10000);
}

void loop() {
  set_duty(d);
  plus ? d++ : d--;
  if(d==0) plus=!plus;
  if(d==100) plus=!plus;
  delayMicroseconds(delay_t);
}

void set_period(int period) { //periodL 100 to 10000 (unit: us)
  p=period;
  p == 10000 ? delay_t = 10000 : delay_t = 5000;
}

void set_duty(int duty) { //duty: 0 to 100 (unit: %)
  digitalWrite(LED, LOW);
  delayMicroseconds(duty*(p/100));
  digitalWrite(LED, HIGH);
  delayMicroseconds(p-duty*(p/100));
}
