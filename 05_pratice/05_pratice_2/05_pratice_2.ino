#define LED 7
bool toggle = false;
int cnt = 0;

void setup() {
  pinMode(LED, OUTPUT);
  digitalWrite(LED, toggle);
  delay(1000);
}

void loop() {
  toggle = toggle_state(toggle);
  digitalWrite(LED, toggle);
  delay(100);
  if(!toggle) cnt++;
  
  while(1) {
    if(cnt < 5) break;
    digitalWrite(LED, 1); //무한 루프로 종료
  }
}

bool toggle_state(bool toggle) {
  return !toggle;
}
