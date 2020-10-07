#include <Servo.h>
#define PIN_SERVO 10
bool plus = true;
int angle = 0;
Servo myservo;

void setup() {
  myservo.attach(PIN_SERVO); 
  myservo.write(0);
  delay(1000);
}

void loop() {
    // add code here. 
    /*myservo.write(0);
    delay(500);
    myservo.write(90);
    delay(500);
    myservo.write(180);
    delay(500);
    myservo.write(90);
    delay(500);*/

    myservo.write(angle);
    delay(10);
    if(plus) angle++;
    else angle--;
    if(angle == 180) plus = false;
    if(angle == 0) plus = true;
}
