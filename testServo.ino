#include <Servo.h>
Servo myservo;
void setup() {
    myservo.attach(2);
    myservo.write(90);
}

void loop() {
    myservo.write(90);
    delau(1000);
    myservo.write(0);
    delay(1000);
}