#include <Servo.h>

Servo myservo;

int pos = 0; // variable to store the servo position
const int servo_startangle = 0;
const int servo_endangle = 70;

void setup()
{
  myservo.attach(9); // attaches the servo on pin 9 to the servo object
}

void loop()
{
  open_servo();
  delay(1000);
  close_servo();
  delay(1000);

}


void open_servo(){
  for (pos = servo_startangle; pos <= servo_endangle; pos += 1)
  { 
    myservo.write(pos);
    delay(15);
  }
}
void close_servo() {
  for (pos = servo_endangle; pos >= servo_startangle; pos -= 1)
  { 
    myservo.write(pos);
    delay(15);
  }
}