
//defining pins and variables
#define leftIn A0
#define rightIn A1

void setup() {
  //declaring pin types
  pinMode(leftIn,INPUT);
  pinMode(rightIn,INPUT);
  //begin serial communication
  Serial.begin(9600);
  
}

void loop(){
  //left sensor state
  int LineSensor1;
  //right sensor state
  int LineSensor2;
  
  //printing values of the sensors to the serial monitor

  Serial.print("Left:  ");
  Serial.println(analogRead(leftIn));
  Serial.print("Right: ");
  Serial.println(analogRead(rightIn));                              
  delay(1000);

//sets left LineSensor1 to high if on tape, else Low
  if(analogRead(leftIn) >= 850){
      LineSensor1 = HIGH;
  }
  else {
      LineSensor1 = LOW;
  }
  //sets right LineSensor2 to high if on tape, else Low
  if(analogRead(rightIn) >= 850){
      LineSensor2 = HIGH;
  }
  else {
      LineSensor2 = LOW;
  }
  //printing hi/lo variables 
  /*
  Serial.println("Left");
  Serial.println(LineSensor1);
  Serial.println("Right");
  Serial.println(LineSensor2);
  delay(1000);
  */
}