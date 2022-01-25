int buttonPin=1;
int pre_button_state=LOW;
int cur_button_state=LOW;
int button_state=LOW;

void setup() {
  Serial.begin(9600);
  pinMode(buttonPin,INPUT);

}

void loop() {
  
  //Serial.println(cur_button_state);
  pre_button_state = cur_button_state;
  cur_button_state = digitalRead(buttonPin);
  if (pre_button_state == HIGH && cur_button_state == LOW){
      button_state = !button_state;
  }
  if (button_state == HIGH) {
    Serial.println("Power ON");
  }
  else {
    Serial.println("Power OFF");
  }
 
}
