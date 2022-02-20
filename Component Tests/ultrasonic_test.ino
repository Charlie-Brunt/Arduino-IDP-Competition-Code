#define echoPin 2
#define trigPin 3


// defines variables
long duration; // variable for the duration of sound wave travel
int dist; // variable for the distance measurement
int prevDist = 990;
bool ifCoarse = false;


void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.begin(9600);
}

void loop() {
  // Clears the trigPin condition
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);

  // record the previous distance and calculate the new distance
  prevDist = dist;
  dist = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)

  // Displays the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.print(dist);
  Serial.println(" cm");

  // check if there is an item less than x cm away
  if(dist < 20 and prevDist < 20) {
      ifCoarse = true;
  }

  else { 
      ifCoarse = false;
  }

  // display whether it is coarse or not.
  Serial.print("Coarse?: ");
  Serial.print(ifCoarse); 
  delay(500);
}