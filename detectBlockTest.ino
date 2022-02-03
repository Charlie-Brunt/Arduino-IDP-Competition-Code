#include <SharpIR.h>

// Define model and input pin:
#define IRPin A2
#define model 1080

// Create variable to store the distance:
int distance_cm;
int bridgeDuration1 = 10000;
long previousMillis = 0;

bool IfCollected = false;

// Create a new instance of the SharpIR class:
SharpIR mySensor = SharpIR(IRPin, model);

void setup() {

}

void loop() {
    distance_cm = mySensor.distance();

    if ((journeyCounter == journey1) && (IfCollected == false){
        unsigned long currentMillis = millis();

        if (currentMillis - previousMillis > bridgeDuration1) {
            checkDistance();
        }
    }
    if ((journeyCounter == journey2) && (junctionCounter == junction2) && (IfCollected == false) {
        checkDistance();
    }
    if ((journeyCounter == journey3) && (junctionCounter == junction2) && (IfCollected == false) {
        checkDistance();
    }
}

/************************ DETECTION ***************************/
void checkDistance() {
    if (distance_cm <= 10) {
        stop();
    }
}