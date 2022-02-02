#include <SharpIR.h>

// Define model and input pin:
#define IRPin A2
#define model 1080

// Create variable to store the distance:
int distance_cm;
int bridgeDuration1 = 10000;
int bridgeDuration2 = 9000;
int bridgeDuration3 = 9000;
long previousMillis = 0;

// Create a new instance of the SharpIR class:
SharpIR mySensor = SharpIR(IRPin, model);

void setup() {

}

void loop() {

    if ((journeyCounter == journey1) && (junctionCounter == junction2) {
        unsigned long currentMillis = millis();

        if (currentMillis - previousMillis > bridgeDuration1) {
            checkDistance();
        }
    }
    if ((journeyCounter == journey2) && (junctionCounter == junction2) {
        unsigned long currentMillis = millis();

        if (currentMillis - previousMillis > bridgeDuration2) {
            checkDistance();
        }
    }
    if ((journeyCounter == journey3) && (junctionCounter == junction2) {
        unsigned long currentMillis = millis();

        if (currentMillis - previousMillis > bridgeDuration2) {
            checkDistance();
        }
    }



}

/************************ DETECTION ***************************/
void checkDistance() {
    if (distance_cm <= 10) {
        stop();
    }
}