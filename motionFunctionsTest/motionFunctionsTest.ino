#include "Motion.h"

void setup() {
    AFMS.begin();  // create with the default frequency 1.6KHz 
}

void loop() {
    forwards(255);
    delay(1000);
}