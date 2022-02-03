# Group 105 IDP Software

Arduino Uno Wifi Rev 2 code for "The Thing" including test programs for testing motors, servos, movement commands, line following and block detection.

To do:
- [x] Test motors, servos, line sensors
- [x] Motion commands with serial control
- [x] Line following algorithm
- [x] Perfect 180s
- [x] Push button / serial command + time delay to start program
- [x] Three journey test protocol
- [x] Switch-case structure
- [ ] Write servo actuation code
- [ ] Write block detection code
- [ ] Write block identification code

- [x] Introduce decision-based LED signals and movement LED
- [ ] Write third pass search, store, fetch algorithm
- [ ] Call functions from .cpp and .h files

Files:
- deliver_block_test - test program to test line following and rotate180 for three journeys
- deliver_test - calibration for delivering blocks
- checkDistance - logic for checking distance for each journey (redundant)
- FirstPassTest - initial first pass code (redundant)
- IRsensorTest - serial write distance readings from IR sensor#
- LineFollowerTest - test for basic line following
- main - first main program including all features
- motionSerialTest - send serial commands to control motion
- MotionTest - all motion functions
- ServoTest - test servo actuation
- test180 - test 180 deg rotations  



By Amanda Ge and Charlie Brunt
