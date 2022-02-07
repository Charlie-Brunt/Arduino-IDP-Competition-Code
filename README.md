# Group 105 IDP Software


By Amanda Ge and Charlie Brunt

Arduino Uno Wifi Rev 2 code for "The Thing" including test programs for testing motors, servos, movement commands, line following and block detection.

## To do

- [x] Test motors, servos, line sensors
- [x] Motion commands with serial control
- [x] Line following algorithm
- [x] Perfect 180s
- [x] Push button / serial command + time delay to start program
- [x] Three journey test protocol
- [x] Switch-case structure
- [x] Introduce decision-based LED signals and movement LED
- [x] Write servo actuation code
- [x] Write block detection code
- [ ] Update deliver_block_test with main code
- [ ] Write block identification code
- [ ] Write third pass search, store, fetch algorithm
- [ ] Call functions from .cpp and .h files
- [ ] Backup standard credit program
## Files

### Component Tests

- **check_distance** - logic for checking distance for each journey
- **deliver_test** - calibration for delivering blocks (COMPLETE)
- **IR_sensor_test** - serial write distance readings from IR sensor#
- **line_follower_test** - test for basic line following
- **motion_serial_test** - send serial commands to control motion
- **motion_test** - all motion functions
- **motor_test** - basic motor testing
- **servo_test** - test servo actuation

### Integration Tests
- **deliver_block_test** - test program to test line following, rotate180 and deliver for three journeys (broken, deprecated)
- **detect_block_test** - program to test millis() timing over ramp, servo actuation, return and delivery
- **first_pass_test** - first pass line follow, rotate, deliver test protocol (COMPLETE)
- **main** - first main program including all features
- **rotate_180_test** - test line follow and 180 deg rotations at junctions 


