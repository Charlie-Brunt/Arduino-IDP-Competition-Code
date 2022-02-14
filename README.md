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
- [x] Fix deliver_block_test for first pass using timer 
- [x] Integrate all modules to date
- [x] Optimise and improve consistency of deliver_block_test
- [x] Write first competition program
- [x] Write third pass search, store, fetch algorithm
- [x] Implement search into program and test
- [ ] Write block identification code
- [ ] Finalise main program and implement thorough testing
- [ ] Refactor code into modules
- [ ] Backup program if all else fails
## Files

### Component Tests

- **check_distance** - logic for checking distance for each journey
- **deliver_test** - calibration for delivering blocks
- **IR_sensor_test** - serial write distance readings from IR sensor
- **LF_sensor_test** - calibration of analogue line follower sensors
- **LF_test_analogue** - test for analogue line following
- **motion_serial_test** - send serial commands to control motion
- **motion_test** - all motion functions
- **motor_test** - basic motor testing
- **servo_test** - test servo actuation
- **ultrasonic_test** - test ultrasonic sensor

### Integration Tests (Analogue LF)
Tests leading up to the first competition with line following through analogue pins.
- **block_identification_test** - first attempt at block identification using ultrasound (broken)
- **deliver_block_test** - test program to test line following, rotate180 and deliver for three journeys (broken, obsolete)
- **detect_block_test** - program to test millis() timing over ramp, servo actuation, return and delivery 
- **detect_block_test2** - program to test robot pushing the block before grabbing it
- **detect_block_test3** - program to test grabbers pushing the block before grabbing it
- **first_pass_test** - first pass line follow, rotate, deliver test protocol (complete)
- **rotate_180_test** - test line follow and 180 degree rotations at junctions
- **search_test**
- **two_pass_test** - identical to first competition program

### Digital Line Following
Post-first-competition tests with improved line following through digital pins.
- **LF_sensor_test_digital** - prints state of line follower sensor for calibration
- **LF_test_digital** - test digital line following algorithm
- **three_pass_test** - three passes with search
- **two_pass_digital** - first competition program with digital line following
- **ultrasonic_two_pass** - testing block identification with ultrasonic sensor
### Main Programs
Finalised code used for competitions.
- **first competition** - first competition program scoring 80 pts
- **main** - initial main program to test latest features (obsolete)