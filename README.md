# Group 105 "The Think Makers" IDP Software

Software files for "The Thing".
Please see "final_competition_refactored" for our most up to date program.
## Files

### Component Tests
Basic component programs to test motors, sensors and servos.
- **analogue_LF_sensor_test** - calibration of analogue line follower sensors
- **digital_LF_sensor_test** - prints state of line follower sensor to calibrate against table
- **IR_sensor_test** - serial write distance readings from IR sensor
- **motion_functions** - all motion functions
- **motion_serial_test** - send serial commands to control motion
- **motor_test** - basic motor testing
- **servo_test** - test servo actuation
- **ultrasonic_test** - test ultrasonic sensor

### Integration Tests

#### Analogue Line Following
Tests leading up to the first competition with line following using analogue pins and thresholds.
- **LF_test_analogue** - test for analogue line following
- **collect_block_test_1** - program to test millis() timing over ramp, servo actuation, return and delivery 
- **collect_block_test_2** - program to test robot pushing the block before grabbing it
- **collect_block_test_3** - program to test grabbers pushing the block before grabbing it
- **deliver_test** - intitial program for deliver block calibration
- **find_block** - initial attempt at end box search algorithm
- **first_pass_test** - first pass line follow, rotate, deliver test protocol
- **rotate_180_test** - test line follow and 180 degree rotations at junctions
- **two_pass_test** - first competition program


#### Digital Line Following
Post-first-competition tests with improved digital line following.
- **digital_LF_test** - tests digital line following algorithm
- **three_pass_search** - three passes with search
- **three_pass_ultrasonic** - three passes with search and identification
- **two_pass_digital** - first competition program modified with digital line following
- **two_pass_ultrasonic** - tests block identification with ultrasonic sensor
### Main Programs
Finalised code used for competitions.
- **first_competition** - first competition program scoring 80 pts
- **final_competition** - final competition program scoring 150 pts
- **final_competition_refactored** - finalised final competition program


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
- [x] Write block identification code
- [x] Finalise main program and implement thorough testing