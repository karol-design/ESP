# Technical demonstration 3 | Control and Steering

### Follow this procedure to test the firmware and tune required variable parameters in the code.

Required for testing:
1) buggy (motors, encoders, nucleo board, motor board, castor wheel, sensors board)
2) laptop, long USB cable, jumper and normal wires, DMM & probes & cables, breadboard, loose TCRT sensor, black paper & white tape, nucleo board, myDAQ & cable, mechanical tools and parts, few basic electronic components
3) batteries, charger, (ideally variable) power supply

## Part 1 - Speed control

1) Pwm class
- [ ] Does the setDutyCycle method works fine with desired pins?

2) Motor class
- [ ] Can you control the direction and voltage for both motors?
- [ ] Is the voltage - velocity relationship relatively linear?

3) Encoder class
- [ ] Can you get the current velocity and is the reading correct?
- [ ] Do startCounter, getCounter and stopCounter methods works as intended (e.g. for turning 90 deg)?
- [ ] TODO: Adjust SAMPLING_FREQUENCY and PULSES_DELTA_T_US.

4) Propulsion class
- [ ] Does the speed stays const. irrespective of the friction/power supply voltage?
- [ ] TODO: Adjust MAX_SPEED_ERROR, Kp, Set/Measure delay
- [ ] Does the turnaround method make the buggy turn by exactly 180 degrees?

## Part 2 - Line sensing

1) Sensors class
- [ ] Does getAmbient method returns correct outputs?
- [ ] Does read and detected methods works fine?
- [ ] TODO: Adjust TRACK_DETECTED_THRESHOLD

2) TrackControl class
- [ ] Does measureError method returns a correct error based on the position of the sensor board above the track.
- [ ] TODO: Adjust error approximations

## Part 3 - Following the line

1) Buggy class
- [ ] Do drive and lookForTrack methods work as required?
- [ ] Does the followTrack method work as intended? How quickly can the buggy drive in that mode?
- [ ] TODO: Improve the angle correction equation in followTrack

2) Main function
- [ ] TODO: Use methods from the Buggy class to prepare the code for the demo