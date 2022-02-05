# Technical demonstration 1 | Motors

This is the code to be used during the first technical demonstration. Presentation modes can be changed using macros in the SETUP part of the code.

## Requirements (for the firmware) for TD1:
- [ ] The successful use of the PWM output features of the micro (presentation of the PWM logic outputs, 2 channels, on the myDAQ oscilloscope)
- [ ] Drive both motors from the microcontroller using the drive board (control the speed/direction of the motors - different speed for each)
- [ ] Ability to measure speed from the wheel encoder (display presenting current readings from both encoders)
- [ ] Wheel control. Independent control of the buggy wheels from the microprocessor (The buggy should draw out a square, with sides of 0.5 m in length and then stop when it reaches the starting position. It should then turn around, and re-trace the square in the opposite direction.)

## The following modes are implemented in the code:
1. **PWM** - Sweep linearly through different PWM values 
2. **MOTORS** - Change the speed of both motors every 2 seconds
3. **ENCODERS** - Change the speed of both motors every 2 seconds and display the angular velocity on the LCD screen
4. **SQUARE** - Drive on the virtual path in the shape of a square (0.5 m side), stop and re-trace the path in the opposite direction

