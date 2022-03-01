# Technical demonstration 1 | Motors

This is the code to be used during the first technical demonstration. Presentation modes (functions) can be called from the main() function.

### Firmware requirements for TD1:
- [ ] Task 1 & 7 (PWM): Presentation of the PWM logic output (2 channels, myDAQ oscilloscope), use of input test-points on the motor drive board, STM32 powered with E5V on the motor board, **motor_test** function to be used
- [ ] Task 2 & 3 (Motors & Encoders): Drive both motors  (control the speed/direction of the motors - different speed for each) & Show an indicator of the turning wheels (LCD/LED) [additional time with no motor excitation for checking encoders by turning the wheels by hand,  **motor_test** function to be used
- [ ] Task 6 (Autonomous driving): The buggy should draw out a square, with sides of 0.5 m in length and then stop when it reaches the starting position. It should then turn around, and re-trace the square in the opposite direction. **square_path** function to be used

### Modes (functions) implemented in the code
2. **motor_test** - Change the speed of both motors & display the velocity readings on the LCD screen
4. **square_path** - Drive on the virtual path in the shape of a square (0.5 m side), stop and re-trace the path in the opposite direction

### Classes implemented in the code:
1. **Pwm** - Used to initialise and set duty cycle of a PWM channel
2. **Encoder** - Used to initialise and read velocity (m/s or normalised) from the encoder
3. **Motor** - Used to initialise and controll the speed or direction of a motor

### Naming convention in the project
- macros - MACRO_NAME
-------------------------------------------
- classes - ClassName
- objects - objectName
- member functions - functionName
- private data members - _variable_name
-------------------------------------------
- variables - variable_name
- functions - function_name

### Contributing and testing requirements
- Commit to the main only the code that **compile without any warnings or errors**.
- To test, compile or flash the code use [online mBed compiler](https://www.ide.mbed.com/compiler)
