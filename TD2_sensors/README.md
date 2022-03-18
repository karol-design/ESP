# Technical demonstration 2 | Sensors

This is the code to be used during the second technical demonstration. Presentation modes (functions) can be called from the main() function.

### Requirements (for the firmware) for TD2:
- [ ] Detects and display on the LCD predicted changes in the all the line sensor outputs
- [ ] Persent response of the MCU to a BLE command

### The following modes are implemented in the code:
1. **bluetooth_test** - Test receiving commands using BLE

### Classes implemented in the code:
1. **Pwm** - Used to initialise and set duty cycle of a PWM channel
2. **Encoder** - Used to initialise and read velocity (m/s or normalised) from the encoder
3. **Motor** - Used to initialise and controll the speed or direction of a motor
4. **Bluetooth** - Used to initialise UART port and read data from the BLE module 

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
