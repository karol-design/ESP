/* Technical Demonstration 1 - Motors
 * Description: STM32 firmware for PWM and Motors control and encoders test
 * Date: 05/02/2022
 */

#include "mbed.h"   // Import the Mbed libraries
#include "C12832.h" // Import the C12832 LCD screen llibrary

#define FORWARD 1   // Forward/backward direction pin logic value (for Motor class)
#define BACKWARD 0
#define BIPOLAR 1   // Bipolar/unipolar mode pin logic value (for Motor class)
#define UNIPOLAR 0

// Define the interface - pin names
#define PIN_MOTOR1_PWM D4
#define PIN_MOTOR2_PWM D5

C12832 lcd(D11, D13, D12, D7, D10); // LCD Initialisation (pin assignment)


/* ----------------------- Pwm class ----------------------- */
class Pwm {

private:
    PwmOut _pwm_pin;    // Pwm pin - object of PwmOut class
    int _frequency;     // Frequency to be set
    int _period_us_val; // Period to be set (= 1/_frequency)

public:
    Pwm(PinName pin, int frequency) : _pwm_pin(pin), _frequency(frequency) {
        _period_us_val = (1000000 / frequency);  // Calculate the period value [micro seconds]
        _pwm_pin.period_us(_period_us_val);      // Set the period of the pwm otuput
    }

    void setDutyCycle(float duty_cycle) {
        _pwm_pin.write(duty_cycle);  // Set the duty cycle (val between 0.0 - 1.0)
    }
};


/* ----------------------- Motor class ----------------------- */
class Motor {

private:
    DigitalOut _mode_pin;       // Motor's mode control pin - object of DigitalOut class
    DigitalOut _direction_pin;  // Motor's direction control pin - object of DigitalOut class
    Pwm _motor;                 // Motor's speed control - object of Pwm class

    void _setMode(int mode) {        // Set control mode (BIPOLAR/UNIPOLAR)
        _mode_pin = mode;
    }

public:
    // Assign pin numbers for bipolar, direction and pwm outputs + pwm frequency
    Motor(PinName mode, PinName dir, PinName pwm, int freq) : _mode_pin(mode), _direction_pin(dir), _motor(pwm, freq) {
        _setMode(UNIPOLAR);     // Set the mode to UNIPOLAR
        setDirection(FORWARD);  // Set direction to FORWARD 
        setSpeed(0.0f);         // Stop the motor
    }

    void setDirection(int direction) {   // Set direction (FORWARD/BACKWARD)
        _direction_pin = direction;
    }

    void setSpeed(float speed) { // Set speed (0.0 - 1.0)
        _motor.setDutyCycle(speed);
    }
};


/* ----------------------- pwm_test function ----------------------- */
void pwm_test() {
    Pwm pwm1(PIN_MOTOR1_PWM, 10000); // First PWM channel, Pin = PIN_MOTOR1_PWM, f = 10 kHz
    Pwm pwm2(PIN_MOTOR2_PWM, 10000); // Second PWM channel, Pin = PIN_MOTOR2_PWM, f = 10 kHz

    for(int i = 0; i < 100; i++) {  // Test the whole range of duty cycle (DC): 0.0 - 1.0 

        float duty_cycle = ((float) i / 100.0f);    // Map i value (0-100) to duty_cycle (0.0 - 1.0) 
        pwm1.setDutyCycle(duty_cycle);  // Set the DC for pwm1

        duty_cycle = 1.0f - duty_cycle; // For pwm2 test the DC from (1.0 - 0.0) 
        pwm2.setDutyCycle(duty_cycle);  // Set the DC for pwm2

        wait(0.1);  // Test procedure duration is 10 seconds
    }
}


/* ----------------------- Main function ----------------------- */
int main() {
    
    while(1) {  // Main while loop of the program

    }
}