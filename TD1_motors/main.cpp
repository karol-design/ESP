/* Technical Demonstration 1 - Motors
 * Description: This code is intended to run on the STM32 MCU
 * Date: 05/02/2022
 */

#include "mbed.h"   // Import the Mbed libraries
#include "C12832.h" // Import the C12832 LCD screen llibrary

C12832 lcd(D11, D13, D12, D7, D10); // LCD Initialisation (pin assignment)

/* -------------------------- Main function --------------------------- */
/* ----------------------- Pwm class ----------------------- */
class Pwm {

private:
    PwmOut _pwm;        // Create PwmOut object
    int _frequency;     // Frequency to be set
    int _period_us_val; // Period to be set (= 1/_frequency)

public:
    Pwm(PinName pin, int frequency) : _pwm(pin), _frequency(frequency) {
        _period_us_val = (1000000 / frequency);  // Calculate the period value [micro seconds]
        _pwm.period_us(_period_us_val);           // Set the period of the pwm otuput
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

    void set(float duty_cycle) {
        _pwm.write(duty_cycle);  // Set the duty cycle (val between 0.0 - 1.0)
    void setSpeed(float speed) { // Set speed (0.0 - 1.0)
        _motor.setDutyCycle(speed);
    }
};


/* ----------------------- Main function ----------------------- */
int main() {
    
    while(1) {  // Main while loop of the program

    }
}