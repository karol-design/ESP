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
    }

    void set(float duty_cycle) {
        _pwm.write(duty_cycle);  // Set the duty cycle (val between 0.0 - 1.0)
    }
};


/* ----------------------- Main function ----------------------- */
int main() {
    
    while(1) {  // Main while loop of the program

    }
}