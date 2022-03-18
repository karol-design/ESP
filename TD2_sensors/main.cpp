/* Technical Demonstration 2 - Sensors
 * Description: STM32 firmware for reading and displaying sensor readings and BLE communication
 * Classes: Pwm, Encoder, Motor, Bluetooth
 * Functions: ...
 * Last modification: 18/03/2022
 */


/* ------------------------------- Pre-processor directives ------------------------------- */
#include "mbed.h"   // Mbed library
#include "C12832.h" // LCD screen library

#define FORWARD 0   // Forward/backward direction pin logic value (for Motor class)
#define BACKWARD 1
#define BIPOLAR 1   // Bipolar/unipolar mode pin logic value (for Motor class)
#define UNIPOLAR 0

// Definition of pins used for "STM32 - Subsystems" interface 
#define PIN_MOTOR_L_PWM PB_15   // Pin connected to PWM1/3
#define PIN_MOTOR_L_MODE PB_14
#define PIN_MOTOR_L_DIR PB_13

#define PIN_MOTOR_R_PWM PC_8    // Pin connected to PWM3/3
#define PIN_MOTOR_R_MODE PC_6
#define PIN_MOTOR_R_DIR PC_5

#define PIN_ENCODER_L_CHA PC_14
#define PIN_ENCODER_R_CHA PC_10

// Physical characteristic 
#define WHEEL_RADIUS 0.04f          // Wheel radius (for velocity measurement) [m]

// Velocity measurement config
#define SAMPLING_FREQUENCY 2        // Velocity measurement sampling frequency [Hz]
#define PULSES_DELTA_T_US 400000    // Delta t for pulses/s measurement [us]
#define PULSES_PER_REV 256          // No. of quadrature encoder pulses per revolution
#define PI 3.141592f                // Pi value (for velocity measurement)

// Motors control config
#define SWITCHING_FREQUENCY 10000.0f  // Set PWM switching frequency to 10 kHz (100 us period) [Hz]


C12832 lcd(D11, D13, D12, D7, D10); // LCD Initialisation (pin assignment)


/* ------------------------------- Pwm class ------------------------------- */
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


/* ------------------------------- Encoder class ------------------------------- */
class Encoder {

private:
    InterruptIn channelA;   // Interrupt channel to receive pulses from the encoder
    Ticker sampler;         // Ticker object to regularly sample current wheel velocity 
    Timeout pulsesDt;       // Timeout object to measure delta t, when measureing pulses/s
    int pulse_count;        // Pulses counter
    int _pulses_per_s;      // Pulses per second
    float _velocity, _velocity_norm;                // Wheel velocity in m/s and normalised (0.0 - 1.0)
    float _sampling_frequency, _sampling_period;    // Sampling frequency and period
    
    void incrementCounter() {
        ++pulse_count;      // Increment the pulse counter
    }
    
    void samplePulses() {
        pulse_count = 0;    // Reset pulses counter
        pulsesDt.attach_us(callback(this, &Encoder::calcVelocity), PULSES_DELTA_T_US);  // Start timeout for calling calcVelocity func
    }

    void calcVelocity() {
        _pulses_per_s = (pulse_count * 1000000 / (int) PULSES_DELTA_T_US); // Divide pulses counter by Dt
        _velocity = ( ((float) _pulses_per_s / PULSES_PER_REV) * (2.0f * PI * WHEEL_RADIUS) ); // Multiply rev/s by wheel circumference
    }

public:
    Encoder(PinName chA, float sf) : channelA(chA), _sampling_frequency(sf) {
        channelA.rise(callback(this, &Encoder::incrementCounter));  // Increment the counter each time channelA goes high (pulse)
        _sampling_period = (1.00f / _sampling_frequency);           // Period = 1 / frequency
        sampler.attach(callback(this, &Encoder::samplePulses), _sampling_period);   // Start a ticker to regularly sample velocity
    }

    float getVelocity(void) const { // Get most recent velocity in m/s
        return _velocity;
    }

    // Methods to start, stop and read raw counter value instead (for precise manouvers, e.g. turning 180 deg)

    void startCounter(void) {  // Stop regular velocity measurements and start pulse counter
        sampler.detach();   // Stop a ticker which regularly sample velocity
        pulse_count = 0;    // Reset pulses counter
    }

    int getCounter(void) const {
        return pulse_count; // Return pulse counter value
    }

    void stopCounter(void) {   // Reinitialize regular velocity sampling
        sampler.attach(callback(this, &Encoder::samplePulses), _sampling_period);   // Start a ticker to regularly sample velocity
    }
};


/* ------------------------------- Motor class ------------------------------- */
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

    void setSpeed(float speed) {    // Set speed (0.0 - 1.0)
        speed = 1.0f - speed;        // Duty cycle reversed, i.e. 100% duty cycle disables the motor
        _motor.setDutyCycle(speed);
    }
};


/* ------------------------------- Bluetooth class ------------------------------- */
class Bluetooth {

private:
public:

    Bluetooth() : {
        
    }

};

/* ------------------------------- Main function ------------------------------- */
int main() {

    ...();   // Test functions: ...
    while(1) {}     // Main while loop of the program
    
}