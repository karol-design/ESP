/* Technical Demonstration 1 - Motors
 * Description: STM32 firmware for PWM and Motors control and encoders test
 * Classes: Pwm, Encoder, Motor
 * Functions: pwm_test, motor_test, encoder_test, square_path
 * Date: 05/02/2022
 */


/* ------------------- Pre-processor directives ------------------- */
#include "mbed.h"   // Mbed library
#include "C12832.h" // LCD screen library
#include "QEI.h"    // Quadrature Encoder Library

#define FORWARD 1   // Forward/backward direction pin logic value (for Motor class)
#define BACKWARD 0
#define BIPOLAR 1   // Bipolar/unipolar mode pin logic value (for Motor class)
#define UNIPOLAR 0

// Pin numbers definition
#define PIN_MOTOR1_PWM PB_15 // PWM1/3 connected pin
#define PIN_MOTOR1_MODE PB_14
#define PIN_MOTOR1_DIR PB_13

#define PIN_MOTOR2_PWM PC_8 // PWM3/3 connected pin
#define PIN_MOTOR2_MODE PC_6
#define PIN_MOTOR2_DIR PC_5

#define PIN_ENCODER1_CHA PC_14
#define PIN_ENCODER1_CHB PC_15
#define PIN_ENCODER2_CHA PC_10
#define PIN_ENCODER2_CHB PC_12

#define SAMPLING_FREQUENCY 1        // Velocity measurement sampling frequency

#define SWITCHING_FREQUENCY 10000.0f  // Set PWM switching frequency to 10 kHz (100 us period)
#define PULSES_PER_REV 256          // No. of quadrature encoder pulses per revolution
#define WHEEL_RADIUS 0.16f          // Wheel radius (for velocity measurement)
#define PI 3.141592f                // Pi value (for velocity measurement)
#define MAX_VELOCITY 10.0f          // Maximum velocity in m/s (for normalised velocity)
#define PULSES_DELTA_T_US 500000    // Delta t (in us) for pulses/s measurement - the wheel makes ~10 revolutions/s


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

/* ----------------------- Encoder class ----------------------- */
class Encoder {

private:
    InterruptIn channelA;
    Ticker sampler;     // Ticker object to regularly sample current wheel velocity 
    Timeout pulsesDt;   // Timeout object to measure delta t, when measureing pulses/s
    int pulse_count;    // Pulses counter
    int _pulses_per_s;  // Pulses per second
    float _velocity, _velocity_norm;                // Wheel velocity in m/s and normalised (0.0 - 1.0)
    float _sampling_frequency, _sampling_period;    // Sampling frequency and period
    
    void incrementCounter() {   // Increment the pulse counter
        ++pulse_count;
    }
    
    void samplePulses() {
        pulse_count = 0;  // Reset pulses counter
        pulsesDt.attach_us(callback(this, &Encoder::calcVelocity), PULSES_DELTA_T_US);   // Start timeout for calling calcVelocity func
    }

    void calcVelocity() {
        _pulses_per_s = (pulse_count * 1000000 / (int) PULSES_DELTA_T_US); // Divide pulses counter by Dt
        _velocity = ( ((float) _pulses_per_s / PULSES_PER_REV) * (2.0f * PI * WHEEL_RADIUS) ); // Multiply rev/s by wheel circumference
    }

public:
    Encoder(PinName chA, PinName chB, float sf) : channelA(chA), _sampling_frequency(sf) {
        channelA.rise(callback(this, &Encoder::incrementCounter));  // Increment the counter each time channelA goes high (pulse)
        _sampling_period = (1.00f / _sampling_frequency);   // Period = 1 / frequency
        sampler.attach(callback(this, &Encoder::samplePulses), _sampling_period);   // Start a ticker to regularly sample velocity
    }

    float getVelocity(void) const { // Get most recent velocity in m/s
        return _velocity;
    }

    float getVelocityNorm(void) const { // Get most recent velocity normalised (0.0 - 1.0)
        if (_velocity < MAX_VELOCITY) {
            return (_velocity / MAX_VELOCITY);
        } else {
            return -1;  // Return an error (-1) if the velocity is outside normalised range
        }
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
    Pwm pwm1(PIN_MOTOR1_PWM, SWITCHING_FREQUENCY); // Two PWM channels, f = SWITCHING_FREQUENCY
    Pwm pwm2(PIN_MOTOR2_PWM, SWITCHING_FREQUENCY);

    for(int i = 0; i < 100; i++) {  // Test the entire range of duty cycle (DC): 0.0 - 1.0 
        float duty_cycle = ((float) i / 100.0f);    // Map i value (0-100) to duty_cycle (0.0 - 1.0) 
        pwm1.setDutyCycle(duty_cycle);  // Set the DC for pwm1

        duty_cycle = 1.0f - duty_cycle; // For pwm2 test the DC from (1.0 - 0.0) 
        pwm2.setDutyCycle(duty_cycle);  // Set the DC for pwm2

        wait(0.1);  // Test procedure duration is 10 seconds
    }
}


/* ----------------------- motor_test function ----------------------- */
void motor_test() {
    Motor motor1(PIN_MOTOR1_MODE, PIN_MOTOR1_DIR, PIN_MOTOR1_PWM, SWITCHING_FREQUENCY);
    Motor motor2(PIN_MOTOR2_MODE, PIN_MOTOR2_DIR, PIN_MOTOR2_PWM, SWITCHING_FREQUENCY);
    Encoder wheel1(PIN_ENCODER1_CHA, PIN_ENCODER1_CHB, SAMPLING_FREQUENCY);
    Encoder wheel2(PIN_ENCODER2_CHA, PIN_ENCODER2_CHB, SAMPLING_FREQUENCY);

    motor1.setDirection(FORWARD);   // Test motors for FORWARD and BACKWARD directions
    motor2.setDirection(FORWARD);

    lcd.cls(); //Clear the screen and display encoders readings [m/s]
    lcd.locate(0, 0);
    lcd.printf("M1 vel = %.2lf m/s", wheel1.getVelocity());
    lcd.locate(0, 10);
    lcd.printf("M2 vel = %.2lf m/s", wheel2.getVelocity());

    for(int i = 0; i<2; i++) {
        for(int i = 0; i < 100; i++) {  // Test the entire range of speed: 0.0 - 1.0 
            float speed = ((float) i / 100.0f);    // Map i value (0-100) to duty_cycle (0.0 - 1.0) 
            motor1.setSpeed(speed);     // Set the speed for motor1

            speed = 1.0f - speed;       // For motor2 test the speed from 1.0 to 0.0
            motor2.setSpeed(speed);     // Set the speed for motor2

            wait(0.1);
            lcd.locate(30, 0);
            lcd.printf("%.2lf", wheel2.getVelocity());
            
        }

        motor1.setDirection(BACKWARD);
        motor2.setDirection(BACKWARD);
    }
}


/* ----------------------- Main function ----------------------- */
int main() {

    // pwm_test();
    // motor_test();
    // square_path();

    while(1) {  // Main while loop of the program
        motor_test();
    }
}
