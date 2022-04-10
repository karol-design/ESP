/* Technical Demonstration 3 - Control
 * Description: STM32 firmware for ...
 * Classes: Pwm, Encoder, Motor, Propulsion, Sensors, TrackControl, Buggy, Bluetooth, Bluetooth
 * Functions: ...
 * Last modification: 10/04/2022
 */


/* ------------------------------- Pre-processor directives ------------------------------- */
#include "mbed.h"   // Mbed library
#include "C12832.h" // LCD screen library

#define FORWARD 0   // Forward/backward direction pin logic value (for Motor class)
#define BACKWARD 1

// Definition of pins used for "STM32 - Subsystems" interface 
#define PIN_MOTOR_L_PWM PB_15   // Pin connected to PWM1/3
#define PIN_MOTOR_L_MODE PB_14
#define PIN_MOTOR_L_DIR PB_13

#define PIN_MOTOR_R_PWM PC_8    // Pin connected to PWM3/3
#define PIN_MOTOR_R_MODE PC_6
#define PIN_MOTOR_R_DIR PC_5

#define PIN_ENCODER_L_CHA PC_14
#define PIN_ENCODER_R_CHA PC_10

#define PIN_SENSOR_IN1 PB_15
#define PIN_SENSOR_IN2 PB_15
#define PIN_SENSOR_IN3 PB_15
#define PIN_SENSOR_IN4 PB_15
#define PIN_SENSOR_IN5 PB_15
#define PIN_SENSOR_IN6 PB_15
#define PIN_SENSOR_OUT1 PB_15
#define PIN_SENSOR_OUT2 PB_15
#define PIN_SENSOR_OUT3 PB_15
#define PIN_SENSOR_OUT4 PB_15
#define PIN_SENSOR_OUT5 PB_15
#define PIN_SENSOR_OUT6 PB_15


#define PIN_BT_TX PA_11 //USART 6
#define PIN_BT_RX PA_12

// Velocity measurement and control config
#define SAMPLING_FREQUENCY 2        // Velocity measurement sampling frequency [Hz]
#define PULSES_DELTA_T_US 400000    // Delta t for pulses/s measurement [us]
#define PULSES_PER_REV 256          // No. of quadrature encoder pulses per revolution
#define MAX_VELOCITY 20.0f          // Max measured velocity of the buggy [rev/s]
#define MAX_SPEED_ERROR 0.05f       // Max speed error [fraction of the max velocity]

// Motors control config
#define SWITCHING_FREQUENCY 10000.0f    // Set PWM switching frequency to 10 kHz (100 us period) [Hz]

// Track control config
#define IR_MEASUREMENT_TIME 0.05f   // Time for which IR led is turned on for line sensing purposes [s]
#define TRACK_DETECTED_THRESHOLD 0.2f   // Threshold value above which track_detected = true [voltage drop as a fraction of 3.3 V]


// LCD Disabled: C12832 lcd(D11, D13, D12, D7, D10); // LCD Initialisation (pin assignment)


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
    float _velocity;        // Wheel velocity in rev/s and normalised (0.0 - 1.0)
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
        _velocity = ((float) _pulses_per_s / PULSES_PER_REV); // Divide pulses/s by pulses/rev to get rev/s
    }

public:
    Encoder(PinName chA, float sf) : channelA(chA), _sampling_frequency(sf) {
        channelA.rise(callback(this, &Encoder::incrementCounter));  // Increment the counter each time channelA goes high (pulse)
        _sampling_period = (1.00f / _sampling_frequency);           // Period = 1 / frequency
        sampler.attach(callback(this, &Encoder::samplePulses), _sampling_period);   // Start a ticker to regularly sample velocity
    }

    float getVelocity(void) const { // Get the most recent normalised velocity (0.0 - 1.0)
        float velocity_norm = (_velocity / MAX_VELOCITY);
        return velocity_norm;
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

public:
    // Assign pin numbers for bipolar, direction and pwm outputs + pwm frequency
    Motor(PinName mode, PinName dir, PinName pwm, int freq) : _mode_pin(mode), _direction_pin(dir), _motor(pwm, freq) {
        _mode_pin = 0;          // Set the mode to UNIPOLAR
        setDirection(FORWARD);  // Set direction to FORWARD 
        setVoltage(0.0f);       // Stop the motor
    }

    void setDirection(int direction) {   // Set direction (FORWARD/BACKWARD)
        _direction_pin = direction;
    }

    void setVoltage(float voltage) {    // Set voltage (0.0 - 1.0)
        voltage = 1.0f - voltage;       // Duty cycle reversed, i.e. 100% duty cycle disables the motor
        _motor.setDutyCycle(voltage);
    }
};


/* ------------------------------- Propulsion class ------------------------------- */
class Propulsion {

private:
    Motor motorLeft;
    Motor motorRight;
    Encoder wheelLeft;
    Encoder wheelRight;

    void setSpeed(float desired_speed, bool right) { // Set speed [0.0 - 1.0] for right / left motor
        float measured_speed, speed = desired_speed;    // At the beginning the speed to be set is assumed to be equal to the desired_speed
        float error = 0.0;  // At the beginning the error is assumed to be 0.0
        do {
            // Higher Kp = quicker correction, less precision (higher steady-state error). Currently Kp = 0.5.
            speed = speed + (0.5f * error);  // Speed based on last set speed and measured error
            if (speed > 1.0f) {speed = 1.0f;} // Keep the speed in 0.0 - 1.0 limits
            if (speed < 0.0f) {speed = 0.0f;}         

            if (right) {
                motorRight.setVoltage(speed);
                wait(0.05); // Wait for some time to ensure the speed stabilises at the new value
                measured_speed = wheelRight.getVelocity(); 
            } else {
                motorLeft.setVoltage(speed);
                wait(0.05);
                measured_speed = wheelLeft.getVelocity(); 
            }
                   
            error = desired_speed - measured_speed; // Calculate current error
        } while (error > MAX_SPEED_ERROR);  // Continue only if the error above the required limit
    }

public:
    Propulsion() :
        motorLeft(PIN_MOTOR_L_MODE, PIN_MOTOR_L_DIR, PIN_MOTOR_L_PWM, SWITCHING_FREQUENCY),
        motorRight(PIN_MOTOR_R_MODE, PIN_MOTOR_R_DIR, PIN_MOTOR_R_PWM, SWITCHING_FREQUENCY),
        wheelLeft(PIN_ENCODER_L_CHA, SAMPLING_FREQUENCY),
        wheelRight(PIN_ENCODER_R_CHA, SAMPLING_FREQUENCY) {

        motorLeft.setDirection(FORWARD);   // Test motors for FORWARD and BACKWARD directions
        motorRight.setDirection(FORWARD);

        motorLeft.setVoltage(0.0);     // Set the speed for motor1
        motorRight.setVoltage(0.0);     // Set the speed for motor2
    }

    void drive(float angle, float speed) {    // Speed [0.0 - 1.0] and angle [-1.0 - 1.0]
        float left_speed = ((angle - 1.0f) / -2.0f) * speed;
        float right_speed = ((angle + 1.0f) / 2.0f) * speed;
        /*  Angle       -1.0    -0.5    0.0     +0.5    +1.0
            Left Motor  1.00    0.75    0.50    0.25    0.00
            Right Motor 0.00    0.25    0.50    0.75    1.00
        */

        setSpeed(left_speed, 0);    // Set the speed of left motor
        setSpeed(right_speed, 1);   // Set the speed of right motor
    }
};


/* ------------------------------- Sensors class ------------------------------- */
class Sensor {

private:
    AnalogIn phototransistor;
    DigitalOut ir_led;

public:
    Sensor(PinName output, PinName input) : phototransistor(output), ir_led(input) {
        ir_led = 0; // Turn the IR LED off by default
    }

    float getAmbient() { // Return sensor reading [0.0 - 1.0], where 0.0 means no IR light detected
        float value = 1.0f - phototransistor.read();
        return value;
    }

    float read() {      // Get normalised reading from the phototransistor [0.0 - 1.0]
        ir_led = 1;     // Turn the IR LED on
        wait(0.005);    // Wait 5ms for the IR LED to turn fully on
        float reading = (1.0f - phototransistor.read());
        wait(0.005);    // Wait 5ms before turning off the LED
        ir_led = 0;     // Turn the IR LED off
        return reading;
    }

    bool detected() {   // Return true if track has been detected by the sensor
        bool track_detected = (read() > TRACK_DETECTED_THRESHOLD);
        return track_detected;
    }
};


/* ----------------------------- TrackControl class ----------------------------- */
class TrackControl {

private:
    Sensor U1, U2, U3, U4, U5, U6;

    float measureError() {
        float error = 0.0;
        
        /* If U1 or U2 detect a white line:
            Calculate what's the normalised error and return it
           If not:
            If U3, U4 U5 or U6 detect a white line
                Approximate the normalised error accordingly and return it
            If not:
                Return -1
        */

        if (U1.detected() || U2.detected()) {    // Check if U1 or U2 detect a white line
            error = U1.read() - U1.read();
        } else if (U3.detected() || U4.detected() || U5.detected() || U6.detected()) {   // Check if any sensor detected a white line
            if(U3.detected()) { // Approximate the error accordingly
                error = 0.5;
            } else if(U4.detected()) {
                error = -0.5;
            } else if(U5.detected()) {
                error = 0.75;
            } else if(U6.detected()) {
                error = -0.75;
            }
        } else {
            error = -1.0; // Return -1 to indicata that the track has not been detected by any sensor
        }

        return error;
    }

public:
    TrackControl() :
        U1(PIN_SENSOR_OUT1, PIN_SENSOR_IN1),
        U2(PIN_SENSOR_OUT2, PIN_SENSOR_IN2),
        U3(PIN_SENSOR_OUT3, PIN_SENSOR_IN3),
        U4(PIN_SENSOR_OUT4, PIN_SENSOR_IN4),
        U5(PIN_SENSOR_OUT5, PIN_SENSOR_IN5),
        U6(PIN_SENSOR_OUT6, PIN_SENSOR_IN6) {}

    float getError() {
        float error = measureError();
        return error;
    }
};


/* ----------------------------- Buggy class ----------------------------- */
class Buggy {

private:
    TrackControl sensors;
    Propulsion motors;

public:
    Buggy(void) : sensors(), motors() {}

    void drive(float angle, float speed) {
        motors.drive(angle, speed); // Move the buggy at 60% speed and angle of 0 degrees
    }

    bool followTrack() {
        bool noTrack = true; // Error flag true by default
        float correction = sensors.getError();

        if (correction > -1.0) {   // Check if the line has been detected by any sensor
            float angle = correction * 0.5f;
            motors.drive(angle, 0.6);
            wait(0.1);  // Drive with no change for 100 ms
            noTrack = false;    // Reset noTrack error flag
        }
        
        return noTrack; // Return error (true) if the track has not been detected by any sensor
    }

    bool lookForTrack() {
        motors.drive(0.7, 0.5); // Slowly drive right to look for the track
        bool noTrack = (sensors.getError() == -1.0f);
        return noTrack;
    }
};


/* ------------------------------- Bluetooth class ------------------------------- */
class Bluetooth {

private:
    Serial hm10; // Set up software serial port for HM-10 BLE module

public:
    Bluetooth(PinName tx, PinName rx) : hm10(tx, rx) {
        hm10.baud(9600);    // Set the baud rate of hm-10 serial port to 9600
    }

    bool commandReceived() {
        bool received = false;    // Flag to indicate if there is anything in the buffer

        while (hm10.readable()) {   // If the buffer isn't empty, flush it and set received flag
            char c = hm10.getc();   // Flushing ensures that the same byte won't be read twice
            received = true;
        }
        return received;
    }
};


/* ------------------------------- Main function ------------------------------- */
int main() {
    Buggy buggy; // Methods: drive(angle, speed); followTrack(); lookForTrack();

    buggy.drive(0.0, 1.0);  // Test driving at angle of 0 deg with 100% speed

    while(1) {
        bool err = buggy.followTrack();    // Follow white track
        if (err) {
            while(buggy.lookForTrack()) {} // Look for the track until it is found
        }
    }
    
}