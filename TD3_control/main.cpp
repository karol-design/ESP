/* Technical Demonstration 3 - Control
 * Description: Final version of the firmware comprising all components required to fully control the buggy
 * Classes: Pwm, Encoder, Motor, Propulsion, Sensors, TrackControl, Buggy, Bluetooth
 * Last modification: 12/04/2022
 */


/* ------------------------------- Pre-processor directives ------------------------------- */
#include "mbed.h"   // Mbed library

#define FORWARD 0   // Forward/backward direction pin logic value (for Motor class)
#define BACKWARD 1

// Definition of STM32 pins used by external components
#define PIN_BT_TX PA_11         // USART 6
#define PIN_BT_RX PA_12

#define PIN_MOTOR_L_PWM PB_15   // PWM1/3
#define PIN_MOTOR_L_MODE PB_14
#define PIN_MOTOR_L_DIR PB_13

#define PIN_MOTOR_R_PWM PC_8    // PWM3/3
#define PIN_MOTOR_R_MODE PC_6
#define PIN_MOTOR_R_DIR PC_5

#define PIN_ENCODER_L_CHA PC_14
#define PIN_ENCODER_R_CHA PC_10

#define PIN_SENSOR_IN1 PA_0     // ADC
#define PIN_SENSOR_IN2 PA_1     // ADC
#define PIN_SENSOR_IN3 PA_4     // ADC
#define PIN_SENSOR_IN4 PB_0     // ADC
#define PIN_SENSOR_IN5 PC_1     // ADC
#define PIN_SENSOR_IN6 PC_0     // ADC

#define PIN_SENSOR_OUT1 PA_8
#define PIN_SENSOR_OUT2 PB_10
#define PIN_SENSOR_OUT3 PB_4
#define PIN_SENSOR_OUT4 PB_5
#define PIN_SENSOR_OUT5 PB_3
#define PIN_SENSOR_OUT6 PA_10

// Velocity measurement and control config
#define SAMPLING_FREQUENCY 2        // Velocity measurement sampling frequency [Hz]
#define PULSES_DELTA_T_US 400000    // Delta t for pulses/s measurement [us] 
#define PULSES_PER_REV 256          // No. of quadrature encoder pulses per revolution [no units]
#define MAX_VELOCITY 40.0f          // Max velocity of the wheel (40 rev/s ~ 27 km/h for r=3 cm) [rev/s] 
#define SPEED_ERROR_COEF 0.5f       // Proportional coefficient (controller) for speed control [no units]
#define SPEED_STABILISATION_DELAY 0.05f // Delay between setting and measuring the speed to see if it is equal to the desired speed [s]
#define MAX_SPEED_ERROR 0.05f       // Max speed error [fraction of MAX_VELOCITY]
#define TURNAROUND_PULSES 768       // Number of pulses for both motors to make turn the buggy by 180 degrees

// Motors control config
#define SWITCHING_FREQUENCY 10000.0f    // Set PWM switching frequency to 10 kHz (100 us period) [Hz]
#define MOTOR_VOLTAGE_OFFSET 0.4f       // Offset, i.e. point at which the motor starts to rotate [fraction of the max supply voltage]

// Track control config
#define TRACK_DETECTED_THRESHOLD 0.2f   // Threshold value above which track_detected = true [voltage drop as a fraction of 3.3 V]


/* ------------------------------- Pwm class ----------------------------------- */
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
    Timeout pulsesDt;       // Timeout object to measure delta t, when measuring pulses/s
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

    // Methods to start, stop and read raw counter value instead (for precise manoeuvres, e.g. turning 180 deg)

    void startCounter(void) {   // Stop regular velocity measurements and start pulse counter
        sampler.detach();       // Stop a ticker which regularly sample velocity
        pulse_count = 0;        // Reset pulses counter
    }

    int getCounter(void) const {
        return pulse_count;     // Return pulse counter value
    }

    void stopCounter(void) {    // Reinitialize regular velocity sampling
        sampler.attach(callback(this, &Encoder::samplePulses), _sampling_period);   // Start a ticker to regularly sample velocity
    }
};


/* ------------------------------- Motor class --------------------------------- */
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

    void setVoltage(float voltage) {        // Set voltage (0.0 - 1.0)
        voltage = MOTOR_VOLTAGE_OFFSET + (voltage / (1.0f / (0.90f - MOTOR_VOLTAGE_OFFSET)));  // offset to get more linear dependency, e.g. .0 -> .4, .5 -> ~.7, 1.0 -> ~1.0 
        voltage = 1.0f - voltage;           // Duty cycle reversed, i.e. 100% duty cycle disables the motor
        _motor.setDutyCycle(voltage);
    }
};


/* ------------------------------- Propulsion class ---------------------------- */
class Propulsion {

private:
    Motor motorLeft;
    Motor motorRight;
    Encoder wheelLeft;
    Encoder wheelRight;

    void setSpeed(float desired_speed, bool right) {    // Set speed [0.0 - 1.0] for right / left motor
        float measured_speed, speed = desired_speed;    // Start with the assumption that the speed to be set = desired_speed
        float error = 0.0;                              // Assume that the error is 0.0
        do {
            // Higher Kp (SPEED_ERROR_COEF) = quicker correction, less precision (higher steady-state error).
            speed = speed + (SPEED_ERROR_COEF * error); // Speed based on last set speed and measured error
            if (speed > 1.0f) {speed = 1.0f;}           // Keep the speed in 0.0 - 1.0 limits
            if (speed < 0.0f) {speed = 0.0f;}

            if (right) {
                motorRight.setVoltage(speed);
                wait(SPEED_STABILISATION_DELAY); // Wait for some time to ensure the speed stabilises at the new value
                measured_speed = wheelRight.getVelocity(); 
            } else {
                motorLeft.setVoltage(speed);
                wait(SPEED_STABILISATION_DELAY);
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
        motorRight.setVoltage(0.0);    // Set the speed for motor2
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

    void turnaround() {
        wheelLeft.startCounter();   // Start counters for both wheels to measure the exact number of pulses from encoders
        wheelRight.startCounter();
        bool left_finished = false;         // Left wheel finished making required number of revolutions/pulses
        bool right_finished = false;        // Right wheel finished -||-
        motorLeft.setDirection(BACKWARD);   // Change the direction of the left motor to BACKWARD for the duration of turnaround

        while(left_finished == false || right_finished == false) {
            if(wheelLeft.getCounter() < TURNAROUND_PULSES) {
                motorLeft.setVoltage(0.3);    // Set the speed of 30% for the left motor
            } else {
                motorLeft.setVoltage(0.0);    // Turn off the left motor
                left_finished = true;
            }

            if(wheelRight.getCounter() < TURNAROUND_PULSES) {
                motorRight.setVoltage(0.3);    // Set the speed of 30% for the right motor
            } else {
                motorRight.setVoltage(0.0);    // Turn off the right motor
                right_finished = true;
            }
        }

        motorLeft.setDirection(FORWARD);   // Change the direction for left motor back to FORWARD
        wheelLeft.stopCounter();
        wheelRight.stopCounter();
    }
};


/* ------------------------------- Sensors class ------------------------------- */
class Sensor {

private:
    AnalogIn phototransistor;
    DigitalOut ir_led;

public:
    Sensor(PinName output, PinName input) : phototransistor(output), ir_led(input) {
        ir_led = 0;     // Turn the IR LED off by default
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


/* ------------------------------- TrackControl class -------------------------- */
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
            error = U1.read() - U2.read();
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


/* ------------------------------- Buggy class --------------------------------- */
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


/* ------------------------------- Bluetooth class ----------------------------- */
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


/* ------------------------------- Test functions ------------------------------ */
void motorClassTest() {
    /* Use this test to:
        --> Check if you can control the direction of rotation for both motors
            and if the direction is correct, i.e. FORWARD will actually move the buggy forward
        --> Check if you can control the voltage for both motors
        --> Check if the relationship between the voltage and the velocity is approximately linear,
            e.g. when you apply 0.2 the motor starts to rotate at ~20% of max velocity
        --> Check if both motors doesn't rotate when "setVoltage(0.0)"
    */

    Serial pc(USBTX, NC);   // Creates an instance of a Serial Connection with default parameters
    pc.printf("\nMotorClassTest initialised\n");  // Print a message

    Motor motorLeft(PIN_MOTOR_L_MODE, PIN_MOTOR_L_DIR, PIN_MOTOR_L_PWM, SWITCHING_FREQUENCY);
    Motor motorRight(PIN_MOTOR_R_MODE, PIN_MOTOR_R_DIR, PIN_MOTOR_R_PWM, SWITCHING_FREQUENCY);
    
    motorLeft.setDirection(FORWARD);   // Test motors for FORWARD and BACKWARD directions
    motorRight.setDirection(BACKWARD);

    motorLeft.setVoltage(0.0);     // Test the speed for left and right motor 
    motorRight.setVoltage(1.0);
}


void encoderClassTest() {
    /* Use this test to:
        --> Check if you can see the velocity (fraction of MAX_VELOCITY) of the left and right wheel and if the values make sense
        --> Check if after 10 seconds you can see the value of the individual impulses counters for both wheels
        --> Check if one rotation of the wheel is equal to 256 pulses
        --> Measure what's the maximum speed in pulses/s and then in rev/s
        --> Check if after 10 seconds you can again see the velocity of both wheels
    */

    Serial pc(USBTX, NC);   // Creates an instance of a Serial Connection with default parameters
    pc.printf("\nEncoderClassTest initialised\n");  // Print a message

    Encoder wheelLeft(PIN_ENCODER_L_CHA, SAMPLING_FREQUENCY);
    Encoder wheelRight(PIN_ENCODER_R_CHA, SAMPLING_FREQUENCY);
    
    // Velocity measurement test

    for(int i=0; i<100; i++) { // Get 100 velocity readings (every 0.1 s)
        // Velocity is measured as a fraction of MAX_VELOCITY, which is set to 40 rev/s
        // (it's 40 rev/s as it has to greater than the actual max velocity of the wheel without any friction)
        pc.printf("v_left = %5.2f | v_right = %5.2f \n", wheelLeft.getVelocity(), wheelRight.getVelocity());  // Print current velocity
        wait(0.1);
    }

    // Pulses counter test

    wheelLeft.startCounter();
    wheelRight.startCounter();

    for(int i=0; i<20; i++) { // Get 20 counter values (every 0.5 s)
        pc.printf("counter_left = %d | counter_right = %d \n", wheelLeft.getCounter(), wheelRight.getCounter());  // Print current velocity
        wait(0.5);
    }

    Motor motorLeft(PIN_MOTOR_L_MODE, PIN_MOTOR_L_DIR, PIN_MOTOR_L_PWM, SWITCHING_FREQUENCY);
    motorLeft.setVoltage(1.0); // Turn the left wheel with max velocity

    int initial_counter_value = wheelLeft.getCounter();
    wait(5);
    int final_counter_value = wheelLeft.getCounter();
    int max_velocity = (( (final_counter_value - initial_counter_value) / 5) / PULSES_PER_REV);

    motorLeft.setVoltage(0.0); // Turn off the left motor
    pc.printf("Max velocity = %d rev/s \n", max_velocity);  // Print the average velocity when "setVoltage(1.0)"

    wheelLeft.stopCounter();
    wheelRight.stopCounter();

    // Velocity measurement test

    for(int i=0; i<100; i++) { // Get 100 velocity readings (every 0.1 s)
        pc.printf("v_left = %5.2f | v_right = %5.2f \n", wheelLeft.getVelocity(), wheelRight.getVelocity());  // Print current velocity
        wait(0.1);
    }
}


void propulsionClassTest() {
    /* Use this test to:
        --> Check if you can control the velocity and the angle of movement for the buggy (test different angles and velocities)
        --> Check if the buggy moves in a straight line when "drive(0.0, speed)"
        --> Check if the velocity stays const. irrespective of the friction / slope of the track
        --> Check if the angle stays const. over time
        --> Check if the turnaround() makes the buggy turn by exactly 180 degrees
    */

    Serial pc(USBTX, NC);   // Creates an instance of a Serial Connection with default parameters
    pc.printf("\nPropulsionClassTest initialised\n");  // Print a message

    Propulsion motors;

    motors.drive(0.0, 1.0); // drive(angle, velocity), angle = -1.0 - 1.0 (only right motor) and velocity = 0.0 - 1.0 (MAX_VELOCITY)
    wait(10); // Test driving for 10 seconds and test turnaround
    motors.drive(0.0, 0.0);
    motors.turnaround();
}


void sensorClassTest() {
    /* Use this test to:
        --> Check if every sensor works fine and returns a reading between 0.0 and 1.0
        --> Check ff the ambient reading is relatively small (e.g. <0.1)
        --> Check if there is a clear difference in the reading above white and black lines
        --> Check if detected() method works fine and is robust (always returns the correct value)
    */

    Serial pc(USBTX, NC);   // Creates an instance of a Serial Connection with default parameters
    pc.printf("\nSensoorClassTest initialised\n");  // Print a message

    Sensor U1(PIN_SENSOR_OUT1, PIN_SENSOR_IN1);
    Sensor U2(PIN_SENSOR_OUT2, PIN_SENSOR_IN2);
    Sensor U3(PIN_SENSOR_OUT3, PIN_SENSOR_IN3);
    Sensor U4(PIN_SENSOR_OUT4, PIN_SENSOR_IN4);
    Sensor U5(PIN_SENSOR_OUT5, PIN_SENSOR_IN5);
    Sensor U6(PIN_SENSOR_OUT6, PIN_SENSOR_IN6);

    for(int i=0; i<10; i++) { // Print 10 readings of the ambient IR light (every 1 s)
        pc.printf("%4.2f | %4.2f | %4.2f | %4.2f | %4.2f | %4.2f",
                  U1.getAmbient(), U2.getAmbient(), U3.getAmbient(), U4.getAmbient(), U5.getAmbient(), U6.getAmbient());
        wait(1);
    }


    for(int i=0; i<10; i++) { // Print 10 readings of the reflected IR light (every 1 s)
        pc.printf("%4.2f | %4.2f | %4.2f | %4.2f | %4.2f | %4.2f", U1.read(), U2.read(), U3.read(), U4.read(), U5.read(), U6.read());
        wait(1);
    }

    for(int i=0; i<10; i++) { // Print 10 values of the detected/not-detected (every 1 s)
        pc.printf("%d | %d | %d | %d | %d | %d",
                  U1.detected(), U2.detected(), U3.detected(), U4.detected(), U5.detected(), U6.detected());
        wait(1);
    }
}

/* ------------------------------- Main function ------------------------------- */
int main() {

    // motorClassTest();
    // encoderClassTest();
    // propulsionClassTest();
    // sensorClassTest();

    while(1) {}
}