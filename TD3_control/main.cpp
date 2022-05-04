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

#define PIN_ENCODER_L_CHA PC_12
#define PIN_ENCODER_R_CHA PC_10

#define PIN_SENSOR_OUT1 PA_0     // ADC
#define PIN_SENSOR_OUT2 PA_1     // ADC
#define PIN_SENSOR_OUT3 PA_4     // ADC
#define PIN_SENSOR_OUT4 PB_0     // ADC
#define PIN_SENSOR_OUT5 PC_1     // ADC
#define PIN_SENSOR_OUT6 PC_0     // ADC

#define PIN_SENSOR_IN1 PB_6
#define PIN_SENSOR_IN2 PA_7
#define PIN_SENSOR_IN3 PC_7
#define PIN_SENSOR_IN4 PA_6
#define PIN_SENSOR_IN5 PA_9
#define PIN_SENSOR_IN6 PA_5

// Motors, velocity measurement and control config
#define SWITCHING_FREQUENCY 10000.0f    // Set PWM switching frequency to 10 kHz (100 us period) [Hz]
#define SAMPLING_FREQUENCY 45           // Velocity measurement sampling frequency [Hz]
#define PULSES_DELTA_T_US 20000         // Delta t for pulses/s measurement [us] 
#define PULSES_PER_REV 256              // No. of quadrature encoder pulses per revolution [no units]
#define MAX_VELOCITY 10.0f              // Max velocity of the wheel (40 rev/s ~ 27 km/h for r=3 cm) [rev/s] 
#define TURNAROUND_PULSES 310           // Number of pulses for both motors to make turn the buggy by 180 degrees

// Debug mode config
#define DEBUG_MODE false                 // Turn the debug messages (serial monitor) on/off 

// Track control config
#define TRACK_DETECTED_THRESHOLD 0.2f   // Threshold value above which track_detected = true [voltage drop as a fraction of 3.3 V]
#define STANDARD_VOLTAGE 0.5f           // Set the standard voltage to be applied to motors
#define TURNAROUND_VOLTAGE 0.3f         // Voltage applied to motors during the turnaround

Serial pc(PA_11, NC);   // Creates an instance of a Serial Connection with default parameters (baud rate: 9600)

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
        voltage = 1.0f - voltage;           // Duty cycle reversed, i.e. 100% duty cycle disables the motor
        _motor.setDutyCycle(voltage);
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

    float getAmbient() {// Return sensor reading [0.0 - 1.0], where 0.0 means no IR light detected
        float value = 1.0f - phototransistor.read();
        return value;
    }

    float read() {      // Get normalised reading from the phototransistor [0.0 - 1.0]
        ir_led = 1;     // Turn the IR LED on
        float reading = (1.0f - phototransistor.read());
        return reading;
    }

    bool detected() {   // Return true if track has been detected by the sensor
        bool track_detected = (read() > TRACK_DETECTED_THRESHOLD);
        return track_detected;
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

/* ------------------------------- Main function ------------------------------- */
int main() {
    Bluetooth bt(PIN_BT_TX, PIN_BT_RX); // Initialise Bluetooth object
    Motor motorLeft(PIN_MOTOR_L_MODE, PIN_MOTOR_L_DIR, PIN_MOTOR_L_PWM, SWITCHING_FREQUENCY);
    Motor motorRight(PIN_MOTOR_R_MODE, PIN_MOTOR_R_DIR, PIN_MOTOR_R_PWM, SWITCHING_FREQUENCY);
    Encoder wheelLeft(PIN_ENCODER_L_CHA, SAMPLING_FREQUENCY);
    Encoder wheelRight(PIN_ENCODER_R_CHA, SAMPLING_FREQUENCY);

    motorLeft.setDirection(FORWARD);
    motorRight.setDirection(FORWARD);
    motorLeft.setVoltage(0.0);  // Stop both motors
    motorRight.setVoltage(0.0);
    if(DEBUG_MODE) {pc.printf("Start the test\n");}
    wait(3);    // Wait for the motors to stop spinning and to give some time to place the buggy on track

    Sensor U1(PIN_SENSOR_OUT1, PIN_SENSOR_IN1);
    Sensor U2(PIN_SENSOR_OUT2, PIN_SENSOR_IN2);
    Sensor U3(PIN_SENSOR_OUT3, PIN_SENSOR_IN3);
    Sensor U4(PIN_SENSOR_OUT4, PIN_SENSOR_IN4);
    Sensor U5(PIN_SENSOR_OUT5, PIN_SENSOR_IN5);
    Sensor U6(PIN_SENSOR_OUT6, PIN_SENSOR_IN6);

    double speed = STANDARD_VOLTAGE;
    bool high_speed = false, low_speed = false;
    int high_speed_counter = 0, stop_counter = 0;

    while(true) {   // Infinite loop
    
        if (bt.commandReceived()) { // If the command has been received
            wheelLeft.startCounter();   // Start counters for both wheels to measure the exact number of pulses from encoders
            wheelRight.startCounter();
            bool left_finished = false;         // Left wheel finished making required number of revolutions/pulses
            bool right_finished = false;        // Right wheel finished -||-
            motorLeft.setDirection(BACKWARD);   // Change the direction of the left motor to BACKWARD for the duration of turnaround

            while(left_finished == false || right_finished == false) {
                if(wheelLeft.getCounter() < TURNAROUND_PULSES) {
                    motorLeft.setVoltage(TURNAROUND_VOLTAGE);    // Set the speed of 30% for the left motor
                } else {
                    motorLeft.setVoltage(0.0);    // Turn off the left motor
                    left_finished = true;
                }

                if(wheelRight.getCounter() < TURNAROUND_PULSES) {
                    motorRight.setVoltage(TURNAROUND_VOLTAGE);    // Set the speed of 30% for the right motor
                } else {
                    motorRight.setVoltage(0.0);    // Turn off the right motor
                    right_finished = true;
                }
            }

            motorLeft.setDirection(FORWARD);   // Change the direction for left motor back to FORWARD
            wheelLeft.stopCounter();
            wheelRight.stopCounter();
        }
        
        if(DEBUG_MODE) {pc.printf("v_left = %5.2f | v_right = %5.2f \n", wheelLeft.getVelocity(), wheelRight.getVelocity());}  // Print current velocity

        if(wheelLeft.getVelocity() < 0.1f && high_speed == false) {
            if(DEBUG_MODE) {pc.printf("Vel < 0.06\n");}
            if(high_speed_counter > 5) {
                if(DEBUG_MODE) {pc.printf("Vel high adj\n");}
                speed = STANDARD_VOLTAGE*1.5f;
                high_speed = true;
                low_speed = false;
                high_speed_counter = 0;
            } else {
                high_speed_counter++;
            }
        } else if(wheelLeft.getVelocity() > 0.35f && low_speed == false) {
            if(DEBUG_MODE) {pc.printf("Vel > 0.35\n");}
            if(DEBUG_MODE) {pc.printf("Vel low adj\n");}
            speed = STANDARD_VOLTAGE;
            high_speed = false;
            low_speed = true;
        }
        if (U1.detected() == false && U2.detected() == false && U3.detected() == false && U4.detected() == false && U5.detected() == false && U6.detected() == false) {
            if (stop_counter > 10) {
                motorLeft.setVoltage(0); 
                motorRight.setVoltage(0);
                stop_counter = 0;  
                continue; 
            } else {
                stop_counter++;
            }
        }
        if (U5.detected() == true) {
            speed = STANDARD_VOLTAGE;
            motorLeft.setVoltage(speed*2f); // Keep driving right until you encounter a white line
            motorRight.setVoltage(speed/2f); 
            continue;
        }
        if (U6.detected() == true) {
            speed = STANDARD_VOLTAGE;
            motorLeft.setVoltage(speed/2f); // Keep driving right until you encounter a white line
            motorRight.setVoltage(speed*2f); 
            continue;
        }
        if (U3.detected() == true) { // when U3 detected line 
            motorLeft.setVoltage(speed*1.15f); // Keep driving right until you encounter a white line
            motorRight.setVoltage(speed); 
            continue;
        }
        if (U4.detected() == true) { // when U4 detected line
            motorLeft.setVoltage(speed); // Keep driving right until you encounter a white line
            motorRight.setVoltage(speed*1.15f);  
            continue;
        }
        if (U1.detected() == true && U2.detected() == true) { // Place the buggy initially on the right side of the line 
            motorLeft.setVoltage(speed); // Keep driving left until you encounter a white line
            motorRight.setVoltage(speed);
        }    
        if (U1.detected() == false && U2.detected() == true) { // Place the buggy initially on the right side of the line 
            motorLeft.setVoltage(speed); // Keep driving left until you encounter a white line
            motorRight.setVoltage(speed*1.1f);
            stop_counter = 0;              
        }
        if (U2.detected() == false && U1.detected() == true) {
            motorLeft.setVoltage(speed*1.1f); // Keep driving right until you encounter a white line
            motorRight.setVoltage(speed); 
            stop_counter = 0;     
        }
    }
}
