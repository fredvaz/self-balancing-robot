
// SELF BALANCING ARDUINO ROBOT WITH STEPPER MOTORS CONTROLLED BASED ON BROBOT EVO 2

// Date: 15/11/2018
// Updated: 02/02/2019
// License: GPL v2

// Original Project URL: http://jjrobots.com/b-robot-evo-2-much-more-than-a-self-balancing-robot (Features,documentation,build instructions,how it works, SHOP,...)
// Modified version: https://github.com/fredvaz/B-ROBOT_EVO2
// Simulink: https://github.com/fredvaz/self_balancing_robot/tree/cd_version

// The board needs at least 10-15 seconds with no motion (robot steady) at beginning to give good values... Robot move slightly when it´s ready!
// MPU6050 IMU connected via I2C bus. Angle estimation using complementary filter (fusion between gyro and accel)
// Angle calculations and control part is running at 100Hz

// The robot is OFF when the angle is high (robot is horizontal). When you start raising the robot it
// automatically switch ON and start a RAISE UP procedure.
// To switch OFF the robot you could manually put the robot down on the floor (horizontal)

// We have a PI controller for speed control and a PD controller for stability (robot angle)
// The output of the control (motors speed) is integrated so it´s really an acceleration not an speed.

// INITIALIZATION

boolean positionControlMode = false;
uint8_t mode; // mode = 0 Normal mode, mode = 1 Pro mode (More agressive)

int16_t motor1;
int16_t motor2;

// position control
volatile int32_t steps1;
volatile int32_t steps2;
int32_t target_steps1;
int32_t target_steps2;
int16_t motor1_control;
int16_t motor2_control;

int16_t speed_M1, speed_M2; // Actual speed of motors
int8_t dir_M1, dir_M2;      // Actual direction of steppers motors
int16_t actual_robot_speed; // overall robot speed (measured from steppers speed)
int16_t actual_robot_speed_Old;
float estimated_speed_filtered; // Estimated robot speed

void setup()
{
    // STEPPER PINS ON JJROBOTS BROBOT BRAIN BOARD
    pinMode(4, OUTPUT);    // ENABLE MOTORS
    pinMode(7, OUTPUT);    // STEP MOTOR 1 PORTE,6
    pinMode(8, OUTPUT);    // DIR MOTOR 1  PORTB,4
    pinMode(11, OUTPUT);   // STEP MOTOR 2 PORTB,7
    pinMode(12, OUTPUT);   // DIR MOTOR 2  PORTD,6
    digitalWrite(4, HIGH); // Disbale motors
    pinMode(10, OUTPUT);   // Servo1 (arm)
    pinMode(13, OUTPUT);   // Servo2

    Serial.begin(115200); // Serial output to console
    Serial1.begin(115200);

    // Initialize I2C bus (MPU6050 is connected via I2C)
    Wire.begin();

    //

    // Serial.println("JJROBOTS");
    delay(200);
    // Serial.println("Don't move for 10 sec...");
    MPU6050_setup(); // setup MPU6050 IMU
    delay(500);

    // Calibrate gyros
    MPU6050_calibrate();

    // STEPPER MOTORS INITIALIZATION
    // Serial.println("Steppers init");
    //  MOTOR1 => TIMER1
    TCCR1A = 0;                          // Timer1 CTC mode 4, OCxA,B outputs disconnected
    TCCR1B = (1 << WGM12) | (1 << CS11); // Prescaler=8, => 2Mhz
    OCR1A = ZERO_SPEED;                  // Motor stopped
    dir_M1 = 0;
    TCNT1 = 0;

    // MOTOR2 => TIMER3
    TCCR3A = 0;                          // Timer3 CTC mode 4, OCxA,B outputs disconnected
    TCCR3B = (1 << WGM32) | (1 << CS31); // Prescaler=8, => 2Mhz
    OCR3A = ZERO_SPEED;                  // Motor stopped
    dir_M2 = 0;
    TCNT3 = 0;
    delay(200);

    // Enable stepper drivers and TIMER interrupts
    digitalWrite(4, LOW); // Enable stepper drivers
    // Enable TIMERs interrupts
    TIMSK1 |= (1 << OCIE1A); // Enable Timer1 interrupt
    TIMSK3 |= (1 << OCIE1A); // Enable Timer1 interrupt
}

// MAIN LOOP
void loop()
{

    // New IMU data? CONTROL LOOP
    if (MPU6050_newData())
    {
        MPU6050_read_3axis();
        loop_counter++;
        slow_loop_counter++;
        dt = (timer_value - timer_old) * 0.000001; // dt in seconds
        // dt = (timer_value - timer_old) * 0.001; // dt in seconds - ADDED
        timer_old = timer_value;

        angle_adjusted_Old = angle_adjusted;
        // Get new orientation angle from IMU (MPU6050)
        float MPU_sensor_angle = MPU6050_getAngle(dt);
        angle_adjusted = MPU_sensor_angle + angle_offset;
        if ((MPU_sensor_angle > -15) && (MPU_sensor_angle < 15))
            angle_adjusted_filtered = angle_adjusted_filtered * 0.99 + MPU_sensor_angle * 0.01;

        if (positionControlMode)
        {
            // POSITION CONTROL. INPUT: Target steps for each motor. Output: motors speed
            motor1_control = positionPDControl(steps1, target_steps1, Kp_position, Kd_position, speed_M1);
            motor2_control = positionPDControl(steps2, target_steps2, Kp_position, Kd_position, speed_M2);

            // Convert from motor position control to throttle / steering commands
            throttle = (motor1_control + motor2_control) / 2;
            throttle = constrain(throttle, -190, 190);
            steering = motor2_control - motor1_control;
            steering = constrain(steering, -50, 50);
        }

        // ROBOT SPEED CONTROL: This is a PI controller.
        //    input:user throttle(robot speed), variable: estimated robot speed, output: target robot angle to get the desired speed
        target_angle = speedPIControl(dt, estimated_speed_filtered, throttle, Kp_thr, Ki_thr);
        target_angle = constrain(target_angle, -max_target_angle, max_target_angle); // limited output
        // TEST - ADDED
        // target_angle = -1.0;

    } // End of new IMU data
}