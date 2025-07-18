#include "mbed.h"

// pes board pin map
#include "PESBoardPinMap.h"

// drivers
#include "DebounceIn.h"
#include "Servo.h"
#include "DCMotor.h"
#include "IMU.h"
#include "IRSensor.h"
#include "SensorBar.h"
#include "LineFollower.h"
#include <Eigen/Dense>

bool do_execute_main_task = false; // this variable will be toggled via the user button (blue button) and
                                   // decides whether to execute the main task or not
bool do_reset_all_once = false;    // this variable is used to reset certain variables and objects and
                                   // shows how you can run a code segment only once

// objects for user button (blue button) handling on nucleo board
DebounceIn user_button(BUTTON1);   // create DebounceIn to evaluate the user button
void toggle_do_execute_main_fcn(); // custom function which is getting executed when user
                                   // button gets pressed, definition at the end

// main runs as an own thread
int main()
{
    // set up states for state machine
    enum RobotState {
        INITIAL,
        LINE, 
        OBSTACLE,
        EMERGENCY
    } robot_state = RobotState::INITIAL;

    // attach button fall function address to user button object
    user_button.fall(&toggle_do_execute_main_fcn);

    // while loop gets executed every main_task_period_ms milliseconds, this is a
    // simple approach to repeatedly execute main
    const int main_task_period_ms = 20; // define main task period time in ms e.g. 20 ms, therefore
                                        // the main task will run 50 times per second
    Timer main_task_timer;              // create Timer object which we use to run the main task
                                        // every main_task_period_ms

    // led on nucleo board
    DigitalOut user_led(LED1);

    // additional led
    // create DigitalOut object to command extra led, you need to add an additional resistor, e.g. 220...500 Ohm
    // a led has an anode (+) and a cathode (-), the cathode needs to be connected to ground via the resistor
    DigitalOut led1(PB_9);

    // --- adding variables and objects and applying functions starts here ---


    /*-----------------------------------DC MOTOR------------------------------------------*/
     // create object to enable power electronics for the DC motors
    DigitalOut enable_motors(PB_ENABLE_DCMOTORS);

    const float voltage_max = 12.0f; // maximum voltage of batteries

    const float gear_ratio = 250.0f; //gear ratio
    const float kn = 55.0f / 12.0f; // motor constant [RPM/V]

    // motor M1 and M2, do NOT enable motion planner when used with the LineFollower (disabled per default)
    DCMotor motor_M1(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1, gear_ratio, kn, voltage_max);
    DCMotor motor_M2(PB_PWM_M2, PB_ENC_A_M2, PB_ENC_B_M2, gear_ratio, kn, voltage_max);

    // enable motion planner
    motor_M1.enableMotionPlanner();
    motor_M2.enableMotionPlanner();

    const float bar_dist = 0.10252f; // distance from wheel axis to leds on sensor bar in meters
    const float d_gear = 0.048; // wheel diameter in meters
    const float b_wheel = 0.12775f; // wheel base in meters

    const float r_gear = d_gear / 2.0f; // wheel radius in meters
    
    const float gear_teeth = 16.0f;
    const float belt_teeth = 47.0f;
    const float belt_gear_ratio = gear_teeth / belt_teeth;
    // compute radius of the timing belt (this is the wheel radius in our specific case)
    const float r_track = r_gear / belt_gear_ratio;
    
    // transforms wheel to robot velocities
    Eigen::Matrix2f Cwheel2robot;
    Cwheel2robot << r_track / 2.0f   ,  r_track / 2.0f,
                    r_track / b_wheel, -r_track / b_wheel;

    // sensor bar
    SensorBar sensor_bar(PB_9, PB_8, bar_dist);

    // angle measured from sensor bar (black line) relative to robot
    float angle{0.0f};

    // rotational velocity controller
    const float Kp = 1.2f * 10.0f;
    const float Kp_nl = 1.2f * 17.0f;
    const float wheel_vel_max = 2.0f * M_PIf * motor_M2.getMaxPhysicalVelocity();
    /*------------------------------CREATE SERVO OBJECTS---------------------------------*/ 
    Servo servo_roll(PB_D0);
    Servo servo_pitch(PB_D1);

    // minimal and maximal pulse width obtained from the calibration process
    // analog reely s-0090
    float servo_ang_min = 0.035f;
    float servo_ang_max = 0.120f;

    // calibrate the servos
    // servo.setPulseWidth: before calibration (0,1) -> (min pwm, max pwm)
    // servo.setPulseWidth: after calibration (0,1) -> (servo_roll_ang_min, servo_roll_ang_max)
    servo_roll.calibratePulseMinMax(servo_ang_min, servo_ang_max);
    servo_pitch.calibratePulseMinMax(servo_ang_min, servo_ang_max);

    // angle limits of the servos
    const float angle_range_min = -M_PIf / 2.0f;
    const float angle_range_max =  M_PIf / 2.0f;

    // angle to pulse width coefficients
    const float normalised_angle_gain = 1.0f / M_PIf;
    const float normalised_angle_offset = 0.5f;

    // pulse width
    static float servo_roll_width = 0.5f;
    static float servo_pitch_width = 0.5f;

    servo_roll.setPulseWidth(servo_roll_width);
    servo_pitch.setPulseWidth(servo_pitch_width);

     // linear 1-D mahony filter
    const float Ts = static_cast<float>(main_task_period_ms) * 1.0e-3f; // sample time in seconds
    const float kp = 3.0f;
    float roll_estimate = 0.0f;
    float pitch_estimate = 0.0f;

    /*--------------------------------------IR SENSOR-----------------------------------------*/
    float ir_distance_cm = 0.0f;
    float thresh = 5;
    IRSensor ir_sensor(PC_2); // before the calibration the read function will return the averaged mV value
    ir_sensor.setCalibration(11.3055803e+03f, -45.6203f); // after the calibration the read function will return the calibrated value

    /*----------------------------------------IMU----------------------------------------*/
    ImuData imu_data;
    IMU imu(PB_IMU_SDA, PB_IMU_SCL);
    Eigen::Vector2f rp(0.0f, 0.0f); // vector for storing roll and pitch angles
    /*------------------------------------------------------------------------------------*/
    // start timer
    main_task_timer.start();
    // timers and flags for EMERGENCY and OBSTACLE states
    Timer emergency_timer;
    bool emergency_timer_started = false;
    Timer obstacle_timer;
    bool obstacle_timer_started = false;

    // this loop will run forever
    while (true) {
        main_task_timer.reset();

        // --- code that runs every cycle at the start goes here ---

        if (do_execute_main_task) {

        // --- code that runs when the blue button was pressed goes here ---

            // visual feedback that the main task is executed, setting this once would actually be enough
            led1 = 1;
            // read imu data
            imu_data = imu.getImuData();
            // read ir distance
            ir_distance_cm = ir_sensor.read();

                // linear 1-D mahony filter
            const float roll_acc = atan2f(imu_data.acc(1), imu_data.acc(2)); // roll angle from accelerometer
            const float pitch_acc = atan2f(-imu_data.acc(0), imu_data.acc(2)); // pitch angle from accelerometer
            
            roll_estimate  += Ts * (imu_data.gyro(0) + kp * (roll_acc  - roll_estimate ));
            pitch_estimate += Ts * (imu_data.gyro(1) + kp * (pitch_acc - pitch_estimate));
            rp(0) = roll_estimate; // roll angle
            rp(1) = pitch_estimate; // pitch angle


            // map to servo commands
            servo_roll_width  = -normalised_angle_gain * rp(0) + normalised_angle_offset;
            servo_pitch_width =  normalised_angle_gain * rp(1) + normalised_angle_offset;

            // enable the servo
            if (!servo_roll.isEnabled())
                servo_roll.enable();
            if (!servo_pitch.isEnabled())
            servo_pitch.enable();

            // state machine
            switch (robot_state) {
                case RobotState::INITIAL: {
                    printf("INITIAL\n");
                    enable_motors = 1;
                    robot_state = RobotState::LINE;
                    break;
                }
                case RobotState::LINE: {
                    printf("LINE\n");

                if (angle_range_min <= rp(0) && rp(0) <= angle_range_max)
                    servo_roll.setPulseWidth(servo_roll_width);
                 if (angle_range_min <= rp(1) && rp(1) <= angle_range_max)
                    servo_pitch.setPulseWidth(servo_pitch_width);
                    
                // only update sensor bar angle if an led is triggered
                if (sensor_bar.isAnyLedActive()) 
                    angle = sensor_bar.getAvgAngleRad();

                // control algorithm for robot velocities
                Eigen::Vector2f robot_coord = {0.4f * wheel_vel_max * r_track,  // half of the max. forward velocity
                                            Kp * angle + Kp_nl * angle * abs(angle)}; // simple proportional angle controller

                // map robot velocities to wheel velocities in rad/sec
                Eigen::Vector2f wheel_speed = Cwheel2robot.inverse() * robot_coord;

                // setpoints for the dc motors in rps
                motor_M1.setVelocity(wheel_speed(1) / (2.0f * M_PIf)); // set a desired speed for speed controlled dc motors M1
                motor_M2.setVelocity(wheel_speed(0) / (2.0f * M_PIf)); // set a desired speed for speed controlled dc motors M2
                  

                if (!sensor_bar.isAnyLedActive()) 
                    robot_state = RobotState::EMERGENCY;

                if (ir_distance_cm < thresh) 
                    robot_state = RobotState::OBSTACLE;

                break;
                }

                case RobotState::OBSTACLE: {
                    printf("OBSTACLE\n");
                    if (!obstacle_timer_started) {
                        obstacle_timer.reset();
                        obstacle_timer.start();
                        obstacle_timer_started = true;
                    }

                    if (angle_range_min <= rp(0) && rp(0) <= angle_range_max)
                        servo_roll.setPulseWidth(servo_roll_width);
                    if (angle_range_min <= rp(1) && rp(1) <= angle_range_max)
                        servo_pitch.setPulseWidth(servo_pitch_width);
                        
                    // only update sensor bar angle if an led is triggered
                    if (sensor_bar.isAnyLedActive()) 
                        angle = sensor_bar.getAvgAngleRad();

                    // control algorithm for robot velocities
                    Eigen::Vector2f robot_coord = {0.10f * wheel_vel_max * r_track,  // half of the max. forward velocity
                                                0.5f * Kp * angle + 0.5f * Kp_nl * angle * abs(angle)}; // simple proportional angle controller

                    // map robot velocities to wheel velocities in rad/sec
                    Eigen::Vector2f wheel_speed = Cwheel2robot.inverse() * robot_coord;

                    // setpoints for the dc motors in rps
                    motor_M1.setVelocity(wheel_speed(1) / (2.0f * M_PIf)); // set a desired speed for speed controlled dc motors M1
                    motor_M2.setVelocity(wheel_speed(0) / (2.0f * M_PIf)); // set a desired speed for speed controlled dc motors M2

                    if (std::chrono::duration_cast<std::chrono::seconds>(obstacle_timer.elapsed_time()).count() >= 20) { // go to LINE after 20 secs
                        robot_state = RobotState::LINE;
                        obstacle_timer.stop();
                        obstacle_timer_started = false;
                    }
                    break;
                }

                case RobotState::EMERGENCY: {
                    printf("EMERGENCY\n");
                    if (!emergency_timer_started) {
                        emergency_timer.reset();
                        emergency_timer.start();
                        emergency_timer_started = true;
                    }
                    if (sensor_bar.isAnyLedActive())
                        robot_state = RobotState::LINE;


                    if (std::chrono::duration_cast<std::chrono::seconds>(emergency_timer.elapsed_time()).count() >= 3) { // reset program if sensor bar does not read line after 3 secs
                        toggle_do_execute_main_fcn();
                        emergency_timer.stop();
                        emergency_timer_started = false;
                    }

                }
                
                default: {
                    break; // do nothing
                }
            }
        } else {
            // the following code block gets executed only once
            if (do_reset_all_once) {
                do_reset_all_once = false;

                // --- variables and objects that should be reset go here ---

                // reset variables and objects
                led1 = 0;
                servo_roll.disable();
                servo_pitch.disable();
                ir_distance_cm = 0.0f;
                enable_motors = 0;
                robot_state = RobotState::INITIAL;
            }
        }

        // toggling the user led
        user_led = !user_led;

        // --- code that runs every cycle at the end goes here ---
        // print ir readings to the serial monitor
        printf("IR distance: %.2f cm\tAngle: %.2f\n", ir_distance_cm, sensor_bar.getAvgAngleRad());
        // read timer and make the main thread sleep for the remaining time span (non blocking)
        int main_task_elapsed_time_ms = duration_cast<milliseconds>(main_task_timer.elapsed_time()).count();
        if (main_task_period_ms - main_task_elapsed_time_ms < 0)
            printf("Warning: Main task took longer than main_task_period_ms\n");
        else
            thread_sleep_for(main_task_period_ms - main_task_elapsed_time_ms);
    }
}

void toggle_do_execute_main_fcn()
{
    // toggle do_execute_main_task if the button was pressed
    do_execute_main_task = !do_execute_main_task;
    // set do_reset_all_once to true if do_execute_main_task changed from false to true
    if (do_execute_main_task)
        do_reset_all_once = true;
}