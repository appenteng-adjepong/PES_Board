#include "mbed.h"

// pes board pin map
#include "PESBoardPinMap.h"

// drivers
#include "DebounceIn.h"
#include "Servo.h"
#include "DCMotor.h"
#include "IMU.h"
#include "IRSensor.h"
#include <Eigen/Dense>
#include "SensorBar.h"

// definitions
#define M_PIf 3.14159265358979323846f // pi

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
    /*------------------------------------ROBOT STATES-----------------------------------*/
    enum RobotState {
        INITIAL,
        SLEEP,
        LINE, 
        OBSTACLE,
        // MAZE,
        SLEEP,
        EMERGENCY
    } 
    robot_state = RobotState::INITIAL; // initialize robot state

    /*-----------------------------------------------------------------------------------*/

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
    // DigitalOut led1(PB_9);

    // start timer
    main_task_timer.start();

    /*-----------------------------------DC MOTOR------------------------------------------*/
    // create object to enable power electronics for the dc motors
    DigitalOut enable_motors(PB_ENABLE_DCMOTORS);
    const float voltage_max = 12.0f; // maximum voltage of battery packs, adjust this to
                                     // 6.0f V if you only use one battery pack

    // https://www.pololu.com/product/3490/specs
    const float gear_ratio = 100.00f;
    const float kn = 140.0f / 12.0f;

    // motor M1 and M2, do NOT enable motion planner when used with the LineFollower (disabled per default)
    // remember to swap the connections of m+ and m-, as well as the A and B terminals of one of the motors
    DCMotor motor_M1(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1, gear_ratio, kn, voltage_max);
    DCMotor motor_M2(PB_PWM_M2, PB_ENC_A_M2, PB_ENC_B_M2, gear_ratio, kn, voltage_max);

    /*----------------------DIFFERENTIAL DRIVE KINEMATICS---------------------------------*/
    const float d_gear = 0.0372f; // driven gear root diameter(from centre to contact area of teeth)
    const float b_wheel = 0.156f;  // wheelbase, distance from wheel to wheel in meters
    const float r1_gear = d_gear / 2.0f; // right driven gear radius in meters
    const float r2_gear = d_gear / 2.0f; // left  driven gear radius in meters

    // transformation matrix for gear to robot velocities
    // velocity of gear is the same as the velocity of track, assuming no slip. 
    Eigen::Matrix2f Cwheel2robot;
    Cwheel2robot << r1_gear / 2.0f   ,  r2_gear / 2.0f   ,
                    r1_gear / b_wheel, -r2_gear / b_wheel;

    /*----------------------------------SENSOR BAR----------------------------------------*/
    const float bar_dist = 0.114f; // distance from wheel axis to leds on sensor bar / array in meters
    SensorBar sensor_bar(PB_9, PB_8, bar_dist);

    // angle measured from sensor bar (black line) relative to robot
    float angle{0.0f};

    // rotational velocity controller
    const float Kp{5.0f};
    const float Knl{3.0f};
    const float wheel_vel_max = 2.0f * M_PIf * motor_M2.getMaxPhysicalVelocity(); //converting from rev/s to rad/s

    /*------------------------------IR PROXIMITY SENSOR----------------------------------*/
    float ir_distance_cm  = 0.0f;
    float ir_distance_min = 4.0f;
    float ir_distance_max = 22.0f;

    IRSensor ir_sensor(PC_2);
    // calibrate the ir sensor
    ir_sensor.setCalibration(11.3055803e+03f, -45.6203f);

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
    /*----------------------------------------IMU----------------------------------------*/
    ImuData imu_data;
    IMU imu(PB_IMU_SDA, PB_IMU_SCL);
    Eigen::Vector2f rp(0.0f, 0.0f); // vector for storing roll and pitch angles
    /*------------------------------------------------------------------------------------*/

    // this loop will run forever
    while (true) {
        main_task_timer.reset();

        if (do_execute_main_task) {

            // visual feedback that the main task is executed, setting this once would actually be enough
            // led1 = 1;
             // read analog input
             /*---------------IR READINGS-----------------------------*/
             const float ir_distance_cm_candidate = ir_sensor.read();
            if (ir_distance_cm_candidate > ir_distance_min && ir_distance_cm_candidate < ir_distance_max)
                ir_distance_cm = ir_distance_cm_candidate;
            // print to the serial monitor
            // static int cntr = 0;
            // if (++cntr % 50 == 0)
            // printf("IR distance cm: %f \n", ir_distance_cm);

            /*-----------------------------IMU READINGS----------------------*/
            // // roll, pitch, yaw according to Tait-Bryan angles ZYX
            // // where R = Rz(yaw) * Ry(pitch) * Rx(roll) for ZYX sequence
            // // singularity at pitch = +/-pi/2 radians (+/- 90 deg)
            rp(0) = imu_data.rpy(0); // roll angle
            rp(1) = imu_data.rpy(1); // pitch angle
            // map to servo commands
            servo_roll_width  = -normalised_angle_gain * rp(0) + normalised_angle_offset;
            servo_pitch_width =  normalised_angle_gain * rp(1) + normalised_angle_offset;

            /*--------------------------SENSORBAR READINGS-------------------------------*/
            if (sensor_bar.isAnyLedActive())
                angle = sensor_bar.getAvgAngleRad();


            switch (robot_state) {
            case RobotState::INITIAL: {
                /*----------------ENABLE DC AND SERVO MOTORS------------------------------*/
                enable_motors = 1;

                if (!servo_roll.isEnabled())
                    servo_roll.enable();
                if (!servo_pitch.isEnabled())
                    servo_pitch.enabe();

                // start balancing the ball immediately the INITIAL state is triggered
                if (angle_range_min <= rp(0) && rp(0) <= angle_range_max)
                    servo_roll.setPulseWidth(servo_roll_width);
                if (angle_range_min <= rp(1) && rp(1) <= angle_range_max)
                    servo_pitch.setPulseWidth(servo_pitch_width);
                // if any led is triggered, go to LINE
                if (sensor_bar.isAnyLedActive())
                    robot_state = RobotState::LINE;

        break;

            }
            
            case RobotState::LINE: {
                // printf("LINE \n");
                // start balancing the ball immediately the LINE state is triggered
                if (angle_range_min <= rp(0) && rp(0) <= angle_range_max)
                    servo_roll.setPulseWidth(servo_roll_width);
                if (angle_range_min <= rp(1) && rp(1) <= angle_range_max)
                    servo_pitch.setPulseWidth(servo_pitch_width);

            // control algorithm for robot velocities
            Eigen::Vector2f robot_coord = {0.5f * wheel_vel_max * r1_gear,  // half of the max. forward velocity
                                           Kp * angle + Knl * angle * abs(angle)}; // simple proportional angle controller

            // map robot velocities to wheel velocities in rad/sec
            Eigen::Vector2f wheel_speed = Cwheel2robot.inverse() * robot_coord;
                // only update sensor bar angle if an led is triggered
                // setpoints for the dc motors in rps
                motor_M1.setVelocity(wheel_speed(0) / (2.0f * M_PIf)); // set a desired speed for speed controlled dc motors M1
                motor_M2.setVelocity(wheel_speed(1) / (2.0f * M_PIf)); // set a desired speed for speed controlled dc motors M2

            

                break;
            }

            case RobotState::OBSTACLE: {
                printf("SLEEP \n");

                // if the obstacle is detected, slow down dc motor speed 
                
                
                // if the pitch goes below -10 (descent), slow down even further

                // else, go to LINE
              

                break;
            }

            case RobotState::MAZE: {
                printf("SLEEP \n");

                // if the measurement is within the min and max limits, go to EXECUTION
                
                
                // if the mechanical button is pressed, go to EMERGENCY
              

                break;

                
            }
            }


            case RobotState::EMERGENCY: {
                printf("EMERGENCY \n");
                // the transition to emergency state causes the execution of the commands contained in the outer else statement scope
                // and since do_reset_all_once is true, the system undergoes a reset

                toggle_do_execute_main_fcn();
                break;
            }

            default: {
                break; // do nothing
            }
        }

            

      
        } else {
            // the following code block gets executed only once
            if (do_reset_all_once) {
                do_reset_all_once = false;

                // reset variables and objects
                // led1 = 0;
                enable_motors = 0;
                servo_roll_width = 0.5f;
                servo_pitch_width = 0.5f;
                servo_roll.setPulseWidth(servo_roll_width);
                servo_pitch.setPulseWidth(servo_pitch_width);
            }
        }

        // toggling the user led
        user_led = !user_led;

        // // print to the serial terminal
         static int cntr = 0;
        if (++cntr % 50 == 0)
            ;
        // printf("Averaged Bar Raw: |  %0.2f  | %0.2f |  %0.2f |  %0.2f |  %0.2f |  %0.2f |  %0.2f |  %0.2f | ", sensor_bar.getAvgBit(0)
        //                                                                                                     , sensor_bar.getAvgBit(1)
        //                                                                                                     , sensor_bar.getAvgBit(2)
        //                                                                                                     , sensor_bar.getAvgBit(3)
        //                                                                                                     , sensor_bar.getAvgBit(4)
        //                                                                                                     , sensor_bar.getAvgBit(5)
        //                                                                                                     , sensor_bar.getAvgBit(6)
        //                                                                                                     , sensor_bar.getAvgBit(7));
        // printf("Mean Left: %0.2f, Mean Center: %0.2f, Mean Right: %0.2f \n", sensor_bar.getMeanThreeAvgBitsLeft()
        //                                                                 , sensor_bar.getMeanFourAvgBitsCenter()
        //                                                                 , sensor_bar.getMeanThreeAvgBitsRight());

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
