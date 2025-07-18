#include "mbed.h"

// pes board pin map
#include "PESBoardPinMap.h"

// drivers
#include "DebounceIn.h"
#include "DCMotor.h"
#include "SensorBar.h"
#include "LineFollower.h"
#include "Servo.h"
#include <Eigen/Dense>
#include <cstdlib>
#include <cmath>
#include <vector>

bool do_execute_main_task = false;
bool do_reset_all_once = false;

DebounceIn user_button(BUTTON1);
void toggle_do_execute_main_fcn();

/*--------------------------------------HELPER FUNCTIONS------------------------------------------------*/
float getrevolutions(float deg_angle, float wr, float wb);
bool isIntersection(float, float, float, bool *, bool *, bool *);
void storePath(char &direction, unsigned char &pathLength, char (&path)[100]);
void simplifyPath(char (&path)[100], unsigned char &pathLength);

int main()
{
    // set up states for state machine
    enum RobotState
    {
        INITIAL,
        SLEEP,
        TRAIN
    } robot_state = RobotState::INITIAL;

    enum MotionState
    {
        FOLLOWLINE,
        PREPARE_TURN,
        LEFT_TURN,
        RIGHT_TURN,
        UTURN,
        STOP
    } motion_state = MotionState::FOLLOWLINE;

    user_button.fall(&toggle_do_execute_main_fcn);

    const int main_task_period_ms = 20; // define main task period time in ms e.g. 20 ms, therefore
    int counter = 0;
    // the main task will run 50 times per second
    Timer main_task_timer;

    // led on nucleo board
    DigitalOut user_led(LED1);
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
    float leftMean = 0.0f, centerMean = 0.0f, rightMean = 0.0f;

    // angle measured from sensor bar (black line) relative to robot
    float angle{0.0f};

    // rotational velocity controller
    const float Kp = 1.2f * 10.0f;
    const float Kp_nl = 1.2f * 17.0f;
    const float wheel_vel_max = 2.0f * M_PIf * motor_M2.getMaxPhysicalVelocity();

    // navigation variables
    bool leftTurn = false, rightTurn = false, crossRoad = false, isIntersection = false, isDeadEnd = false;
    char dir = '0';

    float finalAngle = 0;
    float fixedAngle = 0;

    char path[100] = " ";         //"RSLLLRRRR";
    unsigned char pathLength = 0; // length of the path and index used while training. char is used as uint_8. actually an in
    int pathIndex = 0;            // Index used when the robot done training and completing the path
    unsigned int status = 0;      // solving = 0; reach Maze End = 1

    int deadEndCounter = 0;

    // start timer
    main_task_timer.start();

    // this loop will run forever
    while (true)
    {
        main_task_timer.reset();

        if (counter % 50 == 0)
        {
            printf("Robot state: %d, Motion State: %d, counter: %d\n", robot_state, motion_state, counter);
            counter = 0;
            // if (counter == 20 * 50) // resets every 20 seconds
            // {
            //     counter = 0;
            //     motion_state = MotionState::STOP;
            // }
        }

        if (do_execute_main_task)
        {
            led1 = 1;
            if (sensor_bar.isAnyLedActive())
                angle = sensor_bar.getAvgAngleRad();

            // Intersection and end detection
            leftMean = sensor_bar.getMeanThreeAvgBitsLeft();
            centerMean = sensor_bar.getMeanFourAvgBitsCenter();
            rightMean = sensor_bar.getMeanThreeAvgBitsRight();

            // --- code that runs when the blue button was pressed goes here ---
            switch (robot_state)
            {
            case RobotState::INITIAL:
            {
                // enable hardwaredriver dc motors: 0 -> disabled, 1 -> enabled
                enable_motors = 1;
                robot_state = RobotState::TRAIN;
                motion_state = MotionState::FOLLOWLINE;
                break;
            }
            case RobotState::SLEEP:
            {
                // enable hardwaredriver dc motors: 0 -> disabled, 1 -> enabled
                enable_motors = 0;
                if (do_execute_main_task)
                {
                    do_execute_main_task = false;
                    do_reset_all_once = true;
                    robot_state = RobotState::INITIAL;
                    motion_state = MotionState::FOLLOWLINE;
                }
                break;
            }
            case RobotState::TRAIN:
            {
                switch (motion_state)
                {
                case MotionState::FOLLOWLINE:
                {

                    leftTurn = (leftMean > 0.75f && centerMean > 0.5f && rightMean <= 0.5f);
                    rightTurn = (rightMean > 0.75f && centerMean > 0.5f && leftMean <= 0.5f);
                    crossRoad = (leftMean > 0.5f && centerMean > 0.5f && rightMean > 0.5f);
                    isDeadEnd = (leftMean < 0.1f && centerMean < 0.1f && rightMean < 0.1f);
                    isIntersection = (leftTurn || rightTurn || crossRoad);

                    if (isIntersection)
                    {
                        if (crossRoad)
                        {
                            dir = 'R';
                        }
                        else if (leftTurn)
                            dir = 'L';
                        else
                            dir = 'R';

                        fixedAngle = motor_M1.getRotation() + 0.10; // 0.5f; // should be 0.115 for the new wheels
                        finalAngle = motor_M2.getRotation() + 0.10; // 0.5f; // should be 0.115 for the new wheels
                        motion_state = MotionState::PREPARE_TURN;
                    }
                    else if (isDeadEnd)
                    {
                        deadEndCounter ++;
                        if (deadEndCounter >= 1 * 50)
                        {
                            fixedAngle = motor_M1.getRotation() + getrevolutions(-100, r_track, b_wheel); // fixed
                            finalAngle = motor_M2.getRotation() + getrevolutions(100, r_track, b_wheel);
                            // printf("Final Angle: %.2f, curent angle: %.2f\n", finalAngle, motor_M2.getRotation());
                            dir = 'B';
                            storePath(dir, pathLength, path);
                            motion_state = MotionState::UTURN;
                        }
                    }
                    else
                    {
                        // Normal line-following

                        // Controller implementation
                        deadEndCounter = 0;
                        // control algorithm for robot velocities
                        Eigen::Vector2f robot_coord = {0.4f * wheel_vel_max * r_track,  // half of the max. forward velocity
                                                    Kp * angle + Kp_nl * angle * abs(angle)}; // simple proportional angle controller

                        // map robot velocities to wheel velocities in rad/sec
                        Eigen::Vector2f wheel_speed = Cwheel2robot.inverse() * robot_coord;

                        // setpoints for the dc motors in rps
                        motor_M1.setVelocity(wheel_speed(1) / (2.0f * M_PIf)); // set a desired speed for speed controlled dc motors M1
                        motor_M2.setVelocity(wheel_speed(0) / (2.0f * M_PIf)); // set a desired speed for speed controlled dc motors M2
                  
                    }

                    break;
                }
                case MotionState::PREPARE_TURN:
                {
                    motor_M1.setRotation(fixedAngle);
                    motor_M2.setRotation(finalAngle);

                    if (std::fabs(motor_M1.getRotation() - fixedAngle) < 0.01)
                    {
                        printf("Angle: %f, leftmean: %f, centerMean: %f, rightMean: %f\n", angle, leftMean, centerMean, rightMean);
                        // scout ahead. Checking if there is a straight line ahead of a left turn
                        if (std::max(leftMean, std::max(centerMean, rightMean)) == centerMean)
                        {
                            if (dir == 'L')
                            {
                                dir = 'S';
                            }
                        }

                        // if it still detects something like a crossroad after advancing to turn, then it should stop.
                        if (leftMean > 0.5f && centerMean > 0.5f && rightMean > 0.5f) // end line detected
                        {
                            status = 1;
                            motion_state = MotionState::STOP;
                        }
                        else if (dir == 'R')
                        {
                            // RIGHT 90 DEGREE TURN
                            fixedAngle = motor_M1.getRotation() - 0.10; // fixed
                            finalAngle = motor_M2.getRotation() + getrevolutions(90, r_track, b_wheel);
                            // printf("Final Angle: %.2f, curent angle: %.2f\n", finalAngle, motor_M2.getRotation());

                            storePath(dir, pathLength, path);
                            motion_state = MotionState::RIGHT_TURN;
                        }
                        else if (dir == 'L')
                        {
                            // LEFT 90 DEGREE TURN
                            finalAngle = motor_M1.getRotation() + getrevolutions(90, r_track, b_wheel);
                            fixedAngle = motor_M2.getRotation() - 0.10; // fixed
                            // printf("Final Angle: %.2f, curent angle: %.2f\n", finalAngle, motor_M1.getRotation());

                            storePath(dir, pathLength, path);
                            motion_state = MotionState::LEFT_TURN;
                        }
                        else
                        {
                            storePath(dir, pathLength, path);
                            motion_state = MotionState::FOLLOWLINE;
                        }
                    }
                    break;
                }
                case MotionState::LEFT_TURN:
                {
                    motor_M1.setRotation(finalAngle);
                    motor_M2.setRotation(fixedAngle);

                    if (std::fabs(motor_M1.getRotation() - finalAngle) < 0.01)
                    {
                        motion_state = MotionState::FOLLOWLINE;
                    }

                    break;
                }
                case MotionState::RIGHT_TURN:
                {

                    motor_M1.setRotation(fixedAngle);
                    motor_M2.setRotation(finalAngle);

                    if (std::fabs(motor_M2.getRotation() - finalAngle) < 0.01)
                    {
                        motion_state = MotionState::FOLLOWLINE;
                    }
                    break;
                }
                case MotionState::UTURN:
                {
                    motor_M1.setRotation(fixedAngle);
                    motor_M2.setRotation(finalAngle);

                    if (std::fabs(motor_M2.getRotation() - finalAngle) < 0.01)
                    {
                        motion_state = MotionState::FOLLOWLINE;
                    }

                    break;
                }
                case MotionState::STOP:
                {
                    printf("stopping\n");
                    motor_M1.setVelocity(0);
                    motor_M2.setVelocity(0);

                    robot_state = RobotState::SLEEP;
                    break;
                }
                default:
                    break;
                }
                break;
            }
            }
        }
        else
        {
            // the following code block gets executed only once
            // motion_state = MotionState::FOLLOWLINE;

            if (do_reset_all_once)
            {
                do_reset_all_once = false;

                // --- variables and objects that should be reset go here ---

                // reset variables and objects
                led1 = 0;
                enable_motors = 0;
                robot_state = RobotState::INITIAL;
                // motion_state = MotionState::STOP;
                counter = 0;
            }
        }

        // toggling the user led
        user_led = !user_led;
        counter++;

        // --- code that runs every cycle goes here ---

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
float getrevolutions(float deg_angle, float wr, float wb)
{
    float L = (deg_angle / 360.0f) * 2.0f * M_PIf * wb;
    return L / (2 * M_PIf * wr);
}

// bool isIntersection(float leftMean, float centerMean, float rightMean, bool *leftTurn, bool *rightTurn, bool *crossRoad)
// {
//     *leftTurn = (leftMean > 0.5f && centerMean > 0.3f && rightMean <= 0.5f);
//     *rightTurn = (rightMean > 0.5f && centerMean > 0.3f && leftMean <= 0.5f);
//     *crossRoad = (leftMean > 0.5f && centerMean > 0.5f && rightMean > 0.5f);
//     bool isIntersection = (leftTurn || rightTurn || crossRoad);
//     return isIntersection;
// }
