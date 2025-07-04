#include "mbed.h"

// pes board pin map
#include "PESBoardPinMap.h"

// drivers
#include "DebounceIn.h"
#include "DCMotor.h"
#include <Eigen/Dense>

#define M_PIf 3.14159265358979323846f // pi



bool do_execute_main_task = false; // this variable will be toggled via the user button (blue button) and
                                   // decides whether to execute the main task or not
bool do_reset_all_once = false;    // this variable is used to reset certain variables and objects and
                                   // shows how you can run a code segment only once

// objects for user button (blue button) handling on nucleo board
DebounceIn user_button(BUTTON1);   // create DebounceIn to evaluate the user button
void toggle_do_execute_main_fcn(); // custom function which is getting executed when user
                                   // button gets pressed, definition below

// main runs as an own thread
int main()
{
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

    // enable power electronics for dc motors
    DigitalOut enable_motors(PB_ENABLE_DCMOTORS);

    // set up a DC motor object
    const float voltage_max = 12.0f; // maximum voltage of battery packs, adjust this to
    //                                  //  6.0f if you only used one battery pack

    // 
    const float gear_ratio = 100.0f; //gear ratio
    const float kn = 180.0f / 12.0f; // motor constant [RPM/V]

       
    // connect first motor to M1
    DCMotor motor_right(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1, gear_ratio, kn, voltage_max);
    // connect second motor to M2
    DCMotor motor_left(PB_PWM_M2, PB_ENC_A_M2, PB_ENC_B_M2, gear_ratio, kn, voltage_max);
   
    // // limit max velocity to half the physical possible velocity
    // motor_right.setMaxVelocity(motor_right.getMaxPhysicalVelocity()*0.5f);
    // motor_left.setMaxVelocity(motor_left.getMaxPhysicalVelocity()*0.5f);

    // // limit max acceleration to half of the default acceleration
    // motor_right.setMaxAcceleration(motor_right.getMaxAcceleration()*0.5f);
    // motor_left.setMaxAcceleration(motor_left.getMaxAcceleration()*0.5f);

    // // enable motion planner for smooth movements
    motor_right.enableMotionPlanner();
    motor_left.enableMotionPlanner();

    // robot kinematics
    const float bar_dist = 0.118f; // distance from wheel axis to leds on sensor bar in meters

    const float r_wheel = 0.035f / 2.0f; // wheel radius in meters
    const float b_wheel = 0.1518f; // wheel base in meters

    Eigen::Matrix2f Cwheel2robot; // transform wheel to robot
    Cwheel2robot << r_wheel / 2.0f,     r_wheel / 2.0f,
                    r_wheel / b_wheel, -r_wheel / b_wheel;

    Eigen::Vector2f robot_coord = {0.0f, 0.0f}; // contains v and w (robot translational and rotational velocities)
    Eigen::Vector2f wheel_speed = {0.0f, 0.0f}; // w1 and w2 (wheel speeds)

    //assign desired values to the robot coordinates
    robot_coord(0) = 1.0f;
    robot_coord(1) = 0.5f;

    //assign appropriate values to the individual wheels based on desired values 
    wheel_speed = Cwheel2robot.inverse() * robot_coord;

    // start timer
    main_task_timer.start();

    // this loop will run forever
    while (true) {
        main_task_timer.reset();

        if (do_execute_main_task) {

            // visual feedback that the main task is executed, setting this once would actually be enough
            led1 = 1;
            enable_motors = 1;

            motor_right.setVelocity(wheel_speed(0) / (2.0f * M_PIf));
            motor_left.setVelocity(wheel_speed(1) / (2.0f * M_PIf));

        } else {
            // the following code block gets executed only once
            if (do_reset_all_once) {
                do_reset_all_once = false;

                // reset variables and objects
                led1 = 0;
                enable_motors = 0;
            }
        }

        // toggling the user led
        user_led = !user_led;

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
