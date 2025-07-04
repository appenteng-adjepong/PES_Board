#include "mbed.h"

// pes board pin map
#include "PESBoardPinMap.h"

// drivers
#include "DebounceIn.h"
#include "DCMotor.h"
#include "fastPWM.h"

// define pins

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
    // create object to enable power electronics for the DC motors
    DigitalOut enable_motors(PB_ENABLE_DCMOTORS);


    // additional led
    // create DigitalOut object to command extra led, you need to add an additional resistor, e.g. 220...500 Ohm
    // a led has an anode (+) and a cathode (-), the cathode needs to be connected to ground via the resistor
    DigitalOut led1(PB_9);

    // create fastPWM objects to command the voltage applied to the DC motors
    FastPWM pwm_M1(PB_PWM_M1);
    FastPWM pwm_M2(PB_PWM_M2);
    FastPWM pwm_M3(PB_PWM_M3);

    // set up a DC motor object
    // const float voltage_max = 12.0f; // maximum voltage of battery packs, adjust this to
    //                                  //  6.0f if you only used one battery pack

    // // motor M1
    // const float gear_ratio_M1 = 100.0f; //gear ratio
    // const float kn_M1 = 180.0f / 12.0f; // motor constant [RPM/V]
    // // motor M2
    // const float gear_ratio_M2 = 100.0f; // gear ratio
    // const float kn_M2 = 180.0f / 12.0f; // motor constant [RPM/V]
    //  // motor M3
    // const float gear_ratio_M3 = 100.0f; // gear ratio
    // const float kn_M3 = 55.0f / 12.0f; // motor constant [RPM/V]
    
    // // connect first motor to M1
    // DCMotor motor_M1(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1, gear_ratio_M1, kn_M1, voltage_max);
    // // connect second motor to M2
    // DCMotor motor_M2(PB_PWM_M2, PB_ENC_A_M2, PB_ENC_B_M2, gear_ratio_M2, kn_M2, voltage_max);
    // // connect third motor to M3
    // DCMotor motor_M3(PB_PWM_M3, PB_ENC_A_M3, PB_ENC_B_M3, gear_ratio_M3, kn_M3, voltage_max);

    // // limit max velocity to half the physical possible velocity
    // motor_M1.setMaxVelocity(motor_M1.getMaxPhysicalVelocity()*0.5f);
    // motor_M2.setMaxVelocity(motor_M2.getMaxPhysicalVelocity()*0.5f);
    // motor_M3.setMaxVelocity(motor_M3.getMaxPhysicalVelocity()*0.5f);

    // // drive the motor with half of the maximum rotational velocity
    // motor_M1.setVelocity(motor_M1.getMaxVelocity()*0.5f);
    // motor_M2.setVelocity(motor_M2.getMaxVelocity()*0.5f);
    // motor_M3.setVelocity(motor_M3.getMaxVelocity()*0.5f);

    // // limit max acceleration to half of the default acceleration
    // motor_M1.setMaxAcceleration(motor_M1.getMaxAcceleration()*0.5f);
    // motor_M2.setMaxAcceleration(motor_M2.getMaxAcceleration()*0.5f);
    // motor_M3.setMaxAcceleration(motor_M3.getMaxAcceleration()*0.5f);

    // // enable motion planner for smooth movements
    // motor_M1.enableMotionPlanner();
    // motor_M2.enableMotionPlanner();
    // motor_M3.enableMotionPlanner();

    // start timer
    main_task_timer.start();

    // this loop will run forever
    while (true) {

        main_task_timer.reset();
        // printf("Motor Velocity: %f rpm Motor Position: %f \n", motor_M1.getVelocity(), motor_M1.getRotation());

        if (do_execute_main_task) {

            // visual feedback that the main task is executed, setting this once would actually be enough
            led1 = 1;
            // enable hardware driver DC motors: 0 -> disabled, 1 -> enabled
            enable_motors = 1;

            // rotate the motor three times
            // motor_M1.setRotation(3.0f);
            // motor_M2.setRotation(3.0f);
            // motor_M3.setRotation(3.0f);

            // apply +6V to the motor
            // pwm_M1.write(0.75f);
            // pwm_M2.write(0.75f);
            pwm_M3.write(0.75f);

        } else {
            // the following code block gets executed only once
            if (do_reset_all_once) {
                do_reset_all_once = false;

                // reset variables and objects
                led1 = 0;
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
