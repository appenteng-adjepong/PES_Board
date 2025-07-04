#include "mbed.h"

// pes board pin map
#include "PESBoardPinMap.h"

// drivers
#include "DebounceIn.h"
#include "IRSensor.h"
#include "Servo.h"

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
    // set up states for state machine
    enum RobotState {
        INITIAL,
        EXECUTION, 
        SLEEP,
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

    // start timer
    main_task_timer.start();

    // create servo object
    Servo servo_D0(PB_D0);
    Servo servo_D1(PB_D1);
    Servo servo_D3(PB_D3);

    // minimal pulse width and maximal pulse width obtained from the calibration process
    // analog reely s-0090
    float servo_D0_ang_min = 0.035f;
    float servo_D0_ang_max = 0.120f;

    // analog reely s-0900
    float servo_D1_ang_min = 0.035f;
    float servo_D1_ang_max = 0.120f;

    // futabas3001
    float servo_D3_ang_min = 0.015f;
    float servo_D3_ang_max = 0.115f;

    // calibrate the servos

    servo_D0.calibratePulseMinMax(servo_D0_ang_min, servo_D0_ang_max);
    servo_D1.calibratePulseMinMax(servo_D1_ang_min, servo_D1_ang_max);
    servo_D3.calibratePulseMinMax(servo_D3_ang_min, servo_D3_ang_max);

    // set maximum acceleration of the servos. default value is 1.0e6f
    servo_D0.setMaxAcceleration(0.5f);
    servo_D1.setMaxAcceleration(0.5f);
    servo_D3.setMaxAcceleration(0.5f);

    // ir distance sensor instead of ultrasonic sensor
    IRSensor ir_sensor(PC_2);
    float ir_distance_cm = 0.0f;
    float ir_distance_min = 4.0f;
    float ir_distance_max = 22.0f;
    // calibrate the ir sensor
    ir_sensor.setCalibration(11.3055803e+03f, -45.6203f);

    // mechanical button
    DigitalIn mechanical_button(PC_5); // need to specify the mode for proper usage
    mechanical_button.mode(PullUp); // sets pullup between pin and 3.3 v, so that
    // there is a defined potential

 // define necessary variables
    float servo_input = 0.0f;
    int servo_counter = 0; // servo counter, this is an additional variable used to command the servo
    const int loops_per_seconds = static_cast<int>(ceilf(1.0f / (0.001f * static_cast<float>(main_task_period_ms))));
    // this loop will run forever
    while (true) {
        main_task_timer.reset();

        if (do_execute_main_task) {
            // print to the serial monitor
            printf("IR distance cm: %f \n", ir_distance_cm);
            
            // ir distance sensor instead of ultrasonic sensor
            const float ir_distance_cm_candidate = ir_sensor.read();
            if (ir_distance_cm_candidate > 0.0f && ir_distance_cm_candidate < 30.0f)
                ir_distance_cm = ir_distance_cm_candidate;

            switch (robot_state) {
            case RobotState::INITIAL: {
                printf("INITIAL \n");
                // enable the servo
                if (!servo_D0.isEnabled())
                    servo_D0.enable();
                    robot_state = RobotState::EXECUTION;

                break;
            }
            
            case RobotState::EXECUTION: {
                printf("EXECUTION \n");
                // function to map the distance to the servo movement 
                // (ir_distance_min, ir_distance_max) -> (0.0f, 1.0f)

                servo_input = (ir_distance_cm - ir_distance_min) / (ir_distance_max - ir_distance_min);
                // values smaller than 0.0f or bigger than 1.0f are constrained in setPulseWidth
                servo_D0.setPulseWidth(servo_input);

                // if the measurement is outside the min or max limit, go to SLEEP
                if ((ir_distance_cm < ir_distance_min) || (ir_distance_cm > ir_distance_max))
                    robot_state = RobotState::SLEEP;

                // if the mechanical button is pressed, go to EMERGENCY
                if (mechanical_button.read())
                    robot_state = RobotState::EMERGENCY;

                break;
            }

            case RobotState::SLEEP: {
                printf("SLEEP \n");

                // if the measurement is within the min and max limits, go to EXECUTION
                if ((ir_distance_cm > ir_distance_min) && (ir_distance_cm < ir_distance_max))
                    robot_state = RobotState::EXECUTION;
                
                // if the mechanical button is pressed, go to EMERGENCY
                if (mechanical_button.read())
                    robot_state = RobotState::EMERGENCY;

                break;
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

            // visual feedback that the main task is executed, setting this once would actually be enough
            led1 = 1;
            // read ir sensor distance, only valid measurements will update ir_distance_cm

        } else {
            // the following code block gets executed only once
            if (do_reset_all_once) {
                do_reset_all_once = false;

                // reset variables and objects
                led1 = 0;
                servo_D0.disable();
                ir_distance_cm = 0.0f;
                robot_state = RobotState::INITIAL;

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
        // sate machine
        do_reset_all_once = true;
}
