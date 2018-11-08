/*
 * gripper.hpp
 *
 *  Created on: 2018Äê11ÔÂ8ÈÕ
 *      Author: Pearl
 */

#ifndef SRC_GRIPPER_HPP_
#define SRC_GRIPPER_HPP_

#include "ch.h"
#include "hal.h"

#include "dbus.h"
#include "canBusProcess.h"

#include <stdlib.h>
#include <stdio.h>

static const float pi = 3.14159265358;

class gripper{
private:

    static int16_t motor_output;        //Torque command for motors


    static float motor_error_int; //error integrators for the four motors
    static float motor_error_der; //error derivatives for the four motors
    static int16_t previous_error;

public:
    //These are the parameters of PID controller
    static float gripper_kp;     //Proportional
    static float gripper_ki;     //Integration
    static float gripper_kd;     //Derivative


    static float initial_angle;
    static void grip(bool);
    static void rotate(int);
    static int16_t pid_control(const float setPoint, const float current,
                           float* error_int, float* error_der,
                           int16_t* previous_error);
    static void sub_grip(bool);
};


#endif /* SRC_GRIPPER_HPP_ */
