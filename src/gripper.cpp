/*
 * gripper.cpp
 *
 *  Created on: 2018Äê11ÔÂ8ÈÕ
 *      Author: Pearl
 */
#include "gripper.hpp"

volatile Encoder_canStruct* encoder = can_getEncoder(); //Pointer to motor encoder feedback

float gripper::initial_angle = 0.0;
float gripper::gripper_kp = 150;     //Proportional
float gripper::gripper_ki = 0.5;     //Integration
float gripper::gripper_kd = 1500;     //Derivative
float gripper::motor_error_int = 0.0; //error integrators for the four motors
float gripper::motor_error_der = 0.0; //error derivatives for the four motors
int16_t gripper::previous_error = 0;

int16_t gripper::motor_output = 0;

void gripper::grip(bool close){
    if(close == 0){     //want the griper to grip sth
      palClearPad(GPIOA, 1);
    }
    else{               //relax the griper
      palSetPad(GPIOA, 1);
    }
}

void gripper::rotate(int button){
  while(1){
    if(button == 1){                    //rotate the arm forward
      motor_output = pid_control(105*(pi/180)*19+initial_angle,
                                 (encoder)->radian_angle,
                                 &motor_error_int, &motor_error_der,
                                 &previous_error);
      can_motorSetCurrent(0x1FF, 0.25*motor_output, 0, 0, 0);
      if((encoder)->radian_angle == 105*(pi/180)*19+initial_angle) break;
    }
    else if(button == 3){               //rotate the arm backward, meanwhile close the box
      motor_output = pid_control(initial_angle,
                                 (encoder)->radian_angle,
                                 &motor_error_int, &motor_error_der,
                                 &previous_error);
      can_motorSetCurrent(0x1FF, 15*motor_output, 0, 0, 0);
      if((encoder)->radian_angle == initial_angle) break;
    }
    else if(button == 2){               //maintain the location of the arm, open the box
      palSetPad(GPIOA,2);
      chThdSleepMilliseconds(2000);
    }
    chThdSleepMilliseconds(2);
    if((encoder)->speed_rpm == 0) break;
  }
}

void gripper::sub_grip(bool action){
    if(action == 0){        //action1:down,grip,up
      palClearPad(GPIOA,0);
      chThdSleepMilliseconds(1000);
      palSetPad(GPIOA,7);
      chThdSleepMilliseconds(1000);
      palSetPad(GPIOA,0);
      chThdSleepMilliseconds(200);
    }
    else{                   //action2:down,relax,up
      palClearPad(GPIOA,0);
      chThdSleepMilliseconds(1000);
      palClearPad(GPIOA,7);
      chThdSleepMilliseconds(1000);
      palSetPad(GPIOA,0);
      chThdSleepMilliseconds(200);
    }
}

int16_t gripper::pid_control(const float setPoint, const float current,
                           float* error_int, float* error_der,
                           int16_t* previous_error) {
  int16_t output = 0;

  int16_t error = 0;


  error = setPoint - current;
  *error_int += error;
  *error_int *= 0.5;

  *error_der = (error - *previous_error);

  if(error <= 19*pi/3 && error >= -19*pi/3) gripper_kp = 150;


  output = gripper::gripper_kp * error + gripper::gripper_ki * *error_int
      + gripper::gripper_kd * *error_der; //Proportional controller

      //NOTE: Limit your maximum integrated error is often useful
  float MAX_ERROR_INT = 10000000000;

  if (*error_int > MAX_ERROR_INT)
    *error_int = MAX_ERROR_INT;
  else if (*error_int < -MAX_ERROR_INT)
    *error_int = -MAX_ERROR_INT;

  chThdSleepMilliseconds(2);

  *previous_error = error;

  if (output > 5000)
    output = 5000;
  else if (output < -5000)
    output = -5000;

  return output;
}





