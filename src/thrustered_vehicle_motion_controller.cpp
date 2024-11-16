#include "thrustered_vehicle_motion_controller.h"
#include  "PID_controller.h"
#include  "thrusters_controller.h"
#include  <iostream>


ThrusteredVehicleMotionController::ThrusteredVehicleMotionController(/* args */){

    ThrustersController::init();
    
    surge_control_mode = SURGE_CONTROL_MODE;
    surge_controller.setKp(SURGE_K_P);
    surge_controller.setKi(SURGE_K_I);
    surge_controller.setKd(SURGE_K_D);
    surge_controller.setAcceptableError(SURGE_ACCEPTABLE_ERROR);
    surge_controller.setOutputMin(SURGE_OUTPUT_MIN);
    surge_controller.setOutputMax(SURGE_OUTPUT_MAX);
    surge_controller.setIntegralMin(SURGE_INTEGRAL_MIN);
    surge_controller.setIntegralMax(SURGE_INTEGRAL_MAX);
        
    heave_control_mode = HEAVE_CONTROL_MODE;
    heave_controller.setKp(HEAVE_K_P);
    heave_controller.setKi(HEAVE_K_I);
    heave_controller.setKd(HEAVE_K_D);
    heave_controller.setAcceptableError(HEAVE_ACCEPTABLE_ERROR);
    heave_controller.setOutputMin(HEAVE_OUTPUT_MIN);
    heave_controller.setOutputMax(HEAVE_OUTPUT_MAX);
    heave_controller.setIntegralMin(HEAVE_INTEGRAL_MIN);
    heave_controller.setIntegralMax(HEAVE_INTEGRAL_MAX);

    yaw_control_mode = YAW_CONTROL_MODE;
    yaw_controller.setKp(YAW_K_P);
    yaw_controller.setKi(YAW_K_I);
    yaw_controller.setKd(YAW_K_D);
    yaw_controller.setAcceptableError(YAW_ACCEPTABLE_ERROR);
    yaw_controller.setOutputMin(YAW_OUTPUT_MIN);
    yaw_controller.setOutputMax(YAW_OUTPUT_MAX);
    yaw_controller.setIntegralMin(YAW_INTEGRAL_MIN);
    yaw_controller.setIntegralMax(YAW_INTEGRAL_MAX);

    pitch_control_mode = PITCH_CONTROL_MODE;
    pitch_controller.setKp(PITCH_K_P);
    pitch_controller.setKi(PITCH_K_I);
    pitch_controller.setKd(PITCH_K_D);
    pitch_controller.setAcceptableError(PITCH_ACCEPTABLE_ERROR);
    pitch_controller.setOutputMin(PITCH_OUTPUT_MIN);
    pitch_controller.setOutputMax(PITCH_OUTPUT_MAX);
    pitch_controller.setIntegralMin(PITCH_INTEGRAL_MIN);
    pitch_controller.setIntegralMax(PITCH_INTEGRAL_MAX);
    
    roll_control_mode = ROLL_CONTROL_MODE;
    roll_controller.setKp(ROLL_K_P);
    roll_controller.setKi(ROLL_K_I);
    roll_controller.setKd(ROLL_K_D);
    roll_controller.setAcceptableError(ROLL_ACCEPTABLE_ERROR);
    roll_controller.setOutputMin(ROLL_OUTPUT_MIN);
    roll_controller.setOutputMax(ROLL_OUTPUT_MAX);
    roll_controller.setIntegralMin(ROLL_INTEGRAL_MIN);
    roll_controller.setIntegralMax(ROLL_INTEGRAL_MAX);

    surge_thrust = heave_thrust = yaw_thrust = pitch_thrust = roll_thrust = 0;

    for (int i = 0; i < NUMBER_OF_THRUSTERS; i++)
    {
        thrust_vector[i] = 0;
    }
    
    
}
ThrusteredVehicleMotionController::~ThrusteredVehicleMotionController(/* args */){
    ThrustersController::shutdown();
}

void ThrusteredVehicleMotionController::setSurgeControlMode(bool mode){
    surge_control_mode = mode;

    if (mode == CLOSED_LOOP_MODE)
    {
        surge_controller.reset();
    }
    
}

void ThrusteredVehicleMotionController::setHeaveControlMode(bool mode){
    heave_control_mode = mode;

     if (mode == CLOSED_LOOP_MODE)
    {
        heave_controller.reset();
    }
}

void ThrusteredVehicleMotionController::setYawControlMode(bool mode){
    yaw_control_mode = mode;

     if (mode == CLOSED_LOOP_MODE)
    {
        yaw_controller.reset();
    }
}

void ThrusteredVehicleMotionController::setPitchControlMode(bool mode){
    pitch_control_mode = mode;

     if (mode == CLOSED_LOOP_MODE)
    {
        pitch_controller.reset();
    }
}

void ThrusteredVehicleMotionController::setRollControlMode(bool mode){
    roll_control_mode = mode;

     if (mode == CLOSED_LOOP_MODE)
    {
        roll_controller.reset();
    }
}

void ThrusteredVehicleMotionController::setSurgePIDConstants(float kp,float ki, float kd, float acceptable_error){
    
    surge_controller.setKp(SURGE_K_P);
    surge_controller.setKi(SURGE_K_I);
    surge_controller.setKd(SURGE_K_D);
    surge_controller.setAcceptableError(SURGE_ACCEPTABLE_ERROR);

}

void ThrusteredVehicleMotionController::setHeavePIDConstants(float kp,float ki, float kd, float acceptable_error){
    
    heave_controller.setKp(HEAVE_K_P);
    heave_controller.setKi(HEAVE_K_I);
    heave_controller.setKd(HEAVE_K_D);
    heave_controller.setAcceptableError(HEAVE_ACCEPTABLE_ERROR);

}

void ThrusteredVehicleMotionController::setYawPIDConstants(float kp,float ki, float kd, float acceptable_error){
    
    yaw_controller.setKp(YAW_K_P);
    yaw_controller.setKi(YAW_K_I);
    yaw_controller.setKd(YAW_K_D);
    yaw_controller.setAcceptableError(YAW_ACCEPTABLE_ERROR);

}

void ThrusteredVehicleMotionController::setPitchPIDConstants(float kp,float ki, float kd, float acceptable_error){
    
    pitch_controller.setKp(PITCH_K_P);
    pitch_controller.setKi(PITCH_K_I);
    pitch_controller.setKd(PITCH_K_D);
    pitch_controller.setAcceptableError(PITCH_ACCEPTABLE_ERROR);

}

void ThrusteredVehicleMotionController::setRollPIDConstants(float kp,float ki, float kd, float acceptable_error){
    
    roll_controller.setKp(ROLL_K_P);
    roll_controller.setKi(ROLL_K_I);
    roll_controller.setKd(ROLL_K_D);
    roll_controller.setAcceptableError(ROLL_ACCEPTABLE_ERROR);

}

//Set Min Max Limits
void ThrusteredVehicleMotionController::setSurgePIDLimits(float output_min,float output_max, float integral_min, float integral_max){
    
    surge_controller.setOutputMin(SURGE_OUTPUT_MIN);
    surge_controller.setOutputMax(SURGE_OUTPUT_MAX);
    surge_controller.setIntegralMin(SURGE_INTEGRAL_MIN);
    surge_controller.setIntegralMax(SURGE_INTEGRAL_MAX);

}



void ThrusteredVehicleMotionController::setHeavePIDLimits(float output_min,float output_max, float integral_min, float integral_max){
        
    heave_controller.setOutputMin(HEAVE_OUTPUT_MIN);
    heave_controller.setOutputMax(HEAVE_OUTPUT_MAX);
    heave_controller.setIntegralMin(HEAVE_INTEGRAL_MIN);
    heave_controller.setIntegralMax(HEAVE_INTEGRAL_MAX);

}

void ThrusteredVehicleMotionController::setYawPIDLimits(float output_min,float output_max, float integral_min, float integral_max){
        
    yaw_controller.setOutputMin(YAW_OUTPUT_MIN);
    yaw_controller.setOutputMax(YAW_OUTPUT_MAX);
    yaw_controller.setIntegralMin(YAW_INTEGRAL_MIN);
    yaw_controller.setIntegralMax(YAW_INTEGRAL_MAX);
    
}

void ThrusteredVehicleMotionController::setPitchPIDLimits(float output_min,float output_max, float integral_min, float integral_max){
    
    pitch_controller.setOutputMin(PITCH_OUTPUT_MIN);
    pitch_controller.setOutputMax(PITCH_OUTPUT_MAX);
    pitch_controller.setIntegralMin(PITCH_INTEGRAL_MIN);
    pitch_controller.setIntegralMax(PITCH_INTEGRAL_MAX);

}

void ThrusteredVehicleMotionController::setRollPIDLimits(float output_min,float output_max, float integral_min, float integral_max){
    
    roll_controller.setOutputMin(ROLL_OUTPUT_MIN);
    roll_controller.setOutputMax(ROLL_OUTPUT_MAX);
    roll_controller.setIntegralMin(ROLL_INTEGRAL_MIN);
    roll_controller.setIntegralMax(ROLL_INTEGRAL_MAX);

}


void ThrusteredVehicleMotionController::setTargetSurgePoint(float point){
    if (surge_control_mode == CLOSED_LOOP_MODE)
    {
        surge_controller.setTargetValue(point);
    }
    else
    {
        std::cout<<"Error, closed loop control not enabled"<<std::endl;
    }
    

}


void ThrusteredVehicleMotionController::setTargetHeavePoint(float point){
    if (heave_control_mode == CLOSED_LOOP_MODE)
    {
        heave_controller.setTargetValue(point);
    }
    else
    {
        std::cout<<"Error, closed loop control not enabled"<<std::endl;
    }
    

}

void ThrusteredVehicleMotionController::setTargetYawAngle(float angle){
    if (yaw_control_mode == CLOSED_LOOP_MODE)
    {
        yaw_controller.setTargetValue(angle);
    }
    else
    {
        std::cout<<"Error, closed loop control not enabled"<<std::endl;
    }
    

}

void ThrusteredVehicleMotionController::setTargetPitchAngle(float angle){
    if (pitch_control_mode == CLOSED_LOOP_MODE)
    {
        pitch_controller.setTargetValue(angle);
    }
    else
    {
        std::cout<<"Error, closed loop control not enabled"<<std::endl;
    }
    

}

void ThrusteredVehicleMotionController::setTargetRollAngle(float angle){
    if (roll_control_mode == CLOSED_LOOP_MODE)
    {
        roll_controller.setTargetValue(angle);
    }
    else
    {
        std::cout<<"Error, closed loop control not enabled"<<std::endl;
    }
    

}

void ThrusteredVehicleMotionController::updateCurrentSurgePoint(float point){

    if (surge_control_mode == CLOSED_LOOP_MODE)
    {
         surge_controller.setCurrentValue(point);
         surge_thrust =  surge_controller.updateOutput();
    }
    else
    {
        std::cout<<"Error, closed loop control not enabled"<<std::endl;
    }

}


void ThrusteredVehicleMotionController::updateCurrentHeavePoint(float point){

    if (heave_control_mode == CLOSED_LOOP_MODE)
    {
         heave_controller.setCurrentValue(point);
         heave_thrust =  heave_controller.updateOutput();
    }
    else
    {
        std::cout<<"Error, closed loop control not enabled"<<std::endl;
    }

}

void ThrusteredVehicleMotionController::updateCurrentYawAngle(float angle){

    if (yaw_control_mode == CLOSED_LOOP_MODE)
    {
         yaw_controller.setCurrentValue(angle);
         yaw_thrust =  yaw_controller.updateOutput();
    }
    else
    {
        std::cout<<"Error, closed loop control not enabled"<<std::endl;
    }

}

void ThrusteredVehicleMotionController::updateCurrentPitchAngle(float angle){

    if (pitch_control_mode == CLOSED_LOOP_MODE)
    {
         pitch_controller.setCurrentValue(angle);
         pitch_thrust =  pitch_controller.updateOutput();
    }
    else
    {
        std::cout<<"Error, closed loop control not enabled"<<std::endl;
    }

}

void ThrusteredVehicleMotionController::updateCurrentRollAngle(float angle){

    if (roll_control_mode == CLOSED_LOOP_MODE)
    {
         roll_controller.setCurrentValue(angle);
         roll_thrust =  roll_controller.updateOutput();
    }
    else
    {
        std::cout<<"Error, closed loop control not enabled"<<std::endl;
    }

}


void ThrusteredVehicleMotionController::setSurgeThrust(float thrust){
     if (surge_control_mode != CLOSED_LOOP_MODE)
    {
         
         surge_thrust =  thrust;
    }
    else
    {
        std::cout<<"Error, closed loop control enabled, cannot  set thrust manually."<<std::endl;
    }

}


void ThrusteredVehicleMotionController::setHeaveThrust(float thrust){
     if (heave_control_mode != CLOSED_LOOP_MODE)
    {
         
         heave_thrust =  thrust;
    }
    else
    {
        std::cout<<"Error, closed loop control enabled, cannot  set thrust manually."<<std::endl;
    }

}


void ThrusteredVehicleMotionController::setYawThrust(float thrust){
     if (yaw_control_mode != CLOSED_LOOP_MODE)
    {
         
         yaw_thrust =  thrust;
    }
    else
    {
        std::cout<<"Error, closed loop control enabled, cannot  set thrust manually."<<std::endl;
    }

}

void ThrusteredVehicleMotionController::setPitchThrust(float thrust){
     if (pitch_control_mode != CLOSED_LOOP_MODE)
    {
         
         pitch_thrust =  thrust;
    }
    else
    {
        std::cout<<"Error, closed loop control enabled, cannot  set thrust manually."<<std::endl;
    }

}


void ThrusteredVehicleMotionController::setRollThrust(float thrust){
     if (roll_control_mode != CLOSED_LOOP_MODE)
    {
         
         roll_thrust =  thrust;
    }
    else
    {
        std::cout<<"Error, closed loop control enabled, cannot  set thrust manually."<<std::endl;
    }

}

void ThrusteredVehicleMotionController::resetAllThrusters(){
    surge_thrust = heave_thrust = yaw_thrust = pitch_thrust = roll_thrust = 0;
}

void ThrusteredVehicleMotionController::refresh(){
    ThrustersController::refresh();
}

 void ThrusteredVehicleMotionController::updateThrustValues(){

    for (int i = 0; i < NUMBER_OF_THRUSTERS; i++)
    {
        thrust_vector[i] = (surge_thrust*surge_vector[i]) + (heave_thrust*heave_vector[i]) + (yaw_thrust*yaw_vector[i]) +  (pitch_thrust*pitch_vector[i]) + (roll_thrust*roll_vector[i]);
        thrust_vector[i] = limitToRange(thrust_vector[i],MIN_THRUST,MAX_THRUST);
    }
    ThrustersController::writeThrusterValues(thrust_vector);
 }


float ThrusteredVehicleMotionController::limitToRange(float value, float minimum, float maximum){
    if (value > maximum)
    {
        return maximum;
    }
    else if (value < minimum)
    {
        return minimum;
    }
    else
    {
        return value;
    }
    
    
    
}
