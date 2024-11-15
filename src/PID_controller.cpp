#include "PID_controller.h"

PIDController::PIDController(){
    _Kp = _Ki = _Kd= 0;
    _integral_min = _output_min = -1000;
    _integral_max = _output_max = 1000;
    _acceptable_error = 0.1;
    _frequency = 1;
    reset();
}

PIDController::~PIDController(){

}

void PIDController::setKp(double Kp){
    _Kp = Kp;
}

void PIDController::setKi(double Ki){
    _Ki = Ki;
}

void PIDController::setKd(double Kd){
    _Kd = Kd;
}

void PIDController::setFrequency(double frequency){
    _frequency = frequency;
}


void PIDController::setAcceptableError(double acceptable_error){
    _acceptable_error = acceptable_error;
}

double PIDController::getKp(){
    return _Kp;
}

double PIDController::getKi(){
    return _Ki;
}

double PIDController::getKd(){
    return _Kd;
}

double PIDController::getAcceptableError(){
    return _acceptable_error;
}

double PIDController::getFrequency(){
    return _frequency;
}

void PIDController::setOutputMin(double output_min){
    _output_min = output_min;
}

void PIDController::setOutputMax(double output_max){
    _output_max = output_max;
}

void PIDController::setIntegralMin(double integral_min){
    _integral_min = integral_min;
}

void PIDController::setIntegralMax(double integral_max){
    _integral_max = integral_max;
}

double PIDController::getOutputMin(){
    return _output_min;
}

double PIDController::getOutputMax(){
    return _output_max;
}

double PIDController::getIntegralMin(){
    return _integral_min;
}

double PIDController::getIntegralMax(){
    return _integral_max;
}

void PIDController::setTargetValue(double target_value){
    _target_value = target_value;
}

double PIDController::getTargetValue(){
    return _target_value;
}

void PIDController::updateOutput(double current_value,double rate_of_change,double time_difference){

    double error, p, i, d;    
    error = _target_value - current_value;
    
    if ( ( error >= 0 ) &&    ( error <= _acceptable_error  ) )
    {
        error = 0;
    }
    else if (( error < 0 ) &&    ( error >= _acceptable_error  ))
    {
        error = 0;
    }
    
    p = _Kp * error;

    i += _Ki * error* time_difference;

    i = limitToRange(i,_integral_min,_integral_max);
    _integrated_error += i;
    d = _Kd * rate_of_change;
    
    _output = p + i + d;

    _output = limitToRange(_output,_output_min,_output_max);
}

void PIDController::updateOutput(double current_value,double rate_of_change,double time_difference,double target_value){
    setTargetValue(target_value);
    updateOutput(current_value,rate_of_change,time_difference);
}

void PIDController::updateOutput(double current_value,double rate_of_change){
    updateOutput(current_value,rate_of_change,1/_frequency);
}

double PIDController::getOutput(){
    return _output;
}


void PIDController::reset(){
    _integrated_error = 0;
    _output = 0;
}

double PIDController::limitToRange(double value, double minimum, double maximum){
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