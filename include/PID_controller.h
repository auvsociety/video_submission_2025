#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H


class PIDController
{
private: 
    double _Kp,_Ki,_Kd;
    double _target_value;
    double _integral_max,_integral_min;
    double _output_max,_output_min;
    double _acceptable_error;
    double _integrated_error;
    double _frequency;
    double _output;
    
    double limitToRange(double value, double minimum, double maximum);
public:
   
    
    PIDController();
    ~PIDController();
    
    // PID Parameters
    void setKp(double Kp);
    void setKi(double Ki);
    void setKd(double Kd);
    void setAcceptableError(double acceptable_error);
    void setFrequency(double frequency);
    double getKp();
    double getKi();
    double getKd();
    double getAcceptableError();
    double getFrequency();

    // PID limits
    void setOutputMin(double output_min);
    void setOutputMax(double output_max);
    void setIntegralMin(double integral_min);
    void setIntegralMax(double integral_max);
    double getOutputMin();
    double getOutputMax();
    double getIntegralMin();
    double getIntegralMax();
    // PID target value
    void setTargetValue(double target_value);
    double getTargetValue();
    // PID output
    void updateOutput(double current_value,double rate_of_change);
    void updateOutput(double current_value,double rate_of_change,double time_difference);
    void updateOutput(double current_value,double rate_of_change,double time_difference,double target_value);
    double getOutput();
    void reset();

    
};



#endif