#include <utils_class/PIDController.h>

PIDController::PIDController(){
    p = 1;
    i = 0;
    d = 0;

    error_now = error_before = error_acc = 0;
    out_now = 0;

    cap_error = 9999;
    use_cap_error = false;

    use_acc_error = false;
}

double PIDController::updateError(double error_now_){
    error_now = error_now_;

    if (use_acc_error)
        error_acc += error_now + error_before;
    else   
        error_acc = error_now + error_before;
    

    out_now = (double)(p * error_now + i * (error_acc) + d * (error_now - error_before));
    error_before = error_now;
    return out_now;
}

void PIDController::setP(double p_){
    p = p_;
}

void PIDController::setI(double i_){
    i = i_;
}

void PIDController::setD(double d_){
    d = d_;
}

double PIDController::getP(){
    return p;
}

double PIDController::getI(){
    return i;
}

double PIDController::getD(){
    return d;
}

