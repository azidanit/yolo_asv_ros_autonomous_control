#ifndef SRC_PIDCONTROLLER_H
#define SRC_PIDCONTROLLER_H

class PIDController{
public:
    PIDController();
    double updateError(double error_now_);

    void setP(double p_); 
    void setI(double i_); 
    void setD(double d_); 

    double getP();
    double getI();
    double getD();

    void resetError();

private:
    double p, i, d;
    double error_now, error_before, error_acc;
    double out_now;
    double cap_error;

    bool use_cap_error, use_acc_error;

};

#endif

