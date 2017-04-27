#ifndef PID_H
#define PID_H

class PID {
public:
    /*
     * Errors
     */
    double p_error;
    double i_error;
    double d_error;
    
    /*
     * Coefficients
     */
    double Kp;
    double Ki;
    double Kd;
    
    bool init_d = false;
    double prev_cte;
    
    /*
     * Constructor
     */
    PID();
    
    /*
     * Destructor.
     */
    virtual ~PID();
    
    /*
     * Initialize PID.
     */
    void Init(double Kp, double Ki, double Kd);
    
    /*
     * Update the PID error variables given cross track error.
     */
    void UpdateError(double cte);
    
    void UpdateSpeedError(double set_s, double desired);
    /*
     * Calculate the total PID error.
     */
    double TotalError();
};

#endif /* PID_H */
