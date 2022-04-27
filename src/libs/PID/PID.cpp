#include "PID.h"
/*
    Bryce Williams 11/19/2015
    See PID.h for references as well as method descriptions
*/

PID::PID(float* setpoint, float* feedback, float* output,
         float output_lower, float output_upper,
         float  kp, float ki,  float kd, float Ts){
    _Ts = Ts;           // Init params
    _kp = kp; 
    _ki = ki*Ts;        // Roll sample time into gain  
    _kd = kd / Ts;      // Roll sample time into gain
    
    _setpoint = setpoint; 
    _feedback = feedback; 
    _output = output;
    
    _output_lower = output_lower;
    _output_upper = output_upper;
}

void PID::start(){
    // Start up such we avoid bumps... (see "Initialization" section in
    // the reference link found in the header file).
    last_feedback = *_feedback;         // Eliminate derivative kick at start/restart
    i_accumulator = clip(*_output, _output_lower,
                     _output_upper);    // P and D terms are zero, thus 
                                        // i term is used to keep output unchanged
    /*
        If Ki is set to zero we must "flush" the intergral accumulator. 
        
        Reason:
        If we don't "flush", i_accumulator will hold the output value from line
        above, and because Ki is now zero only zeros will be added to 
        i_accumulator in the sample method, and thus i_accumulator is left 
        unchanged from here on out. i_accumulator is now a constant of value
        output from line above and will ALWAYS appear in the output. i.e.
        
        Here is the BUG if we DON'T FLUSH
        
        _ki = 0;            // User set ki = zero using  PID::set_parameters()
        
        THEN when PID::set_parameters() calls PID::start() (this method)
        
        i_accumulator = output;     // From line above
        
        Then when PID::sample() is called everytime...
        
        sample(){
            i_accumulator += _ki * error;   // Now this is equivalent to 
                                            // i_accumulator = output + 0
                                            // which always equals output
                                            // value from line above

            i_accumulator = clip(i_accumulator, _output_lower, _output_upper);
            
            // Run it!
            *_output = _kp*error + i_accumulator - _kd*(*_feedback - last_feedback);
                // i_accumulator is fixed at value output from line above
                // i.e. i_accumulator = clip(*_output, _output_lower,
                                              _output_upper) = output;
            last_feedback = *_feedback;
            // Clamp Output
            *_output = clip(*_output, _output_lower, _output_upper);
                // Here *_output will always be offset by "output" value
                // from line above 
        }   
    */
    if(-0.00001 <= _ki && _ki <= 0.00001) i_accumulator = 0;
    sample_timer.attach(this, &PID::sample, _Ts);
}

void PID::stop(){
    sample_timer.detach();
}

float PID::getError(){
    return error;
}

void PID::set_parameters(float kp, float ki, float kd, float Ts){
    stop();         // Disable Sample Interrupt... stop()
    _Ts = Ts;       // Set New Sample Time
    _kp = kp;       // Seet New Kp
    _ki = ki*Ts;    // Roll sample time into gain  
    _kd = kd / Ts;  // Roll sample time into gain
    start();        // Enable Sample Interrupt... start()
}

float PID::getKp(){
    return _kp;
}

float PID::getKi(){
    return _ki/_Ts;  // Remove Sample time adjustment so that 
                    // actual set ki is returned...
                    // Remember Sample time is rolled into 
                    // ki inside this class
}

float PID::getKd(){
    return _kd*_Ts;  // Remove Sample time adjustment so that 
                    // actual set kd is returned...
                    // Remember Sample time is rolled into 
                    // kd inside this class
}

float PID::getTs(){
    return _Ts;
}

void PID::sample(){
    error = *_setpoint - *_feedback;

    // Accumulate Integral Term such ki is applied to current error
    // before adding to pool; avoids bumps if ki gain value is changed.
    i_accumulator += _ki * error;
    // Avoid "Windup" by clamping intergral term to output limits;
    // essentially we stop integrating when we reach an upper or 
    // lower bound.
    i_accumulator = clip(i_accumulator, _output_lower, _output_upper);
    
    // Run it!
    *_output = _kp*error + i_accumulator - _kd*(*_feedback - last_feedback);
    last_feedback = *_feedback;
    // Clamp Output
    *_output = clip(*_output, _output_lower, _output_upper);
}

float PID::clip(float value, float lower, float upper){
    return std::max(lower, std::min(value, upper));
}