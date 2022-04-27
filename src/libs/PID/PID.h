#ifndef PID_H
#define PID_H
#include "mbed.h"
#include "algorithm"

/*
    Bryce Williams 11/19/2015
    
    PID Controller Class based on Brett Beauregard's Arduino PID Library
    and PID blog post. 
    
    Brett Beauregard's blog post explains the PID code implementation very well
    and discusses why the actual equation is a bit different than the classical 
    equation, i.e. he explains and implements how to overcome windup, dervative
    kick, etc.  This class uses the same implementation, but adds interrupt 
    driven computation. 
    
    Reference Links:
    1. Arduion Library:
        (http://playground.arduino.cc/Code/PIDLibrary)
    2. Brett Beauregard's PID Blog:
        (http://brettbeauregard.com/blog/2011/04/improving-the-beginners-
         pid-introduction/)
*/

class PID{
    public:
        /*
            Constructor for PID objects. 
            
            Note: PID objects use given pointers, ie setpoint, 
            feedback, output inside interrupts. When reading/ modifying
            these vars make sure we don't have possible read/write 
            conflicts if the interrupt fires. Either ensure reads/writes
            are atomic operations, or call the stop() method perform the 
            read/write and then call the start() method.
            
            @param setpoint   The setpoint
            @param feedback   Pointer to feedback/sensor data var  
            @param output     Pointer to the output var
            @param output_lower    The lower bound of the output value
            @param output_upper    The upper bount of the output value
            @param kp   The Proportional Gain value
            @param ki   The Integral Gain value
            @param kd   The Derivative Gain value 
            @param Ts   The sample period at which the PID algorithm will
                        generate an interrupt and run. 
        */
        PID(float* setpoint, float* feedback, float* output,
            float output_lower, float output_upper,
            float  kp, float ki,  float kd, float Ts);
        /*
            Starts PID Controller; Attaches sample() as callback to Ticker 
            sample_timer and starts the interrupt
        */
        void start();
        /*
            Stops PID Contoller; detaches callback from Ticker sample_timer.
            Allows manual setting of output and read/write of shared vars, 
            DON'T FORGET TO CALL stop() before read/write of shared vars!
            Then after call start()!!!
        */
        void stop();
        /*
            Increments/ decrements Gain values and Sample time 
            by the given value. Gives a simple method to 
            programatically step through different values; just put in a 
            loop and go
            @param delta_"name" The value that will be added to its currently set value         
        */
//        void adjust_parameters(float delta_kp, float delta_ki, float delta_kd, float delta Ts);
        /*
            Overwrite Gain and Sample Time parameters with new 
            values
            Note: sample_timer interrupt is disabled during update
                  to avoid synch issues.
            
        */
        void set_parameters(float kp, float ki, float kd, float Ts);
        
        float getKp();
        float getKi();
        float getKd();
        float getTs();
        
        /*
            returns current error
        */
        float getError();
   
    private:
        float _kp, _ki, _kd;            // PID Gain values
        float _Ts;                      // Sample time is seconds
        float* _setpoint;               // Pointer to setpoint value
        float* _feedback;               // Pointer to sensor feedback value (sensor input)
        float* _output;                 // Pointer to control output value
        float  _output_lower;            // Ouput Lower Limit
        float  _output_upper;            // Output Upper Limit
        
        float i_accumulator;            // Integral Term accumulator
        float last_feedback;            // Previous feedback value
        float error;                    // Feedback error term
        Ticker sample_timer;            // Generates the sample time interrupt and calls sample()
        /* 
            sample() performs next sample calculationand updates command value
        */
        void sample();
        /*
            Clips value to lower/ uppper
            @param value    The value to clip
            @param lower    The mininum allowable value
            @param upper    The maximum allowable value
            @return         The resulting clipped value
        */
        float clip(float value, float lower, float upper);
};

#endif