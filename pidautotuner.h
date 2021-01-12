

/*
   Kinematics for calculating postion
   Author: Venkateswaran Narayanan
   Refs :https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method
   
*/


// PID auto tuning using  Ziegler-Nichols/relay method

#ifndef _PIDAUTOTUNER_H_
#define _PIDAUTOTUNER_H_

#ifndef PI
#define PI 3.14159265358979323846

#endif
class PID_Autotuner {
  public:
   

    PID_Autotuner();
    void setTargetInputValue(float target);
    void setLoopInterval(long interval);
    void setOutputRange(float min, float max);


       // Constants for Ziegler-Nichols tuning mode with Realy method
    typedef enum {
      ZNModeBasicPID,
      ZNModeLessOvershoot,
      ZNModeNoOvershoot,
      PessenIntegralRule
    } ZNMode;

    
    void setZNMode(ZNMode zn);
    void setTuningCycles(int tuneCycles);

    // Must this call immediately before the tuning loop starts
    void startTuningLoop(unsigned long us);
    
    float tunePID(float input, unsigned long us);

    // Get Kp Ki and Kd values  
    float getKp();
    float getKi();
    float getKd();
    
  

    bool isFinished(); // Is the tuning finished?

  private:
    float targetInputValue = 0;
    float loopInterval = 0;
    float minOutput, maxOutput;
    ZNMode znMode = ZNModeNoOvershoot;
    int cycles = 10;

    // See startTuningLoop()
    int i;
    bool output;
    float outputValue;
    long microseconds, t1, t2, tHigh, tLow;
    float max, min;
    float pAverage, iAverage, dAverage;

    float kp, ki, kd;
};


PID_Autotuner::PID_Autotuner() {
}

          // Set target input for tuning
void PID_Autotuner::setTargetInputValue(float target) {
  targetInputValue = target;
}

       // Set loop interval
void PID_Autotuner::setLoopInterval(long interval) {
  loopInterval = interval;
}

      // Set output range
void PID_Autotuner::setOutputRange(float min, float max) {
  minOutput = min;
  maxOutput = max;
}

      // Set Ziegler-Nichols tuning mode
void PID_Autotuner::setZNMode(ZNMode zn) {
  znMode = zn;
}

    // Set tuning cycles
void PID_Autotuner::setTuningCycles(int tuneCycles) {
  cycles = tuneCycles;
}

    // Initialize all variables before loop
void PID_Autotuner::startTuningLoop(unsigned long us) {
  i = 0; // Cycle counter
  output = true; // Current output state
  outputValue = maxOutput;
  t1 = t2 = us; // Times used for calculating period
  microseconds = tHigh = tLow = 0; // More time variables
  max = -1000000000000; // Max input
  min = 1000000000000; // Min input
  pAverage = iAverage = dAverage = 0;
}

// Run one cycle of the loop
float PID_Autotuner::tunePID(float input, unsigned long us) {
  

  // Basic explanation of how this works:
  //  * Turn on the output of the PID controller to full power
  //  * Wait for the output of the system being tuned to reach the target input value
  //      and then turn the controller output off
  //  * Wait for the output of the system being tuned to decrease below the target input
  //      value and turn the controller output back on
  //  * Do this for a certain set of readings Till steady state osciallation / loop innterval is  finshed 
  //  * Calculate the ultimate gain using the amplitude of the controller output and
  //      system output 
  //

  //  * Use this and the period of oscillation to calculate PID gains using the
  //      Ziegler-Nichols method

  // Calculate time delta
  //long prevMicroseconds = microseconds;
  microseconds = us;
  //float deltaT = microseconds - prevMicroseconds;

  // Calculate max and min
  max = (max > input) ? max : input;
  min = (min < input) ? min : input;

  // Output is on and input signal has risen to target
  if (output && input > targetInputValue) {
    // Turn output off, record current time as t1, calculate tHigh, and reset maximum
    output = false;
    outputValue = minOutput;
    t1 = us;
    tHigh = t1 - t2;
    max = targetInputValue;
  }

  // Output is off and input signal has dropped to target
  if (!output && input < targetInputValue) {
    // Turn output on, record current time as t2, calculate tLow
    output = true;
    outputValue = maxOutput;
    t2 = us;
    tLow = t2 - t1;

    // Calculate Ku (ultimate gain)
    // Formula given is Ku = 4d / πa
    // d is the amplitude of the output signal
    // a is the amplitude of the input signal
    float ku = (4.0 * ((maxOutput - minOutput) / 2.0)) / (PI * (max - min) / 2.0);

    // Calculate Tu (period of output oscillations)
    float tu = tLow + tHigh;

    // How gains are calculated
    // PID control algorithm needs Kp, Ki, and Kd
    // Ziegler-Nichols tuning method gives Kp, Ti, and Td
    //
    // Kp = 0.6Ku = Kc
    // Ti = 0.5Tu = Kc/Ki
    // Td = 0.125Tu = Kd/Kc
    //
    // Solving these equations for Kp, Ki, and Kd gives this:
    //
    // Kp = 0.6Ku
    // Ki = Kp / (0.5Tu)
    // Kd = 0.125 * Kp * Tu
 
    // Constants
    // https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method

    float kpConstant, tiConstant, tdConstant;
    if (znMode == ZNModeBasicPID) {
      kpConstant = 0.6;
      tiConstant = 0.5;
      tdConstant = 0.125;
    } else if (znMode == ZNModeLessOvershoot) {
      kpConstant = 0.33;
      tiConstant = 0.4;
      tdConstant = 0.15;
    }else if (znMode == PessenIntegralRule) { //Pssen Integral Rule constants
      kpConstant =  0.7;
      tiConstant = 0.5;
      tdConstant = 0.33;    
      }
    else { // Default to No Overshoot mode as it is the safest
      kpConstant = 0.2;
      tiConstant = 0.5;
      tdConstant = 0.33;
    }

    // Calculate gains
    kp = kpConstant * ku;
    ki = (kp / (tiConstant * tu)) * loopInterval;
    kd = (tdConstant * kp * tu) / loopInterval;

    // Average all gains after the first two cycles
    if (i > 1) {
      pAverage += kp;
      iAverage += ki;
      dAverage += kd;
    }

    // Reset minimum
    min = targetInputValue;

    // Increment cycle count
    i++;
  }

  // If loop is done, disable output and calculate averages
  if (i >= cycles) {
    output = false;
    outputValue = minOutput;
    kp = pAverage / (i - 1);
    ki = iAverage / (i - 1);
    kd = dAverage / (i - 1);
  }

  return outputValue;
}

// Get PID constants after tuning
float PID_Autotuner::getKp() { return kp; };
float PID_Autotuner::getKi() { return ki; };
float PID_Autotuner::getKd() { return kd; };

// Is the tuning loop finished?
bool PID_Autotuner::isFinished() {
  return (i >= cycles);
}


#endif
