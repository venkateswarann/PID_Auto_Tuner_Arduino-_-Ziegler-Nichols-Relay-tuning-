/*
   Encoders for motor rotations.
   Author: Venkateswaran Narayanan
   Ref: https://github.com/paulodowd/EMATM0054_20_21/blob/master/Labsheets/Core/L5_Encoders.ipynb
*/


class PID{
  public:
    // Constructor
    PID();

    // Methods - double
    double compute(double input);

    // Methods - void
    void begin();
    void tune(double _Kp, double _Ki, double _Kd);
    void limit(double min, double max);
    void setpoint(double newSetpoint);
    void minimize(double newMinimize);
    void printForDebugging();  
    void reset();

    // Methods - double, getters
    double getOutput();
  private:
    
    // Variables - long
    unsigned long lastTime;

    // Variables - double
    double output;
    double lastErr;
    double timeChanged;

    // Variables - double, error variables
    double error;
    double errSum;
    double dErr;

    // Variables - bool
    bool doLimit;
    bool init;

    // Variables - double - tuining
    double Kp;
    double Ki;
    double Kd;
    double divisor;
    double minOut;
    double maxOut;
    double setPoint;

    float Kp_output = 0;
    float Ki_output = 0;
    float Kd_output = 0;


    
};




PID::PID() {
  
  
}


void PID::printForDebugging() {

  Serial.print(Kp_output);
  Serial.print(", ");
  Serial.print(Kd_output);
  Serial.print(", ");
  Serial.print(Ki_output);
  Serial.print(", ");
  Serial.println(output);
  
}

void PID::begin () {
  Kp = 1;
  Ki = 1;
  Kd = 1;
  divisor = 10;
  doLimit = false;
  init = true;
}

void PID::setpoint (double newSetpoint) {
  setPoint = newSetpoint;
}

void PID::tune (double _Kp, double _Ki, double _Kd) {
  if (_Kp < 0 || _Ki < 0 || _Kd < 0) return;
  Kp = _Kp;
  Ki = _Ki;
  Kd = _Kd;
}

void PID::limit(double min, double max) {
  minOut = min;
  maxOut = max;
  doLimit = true;
}


void PID::minimize (double newMinimize) {
  divisor = newMinimize;
}

// Getters
double PID::getOutput () {
  return output;
}


double PID::compute (double sensor) {
  // Return false if it could not execute;
  // This is the actual PID algorithm executed every loop();

  // Failsafe, return if the begin() method hasn't been called
  if (!init) return 0;

  // Calculate time difference since last time executed
  unsigned long now = millis();
  double timeChange = (double)(now - lastTime);

  // Calculate error (P, I and D)
  double error = setPoint - sensor;
  errSum += error * timeChange;
  if (doLimit) {
    errSum = constrain(errSum, minOut * 1.1, maxOut * 1.1); 
  }
  double dErr = (error - lastErr) / timeChange;


  Kp_output = Kp * error;
  Ki_output = Ki * errSum;
  Kd_output = Kd * dErr;

  

  // Calculate the new output by adding all three elements together
  double newOutput = (Kp_output  + Ki_output + Kd_output ) / divisor;

 // double newOutput = (Kp * error + Ki * errSum + Kd * dErr);

  // If limit is specifyed, limit the output
  if (doLimit) {
    output = constrain(newOutput, minOut, maxOut);
  } else {
    output = newOutput;  
  }

  // Update lastErr and lastTime to current values for use in next execution
  lastErr = error;
  lastTime = now;

 

  // Return the current output
  return output;
}


void PID::reset() {
  lastErr      = 0;
  errSum  = 0;
  Kp_output       = 0;
  Ki_output       = 0;
  Kd_output       = 0;
  lastTime     = millis();
}
