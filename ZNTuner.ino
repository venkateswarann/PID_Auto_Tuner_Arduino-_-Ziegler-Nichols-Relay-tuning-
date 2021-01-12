/*
   Auto Tunned PID for ROMI's Motor using Zeigler - Nichols + Relay Method
   Author: Venkateswaran Narayanan
   Refs :https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method
   
*/


#include "pidautotuner.h"
#include "Encoder.h"
#include "Motors.h"
#include "PID.h"
#include "timer3.h"
#include "kinematics.h"

#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN  9
#define R_DIR_PIN 15

float targetInputValue = 70.0 ;//Middle speed is given for the system to Auto tune   // change it to as per your demand
int loopInterval = 10 ;

float demandSpeed = 10;


float l_power = 0;
float r_power = 0;



//Setup up initial systems
Motor   l_motor(L_PWM_PIN, L_DIR_PIN), r_motor(R_PWM_PIN, R_DIR_PIN);;
PID l_pid, r_pid;
Kinematics   kinematics;
vector3f way_point;

PID line;          // PID for controlling x coordinate
PID             x_pid;          // PID for controlling x coordinate
PID             y_pid;        // PID for controlling y coordinate
//PID l_pid(0, 0, 0);
double kp ;
double ki ;
double kd ;


void setup() {
  // put your setup code here, to run once:

  //  e0_count_prior = micros();
  //  e1_count_prior = micros();
  // getSpeed();
  Serial.begin(9600);
  delay(1000);

//Setting Up Encoder , PID ,waypoint 
  setupEncoder0();
  setupEncoder1();
  setupTimer3();


  l_pid.begin();
  r_pid.begin();
  x_pid.begin();
  y_pid.begin();
  line.begin();

//Given X = 0.6 and Y = 0 for moving romi to that coordinate
  way_point.x = 1.4;
  way_point.y = 0;
  way_point.theta = 0;
//  speed_r = map(speed_r,0,120,0,255);
//  speed_l =map(speed_l,0,120,0,255);
//Init Auto Tunner routine
  PID_Autotuner tuner = PID_Autotuner();
  tuner.setTargetInputValue(targetInputValue);

  // loop interval in microseconds

  tuner.setLoopInterval(loopInterval);

  // output range from 0 - 255

  tuner.setOutputRange(0, 255);

  // Set the Ziegler-Nichols tuning mode
  // These are the availabe list of modes to try { ZNModeBasicPID,  ZNModeLessOvershoot, ZNModeNoOvershoot ,PessenIntegralRule. Defaults mode is  ZNModeNoOvershoot
  tuner.setZNMode(PID_Autotuner::ZNModeBasicPID);
  tuner.startTuningLoop(micros());

  long microseconds;
  while (!tuner.isFinished()) {


    long prevMicroseconds = microseconds;
    microseconds = micros();

   //getting readings from encoder to auto tune the power for the wheels
   // taking absolute readings
    float speedR = abs(speed_r * 255);
    float speedL = speed_l * 255;
    //getSpeed();
    //power = 2.183 * motor_speed + 6.99
    
    //  int currentLSpeed = abs(map(l_power,0,67,0,255));
    Serial.print(speed_l * 255);

    //// Get input value from encoded speed in terms of power (0 - 255 range)!! send any sensor readings like temperature , encoder etc.
    double input = abs(speed_r * 255);
    //  double input = 40;
    Serial.print(" ");
    double output = tuner.tunePID(input, microseconds);

    Serial.println(output);


    // Do not change the value or the tuning results will be incorrect.

    //Set the output to motor --Setting the output value to the system to take steady state oscillation readings 
    r_motor.setPower(output);

    // While tuning the loop should run at same speed
    while (micros() - microseconds < loopInterval) delayMicroseconds(1);
  }

  // Turn off the output
  r_motor.setPower(0);
  // Get PID gains  and set it to PID controller

  kp = tuner.getKp();
  ki = tuner.getKi();
  kd = tuner.getKd();

  //Debugging 
  Serial.println("Kp");
  Serial.println(kp);
  Serial.println("Ki");
  Serial.println(ki);
  Serial.println("kd");
  Serial.println(kd);
  Serial.println("Tuning Done!");


//reset all things before starting the system ! its mandatory!
 // resetPIDS();

  //  setupEncoder0();
  //  setupEncoder1();
  //  setupTimer3();

//reset encoder counts to after auto tuning!!! else current posting of startig will be at different point
  count_l = 0;
  count_r = 0;
  count_r_old = 0;
  count_l_old = 0;


  line.tune(0.4, 0, 0.0001);
  x_pid.tune( 0.8, 0, 0.01);
  y_pid.tune( 0.8, 0, 0.01);
 
  speed_l = 0;
  speed_r = 0;

  
  //Manually Tuned values for the motor
  // l_pid.setGains(kp,ki,kd);
//  l_pid.tune(1.5, 0.02, 0.1);
//  r_pid.tune(1.5, 0.02, 0.1);
  


  //Values from auto tuner passsed to the system
      l_pid.tune(kp,ki,kd);
     r_pid.tune(kp,ki,kd);

   l_pid.limit(0, 255);
  r_pid.limit(0, 255);

   
  
}

void loop() {
  // put your main code here, to run repeatedly:

//  float SpeedL = map(speed_r,0,120,0,255);
//  float SpeedL =map(speed_l,0,120,0,255);

  vector3f pose = kinematics.update_pose(speed_r, speed_l);
  //  pose.x = abs(pose.x);
  //   pose.y = abs(pose.y);


  float demand_theta = kinematics.calculate_theta(pose.x, pose.y, way_point.x, way_point.y);
  //  demand_theta =  kinematics.constraint_theta(demand_theta);
  // Serial.println(kinematics.is_point_reached(pose, way_point));
  if (kinematics.is_point_reached(pose, way_point) == false) {

    //    Serial.print(pose.x);
    //    Serial.print(" ");
    //    Serial.print(pose.y);
    //    Serial.print(" ");
    //    Serial.print(demand_theta);
    //    Serial.print(" ");
    //    Serial.println( (pose.theta * (180 / pi)));

    line.setpoint(demand_theta);
    // pose.theta = kinematics.constraint_theta(pose.theta);
    float angular_velocity = line.compute(pose.theta);
    angular_velocity = kinematics.constraint_theta(angular_velocity);



    x_pid.setpoint(way_point.x);
    y_pid.setpoint(way_point.y) ;
    float velocity_x = x_pid.compute( pose.x);
    float velocity_y = y_pid.compute( pose.y);


    //   float forward_vel = (float) sqrt(pow((speed_l *255), 2) + pow((speed_r *255), 2));

    float forward_velocity = (float) sqrt(pow(velocity_x, 2) + pow(velocity_y, 2));


    //Automatic power setting for speed adjustment (uncomment the below code code of auto power)
//    l_pid.setpoint(forward_velocity - angular_velocity);
//    r_pid.setpoint(forward_velocity + angular_velocity);

    //Setting manual power setting for speed adjustment (0 -255)
         l_pid.setpoint(demandSpeed);
        r_pid.setpoint(demandSpeed);

    l_power = 255 * ( l_pid.compute(  (speed_l))); // Left PID Output 
    r_power =  255 * ( r_pid.compute((speed_r))); //  Right PID Output 




    Serial.print(speed_l * 255 );
    Serial.print(" ");
    Serial.print(speed_r * 255);
    Serial.print(" ");
    Serial.print(demandSpeed);
     Serial.print(" ");
    Serial.print(  pose.theta);
    Serial.println(" ");

  }
  else {
    l_power = 0;
    r_power = 0;

    Serial.println("Kp");
    Serial.println(kp);
    Serial.println("Ki");
    Serial.println(ki);
    Serial.println("kd");
    Serial.println(kd);
    Serial.println("Tuning Done!");

    l_pid.reset();
    r_pid.reset();
    resetPIDS();
  }

  l_motor.setPower(l_power);
  r_motor.setPower(r_power);

  delay(10);
}


void resetPIDS() {
  x_pid.reset();
  y_pid.reset();
  line.reset();
  l_pid.reset();
  r_pid.reset();
}
