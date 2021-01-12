
#ifndef _KINEMATICS_H_
#define _KINEMATICS_H_

#include "timer3.h"

/*
   Kinematics for calculating postion
   Author: Venkateswaran Narayanan
   Ref: https://github.com/paulodowd/EMATM0054_20_21/blob/master/Labsheets/Core/L8_Kinematics.ipynb
*/

struct vector3f {
  float x = 0;
  float y = 0;
  float theta = 0;
};

struct vector2f {
  float left = 0;
  float right = 0;
};


class Kinematics {
  public:
    vector3f pose;
    vector3f pose_old;

    vector3f update_pose(float speed_r_old, float speed_l_old);
    void print_pose();

    float constraint_theta(float theta);
    float findDistance(float x1, float y1, float x2, float y2) ;
    float calculate_theta(float x1, float y1, float x2, float y2);
    boolean is_point_reached(vector3f pose, vector3f waypoint);
    
  private:
    unsigned long last_millis = 0;
};

vector3f Kinematics::update_pose(float speed_r_old, float speed_l_old) {

  //Calculate how much time (in milliseconds) has
  // bassed since the last update call
  // Note, we do this in type "long", and then
  // typecast the final result to "float".
  long time_now = millis();
  long diff_time = time_now - last_millis;
  last_millis = time_now;

  float dt = (float)diff_time / 1000.0;
  float sum = speed_r_old + speed_l_old;
  float diff = speed_r_old - speed_l_old;

  if (abs(sum) < 0.005) {
    pose.x = pose_old.x;
    pose.y = pose_old.y;
    pose.theta = pose_old.theta + (diff * dt / l);
  }
  else if (abs(diff) < 0.03) {
    float s = sum/2;
    pose.x = pose_old.x + s * cos(pose_old.theta) * dt;
    pose.y = pose_old.y + s * sin(pose_old.theta) * dt;
    pose.theta = pose_old.theta;
  } 

  else {
    pose.x = pose_old.x + (sum * cos(pose_old.theta) * dt) / 2;
    pose.y = pose_old.y + (sum * sin(pose_old.theta) * dt) / 2;
    pose.theta = pose_old.theta + (diff * dt / l);
  }

  pose.theta = constraint_theta(pose.theta);

  pose_old.x = pose.x;
  pose_old.y = pose.y;
  pose_old.theta = pose.theta;

  //print_pose();

  return pose;
}

void Kinematics::print_pose() {
//  Serial.print( "x:  ");
  Serial.print( pose.x );
  Serial.print( ",");
//  Serial.print( "y: " );
  Serial.print( pose.y );
  Serial.print( ",");
//  Serial.print( "theta:  ");
  Serial.println( pose.theta * 180 / pi);
}



//constrain theta between - 0 -2 PI
float   Kinematics::constraint_theta(float theta) {
  if (theta > pi) {
    theta -= 2 * pi;
  } else if (theta < -pi) {
    theta += 2 * pi;
  }
  return theta;
}

//Distance formula - sqrt[ (X2 - X1)^2 + (Y2 - Y1)^2]
float Kinematics:: findDistance(float x1, float y1, float x2, float y2) {
  return sqrt(pow((x2 - x1), 2) + pow((y2 - y1), 2)); 
}

//
float  Kinematics::calculate_theta(float x1, float y1, float x2, float y2){
  float x = x2 -x1;
  float y = y2 -y1;

  float output_theta = atan2(y,x);   //tan inverse of Y over X

  return output_theta;
}

//Check is the given point has been reached
boolean Kinematics:: is_point_reached(vector3f pose, vector3f waypoint) {
  float dist = findDistance(pose.x, pose.y, waypoint.x, waypoint.y);
  if (dist <= 0.1) {
    return true;
  }
  return false;
}


#endif
