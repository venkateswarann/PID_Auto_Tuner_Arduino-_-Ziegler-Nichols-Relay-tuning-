
#ifndef _MOTOR_H
#define _MOTOR_H


#define MAX_PWR 100
float bias = 12;


class Motor{
  public:
    int dir_pin;
    int pwm_pin;
    Motor(int pwm_pin, int dir_pin) {
      this->dir_pin = dir_pin;
      this->pwm_pin = pwm_pin;

      // Set our motor driver pins as outputs.
      pinMode( dir_pin, OUTPUT );
      pinMode( pwm_pin, OUTPUT );

      digitalWrite( dir_pin, LOW );
      digitalWrite( pwm_pin, LOW );
    }

    void setPower( float power) {
      if (power < 0) {
        digitalWrite( dir_pin, HIGH ); //backward
        power *= -1;
      } else {
        digitalWrite( dir_pin, LOW ); //forward
      }
      if (power > MAX_PWR) {
        power = MAX_PWR;
      }
      if (power < bias) {
        power = 0;
      }

      analogWrite(pwm_pin, power);
    }

};

#endif
