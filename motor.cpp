#include "motor.h"

motor::motor(int l_pwm, int r_pwm) {
    pinMode(l_pwm, OUTPUT);
    pinMode(r_pwm, OUTPUT);

    _l_pwm = l_pwm;
    _r_pwm = r_pwm;
}

void motor::move(int speed) {
    if (speed != 0){
        if (speed > 0) {
            analogWrite(_l_pwm, 0);
            analogWrite(_r_pwm, speed);
        } else {
            analogWrite(_l_pwm, abs(speed));
            analogWrite(_r_pwm, 0);
        }
    } else {
        stop();
    }
}

void motor::stop() {
    analogWrite(_l_pwm, 0);
    analogWrite(_r_pwm, 0);
}
