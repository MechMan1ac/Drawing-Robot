#ifndef motor_h
#define motor_h

#include <Arduino.h>

class motor {
    public:
        motor (int l_pwm, int r_pwm);
        void move (int speed);
        void stop ();
    private:
        int _l_pwm;
        int _r_pwm;
};

#endif
