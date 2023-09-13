#include <iostream>
#include <wiringPi.h>
#include "libHCSR04/libHCSR04.h"

HCSR04::HCSR04(int trigger, int echo){
    this->trigger=trigger;
    this->echo=echo;
    wiringPiSetup();
    pinMode(trigger, OUTPUT);
    pinMode(echo, INPUT);
    digitalWrite(trigger, LOW);
    delay(500);
}

double HCSR04::distance(int timeout)
{
    delay(10);

    digitalWrite(trigger, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigger, LOW);

    now=micros();

    while (digitalRead(echo) == LOW && micros()-now<timeout);
        recordPulseLength();

    travelTimeUsec = endTimeUsec - startTimeUsec;
    distanceMeters = 100*((travelTimeUsec/1000000.0)*340.29)/2;

    return distanceMeters;
}


double HCSR04::speed(int timeout)
{
    // calls the distance() function above
    start_distance = this->distance(timeout) * 0.01;   // to m conversion

    // giving a time gap of 1 sec
    delay(1000);

    // calls the distance() function above a second time
    end_distance = this->distance(timeout) * 0.01;     // to m conversion

    // formula change in distance divided by change in time
    // as the time gap is 1 sec we divide it by 1.
    speedMS = (end_distance - start_distance) / 1.0; // m/s

    return speedMS;
}

void HCSR04::recordPulseLength()
{
    startTimeUsec = micros();
    while ( digitalRead(echo) == HIGH );
    endTimeUsec = micros();
}