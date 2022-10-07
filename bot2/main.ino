#include <Arduino.h>
#include <AccelStepper.h>

#define motorInterfaceType 1

int motor_dir[2] = {0, 0}, // 0 for clockwise, 1 anticlockwise
    dir_pin[2] = {6, 8},
    step_pin[2] = {9, 10};

// int step_interval = 500,
//     steps_per_rev = 200; // in microseconds

AccelStepper stepper1(motorInterfaceType, step_pin[0], dir_pin[0]),
    stepper2(motorInterfaceType, step_pin[1], dir_pin[1]);

void setup()
{
    // for (int i = 0; i < 2; i++)
    // {
    //     pinMode(dir_pin[i], OUTPUT);
    //     pinMode(step_pin[i], OUTPUT);
    // }

    stepper1.setMaxSpeed(1000);
    stepper1.setAcceleration(100);
    stepper1.setSpeed(200);
    stepper1.moveTo(2000);

    stepper2.setMaxSpeed(1000);
    stepper2.setAcceleration(100);
    stepper2.setSpeed(200);
    stepper2.moveTo(-2000);
}

void loop()
{
    if (stepper1.distanceToGo() == 0)
        stepper1.moveTo(-stepper1.currentPosition());
    if (stepper2.distanceToGo() == 0)
        stepper2.moveTo(-stepper2.currentPosition());

    stepper2.run();
    stepper1.run();
}