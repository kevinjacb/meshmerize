#include <Arduino.h>

#define MID 70
#define MAX 140

int dir_pin[4] = {8, 9, 10, 12},
    pwm_pin[2] = {6, 11};
int sensor[5] = {A1, A2, A3, A4, A5}; // center sensor is A3
int sensor_reading[5] = {0, 0, 0, 0, 0},
    prev_reading[5] = {0, 0, 0, 0, 0};
byte SPEED = 200;

int condition = 0; // when to switch

bool start = false;

void setup()
{
    Serial.begin(115200);
    for (int i = 0; i < 4; i++)
    {
        pinMode(dir_pin[i], OUTPUT);
        digitalWrite(dir_pin[i], 0);
    }
    for (int i = 0; i < 2; i++)
    {
        pinMode(pwm_pin[i], OUTPUT);
        analogWrite(pwm_pin[i], SPEED);
    }

    for (int i = 0; i < 5; i++)
        pinMode(sensor[i], INPUT);
    forward();
}

void loop()
{
    int state = status();
    while (!state && !start)
    {
        state = status();
        adjPWM(-100);
    }
    start = true;
    if (!state)
        adjPWM(correction(prev_reading));
    else if (state == 5)
        stop();
    else
        adjPWM(correction(sensor_reading));
    if (state > 0)
        for (int i = 0; i < 5; i++)
            prev_reading[i] = sensor_reading[i];
}

void read_sensors()
{
    int state;
    for (int i = 0; i < 5; i++)
    {
        state = digitalRead(sensor[i]);
        sensor_reading[i] = (!condition) ? state : (1 - state);
    }
}

int status()
{
    read_sensors();
    int check = 0;
    for (int i = 0; i < 5; i++)
        check += sensor_reading[i];
    return check;
}
void reverse()
{
    for (int i = 0; i < 4; i++)
        digitalWrite(dir_pin[i], (i % 2 == 0) ? HIGH : LOW);
}

int correction(int sensor_reading[5])
{
    if (sensor_reading[0] || sensor_reading[4])
    {
        if (sensor_reading[0])
            return (sensor_reading[1]) ? -MID : -MAX;
        else
            return (sensor_reading[3]) ? MID : MAX;
    }
    if (sensor_reading[1] || sensor_reading[3])
    {
        if (sensor_reading[1])
            return (sensor_reading[2]) ? -MID : -MAX;
        else
            return (sensor_reading[2]) ? MID : MAX;
    }
    return 0;
}

void adjPWM(int err)
{
    if (!err)
    {
        analogWrite(pwm_pin[0], SPEED);
        analogWrite(pwm_pin[1], SPEED);
    }
    if (err < 0)
    {
        analogWrite(pwm_pin[0], SPEED - abs(err));
        analogWrite(pwm_pin[1], SPEED);
    }
    else
    {
        analogWrite(pwm_pin[1], SPEED - abs(err));
        analogWrite(pwm_pin[0], SPEED);
    }
}

void forward()
{
    for (int i = 0; i < 4; i++)
        digitalWrite(dir_pin[i], (i % 2 == 0) ? LOW : HIGH);
}

void right()
{
    digitalWrite(dir_pin[0], HIGH);
    digitalWrite(dir_pin[1], LOW);
    digitalWrite(dir_pin[2], LOW);
    digitalWrite(dir_pin[3], HIGH);
}

void left()
{
    digitalWrite(dir_pin[0], LOW);
    digitalWrite(dir_pin[1], HIGH);
    digitalWrite(dir_pin[2], HIGH);
    digitalWrite(dir_pin[3], LOW);
}

void stop()
{
    for (int i = 0; i < 4; i++)
        digitalWrite(dir_pin[i], LOW);
}

// void correct(int error)
// {
//     if (!error)
//         return;
//     if (error > 0)
//         left();
//     else
//         right();
//     delay(abs(error));
//     forward();
// }
// int correction()
// {
//     read_sensors();
//     if (sensor_reading[0] || sensor_reading[4])
//     {
//         if (sensor_reading[0])
//             return (sensor_reading[1]) ? -MID : -MAX;
//         else
//             return (sensor_reading[3]) ? MID : MAX;
//     }
//     if (sensor_reading[1] || sensor_reading[3])
//     {
//         if (sensor_reading[1])
//             return (sensor_reading[2]) ? -MID : -MAX;
//         else
//             return (sensor_reading[2]) ? MID : MAX;
//     }
//     return 0;
// }