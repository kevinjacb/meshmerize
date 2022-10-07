#include <Arduino.h>

const int sensor_count = 6;
int dir_pin[4] = {3, 5, 6, 9},
    sensor_pin[sensor_count] = {A0, A1, A2, A3, A4, A5},
    condition = 0;

// stores sensor readings
int sensor_reading[sensor_count] = {0, 0, 0, 0, 0, 0},
    prev_reading[sensor_count] = {0, 0, 0, 0, 0, 0},
    error = 0, prev_error = 0;

// pid constants and variables
int kp = 30, ki = 0, kd = 19,
    pid_p = 0, pid_i = 0, pid_d = 0;

void setup()
{
    for (int i = 0; i < 4; i++)
    {
        pinMode(dir_pin[i], OUTPUT);
        analogWrite(dir_pin[i], 0);
    }
    for (int i = 0; i < sensor_count; i++)
        pinMode(sensor_pin[i], INPUT);
}

int state;
void loop()
{
    readSensors();
    for (int i = 0; i < sensor_count; i++)
        state += sensor_reading[i];
    PID();
    prev_error = error;
    if (state)
        for (int i = 0; i < sensor_count; i++)
            prev_reading[i] = sensor_reading[i];
    delay(50);
}

void readSensors()
{
    for (int i = 0; i < sensor_count; i++)
        sensor_reading[i] = digitalRead(sensor_pin[i]);

    /* errors:
100 -> turn hard left
101 -> turn hard right
102 -> u turn
103 -> stop
*/

    if (sensor_reading[0] && !sensor_reading[1] && !sensor_reading[2] && !sensor_reading[3] && !sensor_reading[4] && !sensor_reading[5])
        error = 5;
    else if (sensor_reading[0] && sensor_reading[1] && !sensor_reading[2] && !sensor_reading[3] && !sensor_reading[4] && !sensor_reading[5])
        error = 4;
    else if (!sensor_reading[0] && sensor_reading[1] && !sensor_reading[2] && !sensor_reading[3] && !sensor_reading[4] && !sensor_reading[5])
        error = 3;
    else if (!sensor_reading[0] && sensor_reading[1] && sensor_reading[2] && !sensor_reading[3] && !sensor_reading[4] && !sensor_reading[5])
        error = 2;
    else if (!sensor_reading[0] && !sensor_reading[1] && sensor_reading[2] && !sensor_reading[3] && !sensor_reading[4] && !sensor_reading[5])
        error = 1;
    else if (!sensor_reading[0] && !sensor_reading[1] && sensor_reading[2] && sensor_reading[3] && !sensor_reading[4] && !sensor_reading[5])
        error = 0;
    else if (!sensor_reading[0] && !sensor_reading[1] && !sensor_reading[2] && sensor_reading[3] && !sensor_reading[4] && !sensor_reading[5])
        error = -1;
    else if (!sensor_reading[0] && !sensor_reading[1] && !sensor_reading[2] && sensor_reading[3] && sensor_reading[4] && !sensor_reading[5])
        error = -2;
    else if (!sensor_reading[0] && !sensor_reading[1] && !sensor_reading[2] && !sensor_reading[3] && sensor_reading[4] && !sensor_reading[5])
        error = -3;
    else if (!sensor_reading[0] && !sensor_reading[1] && !sensor_reading[2] && !sensor_reading[3] && sensor_reading[4] && sensor_reading[5])
        error = -4;
    else if (!sensor_reading[0] && !sensor_reading[1] && !sensor_reading[2] && !sensor_reading[3] && !sensor_reading[4] && sensor_reading[5])
        error = -5;
    else if (sensor_reading[0] && sensor_reading[1] && sensor_reading[2] && !sensor_reading[3] && !sensor_reading[4] && !sensor_reading[5])
        error = 100;
    else if (!sensor_reading[0] && !sensor_reading[1] && !sensor_reading[2] && sensor_reading[3] && sensor_reading[4] && sensor_reading[5])
        error = 101;
    else if (!sensor_reading[0] && !sensor_reading[1] && !sensor_reading[2] && !sensor_reading[3] && !sensor_reading[4] && !sensor_reading[5])
        error = 102;
    else // (sensor_reading[0] && sensor_reading[1] && sensor_reading[2] && sensor_reading[3] && sensor_reading[4] && sensor_reading[5])
        error = 103;
}

void PID()
{
    int forward_delay = 100; // forward delay in ms
    switch (error)
    {
    case 100:
        forward();
        delay(forward_delay);
        stop();
        do
        {
            sharpLeft();
            readSensors();
        } while (error);
        stop();
        break;
    case 101:
        forward();
        delay(forward_delay);
        stop();
        do
        {
            sharpRight();
            readSensors();
        } while (error);
        stop();
        break;

    case 102:
        do
        {
            if (prev_reading[0])
                sharpLeft();
            else
                sharpRight();
            readSensors();
        } while (error);
        stop();
        break;
    case 103:
        stop();
        break;
    default:
        pid_p = error * kp;
        pid_i += error * ki;
        pid_d = (error - prev_error) * kd;
        int pid = pid_p + pid_i + pid_d;
        forward(constrain(SPEED - pid, 0, 255), constrain(SPEED + pid, 0, 255));
    }
}

void forward(int pwm1 = SPEED, int pwm2 = SPEED)
{
    analogWrite(dir_pin[0], 0);
    analogWrite(dir_pin[1], pwm1);
    analogWrite(dir_pin[2], 0);
    analogWrite(dir_pin[3], pwm2);
}
void reverse()
{
    analogWrite(dir_pin[0], SPEED);
    analogWrite(dir_pin[1], 0);
    analogWrite(dir_pin[2], SPEED);
    analogWrite(dir_pin[3], 0);
}
void sharpLeft()
{
    analogWrite(dir_pin[0], 0);
    analogWrite(dir_pin[1], 255);
    analogWrite(dir_pin[2], 255);
    analogWrite(dir_pin[3], 0);
}
void sharpRight()
{
    analogWrite(dir_pin[0], 255);
    analogWrite(dir_pin[1], 0);
    analogWrite(dir_pin[2], 0);
    analogWrite(dir_pin[3], 255);
}
void stop()
{
    analogWrite(dir_pin[0], 0);
    analogWrite(dir_pin[1], 0);
    analogWrite(dir_pin[2], 0);
    analogWrite(dir_pin[3], 0);
}