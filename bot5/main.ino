#include <Arduino.h>

const int sensor_count = 8;
int dir_pin[4] = {3, 5, 6, 9},
    sensor_pin[sensor_count] = {A0, A1, A2, A3, A4, A5, 11, 12},
    condition = 0;

int SPEED = 200, MAX_SPEED = 255, TURN_SPEED = 140;
// stores sensor readings
int sensor_reading[sensor_count] = {0, 0, 0, 0, 0, 0, 0, 0},
    prev_reading[sensor_count] = {0, 0, 0, 0, 0, 0, 0, 0},
    error = 0, prev_error = 0;

// pid constants and variables
int kp = 18, ki = 0, kd = 15,
    pid_p = 0, pid_i = 0, pid_d = 0;

void stop();
void sharpRight();
void sharpLeft();
void reverse();
void forward(int pwm1 = SPEED, int pwm2 = SPEED);
void PID();
void readSensors();

void setup()
{
    Serial.begin(9600);
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
    // Serial.println(String(error) + " -> ");
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
    {
        int state = digitalRead(sensor_pin[sensor_count - i - 1]);
        sensor_reading[i] = (!condition) ? state : (1 - state);
        Serial.print(String(sensor_reading[i]) + " ");
    }
    Serial.println(String(error) + " -> ");
    /* errors:
100 -> turn hard left
101 -> turn hard right
102 -> u turn
103 -> stop
*/
    if (sensor_reading[0] && !sensor_reading[1] && !sensor_reading[2] && !sensor_reading[3] && !sensor_reading[4] && !sensor_reading[5] && !sensor_reading[6] && !sensor_reading[7])
        error = 7;
    else if (sensor_reading[0] && sensor_reading[1] && !sensor_reading[2] && !sensor_reading[3] && !sensor_reading[4] && !sensor_reading[5] && !sensor_reading[6] && !sensor_reading[7])
        error = 6;
    else if (sensor_reading[0] && sensor_reading[1] && sensor_reading[2] && !sensor_reading[3] && !sensor_reading[4] && !sensor_reading[5] && !sensor_reading[6] && !sensor_reading[7])
        error = 5.5;
    else if (!sensor_reading[0] && sensor_reading[1] && !sensor_reading[2] && !sensor_reading[3] && !sensor_reading[4] && !sensor_reading[5] && !sensor_reading[6] && !sensor_reading[7])
        error = 5;
    else if (!sensor_reading[0] && sensor_reading[1] && sensor_reading[2] && !sensor_reading[3] && !sensor_reading[4] && !sensor_reading[5] && !sensor_reading[6] && !sensor_reading[7])
        error = 4;
    else if (!sensor_reading[0] && sensor_reading[1] && sensor_reading[2] && sensor_reading[3] && !sensor_reading[4] && !sensor_reading[5] && !sensor_reading[6] && !sensor_reading[7])
        error = 3.5;
    else if (!sensor_reading[0] && !sensor_reading[1] && sensor_reading[2] && !sensor_reading[3] && !sensor_reading[4] && !sensor_reading[5] && !sensor_reading[6] && !sensor_reading[7])
        error = 3;
    else if (!sensor_reading[0] && !sensor_reading[1] && sensor_reading[2] && sensor_reading[3] && !sensor_reading[4] && !sensor_reading[5] && !sensor_reading[6] && !sensor_reading[7])
        error = 2;
    else if (!sensor_reading[0] && !sensor_reading[1] && sensor_reading[2] && sensor_reading[3] && sensor_reading[4] && !sensor_reading[5] && !sensor_reading[6] && !sensor_reading[7])
        error = 1.5;
    else if (!sensor_reading[0] && !sensor_reading[1] && !sensor_reading[2] && sensor_reading[3] && !sensor_reading[4] && !sensor_reading[5] && !sensor_reading[6] && !sensor_reading[7])
        error = 1;
    else if (!sensor_reading[0] && !sensor_reading[1] && !sensor_reading[2] && sensor_reading[3] && sensor_reading[4] && !sensor_reading[5] && !sensor_reading[6] && !sensor_reading[7])
        error = 0;
    else if (!sensor_reading[0] && !sensor_reading[1] && !sensor_reading[2] && sensor_reading[3] && sensor_reading[4] && sensor_reading[5] && !sensor_reading[6] && !sensor_reading[7])
        error = -0.5;
    else if (!sensor_reading[0] && !sensor_reading[1] && !sensor_reading[2] && !sensor_reading[3] && sensor_reading[4] && !sensor_reading[5] && !sensor_reading[6] && !sensor_reading[7])
        error = -1;
    else if (!sensor_reading[0] && !sensor_reading[1] && !sensor_reading[2] && !sensor_reading[3] && sensor_reading[4] && sensor_reading[5] && !sensor_reading[6] && !sensor_reading[7])
        error = -2;
    else if (!sensor_reading[0] && !sensor_reading[1] && !sensor_reading[2] && !sensor_reading[3] && sensor_reading[4] && sensor_reading[5] && sensor_reading[6] && !sensor_reading[7])
        error = -2.5;
    else if (!sensor_reading[0] && !sensor_reading[1] && !sensor_reading[2] && !sensor_reading[3] && !sensor_reading[4] && sensor_reading[5] && !sensor_reading[6] && !sensor_reading[7])
        error = -3;
    else if (!sensor_reading[0] && !sensor_reading[1] && !sensor_reading[2] && !sensor_reading[3] && !sensor_reading[4] && sensor_reading[5] && sensor_reading[6] && !sensor_reading[7])
        error = -4;
    else if (!sensor_reading[0] && !sensor_reading[1] && !sensor_reading[2] && !sensor_reading[3] && !sensor_reading[4] && sensor_reading[5] && sensor_reading[6] && sensor_reading[7])
        error = -4.5;
    else if (!sensor_reading[0] && !sensor_reading[1] && !sensor_reading[2] && !sensor_reading[3] && !sensor_reading[4] && !sensor_reading[5] && sensor_reading[6] && !sensor_reading[7])
        error = -5;
    else if (!sensor_reading[0] && !sensor_reading[1] && !sensor_reading[2] && !sensor_reading[3] && !sensor_reading[4] && !sensor_reading[5] && sensor_reading[6] && sensor_reading[7])
        error = -6;
    else if (!sensor_reading[0] && !sensor_reading[1] && !sensor_reading[2] && !sensor_reading[3] && !sensor_reading[4] && sensor_reading[5] && sensor_reading[6] && sensor_reading[7])
        error = -6.5;
    else if (!sensor_reading[0] && !sensor_reading[1] && !sensor_reading[2] && !sensor_reading[3] && !sensor_reading[4] && !sensor_reading[5] && !sensor_reading[6] && sensor_reading[7])
        error = -7;
    else if (sensor_reading[0] && sensor_reading[1] && sensor_reading[2] && !sensor_reading[5] && !sensor_reading[6] && !sensor_reading[7])
        error = 100;
    else if (!sensor_reading[0] && !sensor_reading[1] && !sensor_reading[2] && sensor_reading[5] && sensor_reading[6] && sensor_reading[7])
        error = 101;
    else if (!sensor_reading[0] && !sensor_reading[1] && !sensor_reading[2] && !sensor_reading[3] && !sensor_reading[4] && !sensor_reading[5] && !sensor_reading[6] && !sensor_reading[7])
        error = 102;
    else // (sensor_reading[0] && sensor_reading[1] && sensor_reading[2] && sensor_reading[3] && sensor_reading[4] && sensor_reading[5])
        error = 103;
}

void PID()
{
    int tolerance = 1;
    int forward_delay = 0; // forward delay in ms
    switch (error)
    {
    case 100:
        stop();
        forward();
        delay(forward_delay);
        stop();
        do
        {
            sharpLeft();
            readSensors();
        } while (abs(error) > tolerance);
        stop();
        break;
    case 101:
        stop();
        forward();
        delay(forward_delay);
        stop();
        do
        {
            sharpRight();
            readSensors();
        } while (abs(error) > tolerance);
        stop();
        break;

    case 102:
        // do
        // {
        //     if (prev_reading[0])
        //         sharpLeft();
        //     else
        //         sharpRight();
        //     readSensors();
        // } while (error);
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
        forward(constrain(SPEED - pid, -MAX_SPEED, MAX_SPEED), constrain(SPEED + pid, -MAX_SPEED, MAX_SPEED));
    }
}

void forward(int pwm1 = SPEED, int pwm2 = SPEED)
{
    Serial.println(String(pwm1) + " " + String(pwm2));
    if (pwm1 < 0)
    {
        analogWrite(dir_pin[0], abs(pwm1));
        analogWrite(dir_pin[1], 0);
    }
    else
    {
        analogWrite(dir_pin[1], pwm1);
        analogWrite(dir_pin[0], 0);
    }
    if (pwm2 >= 0)
    {
        analogWrite(dir_pin[2], 0);
        analogWrite(dir_pin[3], pwm2);
    }
    else
    {

        analogWrite(dir_pin[3], 0);
        analogWrite(dir_pin[2], abs(pwm2));
    }
}
void reverse()
{
    analogWrite(dir_pin[0], SPEED);
    analogWrite(dir_pin[1], 0);
    analogWrite(dir_pin[2], SPEED);
    analogWrite(dir_pin[3], 0);
}
void sharpRight()
{
    analogWrite(dir_pin[0], 0);
    analogWrite(dir_pin[1], TURN_SPEED);
    analogWrite(dir_pin[2], TURN_SPEED);
    analogWrite(dir_pin[3], 0);
}
void sharpLeft()
{
    analogWrite(dir_pin[0], TURN_SPEED);
    analogWrite(dir_pin[1], 0);
    analogWrite(dir_pin[2], 0);
    analogWrite(dir_pin[3], TURN_SPEED);
}
void stop()
{
    analogWrite(dir_pin[0], 0);
    analogWrite(dir_pin[1], 0);
    analogWrite(dir_pin[2], 0);
    analogWrite(dir_pin[3], 0);
}