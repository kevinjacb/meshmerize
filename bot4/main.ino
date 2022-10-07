#include <Arduino.h>

#define MID 70
#define MAX 140

int dir_pin[4] = {8, 9, 10, 12},
    pwm_pin[2] = {6, 11};
const int sensor_count = 6;
int sensor[sensor_count] = {A1, A2, A3, A4, A5, A0};
// center sensor is A3 and A4
int sensor_reading[sensor_count] = {0, 0, 0, 0, 0, 0},
    prev_reading[sensor_count] = {0, 0, 0, 0, 0, 0};
byte SPEED = 200;

int condition = 0; // when to switch

bool start = false;

int kp = 25, ki = 0, kd = 12,
    pid_p = 0, pid_i = 0, pid_d = 0; // pid constants and variables
int prev_error = 0, error;

int state = 0;

void setSpeed(int mot1 = SPEED, int mot2 = SPEED);
void read_sensors();
void stop();
void left();
void right();
void forward();
void reverse();
int status();
void setError(int reading[sensor_count]);
void PID();

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

    for (int i = 0; i < sensor_count; i++)
        pinMode(sensor[i], INPUT);
    setSpeed();
    forward();
}

void loop()
{
    state = status();
    setError(sensor_reading);
    PID();
    if (state > 0)
        for (int i = 0; i < sensor_count; i++)
            prev_reading[i] = sensor_reading[i];
    Serial.println(error);
    prev_error = error;
    // displayReadings();
    delay(50);
}

void displayReadings()
{
    for (int i = 0; i < sensor_count; i++)
    {
        Serial.print(sensor_reading[i]);
        Serial.print(" ");
    }
    Serial.println();
}

void setSpeed(int mot1 = SPEED, int mot2 = SPEED)
{
    analogWrite(pwm_pin[0], mot1);
    analogWrite(pwm_pin[1], mot2);
}

void read_sensors()
{
    int state;
    for (int i = 0; i < sensor_count; i++)
    {
        state = digitalRead(sensor[i]);
        sensor_reading[i] = (!condition) ? state : (1 - state);
    }
}

void PID()
{
    switch (error)
    {
    case 100:
        stop();
        do
        {
            Serial.println(String(100));
            left();
            read_sensors();
        } while (!sensor_reading[2]);
        stop();
        break;
    case 101:
        stop();
        do
        {
            Serial.println(String(101));
            right();
            read_sensors();
        } while (!sensor_reading[3]);
        stop();
        break;
    case 102:
        stop();
        do
        {
            Serial.println(String(102));
            if (prev_reading[sensor_count - 1])
                right();
            else
                left();
            read_sensors();
        } while (!sensor_reading[2] && !sensor_reading[3]);
        stop();
        break;
    case 103:
        stop();
        break;
    default:
        pid_p = kp * error;
        pid_i += ki * error;
        pid_d = kd * (error - prev_error);
        int pid = pid_p + pid_i + pid_d;
        int pwm1 = constrain(SPEED - pid, 0, 255),
            pwm2 = constrain(SPEED + pid, 0, 255);
        Serial.println(String(pwm1) + " " + String(pwm2));
        setSpeed(pwm1, pwm2);
        forward();
    }
}

void setError(int reading[sensor_count])
{
    /* errors:
    100 -> turn hard left
    101 -> turn hard right
    102 -> u turn
    103 -> stop
    */

    if (reading[0] && !reading[1] && !reading[2] && !reading[3] && !reading[4] && !reading[5])
        error = 101;
    else if (reading[0] && reading[1] && !reading[2] && !reading[3] && !reading[4] && !reading[5])
        error = 4;
    else if (!reading[0] && reading[1] && !reading[2] && !reading[3] && !reading[4] && !reading[5])
        error = 3;
    else if (!reading[0] && reading[1] && reading[2] && !reading[3] && !reading[4] && !reading[5])
        error = 2;
    else if (!reading[0] && !reading[1] && reading[2] && !reading[3] && !reading[4] && !reading[5])
        error = 1;
    else if (!reading[0] && !reading[1] && reading[2] && reading[3] && !reading[4] && !reading[5])
        error = 0;
    else if (!reading[0] && !reading[1] && !reading[2] && reading[3] && !reading[4] && !reading[5])
        error = -1;
    else if (!reading[0] && !reading[1] && !reading[2] && reading[3] && reading[4] && !reading[5])
        error = -2;
    else if (!reading[0] && !reading[1] && !reading[2] && !reading[3] && reading[4] && !reading[5])
        error = -3;
    else if (!reading[0] && !reading[1] && !reading[2] && !reading[3] && reading[4] && reading[5])
        error = -4;
    else if (!reading[0] && !reading[1] && !reading[2] && !reading[3] && !reading[4] && reading[5])
        error = 100;
    else if (reading[0] && reading[1] && reading[2] && !reading[3] && !reading[4] && !reading[5])
        error = 100;
    else if (!reading[0] && !reading[1] && !reading[2] && reading[3] && reading[4] && reading[5])
        error = 101;
    else if (!reading[0] && !reading[1] && !reading[2] && !reading[3] && !reading[4] && !reading[5])
        error = 102;
    else // (reading[0] && reading[1] && reading[2] && reading[3] && reading[4] && reading[5])
        error = 103;
}

int status()
{
    read_sensors();
    int check = 0;
    for (int i = 0; i < sensor_count; i++)
        check += sensor_reading[i];
    return check;
}
void reverse()
{
    setSpeed();
    for (int i = 0; i < 4; i++)
        digitalWrite(dir_pin[i], (i % 2 == 0) ? HIGH : LOW);
}

void forward()
{
    for (int i = 0; i < 4; i++)
        digitalWrite(dir_pin[i], (i % 2 == 0) ? LOW : HIGH);
}

void right()
{
    setSpeed();
    digitalWrite(dir_pin[0], HIGH);
    digitalWrite(dir_pin[1], LOW);
    digitalWrite(dir_pin[2], LOW);
    digitalWrite(dir_pin[3], HIGH);
}

void left()
{
    setSpeed();
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
