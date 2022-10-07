#include <Arduino.h>
#include <MPU9250.h>

#define ADDR 0x68

int dir_pin[4] = {8, 9, 10, 12},
    pwm_pin[2] = {6, 11};

float m_ax = 0, m_ay = 0, yaw = 0;
float ux = 0, vx = 0, uy = 0, vy = 0, ax = 0, ay = 0, dx = 0, dy = 0;

// calculate distance S = ut + 1/2at^2
// calculate velocity from v = u + at

byte SPEED = 255;

MPU9250 mpu;
MPU9250Setting setting;

long prev = millis();
float time_step = 0.01;

void setup()
{

    Serial.begin(250000);
    Wire.begin();

    setting.accel_fs_sel = ACCEL_FS_SEL::A2G;
    setting.gyro_fs_sel = GYRO_FS_SEL::G250DPS;

    while (!mpu.setup(ADDR, setting))
        ;

    Serial.println("Press any key to enter calibration");
    while (!Serial.available())
        ;
    mpu.calibrateAccelGyro();
    mpu.calibrateMag();

    for (int i = 0; i < 4; i++)
    {
        pinMode(dir_pin[i], OUTPUT);
        digitalWrite(dir_pin[i], LOW);
    }

    for (int i = 0; i < 2; i++)
    {
        pinMode(pwm_pin[i], OUTPUT);
        analogWrite(pwm_pin[i], SPEED);
    }
}

void loop()
{

    prev = millis();
    mpu.update();
    yaw = mpu.getYaw();
    m_ax = mpu.getLinearAccX();
    m_ay = mpu.getLinearAccY();

    if (Serial.available() > 0)
    {
        char data = (char)Serial.read();
        Serial.println(data);
        switch (data)
        {
        case 'F':
            forward();
            break;
        case 'B':
            reverse();
            break;
        case 'L':
            left();
            break;
        case 'R':
            right();
            break;
        // case 'G':
        //     forwardLeft();
        //     break;
        // case 'I':
        //     forwardRight();
        //     break;
        // case 'H':
        //     reverseLeft();
        //     break;
        // case 'J':
        //     reverseRight();
        //     break;
        case 'S':
            stop();
            break;
        case '1':
            SPEED = 26;
            break;
        case '2':
            SPEED = 52;
            break;
        case '3':
            SPEED = 78;
            break;
        case '4':
            SPEED = 102;
            break;
        case '5':
            SPEED = 149;
            break;
        case '6':
            SPEED = 180;
            break;
        case '7':
            SPEED = 190;
            break;
        case '8':
            SPEED = 210;
            break;
        case '9':
            SPEED = 230;
            break;
        case '10':
            SPEED = 255;
            break;
        }

        for (int i : pwm_pin)
            analogWrite(i, SPEED);
    }
    calculateVel();
    calculateDist();
    String data = "dist x : " + String(dx) + " dist y : " + String(dy) + "yaw :" + String(yaw) + " accx : " + String(m_ax);
    Serial.println(data);
    while (millis() - prev < time_step * 1000)
        ;
}

void calculateVel()
{
    float yaw_rad = yaw * DEG_TO_RAD;
    ax = m_ax * cos(yaw_rad) - m_ay * sin(yaw_rad);
    ay = m_ay * cos(yaw_rad) + m_ax * sin(yaw_rad);
    vx = ux + ax * time_step;
    vy = uy + ay * time_step;
    ux = vx;
    uy = vy;
}

void calculateDist()
{
    dx += (vx * time_step + 0.5 * ax * time_step * time_step) * 100;
    dy += (vy * time_step + 0.5 * ay * time_step * time_step) * 100;
}

void forward()
{
    for (int i = 0; i < 4; i++)
        digitalWrite(dir_pin[i], (i % 2 == 0) ? HIGH : LOW);
}

void reverse()
{
    for (int i = 0; i < 4; i++)
        digitalWrite(dir_pin[i], (i % 2 == 0) ? LOW : HIGH);
}

void left()
{
    digitalWrite(dir_pin[0], HIGH);
    digitalWrite(dir_pin[1], LOW);
    digitalWrite(dir_pin[2], LOW);
    digitalWrite(dir_pin[3], HIGH);
}

void right()
{
    digitalWrite(dir_pin[0], LOW);
    digitalWrite(dir_pin[1], HIGH);
    digitalWrite(dir_pin[2], HIGH);
    digitalWrite(dir_pin[3], LOW);
}

void forwardRight()
{
}

void stop()
{
    for (int i = 0; i < 4; i++)
        digitalWrite(dir_pin[i], LOW);
}