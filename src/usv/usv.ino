#include <stdlib.h>

const int left_motor_pin = 11;
const int right_motor_pin = 6;
const int left_dir_pin = 5;
const int right_dir_pin = 3;

const int LEFT_FORWARD = 255;
const int LEFT_BACKWARD = 0;
const int RIGHT_FORWARD = 0;
const int RIGHT_BACKWARD = 255;

int left_speed = 0;
int right_speed = 0;
int left_dir = LEFT_FORWARD;
int right_dir = RIGHT_FORWARD;

String com_data;

void setup()
{
    Serial.begin(115200);

    pinMode(left_motor_pin, OUTPUT);
    pinMode(right_motor_pin, OUTPUT);
    pinMode(left_dir_pin, OUTPUT);
    pinMode(right_dir_pin, OUTPUT);

    analogWrite(left_motor_pin, left_speed);
    analogWrite(right_motor_pin, right_speed);
    analogWrite(left_dir_pin, left_dir);
    analogWrite(right_dir_pin, right_dir);
}

void loop()
{
    while (Serial.available() > 0)
    {
        char temp = char(Serial.read());
        if (temp >= '0' && temp <= '9')
        {
            com_data += temp;
        }
        delay(2);
    }
    Serial.flush(); // 串口发送数据结束后清缓存

    if (com_data.length() > 0)
    {
        long data = atol(com_data.c_str()) % 100000;

        int mode = data / 10000;
        left_dir = LEFT_FORWARD;
        right_dir = RIGHT_FORWARD;
        switch (mode)
        {
        case 1:  // 前进
            break;
        case 2:  // 右转
            right_dir = RIGHT_BACKWARD;
            break;
        case 3:  // 左转
            left_dir = LEFT_BACKWARD;
            break;
        case 4:  // 后退
            left_dir = LEFT_BACKWARD;
            right_dir = RIGHT_BACKWARD;
        }

        int speed = data % 10000;
        left_speed = speed / 100;
        right_speed = speed % 100;

        com_data = "";
    }
    analogWrite(left_motor_pin, left_speed);
    analogWrite(right_motor_pin, right_speed);
    analogWrite(left_dir_pin, left_dir);
    analogWrite(right_dir_pin, right_dir);
    delay(2);
}