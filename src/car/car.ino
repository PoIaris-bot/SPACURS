#include <stdlib.h>
#include <Servo.h>

const int left_motor_pin = 11;
const int right_motor_pin = 6;
const int left_dir_pin = 5;
const int right_dir_pin = 3;
const int servo_pin = 9;

const int left_dir = 0;
const int right_dir = 255;

Servo servo;
String com_data;

int speed = 0;
float angle = 95;

void setup() {
    Serial.begin(115200);

    pinMode(left_motor_pin, OUTPUT);
    pinMode(right_motor_pin, OUTPUT);
    pinMode(left_dir_pin, OUTPUT);
    pinMode(right_dir_pin, OUTPUT);

    analogWrite(left_dir_pin, left_dir);
    analogWrite(right_dir_pin, right_dir);

    servo.attach(servo_pin);
    servo.write(angle);
}

void loop() {
    while (Serial.available() > 0) {
        char temp = char(Serial.read());
        if (temp >= '0' && temp <= '9') {
            com_data += temp;
        }
        delay(2);
    }
    Serial.flush(); // 串口发送数据结束后清缓存

    if (com_data.length() > 0) {
        Serial.println(com_data);
        long data = atol(com_data.c_str()) % 10000; // int能表示的范围不够，需要long
        Serial.println(data);
        speed = data / 1000;
        if (speed != 0) speed += 8;
        angle = data % 1000;
        com_data = "";
        Serial.println(speed);
        Serial.println(angle);
        Serial.print('\n');
    }
    servo.write(angle);
    analogWrite(left_motor_pin, speed);
    analogWrite(right_motor_pin, speed);
    delay(2);
}