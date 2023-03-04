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

String command;

void setup() {
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

void loop() {
  while (Serial.available() > 0) {
    char temp = char(Serial.read());
    if ((temp >= '0' && temp <= '9') || (temp >= 'a' && temp <= 'z')) {
      command += temp;
    }
    delay(2);
  }
  Serial.flush();  // 串口发送数据结束后清缓存

  if (command.length() > 0) {
    int idx = command.indexOf("cmd");
    if (idx != -1) {
      int mode = atoi(command.substring(idx + 3, idx + 4).c_str());
      left_speed = atoi(command.substring(idx + 4, idx + 7).c_str());
      right_speed = atoi(command.substring(idx + 7, idx + 10).c_str());

      left_dir = LEFT_FORWARD;
      right_dir = RIGHT_FORWARD;
      switch (mode) {
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
    }
    command = "";
  }
  analogWrite(left_motor_pin, left_speed);
  analogWrite(right_motor_pin, right_speed);
  analogWrite(left_dir_pin, left_dir);
  analogWrite(right_dir_pin, right_dir);
  delay(2);
}