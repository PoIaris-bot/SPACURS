const int LEFT_MOTOR_PIN = 11;
const int RIGHT_MOTOR_PIN = 6;
const int LEFT_DIR_PIN = 5;
const int RIGHT_DIR_PIN = 3;

const int LEFT_DIR = 255;
const int RIGHT_DIR = 0;

int left_output = 0;
int right_output = 0;

String command;

void setup() {
  Serial.begin(115200);

  pinMode(LEFT_MOTOR_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN, OUTPUT);
  pinMode(LEFT_DIR_PIN, OUTPUT);
  pinMode(RIGHT_DIR_PIN, OUTPUT);

  analogWrite(LEFT_MOTOR_PIN, left_output);
  analogWrite(RIGHT_MOTOR_PIN, right_output);
  analogWrite(LEFT_DIR_PIN, LEFT_DIR);
  analogWrite(RIGHT_DIR_PIN, RIGHT_DIR);
}

void loop() {
  while (Serial.available() > 0) {
    char temp = char(Serial.read());
    if ((temp >= '0' && temp <= '9') || (temp >= 'a' && temp <= 'z')) {
      command += temp;
    }
    delay(2);
  }
  Serial.flush();

  if (command.length() > 0) {
    int index = command.indexOf("cmd");
    if (index != -1) {
      left_output = atoi(command.substring(index + 3, index + 5).c_str());
      right_output = atoi(command.substring(index + 5, index + 7).c_str());
    }
    command = "";
  }
  analogWrite(LEFT_MOTOR_PIN, left_output);
  analogWrite(RIGHT_MOTOR_PIN, right_output);
  delay(2);
}