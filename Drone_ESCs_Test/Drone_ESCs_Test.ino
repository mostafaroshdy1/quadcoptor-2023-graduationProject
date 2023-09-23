#include <Servo.h>

Servo esc_front_left, esc_front_right, esc_rear_left, esc_rear_right;  // create servo object to control the ESC
int FR = 6;
int FL = 5;
int RR = 3;
int RL = 9;
int serialRead,escValue, escValue_FR = 1000, escValue_FL = 1000, escValue_RR = 1000, escValue_RL = 1000;
int minPW = 1000;
int maxPW = 2300;
char axis,axis_value;
void setup() {


  Serial.begin(9600);
  setupESCs();

}

void loop() {
  if (Serial.available() > 0) {
       axis = Serial.read();
        serialRead = Serial.parseInt();
        if (serialRead != 0&&axis !="")
          escValue = serialRead;
          axis_value=axis;
    }
  if (axis == 'q') {
    escValue_FL = escValue;
  }
  else if (axis == 'e') {
    escValue_FR = escValue;
  }
  else if (axis == 'a') {
    escValue_RL = escValue;
  }
  else if (axis == 'd') {
    escValue_RR = escValue;
  }
  else if (axis == 'w') {
    escValue_FL = escValue;
    escValue_FR = escValue;
    escValue_RL = escValue;
    escValue_RR = escValue;
  }
  else if (axis == 's') {
    escValue_FL = 0;
    escValue_FR = 0;
    escValue_RL = 0;
    escValue_RR = 0;
  }
  Serial.print("escValue_FL " + (String) escValue_FL);
  Serial.print(" escValue_FR " + (String) escValue_FR);
  Serial.print(" escValue_RL " + (String) escValue_RL);
  Serial.println(" escValue_RR " + (String) escValue_RR);
  updateESCs();
}

void setupESCs() {
  esc_front_right.attach(FR, minPW, maxPW);
  esc_front_left.attach(FL, minPW, maxPW);
  esc_rear_right.attach(RR, minPW, maxPW);
  esc_rear_left.attach(RL, minPW, maxPW);

  esc_front_right.writeMicroseconds(escValue_FR);
  esc_front_left.writeMicroseconds(escValue_FL);
  esc_rear_right.writeMicroseconds(escValue_RR);
  esc_rear_left.writeMicroseconds(escValue_RL);

  delay(2000);
}

void updateESCs() {

  esc_front_right.writeMicroseconds(escValue_FR);
  esc_front_left.writeMicroseconds(escValue_FL);
  esc_rear_right.writeMicroseconds(escValue_RR);
  esc_rear_left.writeMicroseconds(escValue_RL);

}
