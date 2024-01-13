void front(int power) {
  digitalWrite(4, LOW);
  analogWrite(11, power);
  digitalWrite(5, HIGH);
  analogWrite(10, power);
  digitalWrite(6, HIGH);
  analogWrite(9, power);
  digitalWrite(7, LOW);
  analogWrite(3, power);
}

void rightfront(int power) {
  digitalWrite(4, LOW);
  analogWrite(11, power);
  digitalWrite(5, HIGH);
  analogWrite(10, 0);
  digitalWrite(6, HIGH);
  analogWrite(9, 0);
  digitalWrite(7, LOW);
  analogWrite(3, power);
}

void leftfront(int power) {
  digitalWrite(4, HIGH);
  analogWrite(11, 0);
  digitalWrite(5, HIGH);
  analogWrite(10, power);
  digitalWrite(6, HIGH);
  analogWrite(9, power);
  digitalWrite(7, HIGH);
  analogWrite(3, 0);
}

void right(int power) {
  digitalWrite(4, LOW);
  analogWrite(11, power);
  digitalWrite(5, LOW);
  analogWrite(10, power);
  digitalWrite(6, LOW);
  analogWrite(9, power);
  digitalWrite(7, LOW);
  analogWrite(3, power);
}

void left(int power) {
  digitalWrite(4, HIGH);
  analogWrite(11, power);
  digitalWrite(4, HIGH);
  analogWrite(10, power);
  digitalWrite(6, HIGH);
  analogWrite(9, power);
  digitalWrite(7, HIGH);
  analogWrite(3, power);
}

void rightback(int power) {
  digitalWrite(4, HIGH);
  analogWrite(11, 0);
  digitalWrite(5, LOW);
  analogWrite(10, power);
  digitalWrite(6, LOW);
  analogWrite(9, power);
  digitalWrite(7, HIGH);
  analogWrite(3, 0);
}

void leftback(int power) {
  digitalWrite(4, HIGH);
  analogWrite(11, power);
  digitalWrite(5, HIGH);
  analogWrite(10, 0);
  digitalWrite(6, HIGH);
  analogWrite(9, 0);
  digitalWrite(7, HIGH);
  analogWrite(3, power);
}

void back(int power) {
  digitalWrite(4, HIGH);
  analogWrite(11, power);
  digitalWrite(5, LOW);
  analogWrite(10, power);
  digitalWrite(6, LOW);
  analogWrite(9, power);
  digitalWrite(7, HIGH);
  analogWrite(3, power);
}

void rightturn(int power) {
  digitalWrite(4, LOW);
  analogWrite(11, power);
  digitalWrite(5, HIGH);
  analogWrite(10, power);
  digitalWrite(6, LOW);
  analogWrite(9, power);
  digitalWrite(7, HIGH);
  analogWrite(3, power);
}

void leftturn(int power) {
  digitalWrite(4, HIGH);
  analogWrite(11, power);
  digitalWrite(5, LOW);
  analogWrite(10, power);
  digitalWrite(6, HIGH);
  analogWrite(9, power);
  digitalWrite(7, LOW);
  analogWrite(3, power);
}
#include <SPI.h>
#include<Wire.h>
#include <Adafruit_BNO055.h>
#include <MPU6050_6Axis_MotionApps20.h>
#define Gyro_X -315
#define Gyro_Y -93
#define Gyro_Z 5
#define Accel_Z 4668

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

int getVal(int f) {
  byte val = 0;
  Wire.beginTransmission(0x0E);
  Wire.write(f);
  Wire.endTransmission();
  Wire.requestFrom(0x0E, 1);
  while (Wire.available()) {
    val = Wire.read();
  }
  return (int)val;
}

int IRval(int i) {
  int a = getVal(0x04);
  int b = getVal(0x05);
  int c = getVal(0x06);
  int d = getVal(0x07);
  int re_angle;
  int re_strength;

  if (d < 10) {
    re_angle = a;
    re_strength = b;
  }
  else {
    re_angle = c;
    re_strength = d;
  }

  if (i != 1) {
    return re_strength;
  }
  else {
    return re_angle * 5;
  }
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  GyroInit();
  pinMode(7, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(9, OUTPUT);
}

void GyroInit() {
  bno.begin();
  bno.setExtCrystalUse(false);
  delay(500);
}

int GyroGet() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  return euler.x();
}

void loop() {
  int power = 200;
  int ball = IRval(1);
  int linefront = analogRead(A6);
  int lineright = analogRead(A7);
  int lineleft = analogRead(A2);
  int lineback = analogRead(A3);
  GyroGet();
  Serial.print("Ball_");
  Serial.print(ball);
  Serial.print("-Gyro_");
  Serial.print(GyroGet());
  Serial.print("-front_");
  Serial.print(linefront);
  Serial.print("-right_");
  Serial.print(lineright);
  Serial.print("-left_");
  Serial.print(lineleft);
  Serial.print("-back_");
  Serial.println(lineback);

  if (GyroGet() > 20 && GyroGet() < 180) {
    leftturn(100);

  }
  else if (GyroGet() > 180 && GyroGet() < 340) {
    rightturn(100);
  }
  else  {
    if (linefront > 130) {
      back(power);
      delay(800);
    }
    else if (lineright > 130) {
      left(power);
      delay(600);
    }
    else if (lineleft > 130) {
      right(power);
      delay(600);
    }
    else if (lineback > 130) {
      front(power);
      delay(600);
    }
    else  if (ball < 15 || ball > 345) {
      front(power);
    }
    else if (ball <= 45 && ball > 15) {
      right(power);
    }
    else if (ball <= 85 && ball > 95) {
      rightback(power);
    }
    else if (ball < 265 && ball >= 95) {
      back(power);
    }
    else if (ball < 275 && ball >= 265 ) {
      leftback(power);
    }
    else if(ball < 345 && ball > 265) {
      left(power);
    }
  }
}