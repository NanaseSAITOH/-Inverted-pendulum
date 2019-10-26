#include <Wire.h>
#include "KalmanFilter.h"

// MPU-6050のアドレス、レジスタ設定値
#define MPU6050_WHO_AM_I     0x75  // Read Only
#define MPU6050_PWR_MGMT_1   0x6B  // Read and Write
#define MPU_ADDRESS  0x68

#define GYR_GAIN 20.0
#define SAMPLE_TIME 10.0
#define STOP_ANGLE 10

KalmanFilter kalmanX(0.001, 0.003, 0.03);

unsigned long preTime = 0;
unsigned long lastTime;

float angleFiltered, angleFilteredOffset;
float input, output;
float pEffect, iEffect, dEffect, lastpEffect;
float motorPWM;
int timeChange;

// motor pin
int MOTOR1_PWM = 6;
int MOTOR1_IN1 = 4;
int MOTOR1_IN2 = 5;

int MOTOR2_PWM = 9;
int MOTOR2_IN1 = 7;
int MOTOR2_IN2 = 8;
//ジャイロセンサのPIN

bool motor_stop = false;
int16_t ax, ay, az, gx, gy, gz, Temperature;
// デバイス初期化時に実行される
void setup() {
  Wire.begin();

  // PCとの通信を開始
  Serial.begin(115200); //115200bps

  // 初回の読み出し
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(MPU6050_WHO_AM_I);  //MPU6050_PWR_MGMT_1
  Wire.write(0x00);
  Wire.endTransmission();

  // 動作モードの読み出し
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(MPU6050_PWR_MGMT_1);  //MPU6050_PWR_MGMT_1レジスタの設定
  Wire.write(0x00);
  Wire.endTransmission();

  // ピンの初期化
  pinMode(MOTOR1_IN1, OUTPUT);
  pinMode(MOTOR1_IN2, OUTPUT);
  pinMode(MOTOR2_IN1, OUTPUT);
  pinMode(MOTOR2_IN2, OUTPUT);


  // 初期位置のチェック
  for (int i = 0; i < 1000; i++) {
    kalmanFilter();
    angleFilteredOffset = angleFiltered;
  }

  // 最初のPID処理
  //abs():絶対値
  if (abs(angleFiltered) < STOP_ANGLE)    {
    angleFiltered = 0;
    kalmanFilter();
    output = pEffect = iEffect = dEffect = 0;
    pidPorcess();
  }
}

void filter() {
  getjairo();
  float angleX = (atan2(ay, az) * 180 / M_PI);
  float omega =  gx / GYR_GAIN;
  unsigned long now = millis();
  float dt = (now - preTime) / 1000.00;
  preTime = now;
  float K = 0.8;
  float A = K / (K + dt);
  angleFiltered = A * (angleFiltered + omega * dt) + (1 - A) * angleX;
}

void kalmanFilter() {
  getjairo();

  float angleX = (atan2(ay, az) * 180 / M_PI);
  //よくわからんけど計測した角度??
  angleFiltered = kalmanX.update(angleX, gx / GYR_GAIN);
}

// PID処理
void pidPorcess() {
  float kp = 5;
  //float kd = 150;
  float kd = 0;
  //float ki = 0.1;
  float ki = 0;

  // 計測時間チェック
  unsigned long now = millis();
  //dt
  timeChange = (now - lastTime);

  // PID処理
  if (timeChange >= SAMPLE_TIME) {
    //inputはpEffectと同値
    input = angleFiltered - angleFilteredOffset;
    pEffect = input;
    //iEffect += pEffect * timeChange;
    //dEffect = (pEffect - lastpEffect) / timeChange;
    output = kp * pEffect + ki * iEffect + kd * dEffect;

    //PIDした結果のトルクT
    motorPWM = output ;

    lastpEffect = pEffect;
    lastTime = now;
  }
}

void getjairo() {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 14, true);
  while (Wire.available() < 14);

  ax = Wire.read() << 8 | Wire.read();
  ay = Wire.read() << 8 | Wire.read();
  az = Wire.read() << 8 | Wire.read();
  Temperature = Wire.read() << 8 | Wire.read();
  gx = Wire.read() << 8 | Wire.read();
  gy = Wire.read() << 8 | Wire.read();
  gz = Wire.read() << 8 | Wire.read();
}

void loop() {
  /*-----------------------------------------------------------------------------------------------*/
  Serial.print("OK");
  // フィルタ処理
  kalmanFilter();
  // 転倒チェック.転倒していなかった場合
  if (abs(angleFiltered) < STOP_ANGLE && !motor_stop) {
    //pid
    Serial.print("pid");
    Serial.print(abs(angleFiltered));
    pidPorcess();
    Serial.print(motorPWM); Serial.print("\t");
    Serial.print(angleFiltered);
    Serial.print("\n");
    //転倒していた場合.
  } else if (abs(angleFiltered) >= STOP_ANGLE) {
    motor_stop = true;
  }

  // 転倒時はモーター停止
  if (motor_stop) {
    digitalWrite(MOTOR1_IN1, LOW);
    digitalWrite(MOTOR1_IN2, LOW);
    digitalWrite(MOTOR2_IN1, LOW);
    digitalWrite(MOTOR2_IN2, LOW);
  }
  //モーターが順回転の時
  else if (motorPWM > 0) {
    digitalWrite(MOTOR1_IN1, LOW);
    digitalWrite(MOTOR1_IN2, HIGH);
    digitalWrite(MOTOR2_IN1, LOW);
    digitalWrite(MOTOR2_IN2, HIGH);
  }//モーターが逆回転の時
  else if (motorPWM < 0) {
    digitalWrite(MOTOR1_IN1, HIGH);
    digitalWrite(MOTOR1_IN2, LOW);
    digitalWrite(MOTOR2_IN1, HIGH);
    digitalWrite(MOTOR2_IN2, LOW);
  }
  Serial.print("motorPWM=");
  Serial.print(motorPWM);
  analogWrite(MOTOR1_PWM, min(128, abs(motorPWM)));
  analogWrite(MOTOR2_PWM, min(128, abs(motorPWM)));
}
