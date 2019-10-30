#include <Wire.h>
#include "KalmanFilter.h"
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu6050;
MPU6050 initialize;

#define GYR_GAIN 20.0
#define SAMPLE_TIME 10.0
#define STOP_ANGLE 30

KalmanFilter kalmanX(0.001, 0.003, 0.03);

unsigned long preTime = 0;
unsigned long lastTime;

float angleFiltered, angleFilteredOffset;
float input, output;
float pEffect, iEffect, dEffect, lastpEffect;
float motorPWM;
int timeChange;

// motor pin
int MOTOR1_PWM = 10;
int MOTOR1_IN1 = 9;
int MOTOR1_IN2 = 8;

int MOTOR2_PWM = 6;
int MOTOR2_IN1 = 4;
int MOTOR2_IN2 = 5;
//ジャイロセンサのPIN
int STBY_PIN = 13;

bool motor_stop = false;
// デバイス初期化時に実行される
void setup() {

  // ピンの初期化
  pinMode(MOTOR1_IN1, OUTPUT);
  pinMode(MOTOR1_IN2, OUTPUT);
  pinMode(MOTOR2_IN1, OUTPUT);
  pinMode(MOTOR2_IN2, OUTPUT);
  digitalWrite(STBY_PIN, HIGH);

  Wire.begin();
  // PCとの通信を開始
  Serial.begin(115200); //115200bps
  mpu6050.initialize();
  initialize.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);;

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
  int16_t ax, ay, az, gx, gy, gz;
  mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
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
  int16_t ax, ay, az, gx, gy, gz;
  Serial.println("end1");
  mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  Serial.println("end2");
  float angleX = (atan2(ay, az) * 180 / M_PI);
  //よくわからんけど計測した角度??
  angleFiltered = kalmanX.update(angleX, gx / GYR_GAIN);
}

// PID処理
void pidPorcess() {
  float kp = 250;
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
    iEffect += pEffect * timeChange;
    dEffect = (pEffect - lastpEffect) / timeChange;
    output = kp * pEffect + ki * iEffect + kd * dEffect;

    //PIDした結果のトルクT
    motorPWM = output ;

    lastpEffect = pEffect;
    lastTime = now;
  }
}


void loop() {
  /*-----------------------------------------------------------------------------------------------*/
  //Serial.print("OK");
  // フィルタ処理
  kalmanFilter();
  // 転倒チェック.転倒していなかった場合
  if (abs(angleFiltered) < STOP_ANGLE && !motor_stop) {
    pidPorcess();
  } else if (abs(angleFiltered) >= STOP_ANGLE) {
    motor_stop = true;
  }

  // 転倒時はモーター停止
  if (motor_stop) {
    Serial.println("end");
    digitalWrite(MOTOR1_IN1, LOW);
    digitalWrite(MOTOR1_IN2, LOW);
    digitalWrite(MOTOR2_IN1, LOW);
    digitalWrite(MOTOR2_IN2, LOW);
  }
  //モーターが順回転の時
  else if (motorPWM > 0) {
    
    Serial.println("left");
    digitalWrite(MOTOR1_IN1, LOW);
    digitalWrite(MOTOR1_IN2, HIGH);
    digitalWrite(MOTOR2_IN1, LOW);
    digitalWrite(MOTOR2_IN2, HIGH);
  }//モーターが逆回転の時
  else if (motorPWM < 0) {
    Serial.print("right");
    digitalWrite(MOTOR1_IN1, HIGH);
    digitalWrite(MOTOR1_IN2, LOW);
    digitalWrite(MOTOR2_IN1, HIGH);
    digitalWrite(MOTOR2_IN2, LOW);
  }
  //Serial.print("motorPWM=");
  //Serial.print(motorPWM);
  analogWrite(MOTOR1_PWM, (abs(motorPWM)));
  analogWrite(MOTOR2_PWM, (abs(motorPWM)));
}
