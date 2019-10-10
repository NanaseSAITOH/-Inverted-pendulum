# Inverted-Pendulum

## 注意点

MPU 6050を使わないと該当プログラムは動かない.

## モータPINについて

int MOTOR1_PWM = 9;
int MOTOR1_IN1 = 8;
int MOTOR1_IN2 = 7;

|MOTOR_IN1|MOTOR_IN2|モータの回転|
|---------|---------|----------|
|LOW------|HIGH-----|反時計回り--|
|HIGH-----|LOW------|時計回り---|
|LOW------|LOW------|停止-------|
