#include <Servo.h>          //Servo motor library. This is standard library //Robot Lk
#include <NewPing.h>        //Ultrasonic sensor function library. You must install this library //Robot Lk

const int LeftMotorForward = 5;
const int LeftMotorBackward = 4;
const int RightMotorForward = 2;
const int RightMotorBackward = 3;
const int enA = 0;
const int enB = 1;
//LED Robot Lk
/*const int ForwardLED = 8;
const int BackwardLED = 9;
const int LeftLED = 11;
const int RightLED = 12;
const int LeftSensorLED = 13;
const int RightSensorLED = 3;*/
//sensor pins
#define trig_pin A1 //analog input 1
#define echo_pin A2 //analog input 2


NewPing sonar(trig_pin, echo_pin); //sensor function
Servo servo_motor; //our servo name

//Robot Lk
const int maxSpeed = 255; // Максимальная скорость
const int halfSpeed = 50; // Половина максимальной скорости

void setup() {
  // Настройка пинов
  pinMode(trig_pin, OUTPUT);
  pinMode(echo_pin, INPUT);
  pinMode(enA, OUTPUT);
  pinMode(LeftMotorForward, OUTPUT);
  pinMode(LeftMotorBackward, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(RightMotorForward, OUTPUT);
  pinMode(RightMotorBackward, OUTPUT);
  
  // Инициализация серийного порта для отладки
  Serial.begin(9600);
}

void loop() {
  // Измерение расстояния до препятствия
  long duration, distance;
  digitalWrite(trig_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_pin, LOW);
  duration = pulseIn(echo_pin, HIGH);
  distance = (duration / 2) / 29.1; // Перевод в сантиметры

  Serial.print("Distance: ");
  Serial.println(distance);

  // Управление скоростью в зависимости от расстояния до препятствия
  if (distance > 100) {
    // Едем с максимальной скоростью
    analogWrite(enA, maxSpeed);
    analogWrite(enB, maxSpeed);
    digitalWrite(LeftMotorForward, HIGH);
    digitalWrite(LeftMotorBackward, LOW);
    digitalWrite(RightMotorForward, HIGH);
    digitalWrite(RightMotorBackward, LOW);
  } else if (distance > 20 && distance < 100) {
    // Едем с половинной скоростью
    analogWrite(enA, halfSpeed);
    analogWrite(enB, halfSpeed);
    digitalWrite(LeftMotorForward, HIGH);
    digitalWrite(LeftMotorBackward, LOW);
    digitalWrite(RightMotorForward, HIGH);
    digitalWrite(RightMotorBackward, LOW);
  } else {
    // Останавливаемся
    analogWrite(enA, 0);
    analogWrite(enB, 0);
     digitalWrite(LeftMotorForward, LOW);
    digitalWrite(LeftMotorBackward, LOW);
    digitalWrite(RightMotorForward, LOW);
    digitalWrite(RightMotorBackward, LOW);
  }

  // Небольшая задержка перед следующим измерением
  delay(100);
}
