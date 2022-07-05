#include <Servo.h> //Biblioteca para el Servo motor (Esta es una Biblioteca estándar)
#include <NewPing.h> //Biblioteca para el Sensor Ultrasónico (Puedes instalarla en ->Herramientas-> Administrar Bibliotecas-> Tema:NewPing ->Instalar)

//Nombramos los pines de control del módulo L298N

const int MotorDer_Frente = 2;
const int MotorDer_Reversa = 3;
const int MotorIzq_Frente = 4;
const int MotorIzq_Reversa = 7;
const int VelMotor_Izq = 5; //Pines PWM
const int VelMotor_Der = 6; //Pines PWM
const int pwm = 140;

//Pines del sensor ultrasónico
#define trig_pin A1 //Entrada analógica 1
#define echo_pin A2 //Entrada analógica 2

#define distancia_maxima 200
boolean Avanza = false;
int distancia = 100;

NewPing sonar(trig_pin, echo_pin, distancia_maxima); //Funcion del sensor ultrasonico
Servo servo_motor; //Nombramos nuestro motor

void setup() {

  pinMode(MotorDer_Frente, OUTPUT);
  pinMode(MotorDer_Reversa, OUTPUT);
  pinMode(MotorIzq_Frente, OUTPUT);
  pinMode(MotorIzq_Reversa, OUTPUT);

  servo_motor.attach(11); //Pin del servo

  servo_motor.write(90);
  delay(2000);
  distancia = leer_Ping();
  delay(100);
  distancia = leer_Ping();
  delay(100);
  distancia = leer_Ping();
  delay(100);
  distancia = leer_Ping();
  delay(100);
}

/*
  Lógica principal. Se ejecuta una y otra vez.
  Se define la función que ejecutara repetidamente
*/
void loop() {

  int distancia_Derecha = 0;
  int distancia_Izquierda = 0;
  delay(50);

  if (distancia <= 40) {   //distancia de detección de obstaculo (aumenta si quieres que los detecte desde más lejos)
    mover_Alto();
    delay(300);
    mover_Reversa();
    delay(400);
    mover_Alto();
    delay(300);
    distancia_Derecha = mirar_Derecha();
    delay(300);
    distancia_Izquierda = mirar_Izquierda();
    delay(300);

    if (distancia >= distancia_Izquierda) {
      girar_Derecha();
      mover_Alto();
    }
    else {
      girar_Izquierda();
      mover_Alto();
    }
  }
  else {
    mover_Frente();
  }
  distancia = leer_Ping();
}

int mirar_Derecha() {
  servo_motor.write(10);
  delay(500);
  int distancia = leer_Ping();
  delay(100);
  servo_motor.write(90);
  return distancia;
}

int mirar_Izquierda() {
  servo_motor.write(170);
  delay(500);
  int distancia = leer_Ping();
  delay(100);
  servo_motor.write(90);
  return distancia;
  delay(100);
}

int leer_Ping() {
  delay(70);
  int cm = sonar.ping_cm();
  if (cm == 0) {
    cm = 250;
  }
  return cm;
}

void mover_Alto() {
  digitalWrite(MotorDer_Frente, LOW);
  digitalWrite(MotorIzq_Frente, LOW);
  digitalWrite(MotorDer_Reversa, LOW);
  digitalWrite(MotorIzq_Reversa, LOW);
}
//Hay que recordar que los pines Frente y Reversa del mismo motor deben estar en estados opuestos (uno HIGH y uno LOW, para que el motor gire)

void mover_Frente() {
  if (!Avanza) {
    Avanza = true;
    digitalWrite(MotorIzq_Frente, HIGH);
    digitalWrite(MotorDer_Frente, HIGH);
    digitalWrite(MotorIzq_Reversa, LOW);
    digitalWrite(MotorDer_Reversa, LOW);
    analogWrite(VelMotor_Izq, pwm);
    analogWrite(VelMotor_Der, pwm);
  }
}

void mover_Reversa() {
  Avanza = false;
  digitalWrite(MotorIzq_Reversa, HIGH);
  digitalWrite(MotorDer_Reversa, HIGH);
  digitalWrite(MotorIzq_Frente, LOW);
  digitalWrite(MotorDer_Frente, LOW);
  analogWrite(VelMotor_Izq, pwm);
  analogWrite(VelMotor_Der, pwm);
}

void girar_Derecha() {
  digitalWrite(MotorIzq_Frente, HIGH);
  digitalWrite(MotorDer_Reversa, HIGH);
  digitalWrite(MotorIzq_Reversa, LOW);
  digitalWrite(MotorDer_Frente, LOW);
  analogWrite(VelMotor_Izq, pwm);
  analogWrite(VelMotor_Der, pwm);
  delay(250);
  digitalWrite(MotorIzq_Frente, HIGH);
  digitalWrite(MotorDer_Frente, HIGH);
  digitalWrite(MotorIzq_Reversa, LOW);
  digitalWrite(MotorDer_Reversa, LOW);
  analogWrite(VelMotor_Izq, pwm);
  analogWrite(VelMotor_Der, pwm);
}

void girar_Izquierda() {
  digitalWrite(MotorIzq_Reversa, HIGH);
  digitalWrite(MotorDer_Frente, HIGH);
  digitalWrite(MotorIzq_Frente, LOW);
  digitalWrite(MotorDer_Reversa, LOW);
  analogWrite(VelMotor_Izq, pwm);
  analogWrite(VelMotor_Der, pwm);

  delay(250);
  digitalWrite(MotorIzq_Frente, HIGH);
  digitalWrite(MotorDer_Frente, HIGH);
  digitalWrite(MotorIzq_Reversa, LOW);
  digitalWrite(MotorDer_Reversa, LOW);
  analogWrite(VelMotor_Izq, pwm);
  analogWrite(VelMotor_Der, pwm);
}
