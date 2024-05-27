#include <AFMotor.h>
#include <NewPing.h>

#define TRIGGER_PIN  A4 
#define ECHO_PIN     A5
#define MAX_DISTANCE 200 

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); 

#define IR_IZQUIERDO A3  // Sensor izquierdo
#define IR_DERECHO   A2  // Sensor derecho

AF_DCMotor motorI(2);  // Motor Izquierdo
AF_DCMotor motorD(1);  // Motor Derecho

float Kp = 3.3;  // Coeficiente Proporcional
float Ki = 0.0;  // Coeficiente Integral
float Kd = 1.8;  // Coeficiente Derivativo

float lastError = 0;
float integral = 0;

int contador = 0;
bool casa_detectada = false;

void setup() {
  Serial.begin(115200);
  pinMode(IR_IZQUIERDO, INPUT);
  pinMode(IR_DERECHO, INPUT);
}

void loop() {
  detectarCasa();
  if (contador == 2) {
    stop();
    contador = 0;
    delay(5000); // Pausa para observar el comportamiento antes de continuar
    return; 
  }

  int sensorIzquierdo = analogRead(IR_IZQUIERDO);
  int sensorDerecho = analogRead(IR_DERECHO);  

  float error = calculateError(sensorIzquierdo, sensorDerecho);
  float correction = getPIDCorrection(error);

  applyMotorCorrection(correction);
  delay(30);
}

void detectarCasa() {
  int distancia_casa = sonar.ping_cm();
  if (distancia_casa < 20 && !casa_detectada && distancia_casa != 0) {
    contador++;
    casa_detectada = true;
    Serial.println("Casa detectada");
  }
  if (distancia_casa > 20 && casa_detectada && distancia_casa != 0) {
    casa_detectada = false;
    Serial.println("Casa fuera de rango");
  }
}

float calculateError(int sensorIzquierdo, int sensorDerecho) {
  int threshold = 200;  // Ajuste el umbral para tu configuración específica
  float error = 0;
  if (sensorIzquierdo > threshold) error += -5;  // Sensor izquierdo
  if (sensorDerecho > threshold) error += 5;     // Sensor derecho
  return error;
}

float getPIDCorrection(float error) {
  float proportional = error;
  integral += error;
  float derivative = error - lastError;
  lastError = error;
  return Kp * proportional + Ki * integral + Kd * derivative;
}

void applyMotorCorrection(float correction) {
  int baseSpeed = 100;
  int maxSpeed = 130;

  int leftSpeed = baseSpeed - correction;
  int rightSpeed = baseSpeed + correction;

  leftSpeed = constrain(leftSpeed, 0, maxSpeed);
  rightSpeed = constrain(rightSpeed, 0, maxSpeed);

  if (correction < -5) {
    turnLeft(abs(leftSpeed), abs(rightSpeed));
  } else if (correction > 5) {
    turnRight(abs(leftSpeed), abs(rightSpeed));
  } else {
    forward(leftSpeed, rightSpeed);
  }
}

void forward(int leftSpeed, int rightSpeed) {
  motorI.run(FORWARD);
  motorI.setSpeed(leftSpeed);
  motorD.run(FORWARD);
  motorD.setSpeed(rightSpeed);
}

void turnLeft(int leftSpeed, int rightSpeed) {
  motorI.run(BACKWARD);
  motorI.setSpeed(leftSpeed);
  motorD.run(FORWARD);
  motorD.setSpeed(rightSpeed);
  delay(100);
}

void turnRight(int leftSpeed, int rightSpeed) {
  motorI.run(FORWARD);
  motorI.setSpeed(leftSpeed);
  motorD.run(BACKWARD);
  motorD.setSpeed(rightSpeed);
  delay(100);
}

void stop() {
  motorI.run(RELEASE);
  motorI.setSpeed(0);
  motorD.run(RELEASE);
  motorD.setSpeed(0);
}
