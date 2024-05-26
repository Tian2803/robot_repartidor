#include <AFMotor.h>
#include <NewPing.h>

#define der A0
#define izq A1

#define TRIG_PIN A4
#define ECHO_PIN A5
#define distMax 200
int distancia = 0;
int contador = 0;
bool obstaculo_detectado = false;

AF_DCMotor motorL(2, MOTOR12_1KHZ);  // Motor izquierdo
AF_DCMotor motorD(4, MOTOR34_1KHZ);  // Motor derecho
NewPing Dist(TRIG_PIN, ECHO_PIN, distMax);

int pwm = 100;    // Velocidad base de los motores
double Kp = 1.0;  // Constante proporcional
double Ki = 0.0;  // Constante integral
double Kd = 0.0;  // Constante derivativa

double integral = 0.0;
double lastError = 0.0;

void setup() {
  Serial.begin(9600);
  pinMode(izq, INPUT);
  pinMode(der, INPUT);
}

void loop() {
  int left = digitalRead(izq);
  int right = digitalRead(der);
  distancia = medirDistancia();

  // Si la distancia es menor o igual a 25 cm y no se ha detectado un obstáculo antes, incrementa el contador y marca que se ha detectado un obstáculo
  if (distancia <= 25 && !obstaculo_detectado) {
    obstaculo_detectado = true;
    contador++;
    if (contador == 2) {
      stop();
    }
  }

  // Si la distancia es mayor a 25 cm y se había detectado un obstáculo anteriormente, restablece el estado del obstáculo
  if (distancia > 25 && obstaculo_detectado) {
    obstaculo_detectado = false;
  }

  Serial.print("Distancia: ");
  Serial.println(distancia);
  Serial.print("Contador: ");
  Serial.println(contador);


  Serial.print("Izquierda: ");
  Serial.println(left);
  Serial.print("Derecha: ");
  Serial.println(right);

  delay(2000);


  // Calcular error (diferencia entre las lecturas del sensor derecho e izquierdo)
  double error = right - left;

  // Calcular la señal de control usando PID
  double correction = Kp * error + Ki * integral + Kd * (error - lastError);

  // Actualizar término integral
  integral += error;

  // Almacenar el error actual para la próxima iteración
  lastError = error;

  // Ajustar velocidades del motor en base a la corrección
  int leftSpeed = pwm + correction;
  int rightSpeed = pwm - correction;

  Serial.println(leftSpeed);
  Serial.println(rightSpeed);

  // Asegurar que las velocidades del motor estén dentro del rango válido
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  // Mover el robot en base a las lecturas del sensor y la corrección PID
  if (left == 0 && right == 0) {
    forward(leftSpeed, rightSpeed);
  } else if (left == 0 && right == 1) {
    turnRight(leftSpeed, rightSpeed);
  } else if (left == 1 && right == 0) {
    turnLeft(leftSpeed, rightSpeed);
  } else {
    stop();
  }
}

void forward(int leftSpeed, int rightSpeed) {
  motorL.run(FORWARD);
  motorL.setSpeed(leftSpeed);
  motorD.run(FORWARD);
  motorD.setSpeed(rightSpeed);
}

void turnLeft(int leftSpeed, int rightSpeed) {
  motorL.run(BACKWARD);
  motorL.setSpeed(leftSpeed);
  motorD.run(FORWARD);
  motorD.setSpeed(rightSpeed);
}

void turnRight(int leftSpeed, int rightSpeed) {
  motorL.run(FORWARD);
  motorL.setSpeed(leftSpeed);
  motorD.run(BACKWARD);
  motorD.setSpeed(rightSpeed);
}

void stop() {
  motorL.run(RELEASE);
  motorL.setSpeed(0);
  motorD.run(RELEASE);
  motorD.setSpeed(0);
}

int medirDistancia() {
  delay(10);
  int distanciaCM = Dist.ping_cm();
  if (distanciaCM <= 0 || distanciaCM >= 250) {
    distanciaCM = 250;
  }
  return distanciaCM;
}
