#include <Arduino.h>
#include <EEPROM.h>

// ==========================================
//               CONFIGURACIÓN
// ==========================================
const int ledPin = 11;
const int buttonCalib = 9;
const int buttonStart = 10;
const int pinIR = 12;

// Pines Motores
const int pinPwmMotorIzq = 6;
const int pinPwmMotorDer = 5;
const int dirMotorIzq2 = 7;
const int dirMotorIzq1 = 8;
const int dirMotorDer2 = 3;
const int dirMotorDer1 = 2;
const int stbyMotor = 4;

// ==========================================
//           VARIABLES PID DIGITAL
// ==========================================
// AJUSTES
int32_t Kp = 50;
int32_t Ki = 0;
int32_t Kd = 300;

int32_t errorActual = 0;
int32_t errorAnterior = 0;
int32_t errorDerivativo = 0;
int32_t u = 0;

// Variables Sensores
uint16_t sensor[8];
uint16_t sensorMin[8];
uint16_t sensorMax[8];
uint16_t umbral[8];

int16_t pesos[8] = {-4000, -3000, -2000, -1000, 1000, 2000, 3000, 4000};
int32_t ultimoErrorConocido = 0;

bool calibrando = false;
bool iniciarRecorrido = false;

// ==========================================
//             FUNCIONES BASE
// ==========================================
void guardarCalibracion()
{
  for (int i = 0; i < 8; i++)
  {
    EEPROM.put(i * 4, sensorMin[i]);
    EEPROM.put(i * 4 + 2, sensorMax[i]);
  }
}

void cargarCalibracion()
{
  for (int i = 0; i < 8; i++)
  {
    EEPROM.get(i * 4, sensorMin[i]);
    EEPROM.get(i * 4 + 2, sensorMax[i]);
    // Protección por si la memoria está vacía
    if (sensorMin[i] == 0xFFFF)
    {
      sensorMin[i] = 1023;
      sensorMax[i] = 0;
    }
    // Calculamos el punto medio (Umbral)
    umbral[i] = (sensorMin[i] + sensorMax[i]) / 2;
  }
}

void calibracion()
{
  static unsigned long tiempoInicio = 0;
  static unsigned long ultimoParpadeo = 0;
  if (tiempoInicio == 0)
    tiempoInicio = millis();
  for (int i = 0; i < 8; i++)
  {
    if (sensor[i] < sensorMin[i])
      sensorMin[i] = sensor[i];
    if (sensor[i] > sensorMax[i])
      sensorMax[i] = sensor[i];
  }
  if (millis() - ultimoParpadeo > 150)
  {
    ultimoParpadeo = millis();
    digitalWrite(ledPin, !digitalRead(ledPin));
  }
  if (millis() - tiempoInicio > 3000)
  {
    calibrando = false;
    tiempoInicio = 0;
    for (int i = 0; i < 8; i++)
      umbral[i] = (sensorMin[i] + sensorMax[i]) / 2;

    guardarCalibracion();
    digitalWrite(ledPin, LOW);
  }
  return;
}

void moverMotores(int pwmIzq, int pwmDer)
{
  // Motor Izquierdo
  if (pwmIzq >= 0)
  {
    digitalWrite(dirMotorIzq1, LOW);
    digitalWrite(dirMotorIzq2, HIGH);
  }
  else
  {
    digitalWrite(dirMotorIzq1, HIGH);
    digitalWrite(dirMotorIzq2, LOW);
    pwmIzq = -pwmIzq;
  }

  // Motor Derecho
  if (pwmDer >= 0)
  {
    digitalWrite(dirMotorDer1, LOW);
    digitalWrite(dirMotorDer2, HIGH);
  }
  else
  {
    digitalWrite(dirMotorDer1, HIGH);
    digitalWrite(dirMotorDer2, LOW);
    pwmDer = -pwmDer;
  }

  analogWrite(pinPwmMotorIzq, constrain(pwmIzq, 0, 255));
  analogWrite(pinPwmMotorDer, constrain(pwmDer, 0, 255));
  digitalWrite(stbyMotor, HIGH);
}

void setup()
{
  pinMode(ledPin, OUTPUT);
  pinMode(buttonCalib, INPUT);
  pinMode(buttonStart, INPUT);
  pinMode(pinIR, OUTPUT);
  digitalWrite(pinIR, HIGH);

  pinMode(dirMotorIzq1, OUTPUT);
  pinMode(dirMotorIzq2, OUTPUT);
  pinMode(dirMotorDer1, OUTPUT);
  pinMode(dirMotorDer2, OUTPUT);
  pinMode(stbyMotor, OUTPUT);
  pinMode(pinPwmMotorIzq, OUTPUT);
  pinMode(pinPwmMotorDer, OUTPUT);

  Serial.begin(9600);
  cargarCalibracion();
}

void loop()
{
  for (int i = 0; i < 8; i++)
    sensor[i] = analogRead(i);
  if (!calibrando && digitalRead(buttonCalib) == HIGH)
  {
    calibrando = true;
    for (int i = 0; i < 8; i++)
    {
      sensorMin[i] = 1023;
      sensorMax[i] = 0;
    }
  }
  if (calibrando)
  {
    calibracion();
  }
  if (digitalRead(buttonStart) == HIGH)
  {
    iniciarRecorrido = true;
    delay(500);
  }
  if (!iniciarRecorrido)
  {
    moverMotores(0, 0);
    return;
  }
  int32_t sumaPonderada = 0;
  int16_t sensoresActivos = 0;

  for (int i = 0; i < 8; i++)
  {
    if (sensor[i] > umbral[i])
    {
      sumaPonderada += pesos[i];
      sensoresActivos++;
    }
  }
  if (sensoresActivos > 0)
  {
    errorActual = sumaPonderada / sensoresActivos;
    ultimoErrorConocido = errorActual;
  }
  else
  {
    // RECUPERACIÓN DE LÍNEA: Girar hacia donde se vio por última vez
    if (ultimoErrorConocido > 0)
      errorActual = 4500;
    else
      errorActual = -4500;
  }

  // 5. CÁLCULO PID
  errorDerivativo = errorActual - errorAnterior;
  u = (Kp * errorActual + Ki * 0 + Kd * errorDerivativo) / 1000;
  errorAnterior = errorActual;

  // 6. MOTORES CON ACELERACIÓN DINÁMICA
  int velocidadRecta = 200;
  int velocidadCurva = 130;

  int agresividad = constrain(abs(u), 0, 100);

  int velocidadBase = map(agresividad, 0, 100, velocidadRecta, velocidadCurva);

  int pwmIzq = velocidadBase - u;
  int pwmDer = velocidadBase + u;

  moverMotores(pwmIzq, pwmDer);
}