#include <Arduino.h>
#include "A4988.h"
#include <CircularBuffer.h>
#include "Adafruit_VL53L0X.h"

CircularBuffer<double, 6> buffer;

// Define the pin connections for the A4988 driver
#define DIR_PIN 14
#define STEP_PIN 32
#define MS1_PIN 13
#define MS2_PIN 12
#define MS3_PIN 27
#define MOTOR_STEPS 200

#define MAX 45
#define MIN -45

#define motorInterfaceType 1
A4988 stepper(MOTOR_STEPS, DIR_PIN, STEP_PIN, MS1_PIN, MS2_PIN, MS3_PIN);
double angle_P, angle_I, angle_D, angle_PID, angle_now;

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

double distance = 0.0;
double distancePrecedente;
uint16_t temps, tempsPrint; // variable pour le temps
double distance_previous_error, distance_error;
uint16_t period = 10; // Refresh rate period of the loop is 10ms

#define KP_MAX 2
#define KI_MAX 1
#define KD_MAX 2000

double kp = 1.5;
double ki = 0.2;
double kd = 300;
double distance_setpoint = 25;

void mesureDistanceAvecMoyenneGlissante(void);
void Kpot(void);

void setup()
{
  Serial.begin(115200);
  stepper.begin(180, 2); // vitesse est de 1.8°/s
  temps = millis();
  tempsPrint = millis();
  angle_now = 0;
  stepper.setSpeedProfile(stepper.LINEAR_SPEED, 4000, 4000);
  //setup du VL53L0X
  if (!lox.begin())
  {
    Serial.println(F("Failed to boot VL53L0X"));
    while (1)
    {
    }
  }
  lox.startRangeContinuous();
}

void loop()
{

  if (millis() > temps + period)
  {
    temps = millis();
    Kpot();
    mesureDistanceAvecMoyenneGlissante();

    distance_error = distance_setpoint - distance;
    // Proportionnel
    angle_P = kp * distance_error;

    // Intégrale
    angle_I = angle_I + (ki * distance_error);

    // Derivée
    float dist_diference = distance_error - distance_previous_error;
    angle_D = kd * ((distance_error - distance_previous_error) / period);

    angle_PID = angle_P + angle_I + angle_D;

    double target_angle = angle_PID;
    target_angle = target_angle > MAX ? MAX : target_angle;
    target_angle = target_angle < MIN ? MIN : target_angle;
    double rotation = target_angle - angle_now;
    stepper.rotate(rotation);
    angle_now = target_angle;
    distance_previous_error = distance_error;
  }

  if (millis() > tempsPrint + 100)
  {
    tempsPrint = millis();
    printf("distance: %6.2f error: %6.2f angle_PID: %6.2f angle_now: %6.2f ", distance, distance_error, angle_PID, angle_now);
    printf("kp: %f ki: %f kd: %f \n", kp, ki, kd / 1000.0);
  }
}

void mesureDistanceAvecMoyenneGlissante(void)
{

  while (!buffer.isFull())
  {
    distance = lox.readRange() / 10.0;
    buffer.push(distance);
  }

  if (buffer.isFull())
  {
    // trouver la disntance en ce moment
    distance = 0.00;
    for (int i = 0; i < buffer.size() - 1; i++)
    {
      distance += buffer[i];
    }
    distance = distance / (float)buffer.size();

    // trouver la distance precedente
    distancePrecedente = 0.00;
    for (int i = 1; i < buffer.size(); i++)
    {
      distancePrecedente += buffer[i];
    }
    distancePrecedente = distancePrecedente / (float)buffer.size();

    buffer.shift(); // enleve la derniere valeur pour fifo
  }
}

void Kpot(void)
{
  uint16_t ADC_Value1 = analogRead(A2);
  uint16_t ADC_Value2 = analogRead(A3);
  uint16_t ADC_Value3 = analogRead(A4);

  kp = KP_MAX * ADC_Value1 / 4096.0;
  ki = KI_MAX * ADC_Value2 / 4096.0;
  kd = KD_MAX * ADC_Value3 / 4096.0;
}