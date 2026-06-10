#ifndef CYTRON_MOTOR_DRIVER_H
#define CYTRON_MOTOR_DRIVER_H

#include <Arduino.h>
#include <stdint.h>

enum MODE {
  PWM_DIR,
  PWM_PWM,
};

class CytronMD
{
  public:
    CytronMD(MODE mode, uint8_t pin1, uint8_t pin2);
    void setSpeed(int16_t speed);
    
  protected:
    MODE _mode;
  	uint8_t _pin1;
    uint8_t _pin2;
};

#endif

CytronMD::CytronMD(MODE mode, uint8_t pin1, uint8_t pin2)
{
      _mode = mode;
      _pin1 = pin1;
      _pin2 = pin2;

      pinMode(_pin1, OUTPUT);
      pinMode(_pin2, OUTPUT);

      digitalWrite(_pin1, LOW);
      digitalWrite(_pin2, LOW);
}

void CytronMD::setSpeed(int16_t speed)
{
      // Make sure the speed is within the limit.
      if (speed > 255)
      {
            speed = 255;
      }
      else if (speed < -255)
      {
            speed = -255;
      }

      // Set the speed and direction.
      switch (_mode)
      {
      case PWM_DIR:
            if (speed >= 0)
            {
#if defined(ARDUINO_ARCH_ESP32)
                  ledcWrite(_pin1, speed);
#else
                  analogWrite(_pin1, speed);
#endif

                  digitalWrite(_pin2, LOW);
            }
            else
            {

#if defined(ARDUINO_ARCH_ESP32)
                  ledcWrite(_pin1, -speed);
#else
                  analogWrite(_pin1, -speed);
#endif

                  digitalWrite(_pin2, HIGH);
            }
            break;

      case PWM_PWM:
            if (speed >= 0)
            {
#if defined(ARDUINO_ARCH_ESP32)
                  ledcWrite(_pin1, speed);
                  ledcWrite(_pin2, 0);
#else
                  analogWrite(_pin1, speed);
                  analogWrite(_pin2, 0);
#endif
            }
            else
            {
#if defined(ARDUINO_ARCH_ESP32)
                  ledcWrite(_pin1, 0);
                  ledcWrite(_pin2, -speed);
#else
                  analogWrite(_pin1, 0);
                  analogWrite(_pin2, -speed);
#endif
            }
            break;
      }
}


// --- Cytron motor driver setup ---
CytronMD motor1(PWM_PWM, 6, 5);   // Motor 1 on pins 6 & 5 
CytronMD motor2(PWM_PWM, 10, 11); // Motor 2 on pins 10 & 11

// --- Interrupt PWM variables ---
volatile unsigned long pulseStart1 = 0;
volatile unsigned long pulseWidth1 = 1500;
volatile unsigned long pulseStart2 = 0;
volatile unsigned long pulseWidth2 = 1500;

// --- PWM signal specs ---
const int pwmMin = 1000;
const int pwmCenter = 1500;
const int pwmMax = 2000;
const int deadband = 30;

// --- Smoothed values ---
float smoothSpeed1 = 0;
float smoothSpeed2 = 0;
const float alpha = 0.15;  // Smoothing factor (between 0 and 1)

// --- Timing for failsafe ---
unsigned long lastSignalTime1 = 0;
unsigned long lastSignalTime2 = 0;
const unsigned long timeout = 300;  // ms before failsafe stops the motor

void setup() {
  pinMode(2, INPUT);  // PWM Input 1 (INT0)
  pinMode(3, INPUT);  // PWM Input 2 (INT1)

  attachInterrupt(digitalPinToInterrupt(2), pwmInput1Change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), pwmInput2Change, CHANGE);

  Serial.begin(115200);
}

void loop() {
  unsigned long pwm1, pwm2;
  unsigned long now = millis();

  noInterrupts();
  pwm1 = pulseWidth1;
  pwm2 = pulseWidth2;
  interrupts();

  // Map to raw motor speeds
  int targetSpeed1 = mapPulseToSpeed(pwm1);
  int targetSpeed2 = mapPulseToSpeed(pwm2);

  // Smoothing the speeds
  smoothSpeed1 = (alpha * targetSpeed1) + (1 - alpha) * smoothSpeed1;
  smoothSpeed2 = (alpha * targetSpeed2) + (1 - alpha) * smoothSpeed2;

  // Failsafe: Check if signal has timed out
  if (now - lastSignalTime1 > timeout) {
    smoothSpeed1 = 0;
  }

  if (now - lastSignalTime2 > timeout) {
    smoothSpeed2 = 0;
  }

  // Debug output
  Serial.print("PWM1: ");
  Serial.print(pwm1);
  Serial.print(" us | Smoothed Speed1: ");
  Serial.print(smoothSpeed1);
  Serial.print(" | PWM2: ");
  Serial.print(pwm2);
  Serial.print(" us | Smoothed Speed2: ");
  Serial.println(smoothSpeed2);

  // Send smoothed speeds to motors
  motor1.setSpeed((int)smoothSpeed1);
  motor2.setSpeed((int)smoothSpeed2);

  delay(20);
}

// --- Interrupt routines ---

void pwmInput1Change() {
  if (digitalRead(2) == HIGH) {
    pulseStart1 = micros();
  } else {
    pulseWidth1 = micros() - pulseStart1;
    lastSignalTime1 = millis();  // Update last signal time
  }
}

void pwmInput2Change() {
  if (digitalRead(3) == HIGH) {
    pulseStart2 = micros();
  } else {
    pulseWidth2 = micros() - pulseStart2;
    lastSignalTime2 = millis();  // Update last signal time
  }
}

// --- PWM to speed mapping ---
int mapPulseToSpeed(unsigned long pulse) {
  if (pulse >= pwmMin && pulse <= pwmMax) {
    if (pulse > pwmCenter + deadband) {
      return map(pulse, pwmCenter + deadband, pwmMax, 0, 255);
    } else if (pulse < pwmCenter - deadband) {
      return map(pulse, pwmMin, pwmCenter - deadband, -255, 0);
    } else {
      return 0;  // Deadband zone
    }
  } else {
    return 0;    // Invalid signal
  }
}
