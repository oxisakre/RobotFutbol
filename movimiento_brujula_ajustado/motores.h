#ifndef MOTORES_H
#define MOTORES_H

#include <Arduino.h> // CRÍTICO: Le enseña al archivo .h qué es un pinMode, analogWrite, etc.

// ================= PINES =================
// MOTOR 1 (Adelante-Derecha,   45°)
const int M1_IN1 = 2;  const int M1_IN2 = 1;  const int M1_ENA = 3;
// MOTOR 2 (Atras-Derecha,     -45°)
const int M2_IN3 = 4;  const int M2_IN4 = 5;  const int M2_ENB = 0;
// MOTOR 3 (Atras-Izquierda,  -135°)
const int M3_IN1 = 17; const int M3_IN2 = 16; const int M3_ENC = 19;
// MOTOR 4 (Adelante-Izquierda, 135°)
const int M4_IN3 = 14; const int M4_IN4 = 15; const int M4_END = 13;

// SOLENOIDE (Patada)
const int PIN_KICK = 33;

// ================= DEADZONES =================
const int MIN_M1 = 172;
const int MIN_M2 = 180;
const int MIN_M3 = 170;
const int MIN_M4 = 148;

// ================= PID DE ESTABILIZACIÓN =================
float Kp = 0.015;
float Kd = 0.08;
float error_previo = 0;
float W_prev       = 0;

// ================= FUNCIONES =================

void inicializarMotores() {
  int pines_dir[] = {M1_IN1, M1_IN2, M2_IN3, M2_IN4, M3_IN1, M3_IN2, M4_IN3, M4_IN4};
  for (int i = 0; i < 8; i++) {
    pinMode(pines_dir[i], OUTPUT);
  }

  pinMode(M1_ENA, OUTPUT); pinMode(M2_ENB, OUTPUT);
  pinMode(M3_ENC, OUTPUT); pinMode(M4_END, OUTPUT);

  pinMode(PIN_KICK, OUTPUT);
  digitalWrite(PIN_KICK, LOW);
}

void frenar() {
  digitalWrite(M1_IN1, LOW); digitalWrite(M1_IN2, LOW);
  digitalWrite(M2_IN3, LOW); digitalWrite(M2_IN4, LOW);
  digitalWrite(M3_IN1, LOW); digitalWrite(M3_IN2, LOW);
  digitalWrite(M4_IN3, LOW); digitalWrite(M4_IN4, LOW);
  analogWrite(M1_ENA, 0); analogWrite(M2_ENB, 0);
  analogWrite(M3_ENC, 0); analogWrite(M4_END, 0);
}

void patear() {
  digitalWrite(PIN_KICK, HIGH);
  delay(150);
  digitalWrite(PIN_KICK, LOW);
}

// Esta función ahora recibe la dirección, la velocidad y el error ya calculado
void moverEnAngulo(float angulo_grados, int velocidad, float error_brújula) {
  
  // ===== CONTROL PD =====
  // Si estamos muy cerca del objetivo (margen de 6 grados), no corregimos para evitar vibraciones
  if (abs(error_brújula) < 6.0) error_brújula = 0.0;

  float derivativa = error_brújula - error_previo;
  float W_target = (Kp * error_brújula) + (Kd * derivativa);
  error_previo = error_brújula;

  // Limitamos el giro máximo
  if (W_target >  1.0) W_target =  1.0;
  if (W_target < -1.0) W_target = -1.0;

  // Rampa de aceleración para el giro (evita tirones bruscos)
  float max_delta_W = 0.1;
  if      (W_target > W_prev + max_delta_W) W_prev += max_delta_W;
  else if (W_target < W_prev - max_delta_W) W_prev -= max_delta_W;
  else                                      W_prev  = W_target;

  float W = W_prev;

  // ===== CINEMÁTICA X-DRIVE =====
  float radianes     = angulo_grados * (PI / 180.0);
  float ratio_lineal = constrain(velocidad, 0, 255) / 255.0;

  float Vy = cos(radianes) * ratio_lineal;
  float Vx = -sin(radianes) * ratio_lineal;

  float vel_M1 = Vy - Vx - W;
  float vel_M2 = Vy + Vx - W;
  float vel_M3 = Vy - Vx + W;
  float vel_M4 = Vy + Vx + W;

  // Normalización (para que ningún motor pase del 100%)
  float max_vel = max(max(abs(vel_M1), abs(vel_M2)), max(abs(vel_M3), abs(vel_M4)));
  if (max_vel > 1.0) {
    vel_M1 /= max_vel; vel_M2 /= max_vel;
    vel_M3 /= max_vel; vel_M4 /= max_vel;
  }

  // ===== ZONA MUERTA Y PWM =====
  int pwm_M1 = 0, pwm_M2 = 0, pwm_M3 = 0, pwm_M4 = 0;
  float umbral_ruido = max(0.04f, 0.15f * ratio_lineal);

  if (abs(vel_M1) > umbral_ruido) pwm_M1 = MIN_M1 + round(abs(vel_M1) * (255 - MIN_M1));
  if (abs(vel_M2) > umbral_ruido) pwm_M2 = MIN_M2 + round(abs(vel_M2) * (255 - MIN_M2));
  if (abs(vel_M3) > umbral_ruido) pwm_M3 = MIN_M3 + round(abs(vel_M3) * (255 - MIN_M3));
  if (abs(vel_M4) > umbral_ruido) pwm_M4 = MIN_M4 + round(abs(vel_M4) * (255 - MIN_M4));

  // ===== DIRECCIÓN =====
  if (pwm_M1 == 0) { digitalWrite(M1_IN1, LOW);      digitalWrite(M1_IN2, LOW);      }
  else             { digitalWrite(M1_IN1, vel_M1 > 0); digitalWrite(M1_IN2, vel_M1 < 0); }
  if (pwm_M2 == 0) { digitalWrite(M2_IN3, LOW);      digitalWrite(M2_IN4, LOW);      }
  else             { digitalWrite(M2_IN3, vel_M2 > 0); digitalWrite(M2_IN4, vel_M2 < 0); }
  if (pwm_M3 == 0) { digitalWrite(M3_IN1, LOW);      digitalWrite(M3_IN2, LOW);      }
  else             { digitalWrite(M3_IN1, vel_M3 > 0); digitalWrite(M3_IN2, vel_M3 < 0); }
  if (pwm_M4 == 0) { digitalWrite(M4_IN3, LOW);      digitalWrite(M4_IN4, LOW);      }
  else             { digitalWrite(M4_IN3, vel_M4 > 0); digitalWrite(M4_IN4, vel_M4 < 0); }

  // ===== ENERGIZAR =====
  analogWrite(M1_ENA, pwm_M1); analogWrite(M2_ENB, pwm_M2);
  analogWrite(M3_ENC, pwm_M3); analogWrite(M4_END, pwm_M4);
}

#endif