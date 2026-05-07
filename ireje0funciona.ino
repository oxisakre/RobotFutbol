#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>

// ================= IMU =================
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire2);
float offset_heading = 0;
float heading_deseado = 0;

// Constantes de Control y Filtros
float Kp = 0.015; 
float Kd = 0.08; 
float error_previo = 0;
float W_prev = 0;

// ================= PINES =================
// MOTOR 1 (Adelante-Derecha)
const int M1_IN1 = 2; const int M1_IN2 = 1; const int M1_ENA = 3;
// MOTOR 2 (Atrás-Derecha)
const int M2_IN3 = 4; const int M2_IN4 = 5; const int M2_ENB = 0;
// MOTOR 3 (Atrás-Izquierda)
const int M3_IN1 = 17; const int M3_IN2 = 16; const int M3_ENC = 19;
// MOTOR 4 (Adelante-Izquierda)
const int M4_IN3 = 14; const int M4_IN4 = 15; const int M4_END = 13;

// SOLENOIDE
const int PIN_KICK = 33;

// DEADZONE (Valores mínimos de arranque ajustados para reducir fuerza inicial)
const int MIN_M1 = 130;
const int MIN_M2 = 135;
const int MIN_M3 = 130;
const int MIN_M4 = 115;

// ================= FUNCIONES IMU =================

float normalizeAngle(float angle) {
  if (isnan(angle) || isinf(angle)) return 0.0;
  float mod_angle = fmod(angle + 180.0, 360.0);
  if (mod_angle < 0) {
    mod_angle += 360.0;
  }
  return mod_angle - 180.0;
}

float getHeading() {
  sensors_event_t event;
  bno.getEvent(&event);
  return normalizeAngle(event.orientation.x - offset_heading);
}

// ================= SETUP =================

void setup() {
  Serial.begin(115200);

  int pines_dir[] = {M1_IN1, M1_IN2, M2_IN3, M2_IN4, M3_IN1, M3_IN2, M4_IN3, M4_IN4};
  for(int i=0; i<8; i++) pinMode(pines_dir[i], OUTPUT);

  pinMode(M1_ENA, OUTPUT);
  pinMode(M2_ENB, OUTPUT);
  pinMode(M3_ENC, OUTPUT);
  pinMode(M4_END, OUTPUT);

  pinMode(PIN_KICK, OUTPUT);
  digitalWrite(PIN_KICK, LOW);

  Wire2.begin();
  if (!bno.begin()) {
    Serial.println("No BNO055 detectado... revisa cableado.");
    while(1);
  }

  delay(1000);
  bno.setExtCrystalUse(true);

  sensors_event_t event;
  bno.getEvent(&event);
  offset_heading = event.orientation.x;

  frenar();
}

// ================= LOOP =================

void loop() {
  float angulo = 0; 
  int velocidad = 0; 

  moverEnAngulo(angulo, velocidad);
  delay(20);
}

// ==========================================
//        FUNCIÓN PRINCIPAL DE MOVIMIENTO
// ==========================================

void moverEnAngulo(float angulo_grados, int velocidad) {

  // ===== IMU =====
  float heading_actual = getHeading();
  float error = normalizeAngle(heading_deseado - heading_actual);

  // Banda muerta: previene oscilaciones de sobreimpulso (limit cycle) ampliada a 6 grados
  if (abs(error) < 6.0) { 
    error = 0.0;
  }

  // ===== CONTROL PD =====
  float derivativa = error - error_previo;
  float W_target = (Kp * error) + (Kd * derivativa);
  error_previo = error;

  if (W_target > 1) W_target = 1;
  if (W_target < -1) W_target = -1;

  // Filtro Slew Rate: previene caída de tensión por picos de corriente (brownout)
  float max_delta_W = 0.1; 
  if (W_target > W_prev + max_delta_W) {
    W_prev += max_delta_W;
  } else if (W_target < W_prev - max_delta_W) {
    W_prev -= max_delta_W;
  } else {
    W_prev = W_target;
  }
  
  float W = W_prev;

  // ===== CINEMÁTICA =====
  float rad = angulo_grados * PI / 180.0;
  
  float ratio_lineal = constrain(velocidad, 0, 255) / 255.0;
  float Vy = cos(rad) * ratio_lineal;
  float Vx = -sin(rad) * ratio_lineal;

  float vel_M1 = Vy - Vx - W;
  float vel_M2 = Vy + Vx - W;
  float vel_M3 = Vy - Vx + W; 
  float vel_M4 = Vy + Vx + W; 

  float max_vel = max(max(abs(vel_M1), abs(vel_M2)), max(abs(vel_M3), abs(vel_M4)));
  if (max_vel > 1.0) {
    vel_M1 /= max_vel;
    vel_M2 /= max_vel;
    vel_M3 /= max_vel;
    vel_M4 /= max_vel;
  }

  // ===== PWM y ZONA MUERTA =====
  int pwm_M1 = 0, pwm_M2 = 0, pwm_M3 = 0, pwm_M4 = 0;
  float umbral_ruido = 0.15; // Elevado al 15% para ignorar correcciones diminutas

  if (abs(vel_M1) > umbral_ruido) pwm_M1 = MIN_M1 + round(abs(vel_M1) * (255 - MIN_M1));
  if (abs(vel_M2) > umbral_ruido) pwm_M2 = MIN_M2 + round(abs(vel_M2) * (255 - MIN_M2));
  if (abs(vel_M3) > umbral_ruido) pwm_M3 = MIN_M3 + round(abs(vel_M3) * (255 - MIN_M3));
  if (abs(vel_M4) > umbral_ruido) pwm_M4 = MIN_M4 + round(abs(vel_M4) * (255 - MIN_M4));

  // ===== DIRECCIÓN Y FRENO ======
  if (pwm_M1 == 0) { 
    digitalWrite(M1_IN1, LOW); digitalWrite(M1_IN2, LOW); 
  } else { 
    digitalWrite(M1_IN1, vel_M1 > 0); digitalWrite(M1_IN2, vel_M1 < 0); 
  }

  if (pwm_M2 == 0) { 
    digitalWrite(M2_IN3, LOW); digitalWrite(M2_IN4, LOW); 
  } else { 
    digitalWrite(M2_IN3, vel_M2 > 0); digitalWrite(M2_IN4, vel_M2 < 0); 
  }

  if (pwm_M3 == 0) { 
    digitalWrite(M3_IN1, LOW); digitalWrite(M3_IN2, LOW); 
  } else { 
    digitalWrite(M3_IN1, vel_M3 > 0); digitalWrite(M3_IN2, vel_M3 < 0); 
  }

  if (pwm_M4 == 0) { 
    digitalWrite(M4_IN3, LOW); digitalWrite(M4_IN4, LOW); 
  } else { 
    digitalWrite(M4_IN3, vel_M4 > 0); digitalWrite(M4_IN4, vel_M4 < 0); 
  }

  // ===== VELOCIDAD =====
  analogWrite(M1_ENA, pwm_M1);
  analogWrite(M2_ENB, pwm_M2);
  analogWrite(M3_ENC, pwm_M3);
  analogWrite(M4_END, pwm_M4);

  // DEBUG 
  Serial.print("H: "); Serial.print(heading_actual);
  Serial.print(" E: "); Serial.print(error);
  Serial.print(" W: "); Serial.println(W);
}

// ================= AUX =================

void frenar() {
  digitalWrite(M1_IN1, LOW); digitalWrite(M1_IN2, LOW);
  digitalWrite(M2_IN3, LOW); digitalWrite(M2_IN4, LOW);
  digitalWrite(M3_IN1, LOW); digitalWrite(M3_IN2, LOW);
  digitalWrite(M4_IN3, LOW); digitalWrite(M4_IN4, LOW);

  analogWrite(M1_ENA, 0);
  analogWrite(M2_ENB, 0);
  analogWrite(M3_ENC, 0);
  analogWrite(M4_END, 0);
}
