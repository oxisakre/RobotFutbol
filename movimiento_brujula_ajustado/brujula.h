#ifndef BRUJULA_H
#define BRUJULA_H

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <math.h>

// Instancia del sensor (usando Wire2 según tu código)
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire2);
float offset_heading  = 0;

// ==========================================
// 1. INICIALIZACIÓN
// ==========================================
void inicializarBrujula() {
  Wire2.begin();
  if (!bno.begin()) {
    Serial.println("Error: No BNO055 detectado...");
    while (1); 
  }

  bno.setExtCrystalUse(true);

  // Calentamiento del filtro interno (evita el salto brusco)
  for(int i = 0; i < 50; i++) {
    sensors_event_t dummy_event;
    bno.getEvent(&dummy_event);
    delay(50);
  }

  sensors_event_t event;
  bno.getEvent(&event);
  offset_heading = event.orientation.x;
}

// ==========================================
// 2. LECTURA (0° a 360° para lectura humana)
// ==========================================
float getHeading360() {
  sensors_event_t event;
  bno.getEvent(&event);
  
  float angulo_crudo = event.orientation.x - offset_heading;
  if (isnan(angulo_crudo) || isinf(angulo_crudo)) return 0.0;
  
  float mod_angle = fmod(angulo_crudo, 360.0);
  if (mod_angle < 0) mod_angle += 360.0;
  
  return mod_angle;
}

// ==========================================
// 3. CÁLCULO DE ERROR PARA EL PID (-180° a 180°)
// ==========================================
// Usa esta función EXCLUSIVAMENTE para tu variable "error" en la cinemática
float calcularErrorCorto(float target, float current) {
  float error = target - current;
  
  // Forzamos el error a que siempre busque el camino más corto
  while (error > 180.0)  error -= 360.0;
  while (error < -180.0) error += 360.0;
  
  return error;
}

#endif