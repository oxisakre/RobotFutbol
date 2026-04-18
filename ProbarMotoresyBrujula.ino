#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// Tu brújula usa Wire2
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire2);

// TUS PINES REALES
// MOTOR 1
const int M1_IN1 = 2; const int M1_IN2 = 1; const int M1_ENA = 3;
// MOTOR 2
const int M2_IN3 = 4; const int M2_IN4 = 5; const int M2_ENB = 0;
// MOTOR 3
const int M3_IN1 = 17; const int M3_IN2 = 16; const int M3_ENC = 19;
// MOTOR 4
const int M4_IN3 = 14; const int M4_IN4 = 15; const int M4_END = 13;

unsigned long ultimoTiempoDatos = 0;

void setup() {
  Serial.begin(115200);

  // Configurar absolutamente todos los pines de control como salida
  int pines[] = {M1_IN1, M1_IN2, M1_ENA, M2_IN3, M2_IN4, M2_ENB, 
                 M3_IN1, M3_IN2, M3_ENC, M4_IN3, M4_IN4, M4_END};
                 
  for(int i=0; i<12; i++) {
    pinMode(pines[i], OUTPUT);
  }
  
  apagarTodosLosMotores();

  // Iniciar I2C (Wire2) y Brújula
  Wire2.begin();
  Serial.println("Iniciando brújula...");
  if (!bno.begin()) {
    Serial.println("Error: No se detectó la brújula en Wire2.");
    while (1);
  }
  
  Serial.println("Test listo. Ingresa 0, 1, 2, 3 o 4:");
}

void loop() {
  if (millis() - ultimoTiempoDatos > 500) {
    sensors_event_t event;
    bno.getEvent(&event);
    Serial.print("Brújula (Grados): ");
    Serial.println(event.orientation.x);
    ultimoTiempoDatos = millis();
  }

  if (Serial.available() > 0) {
    char comando = Serial.read();
    if (comando == '\n' || comando == '\r') return; 

    int cantidad = comando - '0'; 

    if (cantidad >= 0 && cantidad <= 4) {
      Serial.print(">>> ENCENDIENDO ");
      Serial.print(cantidad);
      Serial.println(" MOTORES <<<");
      activarMotores(cantidad);
    } else {
      Serial.println("Por favor, ingresa solo números del 0 al 4.");
    }
  }
}

void apagarTodosLosMotores() {
  // Cortamos la energía de todos los Enable
  digitalWrite(M1_ENA, LOW);
  digitalWrite(M2_ENB, LOW);
  digitalWrite(M3_ENC, LOW);
  digitalWrite(M4_END, LOW);
  
  // Apagamos direcciones
  digitalWrite(M1_IN1, LOW); digitalWrite(M1_IN2, LOW);
  digitalWrite(M2_IN3, LOW); digitalWrite(M2_IN4, LOW);
  digitalWrite(M3_IN1, LOW); digitalWrite(M3_IN2, LOW);
  digitalWrite(M4_IN3, LOW); digitalWrite(M4_IN4, LOW);
}

void activarMotores(int cantidad) {
  apagarTodosLosMotores(); 

  // Configuramos la dirección hacia adelante para los que se vayan a encender
  if (cantidad >= 1) { digitalWrite(M1_IN1, HIGH); digitalWrite(M1_IN2, LOW); }
  if (cantidad >= 2) { digitalWrite(M2_IN3, HIGH); digitalWrite(M2_IN4, LOW); }
  if (cantidad >= 3) { digitalWrite(M3_IN1, HIGH); digitalWrite(M3_IN2, LOW); }
  if (cantidad >= 4) { digitalWrite(M4_IN3, HIGH); digitalWrite(M4_IN4, LOW); }

  // Encendemos la potencia (PWM/Enable) a máxima velocidad (HIGH = 255)
  // Esto generará el pico de corriente para probar si la fuente resiste
  if (cantidad >= 1) digitalWrite(M1_ENA, HIGH);
  if (cantidad >= 2) digitalWrite(M2_ENB, HIGH);
  if (cantidad >= 3) digitalWrite(M3_ENC, HIGH);
  if (cantidad >= 4) digitalWrite(M4_END, HIGH);
}