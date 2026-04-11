// --- DEFINICIÓN DE PINES (Adaptado para Teensy) ---
// --- MOTOR 1 (Adelante-Derecha, 45°) ---
const int M1_IN1 = 2; const int M1_IN2 = 1; const int M1_ENA = 3;
// --- MOTOR 2 (Atrás-Derecha, -45°) ---
const int M2_IN3 = 4; const int M2_IN4 = 5; const int M2_ENB = 0;
// --- MOTOR 3 (Atrás-Izquierda, -135°) ---
const int M3_IN1 = 17; const int M3_IN2 = 16; const int M3_ENC = 19;
// --- MOTOR 4 (Adelante-Izquierda, 135°) ---
const int M4_IN3 = 14; const int M4_IN4 = 15; const int M4_END = 13;

// --- PIN SOLENOIDE (Pateador) ---
const int PIN_KICK = 33;

const int vel_max = 255;
const int vel_giro = 180;

void setup() {
  // Configuración de los pines de DIRECCIÓN usando las variables
  int pines_dir[] = {M1_IN1, M1_IN2, M2_IN3, M2_IN4, M3_IN1, M3_IN2, M4_IN3, M4_IN4};
  for(int i=0; i<8; i++) { 
    pinMode(pines_dir[i], OUTPUT); 
  }

  // Configuración de los pines de VELOCIDAD (PWM)
  pinMode(M1_ENA, OUTPUT);
  pinMode(M2_ENB, OUTPUT);
  pinMode(M3_ENC, OUTPUT);
  pinMode(M4_END, OUTPUT);

  // Configuración del pin del solenoide
  pinMode(PIN_KICK, OUTPUT);
  digitalWrite(PIN_KICK, LOW); // Aseguramos que arranque apagado por seguridad
}

void loop() {
  // --- COREOGRAFÍA DE PRUEBA DE PATRONES ---
  delay(5000); // Espera 5 segundos antes de empezar

  // Patrón 1: Avanzar (Conjunto)
  avanzar(vel_max);
  frenar();
  patear(); 

  // Patrón 2: Retroceder (Conjunto)
  retroceder(vel_max);
  frenar();
  patear(); 

  // Patrón 3: Desplazamiento Lateral Derecha (Conjunto)
  desplazarDerecha(vel_max);
  frenar();
  patear(); 

  // Patrón 4: Desplazamiento Lateral Izquierda (Conjunto)
  desplazarIzquierda(vel_max);
  frenar();
  patear(); 

  // Patrón 5: Pivotar (Giro sobre eje central)
  pivotarDerecha(vel_giro);
  frenar();
  patear(); 

  // Patrón Independiente 6: Solo Mueve Motor 1 (Individual)
  digitalWrite(M1_IN1, HIGH); digitalWrite(M1_IN2, LOW); 
  analogWrite(M1_ENA, vel_max); // Cambio a analogWrite
  delay(3000); // Agregué un delay para que se note el movimiento
  frenar();
  patear(); 
}

// ==========================================
//        FUNCIÓN DE DISPARO (SOLENOIDE)
// ==========================================

void patear() {
  // Paso 1: Espera para que el Step-Up cargue los capacitores
  delay(150);
  
  digitalWrite(PIN_KICK, HIGH);
  delay(150); // Mantiene el pulso 150ms 
  digitalWrite(PIN_KICK, LOW);
  delay(7000); 
}

// ==========================================
//        FUNCIONES DE MOVIMIENTO HOLONÓMICO
// ==========================================

void frenar() {
  digitalWrite(M1_IN1, LOW); digitalWrite(M1_IN2, LOW);
  digitalWrite(M2_IN3, LOW); digitalWrite(M2_IN4, LOW);
  digitalWrite(M3_IN1, LOW); digitalWrite(M3_IN2, LOW);
  digitalWrite(M4_IN3, LOW); digitalWrite(M4_IN4, LOW);
  
  // Reemplazo de ledcWrite por analogWrite
  analogWrite(M1_ENA, 0); analogWrite(M2_ENB, 0);
  analogWrite(M3_ENC, 0); analogWrite(M4_END, 0);
  delay(3000);
}

void avanzar(int velocidad) {
  digitalWrite(M1_IN1, HIGH); digitalWrite(M1_IN2, LOW);
  digitalWrite(M2_IN3, HIGH); digitalWrite(M2_IN4, LOW);
  digitalWrite(M3_IN1, HIGH); digitalWrite(M3_IN2, LOW);
  digitalWrite(M4_IN3, HIGH); digitalWrite(M4_IN4, LOW);
  
  analogWrite(M1_ENA, velocidad); analogWrite(M2_ENB, velocidad);
  analogWrite(M3_ENC, velocidad); analogWrite(M4_END, velocidad);
  delay(3000);
}

void retroceder(int velocidad) {
  digitalWrite(M1_IN1, LOW); digitalWrite(M1_IN2, HIGH);
  digitalWrite(M2_IN3, LOW); digitalWrite(M2_IN4, HIGH);
  digitalWrite(M3_IN1, LOW); digitalWrite(M3_IN2, HIGH);
  digitalWrite(M4_IN3, LOW); digitalWrite(M4_IN4, HIGH);
  
  analogWrite(M1_ENA, velocidad); analogWrite(M2_ENB, velocidad);
  analogWrite(M3_ENC, velocidad); analogWrite(M4_END, velocidad);
  delay(3000);
}

void desplazarIzquierda(int velocidad) {
  digitalWrite(M1_IN1, HIGH); digitalWrite(M1_IN2, LOW); // M1 Adelante
  digitalWrite(M3_IN1, HIGH); digitalWrite(M3_IN2, LOW); // M3 Adelante
  digitalWrite(M2_IN3, LOW); digitalWrite(M2_IN4, HIGH); // M2 Atrás
  digitalWrite(M4_IN3, LOW); digitalWrite(M4_IN4, HIGH); // M4 Atrás
  
  analogWrite(M1_ENA, velocidad); analogWrite(M2_ENB, velocidad);
  analogWrite(M3_ENC, velocidad); analogWrite(M4_END, velocidad);
  delay(3000);
}

void desplazarDerecha(int velocidad) {
  digitalWrite(M1_IN1, LOW); digitalWrite(M1_IN2, HIGH); // M1 Atrás
  digitalWrite(M3_IN1, LOW); digitalWrite(M3_IN2, HIGH); // M3 Atrás
  digitalWrite(M2_IN3, HIGH); digitalWrite(M2_IN4, LOW); // M2 Adelante
  digitalWrite(M4_IN3, HIGH); digitalWrite(M4_IN4, LOW); // M4 Adelante
  
  analogWrite(M1_ENA, velocidad); analogWrite(M2_ENB, velocidad);
  analogWrite(M3_ENC, velocidad); analogWrite(M4_END, velocidad);
  delay(3000);
}

void pivotarDerecha(int velocidad) {
  digitalWrite(M3_IN1, HIGH); digitalWrite(M3_IN2, LOW); // M3 Adelante
  digitalWrite(M4_IN3, HIGH); digitalWrite(M4_IN4, LOW); // M4 Adelante
  digitalWrite(M1_IN1, LOW); digitalWrite(M1_IN2, HIGH); // M1 Atrás
  digitalWrite(M2_IN3, LOW); digitalWrite(M2_IN4, HIGH); // M2 Atrás
  
  analogWrite(M1_ENA, velocidad); analogWrite(M2_ENB, velocidad);
  analogWrite(M3_ENC, velocidad); analogWrite(M4_END, velocidad);
  delay(3000);
}
