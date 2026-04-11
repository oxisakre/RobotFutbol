// --- DEFINICIÓN DE PINES (Teensy) ---
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

// --- UMBRALES MÍNIMOS DE ARRANQUE (Deadband) medidos en test ---
const int MIN_M1 = 172;
const int MIN_M2 = 180;
const int MIN_M3 = 170;
const int MIN_M4 = 148;

void setup() {
  // Configuración de los pines de DIRECCIÓN 
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
  digitalWrite(PIN_KICK, LOW); // Arranca apagado por seguridad
  
  frenar(); // Asegura que los motores arranquen quietos
}

void loop() {
  delay(5000); // Espera 5 segundos antes de arrancar

  // --- 1. NUEVA PRUEBA LINEAL ---
  // Avanza hacia adelante (0°) a toda velocidad
  moverEnAngulo(0, 255); 
  delay(2000);
  frenar();
  delay(1000); // Pausa corta

  // Vuelve hacia atrás (180°) a mínima velocidad
  // Al pasarle '1', usará los pisos mínimos (172, 180...)
  moverEnAngulo(180, 1); 
  delay(2000);
  frenar();
  delay(2000); // Pausa más larga antes de pasar a lo siguiente

  // --- 2. RUTINA DE TESTEO EN DIAGONALES ---
  // Elegimos 4 ángulos para probar
  float angulos_test[] = {108, -100, -21, 65}; 
  int cantidad_angulos = 4;

  for (int i = 0; i < cantidad_angulos; i++) {
    float angulo_actual = angulos_test[i];

    // Prueba a velocidad máxima
    moverEnAngulo(angulo_actual, 255); 
    delay(2000); 
    frenar();
    delay(1000); 

    // Prueba a velocidad mínima
    moverEnAngulo(angulo_actual, 1); 
    delay(2000); 
    frenar();
    delay(2000); 
  }

  // Termina la coreografía con una demostración del pateador y el giro
  patear();
  pivotarDerecha(180);
  frenar();

  // Detiene el test al terminar
  while(1); 
}


// ==========================================
//        FUNCIÓN MAESTRA DE MOVIMIENTO
// ==========================================

void moverEnAngulo(float angulo_grados, int velocidad) {
  // 1. Convertir grados a radianes
  float radianes = angulo_grados * (PI / 180.0);

  // 2. Calcular ejes (0° es Adelante)
  float Vy = cos(radianes);
  float Vx = -sin(radianes);

  // 3. Ecuaciones cinemáticas del X-Drive
  float vel_M1 = Vy - Vx;
  float vel_M2 = Vy + Vx;
  float vel_M3 = Vy - Vx;
  float vel_M4 = Vy + Vx;

  // 4. Normalización para no superar proporciones (1.0)
  float max_vel = max(max(abs(vel_M1), abs(vel_M2)), max(abs(vel_M3), abs(vel_M4)));
  if (max_vel > 1.0) {
    vel_M1 /= max_vel;
    vel_M2 /= max_vel;
    vel_M3 /= max_vel;
    vel_M4 /= max_vel;
  }

  // 5. Compensación de Zona Muerta
  // Convertimos la velocidad pedida (0-255) en un ratio (0.0 a 1.0)
  float ratio = constrain(velocidad, 0, 255) / 255.0;
  
  int pwm_M1 = 0, pwm_M2 = 0, pwm_M3 = 0, pwm_M4 = 0;
  
  // Si el motor tiene que moverse, mapeamos desde su piso mínimo hasta 255
  if (abs(vel_M1) > 0.01) pwm_M1 = MIN_M1 + round(abs(vel_M1) * ratio * (255 - MIN_M1));
  if (abs(vel_M2) > 0.01) pwm_M2 = MIN_M2 + round(abs(vel_M2) * ratio * (255 - MIN_M2));
  if (abs(vel_M3) > 0.01) pwm_M3 = MIN_M3 + round(abs(vel_M3) * ratio * (255 - MIN_M3));
  if (abs(vel_M4) > 0.01) pwm_M4 = MIN_M4 + round(abs(vel_M4) * ratio * (255 - MIN_M4));

  // 6. Enviar señales a los pines
  // MOTOR 1
  if (vel_M1 >= 0) { digitalWrite(M1_IN1, HIGH); digitalWrite(M1_IN2, LOW); } 
  else { digitalWrite(M1_IN1, LOW); digitalWrite(M1_IN2, HIGH); }
  analogWrite(M1_ENA, pwm_M1);

  // MOTOR 2
  if (vel_M2 >= 0) { digitalWrite(M2_IN3, HIGH); digitalWrite(M2_IN4, LOW); } 
  else { digitalWrite(M2_IN3, LOW); digitalWrite(M2_IN4, HIGH); }
  analogWrite(M2_ENB, pwm_M2);

  // MOTOR 3
  if (vel_M3 >= 0) { digitalWrite(M3_IN1, HIGH); digitalWrite(M3_IN2, LOW); } 
  else { digitalWrite(M3_IN1, LOW); digitalWrite(M3_IN2, HIGH); }
  analogWrite(M3_ENC, pwm_M3);

  // MOTOR 4
  if (vel_M4 >= 0) { digitalWrite(M4_IN3, HIGH); digitalWrite(M4_IN4, LOW); } 
  else { digitalWrite(M4_IN3, LOW); digitalWrite(M4_IN4, HIGH); }
  analogWrite(M4_END, pwm_M4);
}


// ==========================================
//        FUNCIONES AUXILIARES
// ==========================================

void pivotarDerecha(int velocidad) {
  digitalWrite(M3_IN1, HIGH); digitalWrite(M3_IN2, LOW); // M3 Adelante
  digitalWrite(M4_IN3, HIGH); digitalWrite(M4_IN4, LOW); // M4 Adelante
  digitalWrite(M1_IN1, LOW); digitalWrite(M1_IN2, HIGH); // M1 Atrás
  digitalWrite(M2_IN3, LOW); digitalWrite(M2_IN4, HIGH); // M2 Atrás
  
  // En giro enviamos la velocidad directamente
  analogWrite(M1_ENA, velocidad); analogWrite(M2_ENB, velocidad);
  analogWrite(M3_ENC, velocidad); analogWrite(M4_END, velocidad);
  delay(3000);
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
  delay(7000); 
}