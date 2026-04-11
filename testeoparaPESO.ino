// --- DEFINICIÓN DE PINES (Teensy) ---
const int M1_IN1 = 2; const int M1_IN2 = 1; const int M1_ENA = 3;
const int M2_IN3 = 4; const int M2_IN4 = 5; const int M2_ENB = 0;
const int M3_IN1 = 17; const int M3_IN2 = 16; const int M3_ENC = 19;
const int M4_IN3 = 14; const int M4_IN4 = 15; const int M4_END = 13;

void setup() {
  Serial.begin(115200);
  
  // Configuración de pines
  int pines_dir[] = {M1_IN1, M1_IN2, M2_IN3, M2_IN4, M3_IN1, M3_IN2, M4_IN3, M4_IN4};
  for(int i=0; i<8; i++) pinMode(pines_dir[i], OUTPUT); 
  
  pinMode(M1_ENA, OUTPUT); pinMode(M2_ENB, OUTPUT);
  pinMode(M3_ENC, OUTPUT); pinMode(M4_END, OUTPUT);

  apagarTodos();

  // Esperar un poco a que abra el puerto serie y mostrar menú
  delay(2000);
  Serial.println("===================================");
  Serial.println("   MODO CALIBRACION DE MOTORES");
  Serial.println("===================================");
  Serial.println("Escribe: [Motor],[PWM]");
  Serial.println("Ejemplo: 1,40 (Arranca el Motor 1 a velocidad 40)");
  Serial.println("Para apagar todo escribe: 0,0");
  Serial.println("-----------------------------------");
}

void loop() {
  // Verifica si enviaste algo por el Monitor Serie
  if (Serial.available() > 0) {
    // Lee el primer número (Motor) y el segundo (PWM) separados por coma
    int motor = Serial.parseInt();
    int pwm = Serial.parseInt();

    // Limpia cualquier salto de línea sobrante en el buffer
    while(Serial.available() > 0) Serial.read();

    if (motor >= 1 && motor <= 4) {
      Serial.print("[*] Testeando Motor "); Serial.print(motor);
      Serial.print(" a PWM: "); Serial.println(pwm);
      probarMotor(motor, pwm);
    } else if (motor == 0) {
      Serial.println("[!] Freno total activado.");
      apagarTodos();
    } else {
      Serial.println("[!] Error: El motor debe ser 1, 2, 3 o 4.");
    }
  }
}

// ==========================================
//        FUNCIONES AUXILIARES
// ==========================================

void probarMotor(int motor, int pwm) {
  apagarTodos(); // Apaga todo primero por seguridad
  
  // Evita que mandes un PWM mayor al límite
  int velocidad = constrain(pwm, 0, 255);

  if (motor == 1) { 
    digitalWrite(M1_IN1, HIGH); digitalWrite(M1_IN2, LOW); 
    analogWrite(M1_ENA, velocidad); 
  }
  if (motor == 2) { 
    digitalWrite(M2_IN3, HIGH); digitalWrite(M2_IN4, LOW); 
    analogWrite(M2_ENB, velocidad); 
  }
  if (motor == 3) { 
    digitalWrite(M3_IN1, HIGH); digitalWrite(M3_IN2, LOW); 
    analogWrite(M3_ENC, velocidad); 
  }
  if (motor == 4) { 
    digitalWrite(M4_IN3, HIGH); digitalWrite(M4_IN4, LOW); 
    analogWrite(M4_END, velocidad); 
  }
}

void apagarTodos() {
  digitalWrite(M1_IN1, LOW); digitalWrite(M1_IN2, LOW);
  digitalWrite(M2_IN3, LOW); digitalWrite(M2_IN4, LOW);
  digitalWrite(M3_IN1, LOW); digitalWrite(M3_IN2, LOW);
  digitalWrite(M4_IN3, LOW); digitalWrite(M4_IN4, LOW);
  
  analogWrite(M1_ENA, 0); analogWrite(M2_ENB, 0);
  analogWrite(M3_ENC, 0); analogWrite(M4_END, 0);
}