#include "brujula.h"
#include "motores.h"

void setup() {
  Serial.begin(115200);
  
  Serial.println("Iniciando componentes...");
  inicializarBrujula();
  inicializarMotores();
  
  frenar();
  Serial.println("¡Robot Listo! Apuntalo hacia tu frente...");
  Serial.println("Arrancando en 3 segundos...");
  delay(3000);
}

void loop() {
  // Las 4 direcciones hacia donde el robot empujará su cuerpo
  // 0=Adelante, 90=Derecha, 180=Atrás, 270=Izquierda
  float direcciones_traslacion[] = {0, 90, 180, 270};
  
  // El robot SIEMPRE intentará mirar hacia su 0° original
  float heading_objetivo = 0; 
  
  int velocidad_avance = 180; // Ajusta esto entre 0 y 255
  unsigned long tiempo_avance = 1500; // 1.5 segundos por lado del cuadrado
  
  // Recorremos los 4 lados del cuadrado
  for (int i = 0; i < 4; i++) {
    float direccion = direcciones_traslacion[i];
    
    Serial.println("====================================");
    Serial.print("Lado "); Serial.print(i + 1);
    Serial.print(": Deslizandose hacia los ");
    Serial.print(direccion);
    Serial.println(" grados.");
    
    unsigned long inicio = millis();
    
    // Mientras no se cumpla el segundo y medio, seguimos moviéndonos
    while (millis() - inicio < tiempo_avance) {
      
      // 1. Leemos dónde está mirando físicamente el robot
      float actual = getHeading360();
      
      // 2. Calculamos el error (-180 a 180) respecto a nuestro objetivo (0°)
      float error_PID = calcularErrorCorto(heading_objetivo, actual);
      
      // 3. ¡La Magia del X-Drive!
      // Le mandamos la "direccion" de traslación, la velocidad, y el error de giro
      moverEnAngulo(direccion, velocidad_avance, error_PID);
      
      // Imprimimos la telemetría para que veas cómo la brújula corrige en tiempo real
      Serial.print("Brújula: "); Serial.print(actual);
      Serial.print(" | Error corrigiendo: "); Serial.println(error_PID);
      
      delay(15);
    }
    
    // Al terminar el lado del cuadrado, frenamos 1 segundo
    frenar();
    delay(1000);
  }
  
  Serial.println("Cuadrado omnidireccional terminado. Reiniciando en 3 segundos...");
  delay(3000);
}