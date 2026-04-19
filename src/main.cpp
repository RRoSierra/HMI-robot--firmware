// --- Asignación de Pines ---
#include <Arduino.h>
// Ajusta estos pines según la placa que estés utilizando
const int PIN_CHARGE = 5;       // Salida: Habilita la carga (CHARGE_3V)
const int PIN_KICKER = 6;       // Salida: Disparo (SIGNAL_KICKER_3V)
const int PIN_DONE   = 7;       // Entrada: Estado DONE
const int PIN_FAULT  = 8;       // Entrada: Estado FAULT
const int PIN_AN_DIV = A0;      // Entrada Analógica: AN_DIV_200V

// --- Parámetros del Hardware ---
// Cambiar a 5.0 y 1023.0 si usas un Arduino UNO/Nano de 5V
const float V_REF = 3.3;        
const float ADC_MAX = 4095.0;   // 12 bits (Ej. ESP32/STM32) 

// Relación del divisor de tensión: (R9 + R16) / R16 = (475k + 10k) / 10k = 48.5
const float DIVIDER_RATIO = 48.5; 

// --- Variables de Estado ---
bool isCharging = false;
unsigned long lastStatusTime = 0;
const unsigned long STATUS_INTERVAL = 1000; // Imprimir estado cada 1 seg

// --- Declaraciones Forward ---
void printMenu();
void handleSerialCommands();
void printStatus();

void setup() {
  Serial.begin(115200);
  
  // Configuración de pines
  pinMode(PIN_CHARGE, OUTPUT);
  pinMode(PIN_KICKER, OUTPUT);
  
  // Asumiendo que el LT3751 usa pines open-drain, activamos pull-up.
  // Si la PCB ya tiene resistencias pull-up, basta con INPUT.
  pinMode(PIN_DONE, INPUT_PULLUP); 
  pinMode(PIN_FAULT, INPUT_PULLUP);

  // Estados iniciales seguros
  digitalWrite(PIN_CHARGE, LOW);
  digitalWrite(PIN_KICKER, LOW);

  printMenu();
}

void loop() {
  handleSerialCommands();
  
  // Actualización periódica del estado sin bloquear el HMI
  if (millis() - lastStatusTime >= STATUS_INTERVAL) {
    printStatus();
    lastStatusTime = millis();
  }
}

// --- Funciones del HMI ---

void printMenu() {
  Serial.println("\n--- HMI PCB PATEADOR ---");
  Serial.println("Comandos disponibles:");
  Serial.println(" [c] - Alternar estado de CARGA (ON/OFF)");
  Serial.println(" [k] - Disparar KICKER (Pulso de 5ms)");
  Serial.println(" [s] - Forzar lectura de estado");
  Serial.println(" [h] - Mostrar este menú");
  Serial.println("------------------------\n");
}

void handleSerialCommands() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    
    // Limpiar el buffer de saltos de línea
    if (cmd == '\n' || cmd == '\r') return; 

    switch (cmd) {
      case 'c':
      case 'C':
        isCharging = !isCharging;
        digitalWrite(PIN_CHARGE, isCharging ? HIGH : LOW);
        Serial.print(">>> COMANDO: Carga ");
        Serial.println(isCharging ? "ACTIVADA" : "DESACTIVADA");
        break;

      case 'k':
      case 'K':
        Serial.println(">>> COMANDO: ¡DISPARO!");
        // Secuencia de disparo
        digitalWrite(PIN_KICKER, HIGH);
        delay(5); // Tiempo de activación del solenoide (ajusta con cuidado)
        digitalWrite(PIN_KICKER, LOW);
        break;

      case 's':
      case 'S':
        printStatus();
        break;
        
      case 'h':
      case 'H':
        printMenu();
        break;

      default:
        Serial.println("Comando no reconocido.");
        break;
    }
  }
}

void printStatus() {
  // Leer valores
  int rawADC = analogRead(PIN_AN_DIV);
  int stateDone = digitalRead(PIN_DONE);
  int stateFault = digitalRead(PIN_FAULT);

  // Calcular voltaje real del capacitor
  float voltagePin = (rawADC / ADC_MAX) * V_REF;
  float voltageCap = voltagePin * DIVIDER_RATIO;

  // Imprimir telemetría
  Serial.print("[ESTADO] ");
  Serial.print("V_Cap: "); 
  Serial.print(voltageCap, 1); 
  Serial.print("V | ");
  
  Serial.print("CHARGE_CMD: "); 
  Serial.print(isCharging ? "ON " : "OFF"); 
  Serial.print(" | ");

  // Nota: LT3751 suele activar DONE/FAULT enviándolos a GND (lógica inversa)
  Serial.print("DONE: "); 
  Serial.print(stateDone == LOW ? "SI" : "NO"); 
  Serial.print(" | ");
  
  Serial.print("FAULT: "); 
  Serial.println(stateFault == LOW ? "ERROR" : "OK");
}