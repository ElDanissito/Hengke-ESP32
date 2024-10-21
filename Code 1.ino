#include <ModbusMaster.h>

#define RE_PIN 23  // Pin de control para Receive Enable (RE)
#define DE_PIN 22  // Pin de control para Drive Enable (DE)
#define RX_PIN 26  // Pin RX de ESP32
#define TX_PIN 27  // Pin TX de ESP32

ModbusMaster node;  // Crear un objeto ModbusMaster

// Funciones de control de transmisión/recepción
void preTransmission() {
  digitalWrite(RE_PIN, 1);  // Activar modo transmisión
  digitalWrite(DE_PIN, 1);  // Activar modo transmisión
}

void postTransmission() {
  digitalWrite(RE_PIN, 0);  // Activar modo recepción
  digitalWrite(DE_PIN, 0);  // Activar modo recepción
}

void setup() {
  // Configurar pines de control RE y DE
  pinMode(RE_PIN, OUTPUT);
  pinMode(DE_PIN, OUTPUT);
  
  // Iniciar comunicación serial
  Serial.begin(9600);  // Serial para depuración
  Serial2.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);  // Serial2 para Modbus (RS485)
  
  // Configurar el objeto Modbus
  node.begin(1, Serial2);  // Direccion Modbus 1 y utilizar Serial2 (RS485)
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);
}

void loop() {
  uint8_t result;
  uint16_t data[8];

  // Leer 8 registros de entrada desde la dirección 0x00
  result = node.readInputRegisters(0x00, 8);

  if (result == node.ku8MBSuccess) {
    // Si la lectura es exitosa, mostrar los datos
    for (int i = 0; i < 8; i++) {
      data[i] = node.getResponseBuffer(i);
      Serial.print("Registro ");
      Serial.print(i);
      Serial.print(": ");
      Serial.println(data[i]);
    }
  } else {
    // Si falla la lectura, mostrar un mensaje de error
    Serial.println("Error en la comunicación Modbus");
  }

  delay(1000);  // Esperar 1 segundo antes de la siguiente lectura
}
