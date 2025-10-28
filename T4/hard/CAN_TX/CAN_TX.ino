#include <SPI.h>
#include <mcp2515.h>

struct can_frame canMsg;
MCP2515 mcp2515(10);  // Pin CS del MCP2515 (usualmente 10 en Arduino UNO)

uint8_t contador = 0; // contador de 1 byte

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("Iniciando MCP2515 a 125 kbps, 8 MHz...");

  mcp2515.reset();

  // Configura bitrate y frecuencia de cristal
  mcp2515.setBitrate(CAN_125KBPS, MCP_8MHZ);   // Cambia MCP_8MHZ por MCP_16MHZ si tu módulo es de 16 MHz
  mcp2515.setNormalMode();

  Serial.println("CAN MCP2515 configurado y en modo normal.");
}

void loop() {
  // Prepara mensaje CAN estándar (11-bit ID)
  canMsg.can_id  = 0x123;   // ID del mensaje
  canMsg.can_dlc = 1;       // longitud de datos (1 byte)
  canMsg.data[0] = contador;

  // Envía el mensaje
  mcp2515.sendMessage(&canMsg);

  // Imprime por serial
  Serial.print("Enviado ID 0x");
  Serial.print(canMsg.can_id, HEX);
  Serial.print(" -> contador = ");
  Serial.println(contador);

  // Incrementa el contador (overflow automático a 0)
  contador++;

  delay(500);
}
