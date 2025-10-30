#include <SPI.h>
#include <mcp2515.h>

struct can_frame canMsg;
MCP2515 mcp2515(10);  // Pin CS del MCP2515

// Agrega delimitadores de inicio y fin
const char *mensaje = "<WSN Grupo 3>";  
const uint16_t ID_MENSAJE = 0x200;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("Iniciando MCP2515 a 125 kbps, 8 MHz...");
  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
  Serial.println("CAN MCP2515 configurado y en modo normal.");
}

void enviarFrase(const char *frase) {
  int len = strlen(frase);
  int index = 0;
  int trama = 0;

  while (index < len) {
    canMsg.can_id = ID_MENSAJE;
    canMsg.can_dlc = 0;

    for (int i = 0; i < 8 && index < len; i++) {
      canMsg.data[i] = frase[index++];
      canMsg.can_dlc++;
    }

    mcp2515.sendMessage(&canMsg);

    Serial.print("Trama ");
    Serial.print(trama++);
    Serial.print(": ");
    for (int i = 0; i < canMsg.can_dlc; i++) {
      Serial.write(canMsg.data[i]);
      Serial.print(" ");
    }
    Serial.println();
    delay(300);
  }

  Serial.println("Mensaje completo enviado.\n");
}

void loop() {
  enviarFrase(mensaje);
  delay(5000);
}
