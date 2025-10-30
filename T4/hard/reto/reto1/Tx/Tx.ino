#include <SPI.h>
#include <mcp2515.h>

struct can_frame canMsg;
MCP2515 mcp2515(10);  // Pin CS del MCP2515

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("=== Transmisor CAN (TX) ===");

  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

  randomSeed(analogRead(A0));
}

void enviarTemperatura(uint16_t id) {
  int temperatura = random(0, 1023);

  canMsg.can_id  = id;
  canMsg.can_dlc = 2;
  canMsg.data[0] = temperatura >> 8;
  canMsg.data[1] = temperatura & 0xFF;

  mcp2515.sendMessage(&canMsg);

  Serial.print("DATA → ID 0x");
  Serial.print(id, HEX);
  Serial.print("  Valor = ");
  Serial.println(temperatura);
}

void enviarRTR(uint16_t id, uint8_t dlc) {
  struct can_frame rtrMsg;
  rtrMsg.can_id  = id | CAN_RTR_FLAG;  // Marca como RTR
  rtrMsg.can_dlc = dlc;

  mcp2515.sendMessage(&rtrMsg);

  Serial.print("Enviado RTR para ID 0x");
  Serial.println(id, HEX);
}

void loop() {
  // Envío de datos normales
  enviarTemperatura(0x100);
  enviarTemperatura(0x110);
  enviarTemperatura(0x120);

  // Envío de solicitud RTR (pide datos al RX)
  enviarRTR(0x200, 2);

  // Espera una posible respuesta del RX
  unsigned long start = millis();
  bool respuesta_recibida = false;

  while (millis() - start < 500) {  // Espera hasta 500 ms
    if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
      if (!(canMsg.can_id & CAN_RTR_FLAG) && ((canMsg.can_id & CAN_SFF_MASK) == 0x200)) {
        Serial.print("RESPUESTA recibida ID 0x");
        Serial.print(canMsg.can_id & CAN_SFF_MASK, HEX);
        Serial.print(" → Valor = ");
        int valor = (canMsg.data[0] << 8) | canMsg.data[1];
        Serial.println(valor);
        respuesta_recibida = true;
        break;
      }
    }
  }

  if (!respuesta_recibida) {
    Serial.println("No se recibió respuesta al RTR.");
  }

  Serial.println("----------------------------");
  delay(1000);
}
