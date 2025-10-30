#include <SPI.h>
#include <mcp2515.h>

struct can_frame canMsg;
MCP2515 mcp2515(10);  // Pin CS del MCP2515
const uint16_t ID_RESPUESTA = 0x200;
const uint16_t ID_FILTER = 0x100;


void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("=== Receptor CAN (RX) ===");

  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

  randomSeed(analogRead(A0));
}

void responderAlRTR(uint16_t id) {
  struct can_frame resp;
  resp.can_id = id;      // Mismo ID del RTR
  resp.can_dlc = 2;
  int valor = random(20, 35);  // Valor simulado (temperatura en °C)
  resp.data[0] = valor >> 8;
  resp.data[1] = valor & 0xFF;

  mcp2515.sendMessage(&resp);

  Serial.print("Respondido al RTR ID 0x");
  Serial.print(id, HEX);
  Serial.print(" con valor ");
  Serial.println(valor);
}

void loop() {
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    uint16_t id = canMsg.can_id & CAN_SFF_MASK;

    if (canMsg.can_id & CAN_RTR_FLAG) {
      // Es una solicitud remota
      if (id == ID_RESPUESTA) {
        Serial.print("RTR recibido para ID 0x");
        Serial.println(id, HEX);
        responderAlRTR(id);
      }
    } else {

      if(id == ID_FILTER){
      
      // Es un mensaje normal (solo mostramos si quieres debug)
      Serial.print("Dato recibido ID 0x");
      Serial.print(id, HEX);
      Serial.print(" → ");
      for (byte i = 0; i < canMsg.can_dlc; i++) {
        Serial.print(canMsg.data[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
      }
    }
  }
}
