#include <Arduino.h>
#include <Ethernet.h>
#include <EthernetUDP.h>
#include <NBAPark.h>

#define WAIT_BTN 4
#define START_BTN 5
#define ACTIVE LOW

enum Command : uint8_t
{
  WAIT,
  START,
};

uint8_t curr_wait_state;
uint8_t last_wait_state;
uint8_t curr_start_state;
uint8_t last_start_state;

const char *WAIT_RESOLUME_ADDRESS  = "/composition/layers/1/clips/1/connect";
const char *START_RESOLUME_ADDRESS = "/composition/layers/1/clips/2/connect";

const IPAddress resolume_skl001(172, 30, 4, 106);
const IPAddress resolume_skl002(172, 30, 4, 107);
const IPAddress resolume_skl003(172, 30, 4, 108);
const int resolume_in_port = 7000;
const int resolume_out_port = 7001;

const uint8_t board_mac[] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0x99};
const IPAddress board_ip(172, 30, 4, 200);

EthernetUDP udp;

void setup()
{
  Serial.begin(115200);

  pinMode(WAIT_BTN, INPUT);
  pinMode(START_BTN, INPUT);

  curr_wait_state = 0;
  last_wait_state = 0;
  curr_start_state = 0;
  last_start_state = 0;

  Ethernet.begin(board_mac, board_ip);
  udp.begin(resolume_out_port);
}

void loop()
{
  curr_wait_state = digitalRead(WAIT_BTN);
  curr_start_state = digitalRead(START_BTN);

  if (curr_wait_state == ACTIVE && last_wait_state != ACTIVE)
  {
    send_click_to_resolume(WAIT);
    delay(100);
  }
  else if (curr_start_state == ACTIVE && last_start_state != ACTIVE)
  {
    send_click_to_resolume(START);
    delay(100);
  }

  last_wait_state = curr_wait_state;
  last_start_state = curr_start_state;
}

bool send_click_to_resolume(Command in_cmd)
{
  debugSkt("[send_click_to_resolume] ");
  bool result = true;

  char *address = 0;
  switch (in_cmd)
  {
    case START:
      debugSkt("START\n");
      address = START_RESOLUME_ADDRESS;
      break;
    case WAIT:
      debugSkt("WAIT\n");
      address = WAIT_RESOLUME_ADDRESS;
      break;
    default:
      debugSkt("Invalid Command given\n");
      result = false;
  }

  OSCPark msg(address);
  msg.set_int(1);

  // Send message through EthernetUDP global instance
  udp.beginPacket(resolume_skl001, resolume_in_port);
  msg.send(udp);
  udp.endPacket();

  udp.beginPacket(resolume_skl002, resolume_in_port);
  msg.send(udp);
  udp.endPacket();

  udp.beginPacket(resolume_skl003, resolume_in_port);
  msg.send(udp);
  udp.endPacket();

  return result;
}
