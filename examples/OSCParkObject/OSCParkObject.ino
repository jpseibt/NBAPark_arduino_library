/*
 * NBA Park Arduino Library
 * Description: Example program that will parse UDP packets and instantiate OSCPark obj
 * Author: José Paulo Seibt Neto
 * Created: Mar - 2025
 * Last Modified: Apr - 2025
*/

#include <NBAPark.h>
#include <Ethernet.h>
#include <EthernetUDP.h>

const uint8_t board_mac[] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0x04};
const IPAddress board_ip(172, 30, 6, 200);
const IPAddress pc_ip(172, 30, 6, 58);

const int resolume_in_port = 7000;
const int resolume_out_port = 7001;

EthernetUDP udp;

uint8_t osc_msg_buffer[255];
OSCPark osc_msg;

Timer timer;
const uint8_t DELAY = 20;

void setup()
{
    Serial.begin(115200);

    Ethernet.begin(board_mac, board_ip);
    udp.begin(resolume_out_port);

    osc_msg.clear();
    timer.reset();
}

// Prints on the Serial Monitor information about receiving OSC messages for DELAY seconds then pauses the loop for another DELAY seconds
void loop()
{
    if (!udp.parsePacket())
    {
        osc_msg.print();
        osc_msg.info();
    }
    else
    {
        udp.read(osc_msg_buffer, sizeof(osc_msg_buffer));
        osc_msg.init(osc_msg_buffer);
        osc_msg.print();
        osc_msg.info();
        osc_msg.clear();
    }
    Serial.println("/ / / / / / / / / / / / / / / / / / / / / / / / /");

    if (timer.get_elapsed_time() >= DELAY)
    {
        while (timer.get_elapsed_time() < DELAY * 2) { delay(1); }
        timer.reset();
        loop();
    }
}
