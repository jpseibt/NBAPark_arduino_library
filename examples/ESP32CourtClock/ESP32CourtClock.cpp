/*
 * NBA Park Arduino Library (ESP32)
 * Description: Example program for the NBA Park court's game clock. It sends an OSC message via an OSCPark obj to a PC on the NBA House,
                which triggers a buzzer sound track when the clock time reaches zero.
 * Author: José Paulo Seibt Neto
 * Created: Nov - 2025
 * Last Modified: Feb - 2026
 * -> Work in progress...
*/

#define DEBUG_LEVEL 1 /* 0 = None, 1 = Sketch only, 2 = Library only, 3 = Sketch and Library */

#include <NBAPark.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_task_wdt.h>

#define JP_ADDRESS "/composition/layers/3/clips/2/connect"
#define BUZZER_PIN 23
#define CONNECTION_TIMEOUT 60 /* Amount of time (sec) until rebooting if WiFi connection fail on setup() */
#define WDOG_TIMEOUT_AMT 8    /* Watch Dog timeout (secs) */
#define AMT_STATE_CHECKS 3    /* Amount of checks needed to trigger the buzzer (OSC send) */
#define STATE_CHECK_DELAY 80  /* Delay (ms) between checks (total delay = AMT_STATE_CHECKS * STATE_CHECK_DELAY) */

// Prototype
void send_msg_to_resolume();

// Network Credentials
static const char* ssid = "FOO";
static const char* password = "foo_bar_baz";

// Static IP Configuration
static const IPAddress board_ip(255, 255, 255, 255);
static const IPAddress gateway(255, 255, 255, 255);
static const IPAddress subnet_mask(255, 255, 255, 255);
static const IPAddress primary_dns(255, 255, 255, 255);

// Resolume IP/OSC Ports
static const IPAddress resolume_ip(255, 255, 255, 255);
static const int resolume_in_port = 7000;

static WiFiUDP udp;
static uint8_t state_count;
static uint8_t last_state;
static uint8_t state;
static bool trigger_buzzer;
static Timer timer;
static Timer wifi_down_timer;


void setup()
{
    Serial.begin(115200);
    timer.reset();
    wifi_down_timer.reset();

    // WiFi setup
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.setAutoReconnect(true);
    WiFi.setHostname("Court-Clock");
    WiFi.config(board_ip, gateway, subnet_mask, primary_dns);
    WiFi.begin(ssid, password);

    debugSkt("Connecting");

    while (WiFi.status() != WL_CONNECTED)
    {
        timer.hang(500);
        debugSkt(".");

        if (timer.get_elapsed_time(SECONDS) > CONNECTION_TIMEOUT)
        {
            debugSkt("\nWiFi connection timeout. Rebooting...\n");
            timer.hang(1000);
            ESP.restart();
        }
    }
    debugSkt("\n---Connected!---\n");

    pinMode(BUZZER_PIN, INPUT_PULLUP);

    state_count = 0;
    last_state = HIGH;
    state = HIGH;
    trigger_buzzer = false;

    // Watch dog setup
    esp_task_wdt_init(WDOG_TIMEOUT_AMT, true);
    esp_task_wdt_add(NULL);
}

// Prints on the Serial Monitor information about receiving OSC messages for DELAY seconds then pauses the loop for another DELAY seconds
void loop()
{
    esp_task_wdt_reset();

    state = digitalRead(BUZZER_PIN);
    //debugSkt(state);
    //debugSktln();

    // Check WiFi connection and print status every 4 seconds
    if (timer.get_elapsed_time(SECONDS) > 4)
    {
        timer.reset();
        if (WiFi.status() != WL_CONNECTED)
        {
            Serial.print("-WiFi=0");
            if (wifi_down_timer.get_elapsed_time(SECONDS) > CONNECTION_TIMEOUT)
            {
                debugSkt("\nWiFi reconnection timeout. Rebooting...\n");
                timer.hang(1000);
                ESP.restart();
            }
        }
        else
        {
            Serial.print("+WiFi=1");
            wifi_down_timer.reset();
        }
        Serial.printf(" Pin=%d Buzzer=%d\n", state, trigger_buzzer);
    }

    if (trigger_buzzer == true)
    {
        if (WiFi.status() == WL_CONNECTED)
        {
            send_msg_to_resolume();
            trigger_buzzer = false;
        }
    }

    if (trigger_buzzer == false && last_state == HIGH && state == LOW)
    {
        state_count = 0;
        for (int i = 0; i < AMT_STATE_CHECKS; ++i)
        {
            timer.hang(STATE_CHECK_DELAY);
            if (digitalRead(BUZZER_PIN) == LOW)
            {
                ++state_count;
            }
            else
            {
                break;
            }
        }

        if (state_count == AMT_STATE_CHECKS)
        {
            trigger_buzzer = true;
            debugSkt("Buzzer queued.\n");
        }
    }

    last_state = state;
}


// Send OSC message to change the value in the High Score or Score text block in Resolume Arena
void send_msg_to_resolume()
{
    if (WiFi.status() != WL_CONNECTED) { return; }

    char osc_message_buffer[255] = {0};

    snprintf(osc_message_buffer, sizeof(osc_message_buffer), JP_ADDRESS);
    OSCPark msg(osc_message_buffer);

    debugSkt("Sending OSC to: ");
    debugSkt(JP_ADDRESS);
    debugSktln();

    // Send message through EthernetUDP global instance
    udp.beginPacket(resolume_ip, resolume_in_port);
    msg.send(udp);
    udp.endPacket();
}
