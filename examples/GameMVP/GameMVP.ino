/*
 * NBA Park Arduino Library
 * Description: Example program that reads custom OSC messages send by Resolume Arena to correctly read values from three BasketSensor objs
                working in conjunction with a MVPHoops instance to count basketballs that go through the rim at specific times based
                on the clip projected in the Resolume composition, displaying the score count of the game in real time.
 * Author: José Paulo Seibt Neto
 * Created: Mar - 2025
 * Last Modified: Sep - 2025
*/

#include <Arduino.h>
#include <NBAPark.h>
#include <Ethernet.h>
#include <EthernetUDP.h>
#include <string.h>       // strncmp

#define RSTPIN A5 // Pin number used to trigger the board RESET pin

// Time
Timer high_score_timer;  // Instance used to reset the high score of the game or reset the Arduino (sending LOW to RESET pin) based on elapsed time
Timer game_timer;        // Instance used to setup game logic such as the valid sensors to check at any given time
uint32_t now;


// MVP hoops and sensors
MVPHoops mvp_hoops;
BitmapPattern curr_mvp_pattern;
MVPHoops::MVPState mvp_state;

const uint8_t ir_out_pins[] = {2, 4, 6};

IRBasketSensor baskets[] = {
    IRBasketSensor(ir_out_pins[0]),
    IRBasketSensor(ir_out_pins[1]),
    IRBasketSensor(ir_out_pins[2])
};

const MVPHoops::Layout test_layouts[] = {
    MVPHoops::Layout(5, BitmapPattern::LAYOUT_1),  // 0
    MVPHoops::Layout(12, BitmapPattern::LAYOUT_5), // 1
    MVPHoops::Layout(14, BitmapPattern::LAYOUT_4), // 2
    MVPHoops::Layout(21, BitmapPattern::LAYOUT_6), // 3
    MVPHoops::Layout(23, BitmapPattern::LAYOUT_2), // 4
    MVPHoops::Layout(27, BitmapPattern::LAYOUT_3), // 5
    MVPHoops::Layout(29, BitmapPattern::LAYOUT_1), // 6
    MVPHoops::Layout(33, BitmapPattern::LAYOUT_3), // 7
    MVPHoops::Layout(35, BitmapPattern::LAYOUT_2), // 8
    MVPHoops::Layout(41, BitmapPattern::LAYOUT_6), // 9
    MVPHoops::Layout(43, BitmapPattern::LAYOUT_4), // 10
    MVPHoops::Layout(45, BitmapPattern::LAYOUT_6), // 11
    MVPHoops::Layout(47, BitmapPattern::LAYOUT_2), // 12
    MVPHoops::Layout(50, BitmapPattern::LAYOUT_3), // 13
    MVPHoops::Layout(52, BitmapPattern::LAYOUT_1), // 14
    MVPHoops::Layout(54, BitmapPattern::LAYOUT_5), // 15
    MVPHoops::Layout(56, BitmapPattern::LAYOUT_4), // 16
    MVPHoops::Layout(59, BitmapPattern::LAYOUT_6), // 17
    MVPHoops::Layout(61, BitmapPattern::LAYOUT_2), // 18
    MVPHoops::Layout(64, BitmapPattern::LAYOUT_6), // 19
    MVPHoops::Layout(66, BitmapPattern::LAYOUT_4), // 20
    MVPHoops::Layout(68, BitmapPattern::LAYOUT_6), // 21
    MVPHoops::Layout(60, BitmapPattern::LAYOUT_2), // 22
    MVPHoops::Layout(72, BitmapPattern::LAYOUT_6), // 23
    MVPHoops::Layout(74, BitmapPattern::LAYOUT_4), // 24
    MVPHoops::Layout(77, BitmapPattern::LAYOUT_5), // 25
    MVPHoops::Layout(79, BitmapPattern::LAYOUT_1), // 26
    MVPHoops::Layout(80, BitmapPattern::LAYOUT_5), // 27
    MVPHoops::Layout(82, BitmapPattern::LAYOUT_7), // 28
    MVPHoops::Layout(93, BitmapPattern::LAYOUT_0), // 29
    MVPHoops::Layout(102, BitmapPattern::LAYOUT_STOP),
};


// Stat tracking
bool new_high_score;
uint16_t high_score_count;
uint16_t score_count;

// OSC messages and network configuration
uint8_t osc_message_buffer[255];
EthernetUDP udp;
const uint8_t board_mac[] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0x05};
const IPAddress board_ip(172, 30, 6, 199);
const IPAddress pc_ip(172, 30, 6, 58);
const int resolume_in_port = 7000;
const int resolume_out_port = 7001;

enum ScoreType : uint8_t
{
    SCORE,
    HIGH_SCORE
};

// Prototypes
bool send_score_to_resolume(ScoreType in_type, uint16_t in_score);
void reset_game_state();
void game_update();

void setup()
{
    digitalWrite(RSTPIN, HIGH); // Keep a weakly HIGH state on RSTPIN as the board RESET pin only triggers when it is pulled LOW

    Serial.begin(115200);
    debugSkt("[GameMVP.ino] setup\n");

    mvp_hoops.init(test_layouts, sizeof(test_layouts) / sizeof(test_layouts[0]));
    curr_mvp_pattern = mvp_hoops.get_curr_pattern();
    mvp_state = MVPHoops::MVPState::MVP_GAME_OVER;

    Ethernet.begin(board_mac, board_ip);
    udp.begin(resolume_out_port);

    high_score_timer.reset();
    now = game_timer.reset();

    new_high_score = false;
    high_score_count = DEFAULT_HIGH_SCORE;
    score_count = 0;
}

void loop()
{
    // Check for new messages
    if (udp.parsePacket())
    {
        udp.read(osc_message_buffer, sizeof(osc_message_buffer));
        OSCPark msg(osc_message_buffer);

        if (strncmp(msg.get_addr_cmp(), RESOLUME_MVPGAME_ADDRESS, OSC_MAX_ADDRESS_LEN) == 0)
        {
            if (mvp_state == MVPHoops::MVPState::MVP_GAME_OVER)
            {
                reset_game_state();
                send_score_to_resolume(SCORE, 0);
                send_score_to_resolume(HIGH_SCORE, high_score_count);
            }
        }
        else if (strncmp(msg.get_addr_cmp(), RESOLUME_MVPWAIT_ADDRESS, OSC_MAX_ADDRESS_LEN) == 0)
        {
            debugSkt("GOT RESOLUME_MVPWAIT_ADDRESS\n");
            mvp_state = MVPHoops::MVPState::MVP_GAME_OVER;
        }
        else if (strncmp(msg.get_addr_cmp(), MVP_HARD_RESET_OSC, OSC_MAX_ADDRESS_LEN) == 0)
        {   // Hard reset triggered, reseting board (message can be send by Bitfocus Companion)
            debugSkt("GOT MVP_HARD_RESET_OSC\n"); delay(500);
            pinMode(RSTPIN, OUTPUT);
            digitalWrite(RSTPIN, LOW);
        }
    }

    if (mvp_state == MVPHoops::MVPState::MVP_GAME_OVER)
    {
        // Check if it is time to reset the current highest score
        if (high_score_timer.get_elapsed_time() > HIGH_SCORE_RESET_TIME || high_score_count > 100)
        {
            high_score_count = DEFAULT_HIGH_SCORE;
            high_score_timer.reset();
            debugSkt("HIGH SCORE RESET...\n");
        }
        debugSkt("WAITING RESOLUME_MVPGAME_ADDRESS\n");
    }
    else
    {   // GAME
        now = game_timer.get_elapsed_time();
        mvp_state = mvp_hoops.update(now);
        curr_mvp_pattern = mvp_hoops.get_curr_pattern();

        if (mvp_state == MVPHoops::MVPState::MVP_HOLD)
        {
            debugSkt("mvp_state: MVP_HOLD\n");
        }
        else if (mvp_state == MVPHoops::MVPState::MVP_GAME_OVER)
        {
            debugSkt("mvp_state: MVP_GAME_OVER\n");
            mvp_state = MVPHoops::MVPState::MVP_GAME_OVER;
        }
        else
        {
            game_update();
        }
    }
}

// Check sensors and update stat variables
void game_update()
{
    debugSkt("[game_update] Current Layout: ");
    debugSktVal(curr_mvp_pattern, BIN);

    for (uint8_t i = 0; i < NUM_MVP_HOOPS; ++i)
    {
        if (((curr_mvp_pattern >> i) & 1) && baskets[i].ball_detected())
        {
            score_count += 2; // Each shot converted grants 2 points
            send_score_to_resolume(SCORE, score_count);
            debugSkt("BALL DETECTED! ball count: ");
            debugSkt(score_count);

            // Check for new high score
            if (score_count > high_score_count)
            {
                high_score_count = score_count;
                send_score_to_resolume(HIGH_SCORE, high_score_count);

                // Check if the new high score clip was already triggered
                if (!new_high_score)
                {   // Activate the new high score pop-up clip on Resolume Arena
                    debugSkt(" | NEW HIGH SCORE!");
                    snprintf(reinterpret_cast<char*>(osc_message_buffer), sizeof(osc_message_buffer), RESOLUME_NEW_HIGH_SCORE_ADDRESS);
                    OSCPark msg(reinterpret_cast<char*>(osc_message_buffer));
                    udp.beginPacket(pc_ip, resolume_in_port);
                    msg.send(udp);
                    udp.endPacket();
                    new_high_score = true;
                }
            }
        }
    }
    debugSktln();
}

// Send OSC message to change the value in the High Score or Score text block in Resolume Arena
bool send_score_to_resolume(ScoreType in_type, uint16_t in_score)
{
    debugSkt("[send_score_to_resolume] ");
    memset(osc_message_buffer, 0, sizeof(osc_message_buffer));

    // Set string value
    // Construct the OSC address using snprintf to write into the global buffer (based in the ScoreType argument)
    switch (in_type)
    {
        case SCORE:
            debugSkt("Sending score: "); debugSkt(in_score); debugSkt(" points\n");
            snprintf(reinterpret_cast<char*>(osc_message_buffer), sizeof(osc_message_buffer), RESOLUME_SCORE_ADDRESS);
            break;
        case HIGH_SCORE:
            debugSkt("Sending high score: "); debugSkt(in_score); debugSkt(" points\n");
            snprintf(reinterpret_cast<char*>(osc_message_buffer), sizeof(osc_message_buffer), RESOLUME_HIGH_SCORE_ADDRESS);
            break;
        default:
            debugSkt("Invalid ScoreType argument...\n");
            return false;
            break;
    }

    OSCPark msg(reinterpret_cast<char*>(osc_message_buffer));

    char score_buffer[4]; // Store chars for the numbers of the high score or score
    itoa(in_score, score_buffer, 10);

    if (in_score < 10)
    {   // With zero prefix for numbers 0-9, e.g. "07"
        char tmp = score_buffer[0];
        score_buffer[0] = '0';
        score_buffer[1] = tmp;
        score_buffer[2] = '\0';
    }
    else if (in_score < 100)
    {   // Numbers 10-99, e.g. "27"
        score_buffer[2] = '\0';
    }
    else
    {   // Three digit numbers (max), e.g. "127"
        score_buffer[3] = '\0';
    }

    // Set score buffer as the value for the OSCPark instance
    msg.set_string(score_buffer);

    // Send message through EthernetUDP global instance
    udp.beginPacket(pc_ip, resolume_in_port);
    msg.send(udp);
    udp.endPacket();

    return true;
}

// Reset global instances used in the game logic, like layouts, sensors, and some counters
void reset_game_state()
{
    mvp_hoops.reset();
    now = game_timer.reset();
    mvp_state = mvp_hoops.update(now);
    new_high_score = false;
    score_count = 0;
}
