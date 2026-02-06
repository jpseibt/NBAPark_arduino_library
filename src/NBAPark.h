/*
 * NBA Park Arduino Library
 * Description: Declarations of classes and structs
 * Author: José Paulo Seibt Neto
 * Created: Feb - 2025
 * Last Modified: Feb - 2026
*/

#ifndef NBAPARK_H
#define NBAPARK_H

#include <Arduino.h>
#include <stdint.h>  // types uint8_t, uint32_t, etc (Arduino.h should already include this header by default)

// Debug levels
#ifndef DEBUG_LEVEL
    #define DEBUG_LEVEL 3 // 0 = None, 1 = Sketch only, 2 = Library only, 3 = Sketch and Library
#endif

#define DEBUG_OUTPUT Serial

// Debug macros for Sketch
#if DEBUG_LEVEL == 1 || DEBUG_LEVEL == 3
    #define debugSkt(msg) DEBUG_OUTPUT.print(msg)
    #define debugSktVal(val, format) DEBUG_OUTPUT.print(val, format)
    #define debugSktln() DEBUG_OUTPUT.print("\n")
#else
    #define debugSkt(msg)
    #define debugSktVal(val, format)
    #define debugSktln()
#endif

// Debug macros for Library
#if DEBUG_LEVEL == 2 || DEBUG_LEVEL == 3
    #define debugLib(msg) DEBUG_OUTPUT.print(msg)
    #define debugLibVal(val, format) DEBUG_OUTPUT.print(val, format)
    #define debugLibln() DEBUG_OUTPUT.print("\n")
#else
    #define debugLib(msg)
    #define debugLibVal(val, format)
    #define debugLibln()
#endif

// Display related macros
#ifndef DSP_H_CENTER
    #define DSP_H_CENTER 160 // Horizontal center of the ILI9341 display
#endif
#ifndef DSP_V_CENTER
    #define DSP_V_CENTER 120 // Verical center
#endif

// Debug flag for use with Adafruit_GFX and Ucglib (ILI9341 240x320 display)
#ifndef DEBUG_DISPLAY
    #define DEBUG_DISPLAY 0 // 0 = None, 1 = Adafruit, 2 = Ucglib
#endif

#if DEBUG_DISPLAY == 1
    #define debugDrawCrossCenter() ada.drawFastHLine(0, ada.height() / 2, ada.width(), 0xffff); /*Draw horizontal line*/ \
                                   ada.drawFastVLine(ada.width() / 2, 0, ada.height(), 0xffff); /*Draw vertical line*/
#elif DEBUG_DISPLAY == 2
    #define debugDrawCrossCenter() ucg.setColor(255, 255, 255); \
                                   ucg.drawHLine(0, ucg.getHeight() / 2, ucg.getWidth()); /*Draw horizontal line*/ \
                                   ucg.drawVLine(ucg.getWidth() / 2, 0, ucg.getHeight())  /*Draw vertical line*/
    #define debugDrawInnerCross() ucg.drawHLine(0, ucg.getHeight() / 4, ucg.getWidth());                   /*Draw upper horizontal line*/ \
                                  ucg.drawVLine(ucg.getWidth() / 4, 0, ucg.getHeight());                   /*Draw left vertical line*/ \
                                  ucg.drawHLine(0, ucg.getHeight() - ucg.getHeight() / 4, ucg.getWidth()); /*Draw bottom horizontal line*/ \
                                  ucg.drawVLine(ucg.getWidth() - ucg.getWidth() / 4, 0, ucg.getHeight())   /*Draw right vertical line*/
#else
    #define debugDrawCrossCenter()
    #define debugDrawInnerCross()
#endif

// Constants
#define SOUND_SPEED 0.0343f            // Speed of sound in centimeters per microsecond
#define BALL_DETECTION_THRESHOLD 30U   // Value in centimeters
#define BALL_DETECTION_COOLDOWN 500U   // Value in milliseconds
#define BALL_DETECTION_TIMEOUT 5000U   // Value in microseconds (3-5ms timeout should be enough for reads up to ~50cm)
#define BALL_DETECTION_READ_DELAY 7U   // Value in milliseconds (almost always should be greater than the timeout, and can vary depending on the environment)
#define NUM_MVP_HOOPS 3U
#define DEFAULT_HIGH_SCORE 10U         // Default high score value (used in the GameMVP example program)
#define HIGH_SCORE_RESET_TIME 86400U   // Value in seconds
#define RESOLUME_MVPGAME_ADDRESS "/mvp/game" // OSC address of message send by Resolume Arena when the MVP GAME clip is running (transport position)
#define RESOLUME_MVPWAIT_ADDRESS "/mvp/wait" // OSC address of message send by Resolume Arena when the MVP WAIT clip is running (transport position)
#define MVP_HARD_RESET_OSC "/mvp/rst"       // OSC address of message send by Bitfocus Companion software to trigger a hard reset
#define RESOLUME_SCORE_ADDRESS "/composition/layers/2/clips/2/video/effects/textblock2/effect/text/params/lines"      // OSC address in the Resolume Arena composition
#define RESOLUME_HIGH_SCORE_ADDRESS "/composition/layers/4/clips/1/video/effects/textblock2/effect/text/params/lines" // OSC address in the Resolume Arena composition
#define RESOLUME_NEW_HIGH_SCORE_ADDRESS "/composition/layers/3/clips/2/connect"
#define OSC_MAX_ADDRESS_LEN 255U
#define R_BATTLE_DEFAULT_MATCH_DUR 120U // Value in seconds
#define R_BATTLE_OVERTIME 45U           // Value in seconds
#define R_BATTLE_RESET_TRIGGER 2000U       // Value in milliseconds (button release time)
#define R_BATTLE_HARD_RESET_TRIGGER 7000U  // Value in milliseconds (button release time)
#define R_BATTLE_BUZZER_FEEDBACK_DUR 150U  // Duration of buzzer sound in milliseconds
#define R_BATTLE_SCORER_TIMEOUT 3000U      // Duration of highlight for scorer in milliseconds
#define BUTTON_RELEASE_WINDOW 2000U        // Value in milliseconds
#define SECS_24H 86400UL
#define SECS_1H 3600UL
#define SECONDS true // Timer.get_elapsed_time() method
#define MILLIS false // Timer.get_elapsed_time() method

// Helper struct for buttons
struct Button
{
    uint8_t pin;
    uint8_t state;
    uint16_t release_time;      // Stores the duration in milliseconds of the last button press
    uint32_t press_millis_start;
    /* The curr_press_dur is usefull for debugging, commented to save up to ~120 bytes.
       Can be uncommented in the constructor, update(), and reset(). */
    uint32_t curr_press_dur; // Stores current button press duration

    Button(uint8_t in_pin) : pin(in_pin), state(0), release_time(UINT16_MAX), press_millis_start(0) //, curr_press_dur(0)
    {
        pinMode(pin, INPUT_PULLUP);
    }

    // Return the release time of the press, zero otherwise
    uint32_t update()
    {
        if (!state)
        {   // Check for first press
            state = digitalRead(pin);
            if (state) { press_millis_start = millis(); }
        }
        else
        {   // Button was pressed in the last update() call
            // Update held state 
            state = digitalRead(pin);
            // If button released, calculate release time, else release time gets zero
            release_time = (!state) ? millis() - press_millis_start : 0;
        }

        curr_press_dur = (state) ? millis() - press_millis_start : 0;
        return release_time;
    }

    void reset()
    {
        state = 0;
        release_time = UINT16_MAX;
        curr_press_dur = 0;
    }
};


class Timer
{
    uint32_t m_start_time;
    uint32_t m_offset_time;

public:
    // Constructors
    Timer();

    // Accessors
    const uint32_t& get_start_time() const { return m_start_time; }
    const uint32_t& get_offset_time() const { return m_offset_time; }

    // Methods
    uint32_t reset();
    uint32_t get_elapsed_time(bool seconds=true) const;
    void hang(uint32_t in_amt) const; // Hang for in_amt milliseconds
};


class Clock : public Timer
{
    bool m_running;        // A setup() call is expected to change the state (mostly used for countdown mode)
    uint8_t m_mode;        // 0 = clock, 1 = countdown
    uint32_t m_clock_time; // Store the clock time state after the last update() or setup() call (in seconds)

public:
    // Constructors
    Clock();
    Clock(uint8_t in_mode, uint32_t in_clock_time);
    Clock(uint8_t in_hh, uint8_t in_mm, uint8_t in_ss);
    Clock(uint8_t in_mode, uint8_t in_hh, uint8_t in_mm, uint8_t in_ss);

    // Accessors
    bool is_running() const { return m_running; }
    uint8_t get_hh() const { return m_clock_time / SECS_1H; }
    uint8_t get_mm() const { return (m_clock_time % SECS_1H) / 60; }
    uint8_t get_ss() const { return m_clock_time % 60; }
    uint32_t get_time_secs() const { return m_clock_time; }

    // Methods
    uint32_t setup(uint8_t in_mode, uint32_t in_clock_time);
    uint32_t setup(uint8_t in_mode, uint8_t in_hh, uint8_t in_mm, uint8_t in_ss);
    uint32_t run();    // Clock running (time passing)
    uint32_t stop();   // Clock stoped (preserve/stop clock time)
    uint32_t update(); // Update clock time (if running)
    void print() const;
};


// Need a IR sensor
class IRBasketSensor
{
    uint8_t m_out_pin;

    // Separate hoop state that handle the cooldown for checking sensor after a ball is detected (all in-lined for simplicity)
    struct HoopCooldown
    {
        Timer mil_timer;
        bool on_cooldown;
        uint16_t cooldown_time;

        // Constructor
        HoopCooldown() : on_cooldown(false), cooldown_time(0) { mil_timer.reset(); }

        // Methods
        void set_cooldown(uint32_t in_cooldown_amount)
        {
            mil_timer.reset();
            cooldown_time = in_cooldown_amount;
            on_cooldown = true;
        }

        void update()
        {
            if (on_cooldown && mil_timer.get_elapsed_time(false) > cooldown_time)
            {
                on_cooldown = false;
                cooldown_time = 0;
            }
        }

        void reset()
        {
            mil_timer.reset();
            on_cooldown = false;
            cooldown_time = 0;
        }
    } m_hoop_cooldown;

public:
    // Constructor
    IRBasketSensor(uint8_t in_out_pin); 

    // Method
    bool ball_detected();
};


// Need a HC-SR04 sensor
class BasketSensor
{
    // Pins used by the ultrasonic sensor
    uint8_t m_trig_pin;
    uint8_t m_echo_pin;

    // Separate hoop state that handle the cooldown for checking sensor after a ball is detected (all in-lined for simplicity)
    struct HoopCooldown
    {
        Timer mil_timer;
        bool on_cooldown;
        uint16_t cooldown_time;

        // Constructor
        HoopCooldown() : on_cooldown(false), cooldown_time(0) { mil_timer.reset(); }

        // Methods
        void set_cooldown(uint32_t in_cooldown_amount)
        {
            mil_timer.reset();
            cooldown_time = in_cooldown_amount;
            on_cooldown = true;
        }

        void update()
        {
            if (on_cooldown && mil_timer.get_elapsed_time(false) > cooldown_time)
            {
                on_cooldown = false;
                cooldown_time = 0;
            }
        }

        void reset()
        {
            mil_timer.reset();
            on_cooldown = false;
            cooldown_time = 0;
        }
    } m_hoop_cooldown;

public:
    // Constructors
    BasketSensor(uint8_t in_trig_pin, uint8_t in_echo_pin);

    // Acessors
    const uint8_t& get_trig_pin() const { return m_trig_pin; }
    const uint8_t& get_echo_pin() const { return m_echo_pin; }

    // Methods
    float get_ultrasonic_distance();
    bool ball_detected();
};


// Bitmap for all possible patterns for three hoops
// Each pattern element enum NEED to align with it's binary representation, e.g. LAYOUT_9 = 0b1001u
enum BitmapPattern : uint8_t
{
    LAYOUT_0 = 0b0000u,
    LAYOUT_1 = 0b0001u,
    LAYOUT_2 = 0b0010u,
    LAYOUT_3 = 0b0011u,
    LAYOUT_4 = 0b0100u,
    LAYOUT_5 = 0b0101u,
    LAYOUT_6 = 0b0110u,
    LAYOUT_7 = 0b0111u,
    LAYOUT_STOP, // Sentinel value
    NUM_PATTERNS
};


// ThreeBasketSensors (begin)
// Used to check and interpret readings from three ultrasonic sensors (HC-SR04) simultaneously
class ThreeBasketSensors
{
    uint8_t m_trig_pins[3];
    uint8_t m_echo_pins[3];
    bool m_ready; // Flag that indicates if the pin arrays where initialized correctly

    // Separate hoops state that handle the cooldown for checking sensor after a ball is detected (all in-lined for simplicity)
    struct ThreeHoopsCooldown
    {
        Timer mil_timer[3];
        BitmapPattern on_cooldown_pattern;

        // Constructor
        ThreeHoopsCooldown() : on_cooldown_pattern(BitmapPattern::LAYOUT_0)
        {
            for (uint8_t i = 0; i < 3; ++i) mil_timer[i].reset();
        }

        // Methods
        void set_cooldown(uint8_t in_hoop_index)
        {
            if (in_hoop_index > 2)
            {
                debugLib("[ThreeHoopsCooldown::set_cooldown] Invalid arg for in_hoop_index\n");
            }
            else
            {
                mil_timer[in_hoop_index].reset();
                // Set cooldown pattern
                switch (in_hoop_index)
                {   // Activate the bit that corresponds to the hoop_index
                    case 0:
                        on_cooldown_pattern |= 0b0001u;
                        break;
                    case 1:
                        on_cooldown_pattern |= 0b0010u;
                        break;
                    case 2:
                        on_cooldown_pattern |= 0b0100u;
                        break;
                    default:
                        debugLib("[ThreeHoopsCooldown::set_cooldown] Should never get here\n");
                }
            }
        }

        // Check each hoop cooldown sequentially and update the bitmap pattern using a bitmask
        void update()
        {
            uint8_t mask = 0b0000u;

            // Check the first sensor
            if ((on_cooldown_pattern & 0b0001u) && (mil_timer[0].get_elapsed_time(false) > BALL_DETECTION_COOLDOWN))
            {   // Deactivate cooldown on first sensor
                debugLib("Deactivate cooldown on first sensor\n");
                mask |= 0b0001u;
            }

            // Check the second sensor
            if ((on_cooldown_pattern & 0b0010u) && (mil_timer[1].get_elapsed_time(false) > BALL_DETECTION_COOLDOWN))
            {   // Deactivate cooldown on second sensor
                debugLib("Deactivate cooldown on second sensor\n");
                mask |= 0b0010u;
            }

            // Check the third sensor
            if ((on_cooldown_pattern & 0b0100u) && (mil_timer[2].get_elapsed_time(false) > BALL_DETECTION_COOLDOWN))
            {   // Deactivate cooldown on third sensor
                debugLib("Deactivate cooldown on third sensor\n");
                mask |= 0b0100u;
            }

            // Apply mask to the bitmap of the sensor on cooldown
            on_cooldown_pattern &= ~mask;
        }

        void reset()
        {
            for (uint8_t i = 0; i < 3; ++i)
            {
                mil_timer[i].reset();
            }
            on_cooldown_pattern = BitmapPattern::LAYOUT_0;
        }
    } m_hoops_cooldown;

public:
    // Constructors
    ThreeBasketSensors()
        : m_trig_pins{0, 0, 0}, m_echo_pins{0, 0, 0}, m_ready(false) {}

    ThreeBasketSensors(const uint8_t in_trig0, const uint8_t in_trig1, const uint8_t in_trig2,
                       const uint8_t in_echo0, const uint8_t in_echo1, const uint8_t in_echo2)
        : m_trig_pins{in_trig0, in_trig1, in_trig2},
          m_echo_pins{in_echo0, in_echo1, in_echo2},
          m_ready(true)
          {}

    ThreeBasketSensors(const uint8_t* in_trig_pin_arr, const uint8_t* in_echo_pin_arr);

    // Methods
    bool init(const uint8_t in_trig0, const uint8_t in_trig1, const uint8_t in_trig2, const uint8_t in_echo0, const uint8_t in_echo1, const uint8_t in_echo2);
    bool init(const uint8_t* in_trig_pin_arr, const uint8_t* in_echo_pin_arr);

    BitmapPattern check_sensors();
    uint8_t filter_sensor_readings(BitmapPattern in_curr_pattern, BitmapPattern in_sensor_checks);
};


// Handle the multiple states and the dynamic layouts of the MVP Competition basketball rims
// Needs to point to an array of Layout instances with the last Layout.active being a BitmapPattern::LAYOUT_STOP
class MVPHoops
{
public:
    struct Layout
    {
        uint32_t time;
        BitmapPattern active;

        // Layout constructors
        Layout() : time(0), active(LAYOUT_0) {}
        Layout(uint32_t in_time, BitmapPattern in_active) : time(in_time), active(in_active) {}
    };

    enum MVPState : uint8_t
    {
        MVP_GAME_OVER,
        MVP_RUNNING,
        MVP_HOLD
    };

private:
    // Member variables
    const Layout* m_layouts_arr;
    uint8_t m_curr;               // Index for the current Layout obj  
    uint8_t m_next;               // Index for the next Layout obj
    BitmapPattern m_curr_pattern; // Copy of the BitmapPattern member variable Layout.active

public:
    // Constructors
    MVPHoops();
    MVPHoops(const Layout* in_layout_arr, const uint8_t in_size);

    // Methods
    bool init(const Layout* in_layout_arr, const uint8_t in_size);
    MVPState update(uint32_t in_time);
    MVPState reset(); // Always return MVP_GAME_OVER

    // Accessors (copy)
    BitmapPattern get_curr_pattern() const { return m_curr_pattern; }

private:
    // Method to iterate over layouts and ensure correct boundary checking
    bool validate_layouts_arr(const Layout* in_layouts_arr, const uint8_t in_size);
    void copy_pattern(const Layout* in_layout);
};


class OSCPark
{
    // Struct used to store the value data from OSCMessages
    struct Value
    {
        char type_tag;
        union Data
        {
            int32_t i_value;
            float f_value;
            char* s_value;
        } data;

        void setup(const char in_type_tag, const uint8_t* in_ptr);
    };

    char m_addr[80];
    char m_type_tags[8];
    Value m_value;

    uint8_t m_addr_len;
    uint8_t m_type_len;
    uint8_t m_values_len;

public:
    // Constructors
    OSCPark();
    OSCPark(const uint8_t* in_buffer);
    OSCPark(const char* in_address);

    // Destructor
    ~OSCPark() { clear(); }

    // Methods
    void init(const uint8_t* in_buffer);
    void init(const char* in_address);
    void set_int(const int in_int);
    void set_float(const float in_float);
    void set_string(const char* in_str);
    void send(Print& in_p);
    void clear();

    // Prints to DEBUG_OUTPUT
    void print() const;
    void info() const;

    // Accessors
    const char* get_addr() const { return m_addr; }
    const char* get_addr_cmp() { return m_addr; } // Non-const return to be used in strncmp
    const char* get_type() const { return m_type_tags; }
    int32_t get_int() const { return m_value.data.i_value; }
    float get_float() const { return m_value.data.f_value; }
    char* get_str() const { return m_value.data.s_value; }
    uint8_t get_addr_len() const { return m_addr_len; }
    uint8_t get_type_len() const { return m_type_len; }
    uint8_t get_values_len() const { return m_values_len; }
};

#endif // NBAPARK_H
