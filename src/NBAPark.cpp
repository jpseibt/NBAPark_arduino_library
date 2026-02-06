/*
 * NBA Park Arduino Library
 * Description: Definitions of the classes and structs from NBAPark.h
 * Author: José Paulo Seibt Neto
 * Created: Feb - 2025
 * Last Modified: Feb - 2026
*/

#include "NBAPark.h"

// Timer Class (start)
// Constructors
Timer::Timer() : m_start_time(millis()), m_offset_time(UINT32_MAX - m_start_time + 1) {}

// Methods
uint32_t Timer::reset()
{
    m_start_time = millis();
    m_offset_time = UINT32_MAX - m_start_time + 1;
    return 0;
}

uint32_t Timer::get_elapsed_time(bool seconds) const
{
    uint32_t now = millis();

    // Calculate elapsed time, handling overflow by adding offset time
    uint32_t elapsed = (now >= m_start_time) ? (now - m_start_time) : (m_offset_time + now);

    if (seconds)
        elapsed /= 1000; // Converts to seconds

    return elapsed;
}

void Timer::hang(uint32_t in_amt) const
{
    uint32_t start = millis();
    uint32_t offset = UINT32_MAX - start;
    uint32_t now, elapsed;

    do
    {
        now = millis();
        elapsed = (now >= start) ? (now - start) : (offset + now);
    }
    while (elapsed < in_amt);
}
// Timer (end)


// Clock (start)
// Constructors
Clock::Clock() : m_running(false), m_mode(0), m_clock_time(0) {}

Clock::Clock(uint8_t in_mode, uint32_t in_clock_time) : m_running(false)
{
    setup(in_mode, in_clock_time);
}

Clock::Clock(uint8_t in_hh, uint8_t in_mm, uint8_t in_ss) : m_running(false)
{
    setup(0, in_hh, in_mm, in_ss);
}

Clock::Clock(uint8_t in_mode, uint8_t in_hh, uint8_t in_mm, uint8_t in_ss) : m_running(false)
{
    setup(in_mode, in_hh, in_mm, in_ss);
}

// Methods
// Setup the clock time value and mode
uint32_t Clock::setup(uint8_t in_mode, uint32_t in_clock_time)
{
    m_mode = in_mode;
    m_clock_time = in_clock_time;
    reset(); // Reset timer counting elapsed time
    return m_clock_time;
}

uint32_t Clock::setup(uint8_t in_mode, uint8_t in_hh, uint8_t in_mm, uint8_t in_ss)
{
    m_mode = in_mode;
    m_clock_time = (3600UL * in_hh) + (60UL * in_mm) + in_ss;
    return m_clock_time;
}

uint32_t Clock::run()
{
    reset(); // Reset timer counting elapsed time
    m_running = true;
    return m_clock_time;
}

uint32_t Clock::stop()
{
    m_running = false;
    return m_clock_time;
}

// Update the clock time value based on the elapsed seconds since the last update() or setup() call
uint32_t Clock::update()
{
    uint32_t now = get_elapsed_time();
    if (!m_running || now < 1)
    {   // Exit early if not running or not a second has passed since the last update() call
        return m_clock_time;
    }

    /* Sync the m_clock_time with the number of seconds elapsed since the last update() call, then call the reset() Timer method.
       This makes sure that the clock will wrap around and update correctly on each update() call,
       incrementing for clock mode and decrementing for countdown mode. */
    if (m_mode == 1)
    {   // Countdown mode
        m_clock_time = (now <= m_clock_time) ? m_clock_time - now : 0;
    }
    else
    {
        m_clock_time += now;
    }
    reset();
    debugLib("[Clock::update] clock_time: "); debugLib(m_clock_time); debugLibln();

    if (m_mode == 0 && m_clock_time >= SECS_24H) 
    {   // Clock mode: Wrap around 24h
        m_clock_time %= SECS_24H;
    }
    else if (m_mode == 1 && m_clock_time <= 0)
    {   // Countdown mode: End of countdown reached
        debugLib("[Clock::update] Countdown end\n");
        m_clock_time = 0;
        m_running = false;
    }

    return m_clock_time;
}

void Clock::print() const
{
    char buffer[9];
    snprintf(buffer, 9, "%02u:%02u:%02u", get_hh(), get_mm(), get_ss());
    DEBUG_OUTPUT.println(buffer);
}
// Clock (end)


// IRBasketSensor Class (start)
IRBasketSensor::IRBasketSensor(uint8_t in_out_pin) : m_out_pin(in_out_pin)
{
    pinMode(m_out_pin, INPUT);
}

bool IRBasketSensor::ball_detected()
{
    m_hoop_cooldown.update();
    if (!m_hoop_cooldown.on_cooldown && !digitalRead(m_out_pin))
    {   // Ball detected
        m_hoop_cooldown.set_cooldown(BALL_DETECTION_COOLDOWN);
        return true;
    }
    return false;
}
// IRBasketSensor Class (end)


// BasketSensor Class (start)
// Constructors
BasketSensor::BasketSensor(uint8_t in_trig_pin, uint8_t in_echo_pin) : m_trig_pin(in_trig_pin), m_echo_pin(in_echo_pin)
{
    // Set trigger and echo pins
    pinMode(m_trig_pin, OUTPUT);
    pinMode(m_echo_pin, INPUT);
}

// Methods
// Update the sensor cooldown state and check for ball detection
bool BasketSensor::ball_detected()
{
    m_hoop_cooldown.update();
    if (!m_hoop_cooldown.on_cooldown)
    {
        float distance = get_ultrasonic_distance();
        if (distance > 2 && distance < BALL_DETECTION_THRESHOLD)
        {
            m_hoop_cooldown.set_cooldown(BALL_DETECTION_COOLDOWN);
            return true;
        }
    }
    return false;
}

float BasketSensor::get_ultrasonic_distance()
{
    uint32_t start_micros = micros();

    // Send a pulse to the ultrasonic sensor
    digitalWrite(m_trig_pin, LOW);
    while (micros() - start_micros < 2);
    digitalWrite(m_trig_pin, HIGH);
    while (micros() - start_micros < 12);
    digitalWrite(m_trig_pin, LOW);

    // Read the echo signal
    uint32_t duration = pulseIn(m_echo_pin, HIGH, BALL_DETECTION_TIMEOUT);
    if (duration == 0) return -1; // timeout reached

    return (duration * SOUND_SPEED) / 2; // Caculate distance in centimeters
}
// BasketSensor Class (end)


// ThreeBasketSensors Class (start)
// Constructor
ThreeBasketSensors::ThreeBasketSensors(const uint8_t* in_trig_pin_arr, const uint8_t* in_echo_pin_arr)
{
    m_ready = init(in_trig_pin_arr, in_echo_pin_arr);
}

// For the method just assign each argument to the corresponding index element in the pin arrays
// Pin number validation can be added
bool ThreeBasketSensors::init(const uint8_t in_trig0, const uint8_t in_trig1, const uint8_t in_trig2,
                              const uint8_t in_echo0, const uint8_t in_echo1, const uint8_t in_echo2)
{
    m_trig_pins[0] = in_trig0;
    m_trig_pins[1] = in_trig1;
    m_trig_pins[2] = in_trig2;

    m_echo_pins[0] = in_echo0;
    m_echo_pins[1] = in_echo1;
    m_echo_pins[2] = in_echo2;

    for (uint8_t i = 0; i < 3; ++i)
    {
        pinMode(m_trig_pins[i], OUTPUT);
        pinMode(m_echo_pins[i], INPUT);
    }

    return true;
}

// Initializes m_trig_pins and m_echo_pins arrays through an input array as argument
// Can be dangerous if arguments are pointers to an array with less than three elements
bool ThreeBasketSensors::init(const uint8_t* in_trig_pin_arr, const uint8_t* in_echo_pin_arr)
{
    // Check for null
    if (!in_trig_pin_arr || !in_echo_pin_arr)
    {
        return false;
    }

    for (uint8_t i = 0; i < 3; ++i)
    {
        m_trig_pins[i] = in_trig_pin_arr[i];
        m_echo_pins[i] = in_echo_pin_arr[i];
        pinMode(m_trig_pins[i], OUTPUT);
        pinMode(m_echo_pins[i], INPUT);
    }
    return true;
}

// Check all sensors (sA, sB, sC) at the same time and returns a bitmap of the reading
// Returns a uint8_t binary value as 0000_0[sC][sB][sA]
BitmapPattern ThreeBasketSensors::check_sensors()
{
    if (!m_ready) return BitmapPattern::LAYOUT_STOP;

    uint32_t pulse_starts[3] = {0, 0, 0};
    uint32_t pulse_durations[3] = {0, 0, 0};
    uint8_t sensor_states[3] = {0, 0, 0}; // 0 = waiting for HIGH, 1 = measuring HIGH, 2 = done

    // Send a pulse to the ultrasonic sensor on each trigger pin
    digitalWrite(m_trig_pins[0], LOW);
    digitalWrite(m_trig_pins[1], LOW);
    digitalWrite(m_trig_pins[2], LOW);
    //while (micros() - start_micros < 2);
    delayMicroseconds(2);
    digitalWrite(m_trig_pins[0], HIGH);
    digitalWrite(m_trig_pins[1], HIGH);
    digitalWrite(m_trig_pins[2], HIGH);
    //while (micros() - start_micros < 12);
    delayMicroseconds(10);
    digitalWrite(m_trig_pins[0], LOW);
    digitalWrite(m_trig_pins[1], LOW);
    digitalWrite(m_trig_pins[2], LOW);

    // Monitor echo pins
    uint32_t start_micros = micros();
    while (micros() - start_micros < BALL_DETECTION_TIMEOUT)
    {
        for (uint8_t i = 0; i < 3; ++i)
        {
            if (sensor_states[i] == 0 && digitalRead(m_echo_pins[i]) == HIGH)
            {   // Start timing the pulse
                pulse_starts[i] = micros();
                sensor_states[i] = 1;
            }
            else if (sensor_states[i] == 1 && digitalRead(m_echo_pins[i]) == LOW)
            {   // Pulse ended, calculate duration
                pulse_durations[i] = micros() - pulse_starts[i];
                sensor_states[i] = 2;
            }
        }

        // Exit early if all sensors are done
        if (sensor_states[0] == 2 && sensor_states[1] == 2 && sensor_states[2] == 2)
        {
            break;
        }
    }

    // Convert durations to distances and then binary representation
    BitmapPattern result = BitmapPattern::LAYOUT_0;
    for (uint8_t i = 0; i < 3; ++i)
    {
        if (pulse_durations[i])
        {
            uint32_t distance = (pulse_durations[i] * SOUND_SPEED) / 2;
            result |= ((distance > 1 && distance < BALL_DETECTION_THRESHOLD) ? 1 : 0) << i;
        }
    }

    return result;
}

// Return amount of shots converted by validating which sensors where triggered and which ones where valid
// at any given time, by using a bitwise AND on the current valid Layout of rims and the (inputed) sensor readings
uint8_t ThreeBasketSensors::filter_sensor_readings(const BitmapPattern in_curr_pattern, const BitmapPattern in_sensor_checks)
{
    if (!m_ready || in_sensor_checks >= BitmapPattern::LAYOUT_STOP) return 0;

    BitmapPattern valid_rims = in_curr_pattern & in_sensor_checks;
    m_hoops_cooldown.update();
    valid_rims &= ~m_hoops_cooldown.on_cooldown_pattern; // Flip bits corresponding to cooldown of each sensor

    debugLib("[ThreeBasketSensors::filter_sensor_readings] in_curr_pattern AND in_sensor_checks = ");
    debugLibVal(valid_rims, BIN); debugLibln();

    switch (valid_rims)
    {
        case LAYOUT_1:
            m_hoops_cooldown.set_cooldown(0);
        case LAYOUT_2:
            m_hoops_cooldown.set_cooldown(1);
        case LAYOUT_4:
            m_hoops_cooldown.set_cooldown(2);
            return 1; // One shot converted
        case LAYOUT_3:
            m_hoops_cooldown.set_cooldown(0);
            m_hoops_cooldown.set_cooldown(1);
        case LAYOUT_5:
            m_hoops_cooldown.set_cooldown(0);
            m_hoops_cooldown.set_cooldown(2);
        case LAYOUT_6:
            m_hoops_cooldown.set_cooldown(1);
            m_hoops_cooldown.set_cooldown(2);
            return 2; // Two shots converted
        case LAYOUT_7:
            m_hoops_cooldown.set_cooldown(0);
            m_hoops_cooldown.set_cooldown(1);
            m_hoops_cooldown.set_cooldown(2);
            return 3; // Three shots converted
    }
    return 0;         // None conveted
}


// MVPHoops (start)
// Constructors
MVPHoops::MVPHoops() : m_curr(0), m_next(1)
{
    m_layouts_arr = nullptr;
    m_curr_pattern = BitmapPattern::LAYOUT_0; // Default Layout
}

MVPHoops::MVPHoops(const Layout* in_layouts_arr, const uint8_t in_size) : m_curr(0), m_next(1)
{
    init(in_layouts_arr, in_size);
}

bool MVPHoops::init(const Layout* in_layouts_arr, const uint8_t in_size)
{
    if (!validate_layouts_arr(in_layouts_arr, in_size))
    {   // Invalid - raise error
        return false;
    }
    m_layouts_arr = in_layouts_arr;

    // Copy the bool array from the Layout obj id
    copy_pattern(&m_layouts_arr[m_curr]);
    return true;
}

// Iterate over the array to validate its layouts and check its size argument
bool MVPHoops::validate_layouts_arr(const Layout* in_layouts_arr, const uint8_t in_size)
{
    if (!in_layouts_arr)
    {   // Handle nullptr error
        return false;
    }
    else if (in_layouts_arr[0].active == BitmapPattern::LAYOUT_STOP || in_size < 2)
    {   // Empty array or without valid layouts
        return false;
    }

    uint8_t i = 0;
    Layout curr_layout = in_layouts_arr[0];

    // Iterate until the second to last element
    for(; i < in_size - 1 && curr_layout.active != BitmapPattern::LAYOUT_STOP; curr_layout = in_layouts_arr[++i])
    {
        if (curr_layout.active >= BitmapPattern::NUM_PATTERNS) return false;
    }

    // Checks the in_size and if the last element in in_layouts_arr is the sentinel value
    return (i == in_size - 1 && curr_layout.active == BitmapPattern::LAYOUT_STOP);
}

// Copy the BitmapPattern member variable Layout.active to m_curr_pattern
void MVPHoops::copy_pattern(const Layout* in_layout)
{
    m_curr_pattern = in_layout->active;
}

// Update the current valid layout by dereferencing the next available Layout obj by checking the in_time argument and returning the "game state"
MVPHoops::MVPState MVPHoops::update(const uint32_t in_time)
{
    if (!m_layouts_arr)
    {
        return MVP_GAME_OVER;
    }
    else if (in_time >= m_layouts_arr[m_next].time)
    {
        if (m_layouts_arr[++m_curr].active == BitmapPattern::LAYOUT_STOP)
        {
            // End of the transitions
            reset();
            return MVP_GAME_OVER;
        }
        copy_pattern(&m_layouts_arr[m_next++]);
    }
    else if (in_time < m_layouts_arr[m_curr].time)
    {
        return MVP_HOLD;
    }

    return MVP_RUNNING;
}

MVPHoops::MVPState MVPHoops::reset()
{
    m_curr = 0;
    m_next = 1;
    if (m_layouts_arr)
        copy_pattern(&m_layouts_arr[m_curr]);

    return MVP_GAME_OVER;
}
// MVPHoops (end)


// OSCPark (start)
// Constructors
// Default
OSCPark::OSCPark() : m_addr{'\0'}, m_type_tags{'\0'}, m_addr_len(0), m_type_len(0), m_values_len(0) {}

OSCPark::OSCPark(const uint8_t* in_buffer)
{
    init(in_buffer);
}

OSCPark::OSCPark(const char* in_address)
{
    init(in_address);
}

void OSCPark::init(const uint8_t* in_buffer)
{
    debugLib("[OSCPark::init] Raw bytes: ");
    for (int i = 0; i < 65; ++i) // Condition can be adjusted to print enough bytes
    {
        debugLibVal(in_buffer[i], HEX);
        debugLib(" ");
    }
    debugLibln();

    const uint8_t* ptr = in_buffer;

    // Extract address pattern (null-terminated, 4-byte aligned)
    m_addr_len = strnlen((const char*)ptr, sizeof(m_addr));
    memcpy(m_addr, ptr, m_addr_len);
    m_addr[m_addr_len] = '\0';
    ptr += (m_addr_len + 1 + 3) & ~3;  // Move to 4-byte boundary (m_addr_len + 1 null char)

    // Extract type tag string (starts with ',')
    m_type_len = strnlen((const char*)ptr, sizeof(m_type_tags));
    if (m_type_len > 1)
    {   // Skip the byte containing ',' by copying from the ptr + 1
        memcpy(m_type_tags, ptr + 1, m_type_len--); // Decrement m_type_len to store the amount of type tags without the comma
    }
    m_type_tags[m_type_len] = '\0';
    ptr += (m_type_len + 1 + 3) & ~3;

    // Extract value (currently only support messages with one type_tag)
    m_value.setup(m_type_tags[0], ptr);
}

// Initialize an instance with an address but without a value.
// If the object was initialized previously with a value of type String, ensure to call clear() to free any dynamically allocated memory.
void OSCPark::init(const char* in_address)
{
    memcpy(&m_addr, in_address, sizeof(m_addr));
    m_type_tags[0] = '\0';
    m_addr_len = strnlen(m_addr, sizeof(m_addr));
    m_type_len = 0;
    m_values_len = 0;
    m_value.type_tag = '\0';
}

void OSCPark::Value::setup(const char in_type_tag, const uint8_t* in_ptr)
{
    debugLib("[OSCPark::Value::setup] ");
    uint8_t str_size = 0;
    char* str_buffer = nullptr;

    switch (in_type_tag)
    {
        case 'i':
        {
            debugLib("Type: INT\n");
            type_tag = 'i';
            memcpy(&data.i_value, in_ptr, sizeof(data.i_value));
            data.i_value = __builtin_bswap32(data.i_value);
            break;
        }
        case 'f':
        {
            debugLib("Type: FLOAT\n");
            type_tag = 'f';
            memcpy(&data.f_value, in_ptr, sizeof(data.f_value));
            uint32_t temp = __builtin_bswap32(*reinterpret_cast<uint32_t*>(&data.f_value));
            data.f_value = *reinterpret_cast<float*>(&temp);
            break;
        }
        case 's':
        {
            debugLib("Type: STRING\n");
            type_tag = 's';
            str_size = strnlen((const char*)in_ptr, 255);
            str_buffer = new char[str_size + 1];
            memcpy(str_buffer, in_ptr, str_size);
            str_buffer[str_size] = '\0';
            data.s_value = str_buffer;
            break;
        }
        default:
        {
            debugLib("No valid type flag parsed...\n");
            break;
        }
    }
}

// Sets a string value for the instance
void OSCPark::set_int(const int in_int)
{
    // Make sure to free the dinamic memory of s_value
    if (m_value.type_tag == 's' && m_value.data.s_value != nullptr)
    {
        debugLib("[OSCPark::set_int] s_value memory freed");
        delete[] m_value.data.s_value;
    }

    m_value.data.i_value = in_int;

    m_values_len = 1;
    m_type_len = 1;
    m_type_tags[0] = 'i';
    m_value.type_tag = 'i';
}

// Sets a string value for the instance
void OSCPark::set_float(const float in_float)
{
    // Make sure to free the dinamic memory of s_value
    if (m_value.type_tag == 's' && m_value.data.s_value != nullptr)
    {
        debugLib("[OSCPark::set_float] s_value memory freed");
        delete[] m_value.data.s_value;
    }

    m_value.data.f_value = in_float;

    m_values_len = 1;
    m_type_len = 1;
    m_type_tags[0] = 'f';
    m_value.type_tag = 'f';
}

// Sets a string value for the instance
void OSCPark::set_string(const char* in_str)
{
    // Make sure to free the dinamic memory of s_value
    if (m_value.type_tag == 's' && m_value.data.s_value != nullptr)
    {
        debugLib("[OSCPark::set_string] Previous s_value memory freed");
        delete[] m_value.data.s_value;
    }

    uint8_t str_len = strnlen(in_str, 255);

    m_value.data.s_value = new char[str_len + 1];
    memcpy(m_value.data.s_value, in_str, str_len);
    m_value.data.s_value[str_len] = '\0';

    m_values_len = 1;
    m_type_len = 1;
    m_type_tags[0] = 's';
    m_value.type_tag = 's';
}

void OSCPark::send(Print &in_p)
{
    debugLib("[OSCPark::send] SENDING\n");
    uint8_t padding_len;
    uint8_t total_len;

    // Write address to the packet
    in_p.write(reinterpret_cast<uint8_t*>(m_addr), m_addr_len);
    in_p.write('\0'); // Adding null terminator
    // Add padding if neccessary
    total_len = m_addr_len + 1; // Including the null terminator
    padding_len = (4 - (total_len % 4)) % 4;
    while (padding_len--)
    {
       in_p.write('\0');
    }

    // Stop here if no type tag in message
    if (m_values_len < 1) return;

    // Write type tag
    in_p.write(static_cast<uint8_t>(','));
    in_p.write(static_cast<uint8_t>(m_type_tags[0]));
    in_p.write('\0'); // Adding null terminator
    // Add padding if neccessary
    total_len = m_type_len + 2; // +1 for the comma and +1 for the null terminator
    padding_len = (4 - (total_len % 4)) % 4;
    while (padding_len--)
    {
        in_p.write('\0');
    }

    switch (m_type_tags[0])
    {
        case 'i':
        {
            uint32_t temp = __builtin_bswap32(m_value.data.i_value);
            in_p.write(reinterpret_cast<uint8_t*>(&temp), sizeof(uint32_t));
            break;
        }
        case 'f':
        {
            uint32_t temp = *reinterpret_cast<uint32_t*>(&m_value.data.f_value);
            temp = __builtin_bswap32(temp);
            in_p.write(reinterpret_cast<uint8_t*>(&temp), sizeof(uint32_t));
            break;
        }
        case 's':
        {
            int s_value_len = strnlen(m_value.data.s_value, 255);
            in_p.write(reinterpret_cast<uint8_t*>(m_value.data.s_value), s_value_len); // Max length of 255
            in_p.write('\0');
            total_len = s_value_len + 1;
            padding_len = (4 - (total_len % 4)) % 4;
            while (padding_len--)
            {
                in_p.write('\0');
            }
            break;
        }
        default:
        {
            debugLib("Not a supported type...\n");
            break;
        }
    }
}

// Reset the member variables to zero and free any dynamically allocated memory
void OSCPark::clear()
{
    m_addr[0] = '\0';
    m_type_tags[0] = '\0';
    m_addr_len = 0;
    m_type_len = 0;
    m_values_len = 0;

    switch (m_value.type_tag)
    {
        case 'i':
            m_value.data.i_value = 0;
            break;
        case 'f':
            m_value.data.f_value = 0;
            break;
        case 's':
            delete[] m_value.data.s_value;
            m_value.data.s_value = nullptr;
            break;
    }
    m_value.type_tag = '\0';
}

// Prints to the DEBUG_OUTPUT the characters representation of the OSC message in the obj
void OSCPark::print() const
{
    if (m_addr_len <= 0)
    {
        DEBUG_OUTPUT.print("OSCPark obj is empty...\n");
    }
    else
    {
        char osc_message_buffer[255];
        osc_message_buffer[0] = '\0';

        // Currently only support messages with one type_tag
        if (m_values_len > 0)
        { // Parentheses added after the type tags for clarity
            switch (m_type_tags[0])
            {
                case 'i':
                    snprintf((char*)osc_message_buffer, sizeof(osc_message_buffer), "%s,%c(%ld)", m_addr, m_type_tags[0], m_value.data.i_value);
                    break;
                case 'f':
                    // Convert float to string with fixed precision
                    char float_str[7];  // Allocate a buffer for the string representation of the float (4 decimals + '.' + the integer part + '\0')
                    dtostrf(m_value.data.f_value, 1, 4, float_str);  // dtostrf converts float to string with 4 decimals
                    snprintf((char*)osc_message_buffer, sizeof(osc_message_buffer), "%s,%c(%s)", m_addr, m_type_tags[0], float_str);
                    break;
                case 's':
                    if (m_value.data.s_value != nullptr)
                    {
                        snprintf((char*)osc_message_buffer, sizeof(osc_message_buffer), "%s,%c(%s)", m_addr, m_type_tags[0], m_value.data.s_value);
                    }
                    else
                    {
                        snprintf((char*)osc_message_buffer, sizeof(osc_message_buffer), "%s,%c(NULL)", m_addr, m_type_tags[0]);
                    }
                    break;
                default:
                    snprintf((char*)osc_message_buffer, sizeof(osc_message_buffer), "%s,%c(error parsing the value)", m_addr, m_type_tags[0]);
                    break;
            }
        }
        else
        {
            // Only address
            snprintf((char*)osc_message_buffer, sizeof(osc_message_buffer), "%s", m_addr);
        }
        DEBUG_OUTPUT.print(osc_message_buffer); DEBUG_OUTPUT.print("\n");
    }
}

// Prints to the DEBUG_OUTPUT the info of each member var of the obj
void OSCPark::info() const
{
    char buffer[255];

    snprintf((char*)buffer, sizeof(buffer), "Address: %s", m_addr);
    DEBUG_OUTPUT.print(buffer); DEBUG_OUTPUT.print("\n");

    if (m_type_len)
    {
        snprintf((char*)buffer, sizeof(buffer), "Type tags: %c", m_type_tags[0]);
        DEBUG_OUTPUT.print(buffer); DEBUG_OUTPUT.print("\n");
    }

    // Currently only support messages with one type_tag
    switch (m_type_tags[0])
    {
        case 'i':
            snprintf((char*)buffer, sizeof(buffer), "m_value.data.i_value: %ld", m_value.data.i_value);
            break;
        case 'f':
            // Convert float to string with fixed precision
            char float_str[7];  // Allocate a buffer for the string representation of the float (4 decimals + '.' + the integer part + '\0')
            dtostrf(m_value.data.f_value, 1, 4, float_str);  // dtostrf converts float to string with 6 decimals
            snprintf((char*)buffer, sizeof(buffer), "m_value.data.f_value: %s", float_str);
            break;
        case 's':
            if (m_value.data.s_value != nullptr)
            {
                snprintf((char*)buffer, sizeof(buffer), "m_value.data.s_value: %s", m_value.data.s_value);
            }
            else
            {
                snprintf((char*)buffer, sizeof(buffer), "m_value.data.s_value: NULL");
            }
            break;
        default:
            snprintf((char*)buffer, sizeof(buffer), "NO VALUE");
            break;
    }
    DEBUG_OUTPUT.print(buffer); DEBUG_OUTPUT.print("\n");

    snprintf((char*)buffer, sizeof(buffer), "m_addr_len: %d", m_addr_len);
    DEBUG_OUTPUT.print(buffer); DEBUG_OUTPUT.print("\n");

    snprintf((char*)buffer, sizeof(buffer), "m_type_len: %d", m_type_len);
    DEBUG_OUTPUT.print(buffer); DEBUG_OUTPUT.print("\n");
}
// OSCPark (end)
