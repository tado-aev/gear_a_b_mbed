#ifndef BASE_CONTROLLER_H_
#define BASE_CONTROLLER_H_

#include <mbed.h>

#include <vector>
#include <bitset>

class BaseController {
public:
    static const unsigned TO_PC_BAUDRATE = 9600;

    /* Number of LEDs on the board */
    static const unsigned NUM_LEDS = 4;

    /* Constructors, Destructor, and Assignment operators {{{ */
    // Default constructor
    BaseController();

    // Copy constructor
    BaseController(const BaseController& other) = delete;

    // Move constructor
    BaseController(BaseController&& other) = delete;

    // Destructor
    ~BaseController();

    // Assignment operator
    BaseController&
    operator=(const BaseController& other) = delete;

    // Move assignment operator
    BaseController&
    operator=(BaseController&& other) = delete;
    /* }}} */

    /**
     * Writes to the serial port connected to the USB port
     */
    void
    write_to_pc(const std::string& msg);

    /**
     * Changes the on-board LEDs to the given bitwise high/low
     */
    void
    led_output(const std::bitset<NUM_LEDS>& output);

    /**
     * Changes the on-board LEDs according to the percentage
     *
     * Percentage -> Number of LEDs lit
     * 0 -19:  0
     * 20-39:  1
     * 40-59:  2
     * 60-79:  3
     * 80-100: 4
     *
     * \param[in] percentage float in the range of [0, 100]
     */
    void
    led_output(const float percentage);

    /**
     * Limits the given variable in the range
     */
    template<typename T>
    T
    limit_range(const T val, const T min_val, const T max_val);

private:
    // On-board LEDs
    std::vector<DigitalOut> leds;
    Serial to_pc;
};

/* Constructors, Destructor, and Assignment operators {{{ */
// Default constructor
BaseController::BaseController()
    // On-board LEDs
    : leds{DigitalOut{LED1}, DigitalOut{LED2}, DigitalOut{LED3}, DigitalOut{LED4}}
    , to_pc{USBTX, USBRX}
{
    to_pc.baud(TO_PC_BAUDRATE);
}

// Destructor
BaseController::~BaseController()
{
}
/* }}} */

void
BaseController::write_to_pc(const std::string& msg) {
    to_pc.printf("%s\n", msg.c_str());
}

void
BaseController::led_output(const std::bitset<NUM_LEDS>& output) {
    for (unsigned i = 0; i < output.size(); i++) {
        leds[i] = output[i];
    }
}

void
BaseController::led_output(const float percentage) {
    if (percentage >= 80) {
        led_output(std::bitset<NUM_LEDS>{"1111"});
    }
    else if (percentage >= 60) {
        led_output(std::bitset<NUM_LEDS>{"1110"});
    }
    else if (percentage >= 40) {
        led_output(std::bitset<NUM_LEDS>{"1100"});
    }
    else if (percentage >= 20) {
        led_output(std::bitset<NUM_LEDS>{"1000"});
    }
    else {
        led_output(std::bitset<NUM_LEDS>{"0000"});
    }
}
template<typename T>
T
BaseController::limit_range(const T val, const T min_val, const T max_val) {
    if (val < min_val) {
        return min_val;
    }
    else if (val > max_val) {
        return max_val;
    }
    return val;
}

#endif /* end of include guard */
