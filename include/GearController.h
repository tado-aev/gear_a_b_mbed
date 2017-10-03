#ifndef GEAR_CONTROLLER_H_
#define GEAR_CONTROLLER_H_

#include <mbed.h>

#include "BaseController.hpp"

#include <string>

class GearController : public BaseController {
public:
    enum ControlMode {
        MANUAL, PROGRAM,
    };

    enum Gear {
        REVERSE, NEUTRAL, DRIVE,
    };

    /**
     * Pulse duration when switching between manual and program mode
     */
    static const unsigned MODE_SWITCH_PULSE_MS = 100;

    /* Constructors, Destructor, and Assignment operators {{{ */
    GearController(PinName mode_shift_program,
                   PinName mode_shift_manual,
                   PinName shift_r,
                   PinName shift_n,
                   PinName shift_d);

    // Copy constructor
    GearController(const GearController& other) = delete;

    // Move constructor
    GearController(GearController&& other);

    // Destructor
    ~GearController();

    // Assignment operator
    GearController&
    operator=(const GearController& other) = delete;

    // Move assignment operator
    GearController&
    operator=(GearController&& other);
    /* }}} */

    void
    init();

    /**
     * Turn on control from program
     */
    void
    on();

    /**
     * Turn off control from program and switch to manual gear shifting
     */
    void
    off();

    /**
     * Switch to program control
     */
    void
    mode_program();

    /**
     * Switch to manual control (using the gear shifter)
     */
    void
    mode_manual();

    /**
     * Sets the gear to the given gear
     *
     * "R" or "r" for reverse,
     * "N" or "n" for neutral,
     * "D" or "d" for drive.
     */
    void
    set(const std::string& gear);

    /**
     * Same as set(const std::string& gear)
     */
    void
    set(const char g);

    /**
     * Sets gear to reverse
     */
    void
    set_r();

    /**
     * Sets gear to neutral
     */
    void
    set_n();

    /**
     * Sets gear to drive
     */
    void
    set_d();

    ControlMode
    get_mode() const;

    Gear
    get_gear() const;

    /**
     * Sets the duration of the overlap when shifting gears
     */
    void
    set_signal_overlap(const unsigned overlap_duration_ms);

private:
    Mutex com_mutex;

    ControlMode current_mode;
    Gear current_gear;

    // Shifting from a program
    DigitalOut mode_shift_program;
    // Shifting manually with the gear selector
    DigitalOut mode_shift_manual;
    // Reverse, Neutral, and Drive gears
    DigitalOut shift_r;
    DigitalOut shift_n;
    DigitalOut shift_d;
    /**
     * There is an overlap of the signals when shifting gears. For example,
     * when switching from neutral to drive, there is an approximately 10ms
     * overlap where both neutral and drive are high. It is presumed that this
     * overlap is desirable when switching gears programmatically, hence, this
     * member variable is introduced.
     */
    unsigned signal_overlap_ms;
};

#endif /* end of include guard */
