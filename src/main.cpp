// main.cpp (FRDM-KL25Z, 3x TCRT5000 -> 3x LM311 comparators, L298N) + RGB debug LED
// Change requested:
// - If turning left (because left sensor saw line while center did not), keep pivoting left UNTIL center sees line again.
// - Same for right.
// - When center sees line again, go straight.
// - If line is lost (000 for a while), pivot in last_dir until reacquired.

#include "mbed.h"
#include "pinassignments.hpp"
#include "rovercontrol.hpp"

// ===================== User-config =====================
static constexpr bool LINE_ACTIVE_LOW = false; // black = HIGH
static constexpr chrono::milliseconds DT{5};   // 200 Hz

static constexpr int FILTER_N = 7;
static constexpr int FILTER_MAJ = (FILTER_N / 2) + 1;

// Speeds
static constexpr float V_STRAIGHT = 0.55f;  // when centered (010)
static constexpr float TANK_TURN  = 0.85f;  // pivot while correcting (left/right lock)
static constexpr float LOST_TURN  = 0.85f;  // pivot while lost

static constexpr float DUTY_MIN_MOVE = 0.28f;

// LOST detection
static constexpr int LOST_COUNT_TRIP = 6;   // 30 ms

// Clamp helpers
static inline float clamp01(float x) { return (x < 0.0f) ? 0.0f : (x > 1.0f) ? 1.0f : x; }
static inline float clamp11(float x) { return (x < -1.0f) ? -1.0f : (x > 1.0f) ? 1.0f : x; }

static float apply_deadband(float cmd) {
    if (cmd == 0.0f) return 0.0f;
    float s = (cmd > 0.0f) ? 1.0f : -1.0f;
    float mag = fabsf(cmd);
    if (mag < DUTY_MIN_MOVE) mag = DUTY_MIN_MOVE;
    return s * clamp01(mag);
}

static inline int popcount8(uint8_t x) {
    int c = 0;
    for (int i = 0; i < 8; ++i) c += (x >> i) & 1;
    return c;
}

struct FilterBit {
    uint8_t hist = 0;
    bool update(bool raw) {
        const uint8_t mask = (FILTER_N >= 8) ? 0xFFu : (uint8_t)((1u << FILTER_N) - 1u);
        hist = (uint8_t)(((hist << 1) | (raw ? 1u : 0u)) & mask);
        return popcount8(hist) >= FILTER_MAJ;
    }
};

enum class LastDir : uint8_t { LEFT, RIGHT };
enum class Mode : uint8_t { STRAIGHT, TURN_LEFT, TURN_RIGHT, LOST };

// FRDM-KL25Z on-board RGB LED (active-low)
DigitalOut led_r(LED1), led_g(LED2), led_b(LED3);
static inline void led_off() { led_r = 1; led_g = 1; led_b = 1; }
static inline void set_rgb(bool r_on, bool g_on, bool b_on) {
    led_r = r_on ? 0 : 1;
    led_g = g_on ? 0 : 1;
    led_b = b_on ? 0 : 1;
}

int main() {
    Left_TCRT.mode(PullUp);
    Centre_TCRT.mode(PullUp);
    Right_TCRT.mode(PullUp);

    motors_init(0.001f);

    FilterBit fL, fC, fR;
    LastDir last_dir = LastDir::LEFT;
    Mode mode = Mode::STRAIGHT;
    int lost_count = 0;

    while (true) {
        bool rawL = Left_TCRT.read();
        bool rawC = Centre_TCRT.read();
        bool rawR = Right_TCRT.read();

        auto to_line = [](bool raw) { return LINE_ACTIVE_LOW ? (!raw) : raw; };

        bool lineL = fL.update(to_line(rawL));
        bool lineC = fC.update(to_line(rawC));
        bool lineR = fR.update(to_line(rawR));

        // Pattern for LED/debug only
        const uint8_t pat = (uint8_t)((lineL ? 0b100 : 0) | (lineC ? 0b010 : 0) | (lineR ? 0b001 : 0));

        // LOST counter
        if (!lineL && !lineC && !lineR) lost_count++;
        else lost_count = 0;

        // Mode transitions (state machine)
        if (lost_count >= LOST_COUNT_TRIP) {
            mode = Mode::LOST;
        } else if (lineC) {
            // Center sees line: always return to straight state
            mode = Mode::STRAIGHT;
        } else {
            // Center does not see line (lineC == 0)
            // If not already locked into a turn, pick a direction based on which side sees line
            if (mode != Mode::TURN_LEFT && mode != Mode::TURN_RIGHT) {
                if (lineL) { mode = Mode::TURN_LEFT;  last_dir = LastDir::LEFT; }
                else if (lineR) { mode = Mode::TURN_RIGHT; last_dir = LastDir::RIGHT; }
            }
        }

        // LED shows current MODE (not just pattern)
        // STRAIGHT = green, TURN_LEFT = red, TURN_RIGHT = blue, LOST = magenta
        switch (mode) {
            case Mode::STRAIGHT:   set_rgb(false, true,  false); break;
            case Mode::TURN_LEFT:  set_rgb(true,  false, false); break;
            case Mode::TURN_RIGHT: set_rgb(false, false, true ); break;
            case Mode::LOST:       set_rgb(true,  false, true ); break;
            default:               led_off(); break;
        }

        float left_cmd = 0.0f;
        float right_cmd = 0.0f;

        // Actions for each mode
        if (mode == Mode::STRAIGHT) {
            left_cmd = V_STRAIGHT;
            right_cmd = V_STRAIGHT;
        } else if (mode == Mode::TURN_LEFT) {
            // Pivot left until center sees line
            left_cmd = -TANK_TURN;
            right_cmd = +TANK_TURN;
        } else if (mode == Mode::TURN_RIGHT) {
            // Pivot right until center sees line
            left_cmd = +TANK_TURN;
            right_cmd = -TANK_TURN;
        } else { // Mode::LOST
            // Pivot in last known direction until any sensor sees line again (then STRAIGHT logic will take over via lineC)
            if (last_dir == LastDir::LEFT) { left_cmd = -LOST_TURN; right_cmd = +LOST_TURN; }
            else { left_cmd = +LOST_TURN; right_cmd = -LOST_TURN; }
        }

        left_cmd  = clamp11(apply_deadband(left_cmd));
        right_cmd = clamp11(apply_deadband(right_cmd));

        motor_set(left_cmd, right_cmd);

        ThisThread::sleep_for(DT);
    }
}