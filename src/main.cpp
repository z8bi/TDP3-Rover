// main.cpp (FRDM-KL25Z, 3x TCRT5000 -> 3x LM311 comparators, L298N) + RGB debug LED
// Line-follow behaviour unchanged.
// Added:
// - TCS3472 on I2C (D14 SDA, D15 SCL)
// - If RED detected: stop
// - While stopped, if GREEN detected: resume
// - Before ANY mode change (straight <-> turn <-> lost), dynamically brake BOTH motors

#include "mbed.h"
#include "pinassignments.hpp"
#include "rovercontrol.hpp"
#include "tcs3472.hpp"

// ===================== Line follow config =====================
static constexpr bool LINE_ACTIVE_LOW = false; // black = HIGH (after LM311)
static constexpr chrono::milliseconds DT{5};   // 200 Hz

static constexpr int FILTER_N = 7;
static constexpr int FILTER_MAJ = (FILTER_N / 2) + 1;

// Speeds
static constexpr float V_STRAIGHT = 0.55f;
static constexpr float TANK_TURN  = 0.85f;
static constexpr float LOST_TURN  = 0.85f;

static constexpr float DUTY_MIN_MOVE = 0.28f;
static constexpr int   LOST_COUNT_TRIP = 6;   // 30 ms

// ===================== Colour stop/go config =====================
static constexpr chrono::milliseconds COLOR_DT{25}; // 40 Hz
static constexpr uint16_t COLOR_C_MIN = 20;

static constexpr int HITS_TO_STOP = 3; // consecutive RED samples required
static constexpr int HITS_TO_GO   = 3; // consecutive GREEN samples required

// ===================== Transition brake config =====================
static constexpr chrono::milliseconds BRAKE_TIME{50};
static constexpr float BRAKE_STRENGTH = 1.0f;

// ===================== Helpers =====================
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

enum class RunState : uint8_t { RUNNING, STOPPED };
enum class ColourDecision : uint8_t { NONE, RED, GREEN };

// FRDM-KL25Z on-board RGB LED (active-low)
DigitalOut led_r(LED1), led_g(LED2), led_b(LED3);
static inline void set_rgb(bool r_on, bool g_on, bool b_on) {
    led_r = r_on ? 0 : 1;
    led_g = g_on ? 0 : 1;
    led_b = b_on ? 0 : 1;
}

// Dominance-based classifier
static ColourDecision classify_colour(const tcs3472::RGBC& v)
{
    if (!v.valid) return ColourDecision::NONE;
    if (v.c < COLOR_C_MIN) return ColourDecision::NONE;

    const bool red =
        (v.r > (uint16_t)(v.g * 13 / 10)) &&
        (v.r > (uint16_t)(v.b * 13 / 10));

    const bool green =
        (v.g > (uint16_t)(v.r * 13 / 10)) &&
        (v.g > (uint16_t)(v.b * 13 / 10));

    if (red) return ColourDecision::RED;
    if (green) return ColourDecision::GREEN;
    return ColourDecision::NONE;
}

int main() {
    Left_TCRT.mode(PullUp);
    Centre_TCRT.mode(PullUp);
    Right_TCRT.mode(PullUp);

    motors_init(0.001f);

    // TCS3472 on D14 (SDA), D15 (SCL)
    tcs3472::TCS3472 colour(D14, D15, 100000);
    const bool colour_ok = colour.init(154.0f, tcs3472::Gain::X60);

    if (!colour_ok) {
        set_rgb(true, true, true); // white at boot indicates color init fail
        ThisThread::sleep_for(500ms);
    }

    Timer colour_timer;
    colour_timer.start();

    RunState run_state = RunState::RUNNING;
    int red_hits = 0;
    int green_hits = 0;

    FilterBit fL, fC, fR;
    LastDir last_dir = LastDir::LEFT;

    Mode active_mode = Mode::STRAIGHT;
    Mode desired_mode = Mode::STRAIGHT;

    int lost_count = 0;

    bool braking = false;
    Timer brake_timer;
    brake_timer.start();

    while (true) {
        // ---------- Colour sampling + stop/go ----------
        if (colour_ok && (colour_timer.elapsed_time() >= COLOR_DT)) {
            colour_timer.reset();

            const tcs3472::RGBC v = colour.read_raw(true, 120);
            const ColourDecision cd = classify_colour(v);

            if (cd == ColourDecision::RED)   red_hits++;   else red_hits = 0;
            if (cd == ColourDecision::GREEN) green_hits++; else green_hits = 0;

            if (run_state == RunState::RUNNING) {
                if (red_hits >= HITS_TO_STOP) {
                    run_state = RunState::STOPPED;
                    green_hits = 0;
                }
            } else {
                if (green_hits >= HITS_TO_GO) {
                    run_state = RunState::RUNNING;
                    red_hits = 0;
                }
            }
        }

        // ---------- STOPPED overrides everything ----------
        if (run_state == RunState::STOPPED) {
            motor_brake(BRAKE_STRENGTH);
            set_rgb(true, true, false); // yellow = stopped
            ThisThread::sleep_for(DT);
            continue;
        }

        // ---------- Line sensors ----------
        const bool rawL = Left_TCRT.read();
        const bool rawC = Centre_TCRT.read();
        const bool rawR = Right_TCRT.read();

        auto to_line = [](bool raw) { return LINE_ACTIVE_LOW ? (!raw) : raw; };

        const bool lineL = fL.update(to_line(rawL));
        const bool lineC = fC.update(to_line(rawC));
        const bool lineR = fR.update(to_line(rawR));

        if (!lineL && !lineC && !lineR) lost_count++;
        else lost_count = 0;

        // ---------- Desired mode ----------
        if (lost_count >= LOST_COUNT_TRIP) {
            desired_mode = Mode::LOST;
        }
        else if (lineL && lineC && !lineR) {
            desired_mode = Mode::TURN_LEFT;   // 110
            last_dir = LastDir::LEFT;
        }
        else if (!lineL && lineC && lineR) {
            desired_mode = Mode::TURN_RIGHT;  // 011
            last_dir = LastDir::RIGHT;
        }
        else if (lineC) {
            desired_mode = Mode::STRAIGHT;
        }
        else {
            if (active_mode != Mode::TURN_LEFT && active_mode != Mode::TURN_RIGHT) {
                if (lineL) {
                    desired_mode = Mode::TURN_LEFT;
                    last_dir = LastDir::LEFT;
                }
                else if (lineR) {
                    desired_mode = Mode::TURN_RIGHT;
                    last_dir = LastDir::RIGHT;
                }
                else {
                    desired_mode = active_mode; // hold
                }
            } else {
                // stay locked in a turn until center sees line again
                desired_mode = active_mode;
            }
        }

        // ---------- Transition brake before mode change ----------
        if (!braking && (desired_mode != active_mode)) {
            braking = true;
            brake_timer.reset();
        }

        if (braking) {
            motor_brake(BRAKE_STRENGTH);
            set_rgb(true, true, true); // white = braking

            if (brake_timer.elapsed_time() >= BRAKE_TIME) {
                braking = false;
                active_mode = desired_mode;
            }

            ThisThread::sleep_for(DT);
            continue;
        }

        // ---------- LED indication ----------
        switch (active_mode) {
            case Mode::STRAIGHT:   set_rgb(false, true,  false); break;
            case Mode::TURN_LEFT:  set_rgb(true,  false, false); break;
            case Mode::TURN_RIGHT: set_rgb(false, false, true ); break;
            case Mode::LOST:       set_rgb(true,  false, true ); break;
            default:               set_rgb(false, false, false); break;
        }

        // ---------- Motor commands ----------
        float left_cmd = 0.0f;
        float right_cmd = 0.0f;

        if (active_mode == Mode::STRAIGHT) {
            left_cmd = V_STRAIGHT;
            right_cmd = V_STRAIGHT;
        } else if (active_mode == Mode::TURN_LEFT) {
            left_cmd = -TANK_TURN;
            right_cmd = +TANK_TURN;
        } else if (active_mode == Mode::TURN_RIGHT) {
            left_cmd = +TANK_TURN;
            right_cmd = -TANK_TURN;
        } else { // LOST
            if (last_dir == LastDir::LEFT) {
                left_cmd = -LOST_TURN;
                right_cmd = +LOST_TURN;
            } else {
                left_cmd = +LOST_TURN;
                right_cmd = -LOST_TURN;
            }
        }

        left_cmd  = clamp11(apply_deadband(left_cmd));
        right_cmd = clamp11(apply_deadband(right_cmd));

        motor_set(left_cmd, right_cmd);

        ThisThread::sleep_for(DT);
    }
}