// main.cpp (FRDM-KL25Z, 3x TCRT5000 -> 3x LM311 comparators, L298N) + RGB debug LED
#include "mbed.h"
#include "pinassignments.hpp"
#include "rovercontrol.hpp"

// ===================== User-config =====================
static constexpr bool LINE_ACTIVE_LOW = false; // black = HIGH

static constexpr chrono::milliseconds DT{5}; // 200 Hz

static constexpr int FILTER_N = 7;
static constexpr int FILTER_MAJ = (FILTER_N / 2) + 1;

static constexpr float V_BASE = 0.45f;
static constexpr float V_LOST = 0.35f;

static constexpr float D_SOFT = 0.18f;
static constexpr float D_HARD = 0.35f;
static constexpr float D_LOST = 0.45f;

static constexpr float DUTY_MIN_MOVE = 0.28f;

static constexpr int LOST_COUNT_TRIP = 6; // 30ms

// Tank/pivot strength for hard turns and LOST
static constexpr float TANK_TURN = 0.90f;

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
// jejjejejejejedej
// FRDM-KL25Z on-board RGB LED (active-low)
DigitalOut led_r(LED1);
DigitalOut led_g(LED2);
DigitalOut led_b(LED3);

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
    int lost_count = 0;

    led_off();

    while (true) {
        bool rawL = Left_TCRT.read();
        bool rawC = Centre_TCRT.read();
        bool rawR = Right_TCRT.read();

        auto to_line = [](bool raw) { return LINE_ACTIVE_LOW ? (!raw) : raw; };

        bool lineL = fL.update(to_line(rawL));
        bool lineC = fC.update(to_line(rawC));
        bool lineR = fR.update(to_line(rawR));

        // swap left/right after filtering
        bool tmp = lineL;
        lineL = lineR;
        lineR = tmp;

        const uint8_t pat = (uint8_t)((lineL ? 0b100 : 0) | (lineC ? 0b010 : 0) | (lineR ? 0b001 : 0));

        switch (pat) {
            case 0b010: set_rgb(false, true,  false); break; // green
            case 0b110: set_rgb(true,  true,  false); break; // yellow
            case 0b011: set_rgb(false, true,  true ); break; // cyan
            case 0b100: set_rgb(true,  false, false); break; // red
            case 0b001: set_rgb(false, false, true ); break; // blue
            case 0b000: set_rgb(true,  false, true ); break; // magenta
            case 0b111: set_rgb(true,  true,  true ); break; // white
            case 0b101: set_rgb(true,  true,  true ); break;
            default:    led_off(); break;
        }

        float left_cmd = 0.0f;
        float right_cmd = 0.0f;

        if (pat == 0b000) lost_count++;
        else lost_count = 0;

        // LOST: tank pivot in last known direction
        if (lost_count >= LOST_COUNT_TRIP) {
            if (last_dir == LastDir::LEFT) {
                left_cmd  = -TANK_TURN;
                right_cmd = +TANK_TURN;
            } else {
                left_cmd  = +TANK_TURN;
                right_cmd = -TANK_TURN;
            }
        } else {
            switch (pat) {
                case 0b010:
                    left_cmd = V_BASE;
                    right_cmd = V_BASE;
                    break;

                case 0b110:
                    left_cmd  = V_BASE - D_SOFT;
                    right_cmd = V_BASE + D_SOFT;
                    last_dir = LastDir::LEFT;
                    break;

                case 0b011:
                    left_cmd  = V_BASE + D_SOFT;
                    right_cmd = V_BASE - D_SOFT;
                    last_dir = LastDir::RIGHT;
                    break;

                // Hard turns: tank pivot
                case 0b100: // hard left
                    left_cmd  = -TANK_TURN;
                    right_cmd = +TANK_TURN;
                    last_dir = LastDir::LEFT;
                    break;

                case 0b001: // hard right
                    left_cmd  = +TANK_TURN;
                    right_cmd = -TANK_TURN;
                    last_dir = LastDir::RIGHT;
                    break;

                case 0b111:
                    left_cmd = V_BASE;
                    right_cmd = V_BASE;
                    break;

                case 0b101:
                    if (last_dir == LastDir::LEFT) {
                        left_cmd  = V_BASE - D_SOFT;
                        right_cmd = V_BASE + D_SOFT;
                    } else {
                        left_cmd  = V_BASE + D_SOFT;
                        right_cmd = V_BASE - D_SOFT;
                    }
                    break;

                default:
                    left_cmd = V_BASE;
                    right_cmd = V_BASE;
                    break;
            }
        }

        left_cmd = apply_deadband(left_cmd);
        right_cmd = apply_deadband(right_cmd);

        // Signed clamp (allows reverse for tank steering)
        left_cmd = clamp11(left_cmd);
        right_cmd = clamp11(right_cmd);

        motor_set(left_cmd, right_cmd);

        ThisThread::sleep_for(DT);
    }
}
