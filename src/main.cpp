// main.cpp (FRDM-KL25Z, 3x TCRT5000 -> 3x LM311 comparators, L298N)
#include "mbed.h"
#include "pinassignments.hpp"
#include "rovercontrol.hpp"

// ===================== User-config =====================
// LM311 is open-collector. Common setup is pull-up + comparator pulls LOW when active.
// Set this based on what you observe.
static constexpr bool LINE_ACTIVE_LOW = true;

// Control loop timing
static constexpr chrono::milliseconds DT{5}; // 200 Hz

// Filtering (majority over last N samples). N must be <= 8 here.
static constexpr int FILTER_N = 7; // 7-sample window
static constexpr int FILTER_MAJ = (FILTER_N / 2) + 1;

// Speeds (0..1 PWM duty). Tune these on the floor.
static constexpr float V_BASE = 0.55f;
static constexpr float V_LOST = 0.35f;
static constexpr float D_SOFT = 0.18f;
static constexpr float D_HARD = 0.32f;

// Deadband compensation for L298N (many setups will not move below ~0.2 to 0.35).
static constexpr float DUTY_MIN_MOVE = 0.28f;

// LOST detection (avoid spurious 000 patterns)
static constexpr int LOST_COUNT_TRIP = 6; // 6*DT = 30ms

// Clamp helper
static inline float clamp01(float x) { return (x < 0.0f) ? 0.0f : (x > 1.0f) ? 1.0f : x; }

// Apply deadband to a signed motor command (keeps sign, raises magnitude if nonzero)
static float apply_deadband(float cmd) {
    if (cmd == 0.0f) return 0.0f;
    float s = (cmd > 0.0f) ? 1.0f : -1.0f;
    float mag = fabsf(cmd);
    if (mag < DUTY_MIN_MOVE) mag = DUTY_MIN_MOVE;
    return s * clamp01(mag);
}

// Popcount for up to 8 bits
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

int main() {
    // Ensure pull-ups for LM311 open-collector outputs.
    // If you already have external pull-ups, internal pull-ups usually still work.
    Left_TCRT.mode(PullUp);
    Centre_TCRT.mode(PullUp);
    Right_TCRT.mode(PullUp);

    motors_init(0.001f); // 1 kHz PWM period (as you used)

    FilterBit fL, fC, fR;

    LastDir last_dir = LastDir::LEFT;
    int lost_count = 0;

    while (true) {
        // Raw reads
        bool rawL = Left_TCRT.read();
        bool rawC = Centre_TCRT.read();
        bool rawR = Right_TCRT.read();

        // Convert to "line detected" boolean with chosen polarity
        auto to_line = [](bool raw) {
            return LINE_ACTIVE_LOW ? (!raw) : raw;
        };

        bool lineL = fL.update(to_line(rawL));
        bool lineC = fC.update(to_line(rawC));
        bool lineR = fR.update(to_line(rawR));

        // Pattern bits: L C R
        const uint8_t pat = (uint8_t)((lineL ? 0b100 : 0) | (lineC ? 0b010 : 0) | (lineR ? 0b001 : 0));

        float left_cmd = 0.0f;
        float right_cmd = 0.0f;

        // LOST handling
        if (pat == 0b000) {
            lost_count++;
        } else {
            lost_count = 0;
        }

        if (lost_count >= LOST_COUNT_TRIP) {
            // LOST: arc-turn toward last known direction
            if (last_dir == LastDir::LEFT) {
                left_cmd  = V_LOST - D_HARD;
                right_cmd = V_LOST + D_HARD;
            } else {
                left_cmd  = V_LOST + D_HARD;
                right_cmd = V_LOST - D_HARD;
            }
        } else {
            // FOLLOW: pattern map with soft/hard turns
            switch (pat) {
                case 0b010: // centered
                    left_cmd = V_BASE;
                    right_cmd = V_BASE;
                    break;

                case 0b110: // left + center
                    left_cmd  = V_BASE - D_SOFT;
                    right_cmd = V_BASE + D_SOFT;
                    last_dir = LastDir::LEFT;
                    break;

                case 0b011: // center + right
                    left_cmd  = V_BASE + D_SOFT;
                    right_cmd = V_BASE - D_SOFT;
                    last_dir = LastDir::RIGHT;
                    break;

                case 0b100: // left only
                    left_cmd  = V_BASE - D_HARD;
                    right_cmd = V_BASE + D_HARD;
                    last_dir = LastDir::LEFT;
                    break;

                case 0b001: // right only
                    left_cmd  = V_BASE + D_HARD;
                    right_cmd = V_BASE - D_HARD;
                    last_dir = LastDir::RIGHT;
                    break;

                case 0b111: // wide line / intersection
                    // Policy: go straight briefly (keep v), do not update last_dir
                    left_cmd = V_BASE;
                    right_cmd = V_BASE;
                    break;

                case 0b101: // both sides, no center (could happen on edges or wide line)
                    // Policy: keep last_dir bias (prevents indecision)
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

        // Apply deadband compensation and clamp
        left_cmd = apply_deadband(left_cmd);
        right_cmd = apply_deadband(right_cmd);

        // Both forward only in this controller; keep commands non-negative.
        // If you want pivot turns, allow negative here and adjust mapping.
        left_cmd = clamp01(left_cmd);
        right_cmd = clamp01(right_cmd);

        motor_set(left_cmd, right_cmd);

        ThisThread::sleep_for(DT);
    }
}
