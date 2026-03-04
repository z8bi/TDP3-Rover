#pragma once
#include "mbed.h"

// Simple TCS3472/TCS34725 reader for mbed (FRDM-KL25Z).
// Reading only: init + read raw RGBC (+ optional lux/CCT helpers).
namespace tcs3472 {

// Gain settings (CONTROL register)
enum class Gain : uint8_t {
    X1  = 0x00,
    X4  = 0x01,
    X16 = 0x02,
    X60 = 0x03
};

struct RGBC {
    uint16_t c;
    uint16_t r;
    uint16_t g;
    uint16_t b;
    bool     valid;
};

class TCS3472 {
public:
    // Defaults use mbed I2C_SDA/I2C_SCL pins (KL25Z: D14/D15).
    TCS3472(PinName sda = I2C_SDA, PinName scl = I2C_SCL, int hz = 100000);

    // Configure integration time + gain, power on and enable ADC.
    // integration_ms typical: 2.4 .. 614.4 ms (will be clamped).
    bool init(float integration_ms = 50.0f, Gain gain = Gain::X16);

    // Read raw RGBC. If wait_for_valid=true, poll STATUS_AVALID until timeout_ms.
    RGBC read_raw(bool wait_for_valid = true, int timeout_ms = 120);

    // Optional rough helpers (need calibration for absolute accuracy).
    float estimate_lux(const RGBC& v) const;
    float estimate_cct_kelvin(const RGBC& v) const;

private:
    I2C   _i2c;
    float _integration_ms = 50.0f;
    Gain  _gain = Gain::X16;

    static uint8_t atime_from_ms(float ms);
    static float   gain_multiplier(Gain g);

    bool write8(uint8_t reg, uint8_t val);
    bool read8(uint8_t reg, uint8_t &out);
    bool readN(uint8_t start_reg, uint8_t* buf, size_t n);
};

} // namespace tcs3472