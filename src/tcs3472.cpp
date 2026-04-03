#include "tcs3472.hpp"

namespace tcs3472 {

// TCS3472/TCS34725 constants
static constexpr int      ADDR7  = 0x29;
static constexpr uint8_t  CMD    = 0x80;

static constexpr uint8_t REG_ENABLE  = 0x00;
static constexpr uint8_t REG_ATIME   = 0x01;
static constexpr uint8_t REG_CONTROL = 0x0F;
static constexpr uint8_t REG_STATUS  = 0x13;

static constexpr uint8_t REG_CDATAL  = 0x14; // C low byte, then C high, R low, R high, ...
static constexpr uint8_t ENABLE_PON  = 0x01;
static constexpr uint8_t ENABLE_AEN  = 0x02;

static constexpr uint8_t STATUS_AVALID = 0x01;

static uint16_t u16le(const uint8_t *p) {
    return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
}

TCS3472::TCS3472(PinName sda, PinName scl, int hz)
    : _i2c(sda, scl) {
    _i2c.frequency(hz);
}

uint8_t TCS3472::atime_from_ms(float ms) {
    // clamp to sensor range: 2.4ms .. 614.4ms
    if (ms < 2.4f)   ms = 2.4f;
    if (ms > 614.4f) ms = 614.4f;

    // ms = (256 - ATIME) * 2.4
    // ATIME = 256 - ms/2.4
    const float atime_f = 256.0f - (ms / 2.4f);
    int atime = static_cast<int>(atime_f + 0.5f);
    if (atime < 0)   atime = 0;
    if (atime > 255) atime = 255;
    return static_cast<uint8_t>(atime);
}

float TCS3472::gain_multiplier(Gain g) {
    switch (g) {
        case Gain::X1:  return 1.0f;
        case Gain::X4:  return 4.0f;
        case Gain::X16: return 16.0f;
        case Gain::X60: return 60.0f;
        default:        return 1.0f;
    }
}

bool TCS3472::write8(uint8_t reg, uint8_t val) {
    char data[2];
    data[0] = static_cast<char>(CMD | reg);
    data[1] = static_cast<char>(val);
    return _i2c.write(ADDR7 << 1, data, 2) == 0;
}

bool TCS3472::read8(uint8_t reg, uint8_t &out) {
    char r = static_cast<char>(CMD | reg);
    if (_i2c.write(ADDR7 << 1, &r, 1, true) != 0) return false;
    char v = 0;
    if (_i2c.read(ADDR7 << 1, &v, 1) != 0) return false;
    out = static_cast<uint8_t>(v);
    return true;
}

bool TCS3472::readN(uint8_t start_reg, uint8_t *buf, size_t n) {
    char r = static_cast<char>(CMD | start_reg);
    if (_i2c.write(ADDR7 << 1, &r, 1, true) != 0) return false;
    return _i2c.read(ADDR7 << 1, reinterpret_cast<char*>(buf), n) == 0;
}

bool TCS3472::init(float integration_ms, Gain gain) {
    _integration_ms = integration_ms;
    _gain = gain;

    const uint8_t atime = atime_from_ms(integration_ms);

    if (!write8(REG_ENABLE, 0x00)) return false;
    if (!write8(REG_ATIME, atime)) return false;
    if (!write8(REG_CONTROL, static_cast<uint8_t>(gain))) return false;

    if (!write8(REG_ENABLE, ENABLE_PON)) return false;
    ThisThread::sleep_for(3ms);
    if (!write8(REG_ENABLE, ENABLE_PON | ENABLE_AEN)) return false;

    // wait at least one integration period before first read
    const int wait_ms = static_cast<int>(((256 - atime) * 2.4f) + 5.0f);
    ThisThread::sleep_for(chrono::milliseconds(wait_ms));
    return true;
}

RGBC TCS3472::read_raw(bool wait_for_valid, int timeout_ms) {
    RGBC out{};
    out.valid = false;

    if (wait_for_valid) {
        Timer t;
        t.start();
        while (true) {
            uint8_t st = 0;
            if (!read8(REG_STATUS, st)) return out;
            if (st & STATUS_AVALID) break;
            if (chrono::duration_cast<chrono::milliseconds>(t.elapsed_time()).count() > timeout_ms) {
                return out;
            }
            ThisThread::sleep_for(2ms);
        }
    }

    uint8_t buf[8] = {0};
    if (!readN(REG_CDATAL, buf, sizeof(buf))) return out;

    out.c = u16le(buf + 0);
    out.r = u16le(buf + 2);
    out.g = u16le(buf + 4);
    out.b = u16le(buf + 6);
    out.valid = true;
    return out;
}

// These are only rough, uncalibrated helpers.
// If you do not want them, you can remove calls from main.cpp.
float TCS3472::estimate_lux(const RGBC& v) const {
    // crude approximation from RGB, normalized by gain and integration time
    const float it = (_integration_ms <= 0.0f) ? 1.0f : _integration_ms;
    const float gmul = gain_multiplier(_gain);
    const float r = static_cast<float>(v.r);
    const float g = static_cast<float>(v.g);
    const float b = static_cast<float>(v.b);

    const float illum = 0.136f * r + 1.000f * g - 0.444f * b;
    return (illum > 0.0f) ? (illum / (gmul * it)) * 1000.0f : 0.0f;
}

float TCS3472::estimate_cct_kelvin(const RGBC& v) const {
    // McCamy-like rough estimate via XYZ transform (same style as the M5 doc you showed earlier)
    const float R = static_cast<float>(v.r);
    const float G = static_cast<float>(v.g);
    const float B = static_cast<float>(v.b);

    const float X = (2.7688f * R) + (1.7517f * G) + (1.1301f * B);
    const float Y = (1.0000f * R) + (4.5906f * G) + (0.0601f * B);
    const float Z = (0.0565f * G) + (5.5942f * B);

    const float sum = X + Y + Z;
    if (sum <= 0.0f) return 0.0f;

    const float x = X / sum;
    const float y = Y / sum;

    const float n = (x - 0.3320f) / (0.1858f - y);
    const float cct = 449.0f*n*n*n + 3525.0f*n*n + 6823.3f*n + 5520.33f;
    return (cct > 0.0f) ? cct : 0.0f;
}

} // namespace tcs3472
