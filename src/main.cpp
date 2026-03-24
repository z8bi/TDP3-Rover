// main.cpp (FRDM-KL25Z + TCS3472/TCS34725 on I2C1 using A4/A5)
// Prints raw RGBC (and optional lux/CCT) to the USB serial terminal.
//
// Wiring (per your pinout):
//   SDA = A4 = PTC2
//   SCL = A5 = PTC1
//   VCC = 3V3, GND = GND

#include "mbed.h"
#include "tcs3472.hpp"

using namespace tcs3472;

// FRDM-KL25Z USB serial is typically over UART0 (D0/D1): PTA2/PTA1.
// This matches the OpenSDA virtual COM port.
static BufferedSerial pc(USBTX, USBRX, 115200);

static void print_line(const char *s) {
    pc.write(s, strlen(s));
}

int main() {
    // Make printf-style output line-buffered via BufferedSerial
    pc.set_format(8, BufferedSerial::None, 1);
    pc.set_blocking(true);

    print_line("\r\nTCS3472 demo (FRDM-KL25Z, I2C1 on A4/A5)\r\n");

    // Explicitly use A4/A5 (I2C1 SDA/SCL)
    TCS3472 sensor(D4, D5, 100000); // try 400000 if you want faster

    const bool ok = sensor.init(50.0f, Gain::X16);
    if (!ok) {
        print_line("ERROR: sensor.init() failed. Check wiring, 3V3, GND, pullups, address.\r\n");
        while (true) {
            ThisThread::sleep_for(500ms);
        }
    }

    print_line("Init OK. Reading...\r\n");

    while (true) {
        RGBC v = sensor.read_raw(true, 250);

        if (!v.valid) {
            print_line("Read timeout / invalid\r\n");
        } else {
            const float lux = sensor.estimate_lux(v);
            const float cct = sensor.estimate_cct_kelvin(v);

            char buf[160];
            const int n = snprintf(
                buf, sizeof(buf),
                "C:%5u R:%5u G:%5u B:%5u | lux:%8.2f | CCT:%8.1f K\r\n",
                v.c, v.r, v.g, v.b, lux, cct
            );
            pc.write(buf, n);
        }

        ThisThread::sleep_for(200ms);
    }
}