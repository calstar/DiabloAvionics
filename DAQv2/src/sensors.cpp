#include "daqv2/sensors.hpp"

#include <cmath>
#include <cstdint>

#include "daqv2/time_utils.hpp"

namespace {
float fast_rand_unit() {
    static uint32_t state = 0xA5A5A5A5u;
    state = state * 1664525u + 1013904223u;
    return (float)((state >> 8) & 0xFFFFFF) * (1.0f / 16777215.0f);
}

float wobble(float base, float swing, float angle, float noise_shift) {
    float noise = (fast_rand_unit() - 0.5f) * swing * 0.05f;
    return base + swing * 0.5f * std::sin(angle + noise_shift) + noise;
}
} // namespace

void read_sensors_into(SystemParameters& p) {
    const float phase = (float)(now_us() & 0xFFFFFFu) * (1.0f / 16777216.0f);
    const float angle = phase * 6.2831853f;

    p.PT_HP = wobble(350.0f, 40.0f, angle, 0.0f);
    p.PT_LP = wobble(60.0f, 10.0f, angle, 0.5f);
    p.PT_F  = wobble(420.0f, 60.0f, angle, 1.0f);
    p.PT_O  = wobble(400.0f, 55.0f, angle, 1.5f);

    p.RTD_O1 = wobble(70.0f, 5.0f, angle, 0.2f);
    p.RTD_O2 = wobble(68.0f, 5.0f, angle, 1.2f);
    p.RTD_O3 = wobble(66.0f, 5.0f, angle, 2.2f);

    p.PT_I  = wobble(500.0f, 40.0f, angle, 0.3f);
    p.TC_I  = wobble(120.0f, 15.0f, angle, 0.9f);
    p.PT_C1 = wobble(230.0f, 25.0f, angle, 1.6f);
    p.TC_C1 = wobble(95.0f, 12.0f, angle, 2.4f);
    p.TC_C3 = wobble(90.0f, 11.0f, angle, 0.8f);
    p.TC_C2 = wobble(88.0f, 11.0f, angle, 1.4f);
    p.PT_C2 = wobble(210.0f, 20.0f, angle, 2.0f);
    p.TC_C4 = wobble(92.0f, 11.0f, angle, 2.6f);

    const bool toggled = phase < 0.5f;
    p.SOL_PV  = toggled;
    p.SOL_FUP = toggled;
    p.SOL_FV  = !toggled;
    p.SOL_OUP = toggled;
    p.SOL_OV  = !toggled;
    p.SOL_FDP = toggled;
    p.MOT_ODP = !toggled;

    p.ROT_MF = toggled;
    p.ROT_MO = !toggled;
}
