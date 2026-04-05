#pragma once

#include <cstdint>

#include <STAR_ISM330DH.h>

// I2C (match other IC_Testing boards; change if your wiring differs)
#define ISM330_SDA_PIN 0
#define ISM330_SCL_PIN 1

// 7-bit: 0x6A SA0 low, 0x6B SA0 high (ISM330DH::begin defaults to HIGH)
#define ISM330_I2C_ADDR 0x6B

// GPIO connected to ISM330 INT1 (active high after setPinMode(false))
#define ISM330_INT1_PIN 5

// Accelerometer full scale — keep ISM330_ACCEL_FS_G in sync for wake-up math
#define ISM330_ACCEL_FS ISM_16g
#define ISM330_ACCEL_FS_G 16.0f

// Wake-up threshold in g (similar role to LIS3DH INTERRUPT_THRESHOLD_G)
#define ISM330_WAKEUP_THRESHOLD_G 2.5f

// Wake duration field 0–3 (1 LSB = 1 / ODR)
#define ISM330_WAKEUP_DUR 2

// If true, XL/GYRO data-ready also pulse INT1 (very chatty at high ODR)
#define ISM330_ROUTE_DRDY_TO_INT1 false

// ST driver: wake LSB weight FS/64 → mg per threshold step
static constexpr uint8_t ISM330_WKUP_THS = []() -> uint8_t {
	constexpr float mgPerLsb = (ISM330_ACCEL_FS_G * 1000.0f) / 64.0f;
	float raw = (ISM330_WAKEUP_THRESHOLD_G * 1000.0f) / mgPerLsb;
	uint8_t v = (raw > 63.0f) ? 63 : static_cast<uint8_t>(raw);
	return (v < 1) ? 1 : v;
}();
