#include <Arduino.h>
#include <Wire.h>

#include <STAR_ISM330DH.h>

#include "main.h"

static ISM330DH imu;
static sfe_ism_data_t accelData{};
static sfe_ism_data_t gyroData{};

static volatile bool g_intFlag = false;

void IRAM_ATTR onInt1Isr() {
	g_intFlag = true;
}

void setup() {
	Serial.begin(115200);
	while (!Serial)
		delay(10);
	delay(50);

	Wire.begin(ISM330_SDA_PIN, ISM330_SCL_PIN);

	if (!imu.begin(Wire, ISM330_I2C_ADDR)) {
		Serial.println("ERROR: ISM330DHCX not found. Check I2C wiring and address (0x6A / 0x6B).");
		while (true)
			delay(1000);
	}

	imu.deviceReset();
	while (!imu.getDeviceReset())
		delay(1);
	delay(100);

	imu.setDeviceConfig(true);
	imu.setBlockDataUpdate(true);

	imu.setAccelDataRate(ISM_XL_ODR_104Hz);
	imu.setAccelFullScale(ISM330_ACCEL_FS);
	imu.setGyroDataRate(ISM_GY_ODR_104Hz);
	imu.setGyroFullScale(ISM_500dps);

	imu.setAccelFilterLP2(true);
	imu.setAccelSlopeFilter(ISM_LP_ODR_DIV_100);
	imu.setGyroFilterLP1(true);
	imu.setGyroLP1Bandwidth(ISM_MEDIUM);

	if (!imu.configureWakeUpOnInt1(ISM330_WKUP_THS, ISM330_WAKEUP_DUR, ISM330_ROUTE_DRDY_TO_INT1)) {
		Serial.println("ERROR: configureWakeUpOnInt1 failed.");
		while (true)
			delay(1000);
	}

	// Active high on INT1 (use RISING on the MCU)
	imu.setPinMode(false);

	pinMode(ISM330_INT1_PIN, INPUT);
	attachInterrupt(digitalPinToInterrupt(ISM330_INT1_PIN), onInt1Isr, RISING);

	Serial.println();
	Serial.println("ISM330DHCX test (ISM330DH)");
	Serial.printf("Accel FS: %.0f g  wake-up threshold: %.2f g  WK_THS LSBs: %u  wake_dur: %u\r\n",
	              ISM330_ACCEL_FS_G, ISM330_WAKEUP_THRESHOLD_G,
	              static_cast<unsigned>(ISM330_WKUP_THS),
	              static_cast<unsigned>(ISM330_WAKEUP_DUR));
	Serial.printf("INT1 GPIO: %d\r\n", ISM330_INT1_PIN);
	Serial.println("ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps");
}

void loop() {
	if (g_intFlag) {
		g_intFlag = false;
		sfe_ism330_wake_event_t wu{};
		if (imu.serviceWakeUpInterrupt(&wu)) {
			Serial.print("[INT] Wake-up  axes:");
			if (wu.axisX)
				Serial.print(" X");
			if (wu.axisY)
				Serial.print(" Y");
			if (wu.axisZ)
				Serial.print(" Z");
			Serial.println();
		}
	}

	if (imu.checkStatus()) {
		if (imu.getAccel(&accelData) && imu.getGyro(&gyroData)) {
			Serial.print(accelData.xData / 1000.0f, 4);
			Serial.print(",");
			Serial.print(accelData.yData / 1000.0f, 4);
			Serial.print(",");
			Serial.print(accelData.zData / 1000.0f, 4);
			Serial.print(",");
			Serial.print(gyroData.xData / 1000.0f, 4);
			Serial.print(",");
			Serial.print(gyroData.yData / 1000.0f, 4);
			Serial.print(",");
			Serial.println(gyroData.zData / 1000.0f, 4);
		}
	}

	delay(10);
}
