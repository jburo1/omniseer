#pragma once

#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>

namespace omniseer_core {

struct ImuSample {
        // microseconds since Teensy boot
        uint32_t t_us;

        // orientation (quaternion)
        float qx, qy, qz, qw;

        // angular velocity
        float wx, wy, wz;       // rad/s

        // linear acceleration (body frame, gravity removed)
        float ax, ay, az;       // m/s^2

        // calibration status (0–3)
        uint8_t calib_sys;
        uint8_t calib_gyro;
        uint8_t calib_accel;
        uint8_t calib_mag;
    };

class ImuBno055 {
public:

    ImuBno055(TwoWire& wire, int32_t sensor_id = 55, uint8_t addr = 0x28);

    bool init();
    void update();

    const ImuSample& latest() const { return _sample; }
    const bool is_healthy() const { return _healthy; }

private:
    Adafruit_BNO055 _bno;
    bool            _healthy = false;
    ImuSample       _sample{};
};

} // namespace omniseer_core
