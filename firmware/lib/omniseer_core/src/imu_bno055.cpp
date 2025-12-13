#include "omniseer_core/imu_bno055.hpp"
#include <Arduino.h>

namespace omniseer_core {

ImuBno055::ImuBno055(TwoWire& wire, int32_t sensor_id, uint8_t addr)
: _bno(sensor_id, addr, &wire)
{}

bool ImuBno055::init() {
    if (!_bno.begin()) {
        Serial.println("ERROR: BNO055 not detected. Check wiring / I2C addr.");
        _healthy = false;
        return false;
    }

    _bno.setExtCrystalUse(true);

    // gyro + accel only
    _bno.setMode(OPERATION_MODE_IMUPLUS);

    delay(20);

    _healthy = true;

    Serial.println("IMU online.");

    return true;
}

void ImuBno055::update() {
    if (!_healthy) return;

    _sample.t_us = micros();

    imu::Vector<3> ang = _bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

    _sample.wx = ang.x() * DEG_TO_RAD;
    _sample.wy = ang.y() * DEG_TO_RAD;
    _sample.wz = ang.z() * DEG_TO_RAD;

    imu::Vector<3> acc = _bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

    _sample.ax = acc.x();
    _sample.ay = acc.y();
    _sample.az = acc.z();

    imu::Quaternion q = _bno.getQuat();

    _sample.qw = q.w();
    _sample.qx = q.x();
    _sample.qy = q.y();
    _sample.qz = q.z();

    uint8_t sys, gyro, accel, mag;
    _bno.getCalibration(&sys, &gyro, &accel, &mag);

    _sample.calib_sys   = sys;
    _sample.calib_gyro  = gyro;
    _sample.calib_accel = accel;
    _sample.calib_mag   = mag;
}

} //namespace omniseer_core
