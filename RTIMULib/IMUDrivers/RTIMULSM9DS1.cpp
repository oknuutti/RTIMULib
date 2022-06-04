////////////////////////////////////////////////////////////////////////////
//
//  This file is part of RTIMULib
//
//  Copyright (c) 2014-2015, richards-tech, LLC
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of
//  this software and associated documentation files (the "Software"), to deal in
//  the Software without restriction, including without limitation the rights to use,
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
//  Software, and to permit persons to whom the Software is furnished to do so,
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.


#include "RTIMULSM9DS1.h"
#include "RTIMUSettings.h"

//  this sets the learning rate for compass running average calculation

#define COMPASS_ALPHA 0.2f

RTIMULSM9DS1::RTIMULSM9DS1(RTIMUSettings *settings) : RTIMU(settings)
{
    m_sampleRate = 100;
    m_samples = 0;
}

RTIMULSM9DS1::~RTIMULSM9DS1()
{
}

bool RTIMULSM9DS1::IMUInit()
{
    unsigned char result;

    // set validity flags

    m_imuData.fusionPoseValid = false;
    m_imuData.fusionQPoseValid = false;
    m_imuData.gyroValid = true;
    m_imuData.accelValid = true;
    m_imuData.compassValid = true;
    m_imuData.pressureValid = false;
    m_imuData.temperatureValid = false;
    m_imuData.humidityValid = false;

    //  configure IMU

    m_accelGyroSlaveAddr = m_settings->m_I2CSlaveAddress;

    // work outmag address

    if (m_settings->HALRead(LSM9DS1_MAG_ADDRESS0, LSM9DS1_MAG_WHO_AM_I, 1, &result, "")) {
        if (result == LSM9DS1_MAG_ID) {
            m_magSlaveAddr = LSM9DS1_MAG_ADDRESS0;
        }
    } else if (m_settings->HALRead(LSM9DS1_MAG_ADDRESS1, LSM9DS1_MAG_WHO_AM_I, 1, &result, "")) {
        if (result == LSM9DS1_MAG_ID) {
            m_magSlaveAddr = LSM9DS1_MAG_ADDRESS1;
        }
    } else if (m_settings->HALRead(LSM9DS1_MAG_ADDRESS2, LSM9DS1_MAG_WHO_AM_I, 1, &result, "")) {
        if (result == LSM9DS1_MAG_ID) {
            m_magSlaveAddr = LSM9DS1_MAG_ADDRESS2;
        }
    } else if (m_settings->HALRead(LSM9DS1_MAG_ADDRESS3, LSM9DS1_MAG_WHO_AM_I, 1, &result, "")) {
        if (result == LSM9DS1_MAG_ID) {
            m_magSlaveAddr = LSM9DS1_MAG_ADDRESS3;
        }
    }

    setCalibrationData();

    //  enable the I2C bus

    if (!m_settings->HALOpen())
        return false;

    //  Set up the gyro/accel

    if (!m_settings->HALWrite(m_accelGyroSlaveAddr, LSM9DS1_CTRL8, 0x84, "Failed to boot LSM9DS1"))
        return false;

    m_settings->delayMs(100);

    if (!m_settings->HALRead(m_accelGyroSlaveAddr, LSM9DS1_WHO_AM_I, 1, &result, "Failed to read LSM9DS1 accel/gyro id"))
        return false;

    if (result != LSM9DS1_ID) {
        HAL_ERROR1("Incorrect LSM9DS1 gyro id %d\n", result);
        return false;
    }

    if (!setGyroSampleRate())
            return false;

    if (!setGyroCTRL3())
            return false;

    //  Set up the mag

    if (!m_settings->HALRead(m_magSlaveAddr, LSM9DS1_MAG_WHO_AM_I, 1, &result, "Failed to read LSM9DS1 accel/mag id"))
        return false;

    if (result != LSM9DS1_MAG_ID) {
        HAL_ERROR1("Incorrect LSM9DS1 accel/mag id %d\n", result);
        return false;
    }

    if (!setAccelCTRL6())
        return false;

    if (!setAccelCTRL7())
        return false;

    if (!setAccelGyroCTRL9())
        return false;

    if (!setFifoCTRL())
        return false;

    if (!setCompassCTRL1())
        return false;

    if (!setCompassCTRL2())
        return false;

    if (!setCompassCTRL3())
        return false;

    gyroBiasInit();

    HAL_INFO("LSM9DS1 init complete\n");
    return true;
}

bool RTIMULSM9DS1::setGyroSampleRate()
{
    unsigned char ctrl1;

    switch (m_settings->m_LSM9DS1GyroSampleRate) {
    case LSM9DS1_GYRO_SAMPLERATE_14_9:
        ctrl1 = 0x20;
        m_sampleRate = 15;
        break;

    case LSM9DS1_GYRO_SAMPLERATE_59_5:
        ctrl1 = 0x40;
        m_sampleRate = 60;
        break;

    case LSM9DS1_GYRO_SAMPLERATE_119:
        ctrl1 = 0x60;
        m_sampleRate = 119;
        break;

    case LSM9DS1_GYRO_SAMPLERATE_238:
        ctrl1 = 0x80;
        m_sampleRate = 238;
        break;

    case LSM9DS1_GYRO_SAMPLERATE_476:
        ctrl1 = 0xa0;
        m_sampleRate = 476;
        break;

    case LSM9DS1_GYRO_SAMPLERATE_952:
        ctrl1 = 0xc0;
        m_sampleRate = 952;
        break;

    default:
        HAL_ERROR1("Illegal LSM9DS1 gyro sample rate code %d\n", m_settings->m_LSM9DS1GyroSampleRate);
        return false;
    }

    m_sampleInterval = (uint64_t)1000000 / m_sampleRate;

    switch (m_settings->m_LSM9DS1GyroBW) {
    case LSM9DS1_GYRO_BANDWIDTH_0:
        ctrl1 |= 0x00;
        break;

    case LSM9DS1_GYRO_BANDWIDTH_1:
        ctrl1 |= 0x01;
        break;

    case LSM9DS1_GYRO_BANDWIDTH_2:
        ctrl1 |= 0x02;
        break;

    case LSM9DS1_GYRO_BANDWIDTH_3:
        ctrl1 |= 0x03;
        break;
    }

    switch (m_settings->m_LSM9DS1GyroFsr) {
    case LSM9DS1_GYRO_FSR_250:
        ctrl1 |= 0x00;
        m_gyroScale = (RTFLOAT)0.00875 * RTMATH_DEGREE_TO_RAD;
        break;

    case LSM9DS1_GYRO_FSR_500:
        ctrl1 |= 0x08;
        m_gyroScale = (RTFLOAT)0.0175 * RTMATH_DEGREE_TO_RAD;
        break;

    case LSM9DS1_GYRO_FSR_2000:
        ctrl1 |= 0x18;
        m_gyroScale = (RTFLOAT)0.07 * RTMATH_DEGREE_TO_RAD;
        break;

    default:
        HAL_ERROR1("Illegal LSM9DS1 gyro FSR code %d\n", m_settings->m_LSM9DS1GyroFsr);
        return false;
    }
    return (m_settings->HALWrite(m_accelGyroSlaveAddr, LSM9DS1_CTRL1, ctrl1, "Failed to set LSM9DS1 gyro CTRL1"));
}

bool RTIMULSM9DS1::setGyroCTRL3()
{
    unsigned char ctrl3;

    if ((m_settings->m_LSM9DS1GyroHpf < LSM9DS1_GYRO_HPF_0) || (m_settings->m_LSM9DS1GyroHpf > LSM9DS1_GYRO_HPF_9)) {
        HAL_ERROR1("Illegal LSM9DS1 gyro high pass filter code %d\n", m_settings->m_LSM9DS1GyroHpf);
        return false;
    }
    ctrl3 = m_settings->m_LSM9DS1GyroHpf;

    //  Turn on hpf
    ctrl3 |= 0x40;

    return m_settings->HALWrite(m_accelGyroSlaveAddr,  LSM9DS1_CTRL3, ctrl3, "Failed to set LSM9DS1 gyro CTRL3");
}

bool RTIMULSM9DS1::setAccelCTRL6()
{
    unsigned char ctrl6;

    if ((m_settings->m_LSM9DS1AccelSampleRate < 0) || (m_settings->m_LSM9DS1AccelSampleRate > 6)) {
        HAL_ERROR1("Illegal LSM9DS1 accel sample rate code %d\n", m_settings->m_LSM9DS1AccelSampleRate);
        return false;
    }

    ctrl6 = (m_settings->m_LSM9DS1AccelSampleRate << 5);

    if ((m_settings->m_LSM9DS1AccelLpf < 0) || (m_settings->m_LSM9DS1AccelLpf > 3)) {
        HAL_ERROR1("Illegal LSM9DS1 accel low pass fiter code %d\n", m_settings->m_LSM9DS1AccelLpf);
        return false;
    }

    switch (m_settings->m_LSM9DS1AccelFsr) {
    case LSM9DS1_ACCEL_FSR_2:
        m_accelScale = (RTFLOAT)0.000061;
        break;

    case LSM9DS1_ACCEL_FSR_4:
        m_accelScale = (RTFLOAT)0.000122;
        break;

    case LSM9DS1_ACCEL_FSR_8:
        m_accelScale = (RTFLOAT)0.000244;
        break;

    case LSM9DS1_ACCEL_FSR_16:
        m_accelScale = (RTFLOAT)0.000732;
        break;

    default:
        HAL_ERROR1("Illegal LSM9DS1 accel FSR code %d\n", m_settings->m_LSM9DS1AccelFsr);
        return false;
    }

    ctrl6 |= (m_settings->m_LSM9DS1AccelLpf) | (m_settings->m_LSM9DS1AccelFsr << 3);

    // if not set, LPF cutoff defined based on ODR - 119 Hz: 50 Hz, 238 Hz: 105 Hz, etc
    ctrl6 |= 0x04;   // enable manual LPF cutoff selection (m_LSM9DS1AccelLpf)

    return m_settings->HALWrite(m_accelGyroSlaveAddr,  LSM9DS1_CTRL6, ctrl6, "Failed to set LSM9DS1 accel CTRL6");
}

bool RTIMULSM9DS1::setAccelCTRL7()
{
    unsigned char ctrl7;

    ctrl7 = 0x00;
    //Bug: Bad things happen.
    //ctrl7 = 0x05;

    // possibility for an extra (very) low pass filter
    // 0xC4: ODR/9, 0x84: ODR/50, 0xA4: ODR/100, 0xE4: ODR/400
    // ctrl7 = 0xC4;

    return m_settings->HALWrite(m_accelGyroSlaveAddr,  LSM9DS1_CTRL7, ctrl7, "Failed to set LSM9DS1 accel CTRL7");
}

bool RTIMULSM9DS1::setAccelGyroCTRL9()
{
    unsigned char ctrl9;
    // ctrl9 = 0x00;   // FIFO disabled (default)
    ctrl9 = 0x02;   // FIFO enabled
    return m_settings->HALWrite(m_accelGyroSlaveAddr,  LSM9DS1_CTRL9, ctrl9, "Failed to set LSM9DS1 accel CTRL9");
}

bool RTIMULSM9DS1::setFifoCTRL()
{
    unsigned char ctrl = 0xC0;   // continuous FIFO mode, full 32 slots available
    return m_settings->HALWrite(m_accelGyroSlaveAddr,  LSM9DS1_FIFO_CTRL, ctrl, "Failed to set LSM9DS1 FIFO_CTRL");
}

bool RTIMULSM9DS1::setCompassCTRL1()
{
    unsigned char ctrl1;

    if ((m_settings->m_LSM9DS1CompassSampleRate < 0) || (m_settings->m_LSM9DS1CompassSampleRate > 5)) {
        HAL_ERROR1("Illegal LSM9DS1 compass sample rate code %d\n", m_settings->m_LSM9DS1CompassSampleRate);
        return false;
    }

    ctrl1 = (m_settings->m_LSM9DS1CompassSampleRate << 2);

    return m_settings->HALWrite(m_magSlaveAddr, LSM9DS1_MAG_CTRL1, ctrl1, "Failed to set LSM9DS1 compass CTRL5");
}

bool RTIMULSM9DS1::setCompassCTRL2()
{
    unsigned char ctrl2;

    //  convert FSR to uT

    switch (m_settings->m_LSM9DS1CompassFsr) {
    case LSM9DS1_COMPASS_FSR_4:
        ctrl2 = 0;
        m_compassScale = (RTFLOAT)0.014;
        break;

    case LSM9DS1_COMPASS_FSR_8:
        ctrl2 = 0x20;
        m_compassScale = (RTFLOAT)0.029;
        break;

    case LSM9DS1_COMPASS_FSR_12:
        ctrl2 = 0x40;
        m_compassScale = (RTFLOAT)0.043;
        break;

    case LSM9DS1_COMPASS_FSR_16:
        ctrl2 = 0x60;
        m_compassScale = (RTFLOAT)0.058;
        break;

    default:
        HAL_ERROR1("Illegal LSM9DS1 compass FSR code %d\n", m_settings->m_LSM9DS1CompassFsr);
        return false;
    }

    return m_settings->HALWrite(m_magSlaveAddr, LSM9DS1_MAG_CTRL2, ctrl2, "Failed to set LSM9DS1 compass CTRL6");
}

bool RTIMULSM9DS1::setCompassCTRL3()
{
     return m_settings->HALWrite(m_magSlaveAddr,  LSM9DS1_MAG_CTRL3, 0x00, "Failed to set LSM9DS1 compass CTRL3");
}

int RTIMULSM9DS1::IMUGetPollInterval()
{
    // return (400 / m_sampleRate);
    return (int)m_samples;
}

bool RTIMULSM9DS1::IMURead()
{
    unsigned char status=1;
    unsigned char mag_status=1;
    unsigned char gyroAccData[12];
    unsigned char compassData[6];

    if (!m_settings->HALRead(m_accelGyroSlaveAddr, LSM9DS1_FIFO_SRC, 1, &status, "Failed to read LSM9DS1 FIFO status"))
        return false;

    //if (!m_settings->HALRead(m_accelGyroSlaveAddr, LSM9DS1_STATUS, 1, &status, "Failed to read LSM9DS1 status"))
    //    return false;
    
    //if (!m_settings->HALRead(m_magSlaveAddr, LSM9DS1_MAG_STATUS, 1, &mag_status, "Failed to read LSM9DS1 mag status"))
    //    return false;

    m_samples = status & 0x3F;   // number of unread measurements in the queue
    if (m_samples == 0)
        return false;
    status = 1;

    //if ((status & 0x3) != 3)
    //    return false;

    if (!m_settings->HALRead(m_accelGyroSlaveAddr, LSM9DS1_OUT_X_L_G, 12, gyroAccData, "Failed to read LSM9DS1 gyro and acc data"))
        status = 0;

    if (mag_status && !m_settings->HALRead(m_magSlaveAddr, LSM9DS1_MAG_OUT_X_L | 0x80, 6, compassData, "Failed to read LSM9DS1 compass data"))
        mag_status = 0;

    m_imuData.timestamp = RTMath::currentUSecsSinceEpoch();
    m_imuData.gyroValid = (bool)status;
    m_imuData.accelValid = (bool)status;
    m_imuData.compassValid = (bool)mag_status;

    if (status)
        RTMath::convertToVector(gyroAccData, m_imuData.gyro, m_gyroScale, false);
    if (status)
        RTMath::convertToVector(&gyroAccData[6], m_imuData.accel, m_accelScale, false);
    if (mag_status)
        RTMath::convertToVector(compassData, m_imuData.compass, m_compassScale, false);

    //  sort out gyro axes and correct for bias

    ///m_imuData.gyro.setZ(-m_imuData.gyro.z());

    //  sort out accel data;

    ///m_imuData.accel.setX(-m_imuData.accel.x());
    ///m_imuData.accel.setY(-m_imuData.accel.y());

    //  sort out compass axes

    ///m_imuData.compass.setX(-m_imuData.compass.x());
    ///m_imuData.compass.setZ(-m_imuData.compass.z());

    //  now do standard processing

    ///handleGyroBias();
    ///calibrateAverageCompass();
    ///calibrateAccel();

    //  now update the filter

    ///updateFusion();

    return true;
}
