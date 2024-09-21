/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

#include "usfsmax.hpp"
#include "usfsmax-def.h"

#include <Arduino.h>
#include <Wire.h>

#define SENS_ERR_STAT                 0x00
#define CALIBRATION_STATUS            0x01
#define ACCEL_CAL_POS                 0x02
#define FUSION_STATUS                 0x03
#define COMBO_DRDY_STAT               0x04

#define G_X_L                         0x05
#define G_X_H                         0x06
#define G_Y_L                         0x07
#define G_Y_H                         0x08
#define G_Z_L                         0x09
#define G_Z_H                         0x0A
#define A_X_L                         0x0B
#define A_X_H                         0x0C
#define A_Y_L                         0x0D
#define A_Y_H                         0x0E
#define A_Z_L                         0x0F
#define A_Z_H                         0x10
#define M_X_L                         0x11
#define M_X_H                         0x12
#define M_Y_L                         0x13
#define M_Y_H                         0x14
#define M_Z_L                         0x15
#define M_Z_H                         0x16
#define BARO_XL                       0x17
#define BARO_L                        0x18
#define BARO_H                        0x19
#define Q0_BYTE0                      0x1A
#define Q0_BYTE1                      0x1B
#define Q0_BYTE2                      0x1C
#define Q0_BYTE3                      0x1D
#define Q1_BYTE0                      0x1E
#define Q1_BYTE1                      0x1F
#define Q1_BYTE2                      0x20
#define Q1_BYTE3                      0x21
#define Q2_BYTE0                      0x22
#define Q2_BYTE1                      0x23
#define Q2_BYTE2                      0x24
#define Q2_BYTE3                      0x25
#define Q3_BYTE0                      0x26
#define Q3_BYTE1                      0x27
#define Q3_BYTE2                      0x28
#define Q3_BYTE3                      0x29
#define LIN_X_L                       0x2A
#define LIN_X_H                       0x2B
#define LIN_Y_L                       0x2C
#define LIN_Y_H                       0x2D
#define LIN_Z_L                       0x2E
#define LIN_Z_H                       0x2F
#define GRAV_X_L                      0x30
#define GRAV_X_H                      0x31
#define GRAV_Y_L                      0x32
#define GRAV_Y_H                      0x33
#define GRAV_Z_L                      0x34
#define GRAV_Z_H                      0x35
#define YAW_BYTE0                     0x36
#define YAW_BYTE1                     0x37
#define YAW_BYTE2                     0x38
#define YAW_BYTE3                     0x39
#define PITCH_BYTE0                   0x3A
#define PITCH_BYTE1                   0x3B
#define PITCH_BYTE2                   0x3C
#define PITCH_BYTE3                   0x3D
#define ROLL_BYTE0                    0x3E
#define ROLL_BYTE1                    0x3F
#define ROLL_BYTE2                    0x40
#define ROLL_BYTE3                    0x41
#define AG_TEMP_L                     0x42
#define AG_TEMP_H                     0x43
#define M_TEMP_L                      0x44
#define M_TEMP_H                      0x45
#define B_TEMP_L                      0x46
#define B_TEMP_H                      0x47
#define AUX_1_X_L                     0x48
#define AUX_1_X_H                     0x49
#define AUX_1_Y_L                     0x4A
#define AUX_1_Y_H                     0x4B
#define AUX_1_Z_L                     0x4C
#define AUX_1_Z_H                     0x4D
#define AUX_2_X_L                     0x4E
#define AUX_2_X_H                     0x4F
#define AUX_2_Y_L                     0x50
#define AUX_2_Y_H                     0x51
#define AUX_2_Z_L                     0x52
#define AUX_2_Z_H                     0x53
#define AUX_3_X_L                     0x54
#define AUX_3_X_H                     0x55
#define AUX_3_Y_L                     0x56
#define AUX_3_Y_H                     0x57
#define AUX_3_Z_L                     0x58
#define AUX_3_Z_H                     0x59
#define MX_L                          0x5A
#define MX_H                          0x5B
#define MY_L                          0x5C
#define MY_H                          0x5D
#define DHI_RSQ_L                     0x5E
#define DHI_RSQ_H                     0x5F

#define FUSION_START_STOP             0x60
#define CALIBRATION_REQUEST           0x61

#define COPRO_CFG_DATA0               0x62
#define COPRO_CFG_DATA1               0x63
#define GYRO_CAL_DATA0                0x64
#define GYRO_CAL_DATA1                0x65
#define ACCEL_CAL_DATA0               0x66
#define ACCEL_CAL_DATA1               0x67
#define ELLIP_MAG_CAL_DATA0           0x68
#define ELLIP_MAG_CAL_DATA1           0x69
#define FINE_MAG_CAL_DATA0            0x6A
#define FINE_MAG_CAL_DATA1            0x6B
#define NEW_I2C_SLAVE_ADDR            0x6C
#define GO_TO_SLEEP                   0x6D
#define FIRMWARE_ID                   0x7F

#define FUSION_RUNNING_MASK           0x01
#define HI_CORRECTOR_MASK             0x10

namespace {
  uint8_t readBytes(uint8_t address, uint8_t reg, uint8_t *data, uint8_t len) {
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(address, (size_t) len);
    uint8_t b = 0;
    while (Wire.available()) {
      data[b++] = Wire.read();
    }
    return b;
  }

  uint8_t readByte(uint8_t address, uint8_t reg) {
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(address, (size_t) 1);
    return Wire.read();
  }

  bool writeBytes(uint8_t address, uint8_t reg, uint8_t *data, uint8_t len) {
    Wire.beginTransmission(address);
    if (Wire.write(reg) != 1) return false;
    if (Wire.write(data, (size_t) len) != len) return false;
    if (Wire.endTransmission() != 0) return false;
    return true;
  }

  bool writeByte(uint8_t address, uint8_t reg, uint8_t b) {
    return writeBytes(address, reg, &b, 1);
  }
}

USFSMAX::USFSMAX(uint8_t address)
    : address(address), status(0xff), orientation({0}) {
  // stuffy stuff
}


void USFSMAX::GyroAccelMagBaro_getADC()
{
  uint8_t bytes[21];

  readBytes(address, G_X_L, 21, bytes);
}

void USFSMAX::GyroAccel_getADC()
{
  uint8_t bytes[12];

  readBytes(address, G_X_L, 12, bytes);
}

void USFSMAX::MagBaro_getADC()
{
  uint8_t bytes[9];

  readBytes(address, M_X_L, 9, bytes);
}

void USFSMAX::Gyro_getADC()
{
  uint8_t bytes[6];
  
  readBytes(address, G_X_L, 6, bytes);
}

void USFSMAX::ACC_getADC()
{
  uint8_t bytes[6];
  
  readBytes(address, A_X_L, 6, bytes);
}

void USFSMAX::MAG_getADC()
{
  uint8_t bytes[6];
  
  readBytes(address, M_X_L, 6, bytes);
}

void USFSMAX::GetMxMy()
{
  uint8_t bytes[4];

  readBytes(address, MX_L, 4, bytes);
}

void USFSMAX::getQUAT()
{
  readBytes(address, Q0_BYTE0, (uint8_t*) &orientation, 16);
}

void USFSMAX::getEULER()
{
  uint8_t bytes[12];

  readBytes(address, YAW_BYTE0, 12, bytes);
}

void USFSMAX::getQUAT_Lin()
{
  readBytes(address, Q0_BYTE0, (uint8_t*) &orientation, 28);
}

void USFSMAX::LIN_ACC_getADC()
{
  uint8_t bytes[12];
  
  readBytes(address, LIN_X_L, 12, bytes);
}

void USFSMAX::BARO_getADC()
{
  uint8_t bytes[3];
  
  readBytes(address, BARO_XL, 3, bytes);
}

bool USFSMAX::poll() {
  uint8_t drdy = readByte(address, COMBO_DRDY_STAT);
  
  // Optimize the I2C read function with respect to whatever sensor data is ready
  /*switch(drdy & 0x0f)
  {
   case 0x01:
     GyroAccel_getADC();
     break;
   case 0x02:
     GyroAccel_getADC();
     break;
   case 0x03:
     GyroAccel_getADC();
     break;
   case 0x07:
     GyroAccelMagBaro_getADC();
     break;
   case 0x0B:
     GyroAccelMagBaro_getADC();
     break;
   case 0x0F:
     GyroAccelMagBaro_getADC();
     break;
   case 0x0C:
     MagBaro_getADC();
     break;
   case 0x04:
     MAG_getADC();
     break;
   case 0x08:
     BARO_getADC();
     break;
   default:
     break;
  };*/

  // abort if a quaternion isn't available
  if (!(drdy & 0x10)) {
    return false;
  }

  // fetch quaternion
  getQUAT_Lin();
  return true;
}

void USFSMAX::start() {
  // check if the sensor responds
  Wire.beginTransmission(address);
  if (Wire.endTransmission() != 0) {
    // sensor not responding
    status = 1;
    return;
  }

  // get the fusion status
  uint8_t ret = readByte(address, FUSION_STATUS);
  if (!ret) {
    // ensure fusion is stopped
    writeByte(address, FUSION_START_STOP, 0x00);
    delay(100);

    // configuration upload                                                           
    writeByte(address, FUSION_START_STOP, 0x08);
    delay(1000);
    
    // Assign configuration values
    CoProcessorConfig_t Config = {
      .cal_points        = CAL_POINTS,
      .Ascale            = ACC_SCALE,
      .AODR              = ACC_ODR,
      .Alpf              = LSM6DSM_ACC_DLPF_CFG,
      .Ahpf              = LSM6DSM_ACC_DHPF_CFG,
      .Gscale            = GYRO_SCALE,
      .GODR              = GYRO_ODR,
      .Glpf              = LSM6DSM_GYRO_DLPF_CFG,
      .Ghpf              = LSM6DSM_GYRO_DHPF_CFG,
      .Mscale            = MAG_SCALE,
      .MODR              = MAG_ODR,
      .Mlpf              = MMC5983MA_MAG_LPF,
      .Mhpf              = MMC5983MA_MAG_HPF,
      .Pscale            = BARO_SCALE,
      .PODR              = BARO_ODR,
      .Plpf              = LPS22HB_BARO_LPF,
      .Phpf              = LPS22HB_BARO_HPF,
      .AUX1scale         = AUX1_SCALE,
      .AUX1ODR           = AUX1_ODR,
      .AUX1lpf           = AUX1_LPF,
      .AUX1hpf           = AUX1_HPF,
      .AUX2scale         = AUX2_SCALE,
      .AUX2ODR           = AUX2_ODR,
      .AUX2lpf           = AUX2_LPF,
      .AUX2hpf           = AUX2_HPF,
      .AUX3scale         = AUX3_SCALE,
      .AUX3ODR           = AUX3_ODR,
      .AUX3lpf           = AUX3_LPF,
      .AUX3hpf           = AUX3_HPF,
      .m_v               = M_V,
      .m_h               = M_H,
      .m_dec             = MAG_DECLINIATION,
      .quat_div          = QUAT_DIV
    };

    // Upload configuration bytes
    uint8_t *pConfig = (uint8_t*) &Config;
    if (!writeBytes(address, COPRO_CFG_DATA0, &pConfig[0], 30)) {
      status = 240;
      return;
    }
    delay(100);
    if (!writeBytes(address, COPRO_CFG_DATA1, &pConfig[30], (sizeof(CoProcessorConfig_t) - 30))) {
      status = 241;
      return;
    }
    delay(100);

    // Re-start sensor fusion
    // Set bit0 to re-start fusion; adjust bit1, bit2 for desired output options
    writeByte(address, FUSION_START_STOP, 0x01 | (OUTPUT_EULER_ANGLES << 1) | (SCALED_SENSOR_DATA << 2));
    delay(100);

    // Poll the FUSION_STATUS register to see if fusion has resumed
    ret = 100;
    do {
      delay(10);
      ret = readByte(address, FUSION_STATUS);
      if ((ret & FUSION_RUNNING_MASK)) {
        break;
      }
    } while (--ret);

    if (!ret) {
      // sensor fusion failed to start
      status = 2;
      return;
    }
  }

  ret = readByte(address, SENS_ERR_STAT);
  if (ret) {
    // sensor error
    status = 3;
    return;
  }

  // start whatever corrector we asked for
  if(ENABLE_DHI_CORRECTOR) {
    if(USE_2D_DHI_CORRECTOR) {
      // Enable DHI corrector, 2D (0x10|0x50)
      writeByte(address, CALIBRATION_REQUEST, 0x50);
    } else {
      // Enable DHI corrector, 3D (0x10)
      writeByte(address, CALIBRATION_REQUEST, 0x10);
    }
  }

  // check if sensor is calibrated
  delay(100);
  ret = readByte(address, CALIBRATION_STATUS);
  if (ret & 0x80 != 0x80) {
    // sensor is uncalibrated
    status = 4;
    return;
  }

  // ask for gyro calibration
  // 0x01 - assert bit 0, start gyro cal
  writeByte(address, CALIBRATION_REQUEST, 0x01);

  // sensor started successfully
  delay(100);
  status = 0;
}

bool USFSMAX::isConnected() {
  return status == 0;
}

uint8_t USFSMAX::getStatus() {
  return status;
}

quat_lin_t USFSMAX::getOrientation() {
  return orientation;
}