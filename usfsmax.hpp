/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

#ifndef USFSMAX_HPP_
#define USFSMAX_HPP_

extern "C" {
  #include <inttypes.h>
}

#include "usfsmax-types.h"

class USFSMAX {
  uint8_t address;
  uint8_t status;
  quat_lin_t orientation;

public:
  USFSMAX(uint8_t address);

  void GyroAccelMagBaro_getADC();
  void GyroAccel_getADC();
  void MagBaro_getADC();
  void Gyro_getADC();
  void ACC_getADC();
  void MAG_getADC();
  void GetMxMy();
  void getQUAT();
  void getEULER();
  void LIN_ACC_getADC();
  void getQUAT_Lin();
  void BARO_getADC();

  bool poll();

  void start();

  bool isConnected();
  uint8_t getStatus();
  quat_lin_t getOrientation();
};

#endif // USFSMAX_HPP_