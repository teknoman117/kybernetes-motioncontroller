/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

#ifndef KYBERNETES_KILLSWITCH_TYPES_H
#define KYBERNETES_KILLSWITCH_TYPES_H

constexpr uint8_t commandDisarm = 0xFF;
constexpr uint8_t commandArm = 0xAA;
constexpr uint8_t commandKeepalive = commandArm;

enum class KillSwitchState : uint8_t {
  Disarmed           = 0,
  DisarmingRequested = 1,
  DisarmingRemote    = 2,
  DisarmingSoftware  = 3,
  Armed              = 4,
};

struct KillSwitchStatusPacket {
  uint16_t servoSteeringInput;
  uint16_t servoThrottleInput;

  // NOTE: not atomically assignable
  uint8_t servoSteeringInputUpdated : 1;
  uint8_t servoThrottleInputUpdated : 1;
  uint8_t armable                   : 1;
  uint8_t unused1                   : 2;
  KillSwitchState state             : 3;

  // handy shortcuts to the state
  KillSwitchState operator=(const KillSwitchState state_) {
    state = state_;
    return state;
  }
} __attribute__((packed));

#endif /* KYBERNETES_KILLSWITCH_TYPES_H */