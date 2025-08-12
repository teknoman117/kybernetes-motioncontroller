/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

#ifndef MOTIONCONTROLLER_TYPES_H
#define MOTIONCONTROLLER_TYPES_H

extern "C" {
  #include <inttypes.h>
}

#include "killswitch-types.hpp"
#include "pid.hpp"

enum class MotionControllerState : uint8_t {
  // ESC and Steering are disabled
  Disabled = 0x00,
  Disabling = 0x10,

  // ESC and Steering are enabled
  Stopped = 0x20,
  MovingForwardPWM = 0x30,
  MovingForwardPID = 0x31,
  MovingBackwardPWM = 0x40,
  MovingBackwardPID = 0x41,
  CrawlingForward = 0x50,
};

enum class PacketType : uint8_t {
  None = 0x00,
  ConfigurationSet = 0x10,
  ConfigurationGet = 0x11,
  SteeringSet = 0x20,
  ThrottleSetPWM = 0x30,
  ThrottleSetPID = 0x40,
  Crawl = 0x50,
  Orientation = 0x60,
  SendArm = 0xA0,
  SendKeepalive = 0xA1,
  SendDisarm = 0xA2,
  ResetOdometer = 0xB0,
  Status = 0xD0,
  Nack = 0xE0,
  ChecksumError = 0xE1,
  Sync = 0xFF,
};

enum class FailureType : uint8_t {
  InvalidStateTransition = 0xC0,
  RemotelyDisabled = 0xD0,
  InvalidWhenActive = 0xE0,
  InvalidWhenDisabled = 0xE1,
  UnsupportedPacketType = 0xFE,
  BadChecksum = 0xFF,
};

struct ConfigurationPacket {
  static constexpr auto ReceiveType = PacketType::ConfigurationGet;
  static constexpr auto SendType = PacketType::ConfigurationSet;
  static constexpr size_t ReceiveTransportSize = 20;
  static constexpr size_t SendTransportSize = 20;

  // Servo Deadzone Configuration
  uint16_t servoSteeringDeadzoneLeft;
  uint16_t servoSteeringDeadzoneRight;
  uint16_t servoThrottleDeadzoneForward;
  uint16_t servoThrottleDeadzoneBackward;

  // PID Controller Configuration
  float Kp;
  float Ki;
  float Kd;
} __attribute__((packed));

// send once per 20 ms (every PID controller update)
struct StatusPacket {
  static constexpr auto ReceiveType = PacketType::Status;
  static constexpr size_t ReceiveTransportSize = 19 + sizeof(PIDFrame);

  // System state
  KillSwitchStatusPacket remote;
  MotionControllerState state;
  uint8_t batteryLow;
  uint8_t bumperPressed;
  int32_t odometer;

  // Motion Packet
  PIDFrame motion;

  // current sensor status
  int16_t batteryVoltage;
  int16_t batteryCurrent;
  int16_t batteryPower;

  // imu status
  uint8_t imuStatus;
} __attribute__((packed));

/*struct OrientationPacket {
  static constexpr auto ReceiveType = PacketType::Orientation;
  static constexpr size_t ReceiveTransportSize = sizeof(quat_t);

  quat_t orientation;
} __attribute__((packed));*/

struct SteeringSetPacket {
  static constexpr auto ReceiveType = PacketType::SteeringSet;
  static constexpr auto SendType = PacketType::SteeringSet;
  static constexpr size_t ReceiveTransportSize = 2;
  static constexpr size_t SendTransportSize = 2;
  int16_t servoSteeringOutput;
} __attribute__((packed));

struct ThrottleSetPWMPacket {
  static constexpr auto ReceiveType = PacketType::ThrottleSetPWM;
  static constexpr auto SendType = PacketType::ThrottleSetPWM;
  static constexpr size_t ReceiveTransportSize = 2;
  static constexpr size_t SendTransportSize = 2;
  int16_t Target;
} __attribute__((packed));

struct ThrottleSetPIDPacket {
  static constexpr auto ReceiveType = PacketType::ThrottleSetPID;
  static constexpr auto SendType = PacketType::ThrottleSetPID;
  static constexpr size_t ReceiveTransportSize = 2;
  static constexpr size_t SendTransportSize = 2;
  int16_t Target;
} __attribute__((packed));

struct CrawlPacket {
  static constexpr auto ReceiveType = PacketType::Crawl;
  static constexpr auto SendType = PacketType::Crawl;
  static constexpr size_t ReceiveTransportSize = 0;
  static constexpr size_t SendTransportSize = 0;
} __attribute__((packed));

struct SendArmPacket {
  static constexpr auto ReceiveType = PacketType::SendArm;
  static constexpr auto SendType = PacketType::SendArm;
  static constexpr size_t ReceiveTransportSize = 0;
  static constexpr size_t SendTransportSize = 0;
} __attribute__((packed));

struct SendKeepalivePacket {
  static constexpr auto ReceiveType = PacketType::SendKeepalive;
  static constexpr auto SendType = PacketType::SendKeepalive;
  static constexpr size_t ReceiveTransportSize = 0;
  static constexpr size_t SendTransportSize = 0;
} __attribute__((packed));

struct SendDisarmPacket {
  static constexpr auto ReceiveType = PacketType::SendDisarm;
  static constexpr auto SendType = PacketType::SendDisarm;
  static constexpr size_t ReceiveTransportSize = 0;
  static constexpr size_t SendTransportSize = 0;
} __attribute__((packed));

struct ResetOdometerPacket {
  static constexpr auto ReceiveType = PacketType::ResetOdometer;
  static constexpr auto SendType = PacketType::ResetOdometer;
  static constexpr size_t ReceiveTransportSize = 0;
  static constexpr size_t SendTransportSize = 0;
} __attribute__((packed));

// send if command failed
struct NackPacket {
  static constexpr auto ReceiveType = PacketType::Nack;
  static constexpr size_t ReceiveTransportSize = 2;
  PacketType packetType;
  FailureType failureType;
} __attribute__((packed));

// send if a framing error occurred
struct ChecksumErrorPacket {
  static constexpr auto ReceiveType = PacketType::ChecksumError;
  static constexpr size_t ReceiveTransportSize = 0;
} __attribute__((packed));

// sync packet
struct SyncPacket {
  static constexpr auto ReceiveType = PacketType::Sync;
  static constexpr auto SendType = PacketType::Sync;
  static constexpr auto ReceiveTransportSize = 0;
  static constexpr auto SendTransportSize = 0;
} __attribute__((packed));

// empty type for no argument commands and responses
template <typename N> struct NoArgumentPacket {
  static constexpr auto SendType = N::ReceiveType;
  static constexpr auto ReceiveType = N::SendType;
  static constexpr size_t SendTransportSize = 0;
  static constexpr size_t ReceiveTransportSize = 0;
} __attribute__((packed));

#endif /* MOTIONCONTROLLER_TYPES_H */