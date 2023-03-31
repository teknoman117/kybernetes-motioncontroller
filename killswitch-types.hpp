#ifndef KYBERNETES_KILLSWITCH_TYPES_H
#define KYBERNETES_KILLSWITCH_TYPES_H

constexpr uint8_t commandDisarm = 0x68;
constexpr uint8_t commandArm = 0x69;
constexpr uint8_t commandKeepalive = commandArm;

enum class KillSwitchState : uint8_t {
  Disarmed           = 0,
  DisarmingRequested = 1,
  DisarmingRemote    = 2,
  DisarmingSoftware  = 3,
  Armed              = 4,
};

// TODO: change kill controller status to have the uint16_t's on top rather than at the end
struct KillSwitchStatusPacket {
  uint8_t servoSteeringInputUpdated : 1;
  uint8_t servoThrottleInputUpdated : 1;
  uint8_t armable                   : 1;
  uint8_t unused1                   : 2;
  KillSwitchState state             : 3;
  uint16_t servoSteeringInput;
  uint16_t servoThrottleInput;

  // handy shortcuts to the state
  KillSwitchState operator=(const KillSwitchState state_) {
    state = state_;
  }
} __attribute__((packed));

#endif /* KYBERNETES_KILLSWITCH_TYPES_H */