/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

#define ENCODER_OPTIMIZE_INTERRUPTS
#define ENCODER_USE_INTERRUPTS

#include <avr/io.h>

#include "cfg/chconf.h"

#include "hal.h"
#include "ch.h"

#include "crc8.hpp"
#include "killswitch-types.hpp"
#include "motioncontroller-types.hpp"
#include "pid.hpp"
//#include "usfsmax.hpp"
#include "lock.hpp"

constexpr int killSwitchAddress = 0x08;

#define ENCODER_CHANNEL_A_LINE PAL_LINE(IOPORT4, 2)
#define ENCODER_CHANNEL_B_LINE PAL_LINE(IOPORT4, 3)
#define BUMPER_A_LINE          PAL_LINE(IOPORT4, 4)
#define BUMPER_B_LINE          PAL_LINE(IOPORT4, 5)
#define IMU_DATA_READ_LINE     PAL_LINE(IOPORT4, 6)
#define BATTERY_LOW_LINE       PAL_LINE(IOPORT4, 7)

#define SERVO_STEERING_LINE    PAL_LINE(IOPORT2, 1)
#define SERVO_THROTTLE_LINE    PAL_LINE(IOPORT2, 2)

constexpr float defaultKp = 0.1f;
constexpr float defaultKi = 0.25f;
constexpr float defaultKd = 0.01f;

namespace {
  template <typename T> bool getStructFromWireCRC8(uint8_t address, T& obj) {
    uint8_t buffer[sizeof(T) + 1];
    i2cAcquireBus(&I2CD1);
    auto ret = i2cMasterReceive(&I2CD1, address, buffer, sizeof buffer);
    i2cReleaseBus(&I2CD1);

    if (ret != MSG_OK
        || buffer[sizeof buffer - 1] != calculateCRC8(0, buffer, sizeof(T))) {
      return false;
    }

    memcpy(reinterpret_cast<uint8_t*>(&obj), buffer, sizeof(T));
    return true;
  }

  template <typename Chan, typename T> bool sendPacket(Chan *channel, T& packet) {
    const uint8_t type = (const uint8_t) T::ReceiveType;

    if (chnWrite(channel, &type, 1) != 1) {
      // something went wrong
      return false;
    }

    if (chnWrite(channel, reinterpret_cast<const uint8_t*>(&packet), T::ReceiveTransportSize)
        != T::ReceiveTransportSize) {
      // something went wrong
      return false;
    }

    uint8_t crc8 = 0;
    crc8 = calculateCRC8(crc8, &type, 1);
    crc8 = calculateCRC8(crc8, reinterpret_cast<const uint8_t*>(&packet), T::ReceiveTransportSize);
    return chnWrite(channel, &crc8, 1) == 1;
  }

  template <typename Chan, typename T> bool receivePacket(Chan *channel, T& packet) {
    const uint8_t type = (const uint8_t) T::SendType;

    if (chnRead(channel, reinterpret_cast<uint8_t*>(&packet), T::SendTransportSize)
        != T::SendTransportSize) {
      // something went wrong
      return false;
    }

    uint8_t checksum = 0;
    if (chnRead(channel, &checksum, 1) != 1) {
      // something went wrong
      return false;
    }

    uint8_t crc8 = 0;
    crc8 = calculateCRC8(crc8, &type, 1);
    crc8 = calculateCRC8(crc8, reinterpret_cast<const uint8_t*>(&packet), T::SendTransportSize);
    return (crc8 == checksum);
  }

  bool sendSyncPacket() {
    const PacketType packet[16] = {
      PacketType::Sync, PacketType::Sync, PacketType::Sync, PacketType::Sync,
      PacketType::Sync, PacketType::Sync, PacketType::Sync, PacketType::Sync,
      PacketType::Sync, PacketType::Sync, PacketType::Sync, PacketType::Sync,
      PacketType::Sync, PacketType::Sync, PacketType::Sync, PacketType::Sync,
    };

    return (chnWrite(&SD1, reinterpret_cast<const uint8_t*>(&packet), sizeof packet)
        == sizeof packet);
  }
}

//USFSMAX imu(0x57);

PID<20> pid(defaultKp, defaultKi, defaultKd);

ConfigurationPacket configuration = {
  .servoSteeringDeadzoneLeft = 1500,
  .servoSteeringDeadzoneRight = 1500,
  .servoThrottleDeadzoneForward = 1500,
  .servoThrottleDeadzoneBackward = 1500,
  .Kp = defaultKp,
  .Ki = defaultKi,
  .Kd = defaultKd,
};

StatusPacket status = {
  .remote = {0},
  .state = MotionControllerState::Disabled,
  .batteryLow = 0,
  .bumperPressed = 0,
  .odometer = 0,
  .motion = PIDFrame()
};

MotionControllerState localState = MotionControllerState::Disabled;

KillSwitchState remoteState = KillSwitchState::Disarmed;

volatile bool bumperContacted = false;
volatile bool imuDataPending = false;
volatile int32_t encoderTicks = 0;
volatile uint8_t encoderState = 0;

SEMAPHORE_DECL(stateSemaphore, 1);

static void bumperContact() {
  bumperContacted = true;
}

static void imuDataReady() {
  imuDataPending = true;
}

static void encoderEvent() {
  chSysLockFromISR();

  // get the encoder transition map
  const auto bits = (palReadPort(PAL_PORT(ENCODER_CHANNEL_A_LINE))
      >> PAL_PAD(ENCODER_CHANNEL_A_LINE)) & 0x3;
  const auto transitions = (encoderState << 2) | bits;
  encoderState = bits;

  switch (transitions) {
    // no change
    case 0b0000:
    case 0b0101:
    case 0b1010:
    case 0b1111:
      break;

    // double step
    case 0b0011:
    case 0b0110:
    case 0b1001:
    case 0b1100:
      break;

    // backward
    case 0b0001:
    case 0b0111:
    case 0b1000:
    case 0b1110:
      encoderTicks--;
      break;

    // forward
    case 0b0010:
    case 0b0100:
    case 0b1011:
    case 0b1101:
      encoderTicks++;
      break;
  }

  chSysUnlockFromISR();
}

OSAL_IRQ_HANDLER(INT0_vect) {
  OSAL_IRQ_PROLOGUE();
  encoderEvent();
  OSAL_IRQ_EPILOGUE();
}

OSAL_IRQ_HANDLER(INT1_vect) {
  OSAL_IRQ_PROLOGUE();
  encoderEvent();
  OSAL_IRQ_EPILOGUE();
}

// TODO: detect imu interrupt as well
OSAL_IRQ_HANDLER(PCINT2_vect) {
  OSAL_IRQ_PROLOGUE();
  bumperContact();
  OSAL_IRQ_EPILOGUE();
}

void setSteering(int16_t value) {
  if (value > 0) {
    value = value + configuration.servoSteeringDeadzoneLeft;
  } else if (value < 0) {
    value = value + configuration.servoSteeringDeadzoneRight;
  } else {
    value = (configuration.servoSteeringDeadzoneLeft + configuration.servoSteeringDeadzoneRight) / 2;
  }

  // 1 ms = 16000
  // 1 us = 16
  pwmEnableChannel(&PWMD1, 0, value * 16);
}

void setThrottle(int16_t value) {
  if (value > 0) {
    value = value + configuration.servoThrottleDeadzoneForward;
  } else if (value < 0) {
    value = value + configuration.servoThrottleDeadzoneBackward;
  } else {
    value = (configuration.servoThrottleDeadzoneForward + configuration.servoThrottleDeadzoneBackward) / 2;
  }

  // 1 ms = 16000
  // 1 us = 16
  pwmEnableChannel(&PWMD1, 1, value * 16);
}

bool sendRemoteCommand(uint8_t command) {
  i2cAcquireBus(&I2CD1);
  auto ret = i2cMasterTransmit(&I2CD1, killSwitchAddress, &command, sizeof command, nullptr, 0);
  i2cReleaseBus(&I2CD1);
  return ret == MSG_OK;
}

void controllerUpdate() {
  int16_t servoThrottleOutput = 0;

  // Fetch encoder ticks
  chSysLock();
  auto ticks = encoderTicks;
  encoderTicks = 0;
  chSysUnlock();

  // Update PID controller
  Lock lock(&stateSemaphore);
  status.odometer += ticks;
  status.motion.nextInput(ticks);
  pid.compute(status.motion);

  if (localState == MotionControllerState::MovingForwardPWM
      || localState == MotionControllerState::MovingForwardPID
      || localState == MotionControllerState::CrawlingForward) {
    if (status.motion.Output >= 0) {
      servoThrottleOutput = status.motion.Output;
    }
  } else if (localState == MotionControllerState::MovingBackwardPWM
      || localState == MotionControllerState::MovingBackwardPID) {
    if (status.motion.Output <= 0) {
      servoThrottleOutput = status.motion.Output;
    }
  }

  // If we're in CrawlingForward and the bumper has been pressed, transition to stopped
  uint8_t bumperPressed = palReadLine(BUMPER_A_LINE) | palReadLine(BUMPER_B_LINE) | bumperContacted;
  if (localState == MotionControllerState::CrawlingForward
      && bumperPressed) {
    localState = MotionControllerState::Stopped;
    servoThrottleOutput = 0;
    status.motion.reset();
  }
  bumperContacted = false;

  // Transition to Stopped (from Moving*) if we have stopped moving and had intended to do so
  if (localState == MotionControllerState::MovingForwardPID
      || localState == MotionControllerState::MovingBackwardPID
      || localState == MotionControllerState::MovingForwardPWM
      || localState == MotionControllerState::MovingBackwardPWM
      || localState == MotionControllerState::CrawlingForward) {
    if (status.motion.Input[0] == 0 && status.motion.Target == 0) {
      localState = MotionControllerState::Stopped;
      servoThrottleOutput = 0;
      status.motion.reset();
    }
  }
  setThrottle(servoThrottleOutput);

  // Check if any transitions need to happen
  KillSwitchStatusPacket packet;
  if (!getStructFromWireCRC8(killSwitchAddress, packet)) {
    return;
  }

  // TODO: use these to detect state transitions rather than just react to the immedate state
  // e.g. allow for top level state machine "these actions cause transitions" rather than this
  //      piecemeal method
  remoteState = packet.state;
  switch (remoteState) {
    case KillSwitchState::Disarmed:
      if (localState != MotionControllerState::Disabled) {
        localState = MotionControllerState::Disabled;
      }
      break;

    case KillSwitchState::DisarmingRequested:
    case KillSwitchState::DisarmingRemote:
    case KillSwitchState::DisarmingSoftware:
      if (localState != MotionControllerState::Disabling) {
        localState = MotionControllerState::Disabling;
        status.motion.reset();
        setThrottle(0);
      }
      break;

    case KillSwitchState::Armed:
      if (localState == MotionControllerState::Disabled
          || localState == MotionControllerState::Disabling) {
        localState = MotionControllerState::Stopped;
      }
      break;
  }

  // Send status packet
  status.state = localState;
  status.remote = packet;
  status.batteryLow = palReadLine(BATTERY_LOW_LINE) ? 0 : 1;
  status.bumperPressed = bumperPressed;
  status.imuStatus = /* imu.getStatus() */ 0;
  sendPacket(&SD1, status);
}

void handlePacketConfigurationGet() {
  using CommandPacket = NoArgumentPacket<ConfigurationPacket>;

  CommandPacket packet;
  if (receivePacket(&SD1, packet)) {
    Lock lock(&stateSemaphore);
    sendPacket(&SD1, configuration);
  } else {
    auto nack = NackPacket{CommandPacket::SendType, FailureType::BadChecksum};
    sendPacket(&SD1, nack);
  }
}

void handlePacketConfigurationSet() {
  using CommandPacket = ConfigurationPacket;
  using ResponsePacket = NoArgumentPacket<ConfigurationPacket>;

  auto failure = FailureType::BadChecksum;
  CommandPacket packet;
  if (receivePacket(&SD1, packet)) {
    if (localState == MotionControllerState::Disabled) {
      {
        Lock lock(&stateSemaphore);
        configuration = packet;
        pid.setTunings(configuration.Kp, configuration.Ki, configuration.Kd);
      }

      ResponsePacket response;
      sendPacket(&SD1, response);
      return;
    } else {
      failure = FailureType::InvalidWhenActive;
    }
  }

  auto nack = NackPacket{CommandPacket::SendType, failure};
  sendPacket(&SD1, nack);
}

void handlePacketSteeringSet() {
  using CommandPacket = SteeringSetPacket;
  using ResponsePacket = NoArgumentPacket<SteeringSetPacket>;

  CommandPacket packet;
  if (receivePacket(&SD1, packet)) {
    {
      Lock lock(&stateSemaphore);
      setSteering(packet.servoSteeringOutput);
    }

    ResponsePacket response;
    sendPacket(&SD1, response);
  } else {
    auto nack = NackPacket{CommandPacket::SendType, FailureType::BadChecksum};
    sendPacket(&SD1, nack);
  }
}

void handlePacketThrottleSetPWM() {
  using CommandPacket = ThrottleSetPWMPacket;
  using ResponsePacket = NoArgumentPacket<ThrottleSetPWMPacket>;

  auto failure = FailureType::BadChecksum;
  ResponsePacket response;
  CommandPacket packet;
  if (receivePacket(&SD1, packet)) {
    Lock lock(&stateSemaphore);
    switch (localState) {
      case MotionControllerState::Disabled:
      case MotionControllerState::Disabling:
        failure = FailureType::InvalidWhenDisabled;
        break;

      // we can enter PWM controlled motion if we're not moving
      case MotionControllerState::Stopped:
        if (packet.Target > 0) {
          localState = MotionControllerState::MovingForwardPWM;
        } else if (packet.Target < 0) {
          localState = MotionControllerState::MovingBackwardPWM;
        }

        status.motion.Target = packet.Target;
        status.motion.Enabled = 0;
        sendPacket(&SD1, response);
        return;

      // we can stay in forward PWM controlled motion or downgrade PID controlled motion
      case MotionControllerState::MovingForwardPWM:
      case MotionControllerState::MovingForwardPID:
      case MotionControllerState::CrawlingForward:
        if (packet.Target < 0) {
          // can't immediately transition to backward motion
          failure = FailureType::InvalidStateTransition;
          break;
        }

        status.motion.Target = packet.Target;
        status.motion.Enabled = 0;
        sendPacket(&SD1, response);
        return;

      // we can stay in backward PWM controlled motion or downgrade PID controlled motion
      case MotionControllerState::MovingBackwardPWM:
      case MotionControllerState::MovingBackwardPID:
        if (packet.Target > 0) {
          // can't immediately transition to forward motion
          failure = FailureType::InvalidStateTransition;
          break;
        }

        status.motion.Target = packet.Target;
        status.motion.Enabled = 0;
        sendPacket(&SD1, response);
        return;
    }
  }

  auto nack = NackPacket{CommandPacket::SendType, failure};
  sendPacket(&SD1, nack);
}

void handlePacketThrottleSetPID() {
  using CommandPacket = ThrottleSetPIDPacket;
  using ResponsePacket = NoArgumentPacket<ThrottleSetPIDPacket>;

  auto failure = FailureType::BadChecksum;
  ResponsePacket response;
  CommandPacket packet;
  if (receivePacket(&SD1, packet)) {
    Lock lock(&stateSemaphore);
    switch (localState) {
      case MotionControllerState::Disabled:
      case MotionControllerState::Disabling:
        failure = FailureType::InvalidWhenDisabled;
        break;

      // we can enter the PID controlled state only if we're not moving
      case MotionControllerState::Stopped:
        if (packet.Target > 0) {
          localState = MotionControllerState::MovingForwardPID;
        } else if (packet.Target < 0) {
          localState = MotionControllerState::MovingBackwardPID;
        }

        status.motion.Target = packet.Target;
        status.motion.Enabled = 1;
        sendPacket(&SD1, response);
        return;

      // we can stay in forward PID controlled motion
      case MotionControllerState::MovingForwardPID:
      case MotionControllerState::CrawlingForward:
        if (packet.Target < 0) {
          // can't immediately transition to backward motion
          failure = FailureType::InvalidStateTransition;
          break;
        }

        status.motion.Target = packet.Target;
        status.motion.Enabled = 1;
        sendPacket(&SD1, response);
        return;

      // we can stay in backward PID controlled motion
      case MotionControllerState::MovingBackwardPID:
        if (packet.Target > 0) {
          // can't immediately transition to forward motion
          failure = FailureType::InvalidStateTransition;
          break;
        }

        status.motion.Target = packet.Target;
        status.motion.Enabled = 1;
        sendPacket(&SD1, response);
        return;

      // we can't directly promote PWM controlled motion to PID controlled motion
      case MotionControllerState::MovingForwardPWM:
      case MotionControllerState::MovingBackwardPWM:
        failure = FailureType::InvalidStateTransition;
        break;
    }
  }

  auto nack = NackPacket{CommandPacket::SendType, failure};
  sendPacket(&SD1, nack);
}

void handlePacketCrawl() {
  using CommandPacket = CrawlPacket;
  using ResponsePacket = NoArgumentPacket<CrawlPacket>;

  auto failure = FailureType::BadChecksum;
  ResponsePacket response;
  CommandPacket packet;
  if (receivePacket(&SD1, packet)) {
    Lock lock(&stateSemaphore);
    switch (localState) {
      case MotionControllerState::Disabled:
      case MotionControllerState::Disabling:
        failure = FailureType::InvalidWhenDisabled;
        break;

      // we can enter the CrawlingForward state only if we're not moving
      case MotionControllerState::Stopped:
      // we can stay in the CrawlingForward state
      case MotionControllerState::CrawlingForward:
      // we can degrade the MovingForwardPID state into CrawlingForward
      case MotionControllerState::MovingForwardPID:
        localState = MotionControllerState::CrawlingForward;
        status.motion.Target = 50;
        status.motion.Enabled = 1;
        sendPacket(&SD1, response);
        return;

      // all other state transitions are invalid
      default:
        failure = FailureType::InvalidStateTransition;
        break;
    }
  }

  auto nack = NackPacket{CommandPacket::SendType, failure};
  sendPacket(&SD1, nack);
}

void handlePacketSendArm() {
  using CommandPacket = SendArmPacket;
  using ResponsePacket = NoArgumentPacket<SendArmPacket>;

  auto failure = FailureType::BadChecksum;
  CommandPacket packet;
  if (receivePacket(&SD1, packet)) {
    Lock lock(&stateSemaphore);
    if (localState == MotionControllerState::Disabled) {
      sendRemoteCommand(commandArm);

      ResponsePacket response;
      sendPacket(&SD1, response);
      return;
    } else {
      failure = FailureType::InvalidWhenActive;
    }
  }

  auto nack = NackPacket{CommandPacket::SendType, failure};
  sendPacket(&SD1, nack);
}

void handlePacketSendKeepalive() {
  using CommandPacket = SendKeepalivePacket;
  using ResponsePacket = NoArgumentPacket<SendKeepalivePacket>;

  auto failure = FailureType::BadChecksum;
  CommandPacket packet;
  if (receivePacket(&SD1, packet)) {
    Lock lock(&stateSemaphore);
    if (localState != MotionControllerState::Disabled
        && localState != MotionControllerState::Disabling) {
      sendRemoteCommand(commandKeepalive);

      ResponsePacket response;
      sendPacket(&SD1, response);
      return;
    } else {
      failure = FailureType::InvalidWhenDisabled;
    }
  }

  auto nack = NackPacket{CommandPacket::SendType, failure};
  sendPacket(&SD1, nack);
}

void handlePacketSendDisarm() {
  using CommandPacket = SendDisarmPacket;
  using ResponsePacket = NoArgumentPacket<SendDisarmPacket>;

  auto failure = FailureType::BadChecksum;
  CommandPacket packet;
  if (receivePacket(&SD1, packet)) {
    sendRemoteCommand(commandDisarm);

    ResponsePacket response;
    sendPacket(&SD1, response);
  } else {
    auto nack = NackPacket{CommandPacket::SendType, failure};
    sendPacket(&SD1, nack);
  }
}

void handlePacketResetOdometer() {
  using CommandPacket = ResetOdometerPacket;
  using ResponsePacket = NoArgumentPacket<ResetOdometerPacket>;

  auto failure = FailureType::BadChecksum;
  CommandPacket packet;
  if (receivePacket(&SD1, packet)) {
    {
      Lock lock(&stateSemaphore);
      status.odometer = 0;
    }

    ResponsePacket response;
    sendPacket(&SD1, response);
  } else {
    auto nack = NackPacket{CommandPacket::SendType, failure};
    sendPacket(&SD1, nack);
  }
}

void handlePacketSync() {
  sendSyncPacket();
}

/*void imuUpdate(unsigned long now) {
  // TODO: millis() can wrap (once every 49 days)
  if (imuNextUpdate > now) {
    return;
  }

  // Next update in 10 ms (from start)
  imuNextUpdate += 10UL;

  if (imu.isConnected()) {
    imu.getQUAT();
    OrientationPacket packet;
    packet.orientation = imu.getOrientation().quat;
    sendPacket(&SD1, packet);
  }
}*/

THD_WORKING_AREA(waPacketThread, 128);
THD_FUNCTION(PacketThread, arg) {
  sdStart(&SD1, NULL);
  sendSyncPacket();

  while (1) {
    PacketType packetType = PacketType::None;
    if (chnRead(&SD1, reinterpret_cast<uint8_t *>(&packetType), sizeof packetType)
        != sizeof packetType) {
      // something went wrong
      return;
    }

    switch (packetType) {
      case PacketType::ConfigurationSet:
        handlePacketConfigurationSet();
        break;

      case PacketType::ConfigurationGet:
        handlePacketConfigurationGet();
        break;

      case PacketType::SteeringSet:
        handlePacketSteeringSet();
        break;

      case PacketType::ThrottleSetPWM:
        handlePacketThrottleSetPWM();
        break;

      case PacketType::ThrottleSetPID:
        handlePacketThrottleSetPID();
        break;

      case PacketType::Crawl:
        handlePacketCrawl();
        break;

      case PacketType::SendArm:
        handlePacketSendArm();
        break;

      case PacketType::SendKeepalive:
        handlePacketSendKeepalive();
        break;

      case PacketType::SendDisarm:
        handlePacketSendDisarm();
        break;

      case PacketType::ResetOdometer:
        handlePacketResetOdometer();
        break;

      case PacketType::Sync:
        handlePacketSync();
        break;

      case PacketType::None:
      default:
        /* ruh roh */
        break;
    }
  }
}

THD_WORKING_AREA(waMainThread, 128);
THD_FUNCTION(MainThread, arg) {
  // Set up pin directions
  palSetLineMode(ENCODER_CHANNEL_A_LINE, PAL_MODE_INPUT);
  palSetLineMode(ENCODER_CHANNEL_B_LINE, PAL_MODE_INPUT);
  palSetLineMode(BUMPER_A_LINE, PAL_MODE_INPUT_PULLUP);
  palSetLineMode(BUMPER_B_LINE, PAL_MODE_INPUT_PULLUP);
  palSetLineMode(IMU_DATA_READ_LINE, PAL_MODE_INPUT);
  palSetLineMode(BATTERY_LOW_LINE, PAL_MODE_INPUT_PULLUP);
  palSetLineMode(SERVO_STEERING_LINE, PAL_MODE_OUTPUT_PUSHPULL);
  palSetLineMode(SERVO_THROTTLE_LINE, PAL_MODE_OUTPUT_PUSHPULL);

  // Configure I2C
  // TODO: check if we need to start at 100 Kbps, because we do end up in 400 Kbps
  static const I2CConfig config = {
    .clock_speed = 400000UL
  };
  i2cStart(&I2CD1, &config);

  // Configure Fast-PWM to overflow at 333 Hz.
  // At 16 MHz, gives 32000 steps across a 180 degree range
  // 1 ms = 16000
  static const PWMConfig pwm1cfg = {
    F_CPU,                            /* PWM frequency.         */
    (F_CPU / 333) - 1,                /* PWM period.            */
    NULL,                             /* TODO: comment.         */
    {
      {PWM_OUTPUT_ACTIVE_HIGH, NULL}, /* PWM channel 1 actived. */
      {PWM_OUTPUT_ACTIVE_HIGH, NULL}, /* PWM channel 2 actived. */
    },
  };
  pwmStart(&PWMD1, &pwm1cfg);
  setSteering(0);
  setThrottle(0);

  // set up encoder interrupts (get BITS 2 and 3 into state BITS 0 and 1)
  EICRA = _BV(ISC10) | _BV(ISC00);
  EIMSK = _BV(INT1) | _BV(INT0);
  encoderState = (palReadPort(PAL_PORT(ENCODER_CHANNEL_A_LINE))
      >> PAL_PAD(ENCODER_CHANNEL_A_LINE)) & 0x3;

  // set up bumper interrupts
  PCMSK2 = _BV(PAL_PAD(BUMPER_A_LINE)) | _BV(PAL_PAD(BUMPER_B_LINE));
  PCICR = _BV(PCIE2);

  //attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(imuDataReadyPin), imuDataReady, RISING);

  // Configure IMU
  //imu.start();

  // call out to what was the arduino loop function
  while (1) {
    //imuUpdate(now);
    controllerUpdate();

    // Process IMU
    /*if (imu.isConnected()) {
      if (imuDataPending) {
        imuDataPending = false;
        if (imu.poll()) {
          // send orientation packet
          OrientationPacket packet;
          packet.orientation = imu.getOrientation();
          sendPacket(packet);
        }
      }
    }*/

    chThdSleepMilliseconds(20);
  }
}

THD_TABLE_BEGIN
  THD_TABLE_THREAD(1, "packet", waPacketThread, PacketThread,  NULL)
  THD_TABLE_THREAD(0, "main", waMainThread, MainThread,  NULL)
THD_TABLE_END

int main(void) {
  halInit();
  chSysInit();
  while (true) {
    // idle loop, don't sleep
  }
}
