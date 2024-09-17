/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

#define ENCODER_OPTIMIZE_INTERRUPTS
#define ENCODER_USE_INTERRUPTS

#include <Encoder.h>
#include <Servo.h>
#include <Wire.h>

#include <PinChangeInterrupt.h>

#include "crc8.hpp"
#include "killswitch-types.hpp"
#include "motioncontroller-types.hpp"
#include "pid.hpp"

constexpr int killSwitchAddress = 0x08;

constexpr uint8_t encoderChannelAPin = 2;
constexpr uint8_t encoderChannelBPin = 3;
constexpr uint8_t bumperAPin = 4;
constexpr uint8_t bumperBPin = 5;
constexpr uint8_t imuDataReadyPin = 6;
constexpr uint8_t batteryLowPin = 7;
constexpr uint8_t servoThrottlePin = 10;
constexpr uint8_t servoSteeringPin = 9;

constexpr float defaultKp = 0.1f;
constexpr float defaultKi = 0.25f;
constexpr float defaultKd = 0.01f;

namespace {
  template <typename T> bool getStructFromWireCRC8(uint8_t address, T& obj) {
    auto length = Wire.requestFrom(address, sizeof(T) + 1);
    if (length != sizeof(T) + 1) {
      // drop buffer contents on error
      while (Wire.available()) {
        Wire.read();
      }
      return false;
    }

    for (int i = 0; i < sizeof(T); i++) {
      reinterpret_cast<uint8_t*>(&obj)[i] = Wire.read();
    }

    uint8_t checksum = Wire.read();
    return checksum == calculateCRC8(0, reinterpret_cast<uint8_t*>(&obj), sizeof(T));
  }

  template <typename T> void sendPacket(T& packet) {
    const uint8_t type = (const uint8_t) T::ReceiveType;
  
    while (Serial.availableForWrite() < T::ReceiveTransportSize + 2);
    Serial.write(type);
    Serial.write(reinterpret_cast<const uint8_t*>(&packet), T::ReceiveTransportSize);

    uint8_t crc8 = 0;
    crc8 = calculateCRC8(crc8, &type, 1);
    crc8 = calculateCRC8(crc8, reinterpret_cast<const uint8_t*>(&packet), T::ReceiveTransportSize);
    Serial.write(crc8);
    return true;
  }

  template <typename T> bool receivePacket(T& packet) {
    const uint8_t type = Serial.read();
    Serial.readBytes(reinterpret_cast<uint8_t*>(&packet), T::SendTransportSize);
    const uint8_t checksum = Serial.read();

    uint8_t crc8 = 0;
    crc8 = calculateCRC8(crc8, &type, 1);
    crc8 = calculateCRC8(crc8, reinterpret_cast<const uint8_t*>(&packet), T::SendTransportSize);
    return (crc8 == checksum);
  }

  void sendSyncPacket() {
    for (int i = 0; i < 16; i++) {
      Serial.write((uint8_t) PacketType::Sync);
    }
  }
}

Encoder encoder(encoderChannelAPin, encoderChannelBPin);

Servo servoSteering;
Servo servoThrottle;

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

unsigned long controllerNextUpdate = 0;

volatile bool bumperContacted = false;

void bumperContact() {
  bumperContacted = true;
}

void setSteering(int16_t value) {
  if (value > 0) {
    servoSteering.writeMicroseconds(value + configuration.servoSteeringDeadzoneLeft);
  } else if (value < 0) {
    servoSteering.writeMicroseconds(value + configuration.servoSteeringDeadzoneRight);
  } else {
    servoSteering.writeMicroseconds((configuration.servoSteeringDeadzoneLeft + configuration.servoSteeringDeadzoneRight) / 2);
  }
}

void setThrottle(int16_t value) {
  if (value > 0) {
    servoThrottle.writeMicroseconds(value + configuration.servoThrottleDeadzoneForward);
  } else if (value < 0) {
    servoThrottle.writeMicroseconds(value + configuration.servoThrottleDeadzoneBackward);
  } else {
    servoThrottle.writeMicroseconds((configuration.servoThrottleDeadzoneForward + configuration.servoThrottleDeadzoneBackward) / 2);
  }
}

bool sendRemoteCommand(uint8_t command) {
  Wire.beginTransmission(killSwitchAddress);
  Wire.write(command);
  return Wire.endTransmission() == 0;
}

void controllerUpdate(unsigned long now) {
  int16_t servoThrottleOutput = 0;

  // TODO: millis() can wrap (once every 49 days)
  if (controllerNextUpdate > now) {
    return;
  }

  // Next update in 20 ms (from start)
  controllerNextUpdate += 20UL;

  // Update PID controller
  auto ticks = encoder.readAndReset();
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
  uint8_t bumperPressed = digitalRead(bumperAPin) | digitalRead(bumperBPin) | bumperContacted;
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
  status.batteryLow = digitalRead(batteryLowPin) ? 0 : 1;
  status.bumperPressed = bumperPressed;
  sendPacket(status);
}

void handlePacketConfigurationGet() {
  using CommandPacket = NoArgumentPacket<ConfigurationPacket>;

  if (Serial.available() < CommandPacket::SendTransportSize + 2) {
    return;
  }

  CommandPacket packet;
  if (receivePacket(packet)) {
    sendPacket(configuration);
  } else {
    auto nack = NackPacket{CommandPacket::SendType, FailureType::BadChecksum};
    sendPacket(nack);
  }
}

void handlePacketConfigurationSet() {
  using CommandPacket = ConfigurationPacket;
  using ResponsePacket = NoArgumentPacket<ConfigurationPacket>;

  if (Serial.available() < CommandPacket::SendTransportSize + 2) {
    return;
  }

  auto failure = FailureType::BadChecksum;
  CommandPacket packet;
  if (receivePacket(packet)) {
    if (localState == MotionControllerState::Disabled) {
      configuration = packet;
      pid.setTunings(configuration.Kp, configuration.Ki, configuration.Kd);

      ResponsePacket response;
      sendPacket(response);
      return;
    } else {
      failure = FailureType::InvalidWhenActive;
    }
  }

  auto nack = NackPacket{CommandPacket::SendType, failure};
  sendPacket(nack);
}

void handlePacketSteeringSet() {
  using CommandPacket = SteeringSetPacket;
  using ResponsePacket = NoArgumentPacket<SteeringSetPacket>;

  if (Serial.available() < CommandPacket::SendTransportSize + 2) {
    return;
  }

  CommandPacket packet;
  if (receivePacket(packet)) {
    setSteering(packet.servoSteeringOutput);

    ResponsePacket response;
    sendPacket(response);
  } else {
    auto nack = NackPacket{CommandPacket::SendType, FailureType::BadChecksum};
    sendPacket(nack);
  }
}

void handlePacketThrottleSetPWM() {
  using CommandPacket = ThrottleSetPWMPacket;
  using ResponsePacket = NoArgumentPacket<ThrottleSetPWMPacket>;

  if (Serial.available() < CommandPacket::SendTransportSize + 2) {
    return;
  }

  auto failure = FailureType::BadChecksum;
  ResponsePacket response;
  CommandPacket packet;
  if (receivePacket(packet)) {
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
        sendPacket(response);
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
        sendPacket(response);
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
        sendPacket(response);
        return;
    }
  }

  auto nack = NackPacket{CommandPacket::SendType, failure};
  sendPacket(nack);
}

void handlePacketThrottleSetPID() {
  using CommandPacket = ThrottleSetPIDPacket;
  using ResponsePacket = NoArgumentPacket<ThrottleSetPIDPacket>;

  if (Serial.available() < CommandPacket::SendTransportSize + 2) {
    return;
  }

  auto failure = FailureType::BadChecksum;
  ResponsePacket response;
  CommandPacket packet;
  if (receivePacket(packet)) {
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
        sendPacket(response);
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
        sendPacket(response);
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
        sendPacket(response);
        return;
      
      // we can't directly promote PWM controlled motion to PID controlled motion
      case MotionControllerState::MovingForwardPWM:
      case MotionControllerState::MovingBackwardPWM:
        failure = FailureType::InvalidStateTransition;
        break;
    }
  }

  auto nack = NackPacket{CommandPacket::SendType, failure};
  sendPacket(nack);
}

void handlePacketCrawl() {
  using CommandPacket = CrawlPacket;
  using ResponsePacket = NoArgumentPacket<CrawlPacket>;

  if (Serial.available() < CommandPacket::SendTransportSize + 2) {
    return;
  }

  auto failure = FailureType::BadChecksum;
  ResponsePacket response;
  CommandPacket packet;
  if (receivePacket(packet)) {
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
        sendPacket(response);
        return;

      // all other state transitions are invalid
      default:
        failure = FailureType::InvalidStateTransition;
        break;
    }
  }

  auto nack = NackPacket{CommandPacket::SendType, failure};
  sendPacket(nack);
}

void handlePacketSendArm() {
  using CommandPacket = SendArmPacket;
  using ResponsePacket = NoArgumentPacket<SendArmPacket>;

  if (Serial.available() < CommandPacket::SendTransportSize + 2) {
    return;
  }

  auto failure = FailureType::BadChecksum;
  CommandPacket packet;
  if (receivePacket(packet)) {
    if (localState == MotionControllerState::Disabled) {
      sendRemoteCommand(commandArm);

      ResponsePacket response;
      sendPacket(response);
      return;
    } else {
      failure = FailureType::InvalidWhenActive;
    }
  }

  auto nack = NackPacket{CommandPacket::SendType, failure};
  sendPacket(nack);
}

void handlePacketSendKeepalive() {
  using CommandPacket = SendKeepalivePacket;
  using ResponsePacket = NoArgumentPacket<SendKeepalivePacket>;

  if (Serial.available() < CommandPacket::SendTransportSize + 2) {
    return;
  }

  auto failure = FailureType::BadChecksum;
  CommandPacket packet;
  if (receivePacket(packet)) {
    if (localState != MotionControllerState::Disabled
        && localState != MotionControllerState::Disabling) {
      sendRemoteCommand(commandKeepalive);

      ResponsePacket response;
      sendPacket(response);
      return;
    } else {
      failure = FailureType::InvalidWhenDisabled;
    }
  }

  auto nack = NackPacket{CommandPacket::SendType, failure};
  sendPacket(nack);
}

void handlePacketSendDisarm() {
  using CommandPacket = SendDisarmPacket;
  using ResponsePacket = NoArgumentPacket<SendDisarmPacket>;

  if (Serial.available() < CommandPacket::SendTransportSize + 2) {
    return;
  }

  auto failure = FailureType::BadChecksum;
  CommandPacket packet;
  if (receivePacket(packet)) {
    sendRemoteCommand(commandDisarm);

    ResponsePacket response;
    sendPacket(response);
  } else {
    auto nack = NackPacket{CommandPacket::SendType, failure};
    sendPacket(nack);
  }
}

void handlePacketResetOdometer() {
  using CommandPacket = ResetOdometerPacket;
  using ResponsePacket = NoArgumentPacket<ResetOdometerPacket>;

  if (Serial.available() < CommandPacket::SendTransportSize + 2) {
    return;
  }

  auto failure = FailureType::BadChecksum;
  CommandPacket packet;
  if (receivePacket(packet)) {
    status.odometer = 0;

    ResponsePacket response;
    sendPacket(response);
  } else {
    auto nack = NackPacket{CommandPacket::SendType, failure};
    sendPacket(nack);
  }
}

void handlePacketSync() {
  Serial.read();
  sendSyncPacket();
}

void loop() {
  auto now = millis();
  controllerUpdate(now);

  // Process packets
  if (!Serial.available()) {
    return;
  }

  PacketType packetType = (PacketType) Serial.peek();
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
      if (controllerNextUpdate - now < 2) {
        // prevent us from missing the next status update
        break;
      }
      handlePacketSendArm();
      break;
    
    case PacketType::SendKeepalive:
      if (controllerNextUpdate - now < 2) {
        // prevent us from missing the next status update
        break;
      }
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
  
    default:
      /* ruh roh */
      Serial.read();
      break;
  }
}

void setup() {
  // 1 Mbps UART
  Serial.begin(250000UL);

  // Configure I2C
  Wire.begin();
  Wire.setClock(250000UL);
  Wire.setWireTimeout(1000UL);

  pinMode(encoderChannelAPin, INPUT);
  pinMode(encoderChannelBPin, INPUT);
  pinMode(bumperAPin, INPUT_PULLUP);
  pinMode(bumperBPin, INPUT_PULLUP);
  pinMode(imuDataReadyPin, INPUT);
  pinMode(batteryLowPin, INPUT_PULLUP);
  pinMode(servoThrottlePin, OUTPUT);
  pinMode(servoSteeringPin, OUTPUT);

  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(bumperAPin), bumperContact, RISING);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(bumperBPin), bumperContact, RISING);

  servoSteering.attach(servoSteeringPin);
  servoThrottle.attach(servoThrottlePin);
  setSteering(0);
  setThrottle(0);

  sendSyncPacket();
  controllerNextUpdate = millis() + 20UL;
}  
