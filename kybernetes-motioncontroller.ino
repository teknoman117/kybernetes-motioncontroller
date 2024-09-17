/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

#define ENCODER_OPTIMIZE_INTERRUPTS
#define ENCODER_USE_INTERRUPTS

#include <Encoder.h>
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
}

Encoder encoder(encoderChannelAPin, encoderChannelBPin);

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

unsigned long controllerNextUpdate = 0;

volatile bool bumperContacted = false;

void bumperContact() {
  bumperContacted = true;
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
  OCR1A = value * 16;
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
  OCR1B = value * 16;
}

bool sendRemoteCommand(uint8_t command) {
  Wire.beginTransmission(killSwitchAddress);
  Wire.write(command);
  return Wire.endTransmission() == 0;
}

void loop() {
  auto now = millis();
  if (now < controllerNextUpdate) {
    return;
  }
  controllerNextUpdate += 2;

  // Check if any transitions need to happen
  KillSwitchStatusPacket packet;
  if (!getStructFromWireCRC8(killSwitchAddress, packet)) {
    return;
  }

  if (packet.servoSteeringInputUpdated) {
    Serial.print("steering: ");
    Serial.println(packet.servoSteeringInput);
    //setSteering(packet.servoSteeringInput - 1500);
  }

  if (packet.servoThrottleInputUpdated) {
    Serial.print("throttle: ");
    Serial.println(packet.servoThrottleInput);
  }

  // if the kill switch is disarmed
  if (packet.armable && packet.state == KillSwitchState::Disarmed) {
    sendRemoteCommand(commandArm);
    Serial.println("arming");
  } else if (packet.state == KillSwitchState::Armed) {
    sendRemoteCommand(commandKeepalive);
  } else if (packet.state == KillSwitchState::DisarmingRemote) {
    Serial.println("disarming...");
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
  
  //pinMode(servoThrottlePin, OUTPUT);
  //pinMode(servoSteeringPin, OUTPUT);

  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(bumperAPin), bumperContact, RISING);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(bumperBPin), bumperContact, RISING);

  // Configure Fast-PWM to overflow at 333 Hz.
  // At 16 MHz, gives 32000 steps across a 180 degree range
  // 1 ms = 16000
  DDRB |= _BV(PORTB2) | _BV(PORTB1);
  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10);
  ICR1 = (F_CPU / 333) - 1;
  setSteering(0);
  setThrottle(0);

  controllerNextUpdate = millis() + 20UL;
}  
