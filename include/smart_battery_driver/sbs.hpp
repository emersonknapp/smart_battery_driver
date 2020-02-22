// Copyright 2020 Emerson Knapp
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef SMART_BATTERY_DRIVER__SBS_HPP_
#define SMART_BATTERY_DRIVER__SBS_HPP_

#include <string>

namespace SBS
{

enum class SBSCommand
{
  ManufacturerAccess = 0x00,
  RemainingCapacityAlarm = 0x01,
  RemainingTimeAlarm = 0x02,
  BatteryMode = 0x03,
  AtRate = 0x04,
  AtRateTimeToFull = 0x05,
  AtRateTimeToEmpty = 0x06,
  AtRateOK = 0x07,
  Temperature = 0x08,
  Voltage = 0x09,
  Current = 0x0a,
  AverageCurrent = 0x0b,
  MaxError = 0x0c,
  RelativeStateOfCharge = 0x0d,
  AbsoluteStateOfCharge = 0x0e,
  RemainingCapacity = 0x0f,
  FullChargeCapacity = 0x10,
  RunTimeToEmpty = 0x11,
  AverageTimeToEmpty = 0x12,
  AverageTimeToFull = 0x13,
  ChargingCurrent = 0x14,
  ChargingVoltage = 0x15,
  BatteryStatus = 0x16,
  CycleCount = 0x17,
  DesignCapacity = 0x18,
  DesignVoltage = 0x19,
  SpecificationInfo = 0x1a,
  ManufactureDate = 0x1b,
  SerialNumber = 0x1c,
  // 0x1d-0x1f reserved
  ManufacturerName = 0x20,
  DeviceName = 0x21,
  CellChemistry = 0x22,
  ManufacturerData = 0x23,
};

enum class BatteryStatus : unsigned
{
  Initialized     = 0x0080,
  Discharging     = 0x0040,
  FullyCharged    = 0x0020,
  FullyDischarged = 0x0010,
};

enum class BatteryStatusAlarm : unsigned
{
  OverCharged       = 0x8000,
  TerminateCharge   = 0x4000,
  OverTemp          = 0x1000,
  RemainingCapacity = 0x0200,
  RemainingTime     = 0x0100,
  Mask              = 0xFF00,
};

enum class BatteryStatusError : unsigned
{
  OK                 = 0x0,
  Busy               = 0x1,
  ReservedCommand    = 0x2,
  UnsupportedCommand = 0x3,
  AccessDenied       = 0x4,
  OverflowUnderflow  = 0x5,
  BadSize            = 0x6,
  UnknownError       = 0x7,
};

class SmartBattery
{
public:
  SmartBattery(unsigned int i2cbus, unsigned int address);
  virtual ~SmartBattery() = default;

  bool manufacturerName(std::string & data) const;
  bool deviceName(std::string & data) const;
  bool cellChemistry(std::string & data) const;

  // mV
  int voltage() const;
  // mA
  int16_t current() const;
  // mAh
  int fullChargeCapacity() const;
  // mAh
  int designCapacity() const;
  // percent
  int relativeStateOfCharge() const;
  // bit flags, use above enums to interpret
  int batteryStatus() const;
  // number
  int serialNumber() const;
  // Celsius
  float temperature() const;

private:
  int readWord(SBSCommand command) const;
  int i2cbus_;
  int address_;
  int file_;
};

}  // namespace SBS

#endif  // SMART_BATTERY_DRIVER__SBS_HPP_
