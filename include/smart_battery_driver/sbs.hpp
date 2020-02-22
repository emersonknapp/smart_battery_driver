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

class SmartBattery
{
public:
  SmartBattery(unsigned int i2cbus, unsigned int address);
  virtual ~SmartBattery() = default;

  bool manufacurerName(std::string & data);
  bool deviceName(std::string & data);
  bool cellChemistry(std::string & data);

private:
  int i2cbus_;
  int address_;
  int file_;
};

#endif  // SMART_BATTERY_DRIVER__SBS_HPP_
