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
#include <sys/ioctl.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

extern "C" {
  #include <linux/i2c.h>
  #include <linux/i2c-dev.h>
  #include <i2c/smbus.h>
}

#include <string>

#include "smart_battery_driver/sbs.hpp"

namespace SBS
{

#define MISSING_FUNC_FMT        "Error: Adapter does not have %s capability\n"

int check_smbus_capabilities(const int file)
{
  unsigned long funcs = 0;
  /* check adapter functionality */
  if (ioctl(file, I2C_FUNCS, &funcs) < 0) {
    fprintf(
      stderr, "Error: Could not get the adapter "
      "functionality matrix: %s\n", strerror(errno));
    return -1;
  }
  if (!(funcs & I2C_FUNC_SMBUS_READ_BYTE)) {
    fprintf(stderr, MISSING_FUNC_FMT, "SMBus receive byte");
    return -1;
  }
  if (!(funcs & I2C_FUNC_SMBUS_READ_BYTE_DATA)) {
    fprintf(stderr, MISSING_FUNC_FMT, "SMBus read byte");
    return -1;
  }

  if (!(funcs & I2C_FUNC_SMBUS_READ_WORD_DATA)) {
    fprintf(stderr, MISSING_FUNC_FMT, "SMBus read word");
    return -1;
  }
  if (!(funcs & (I2C_FUNC_SMBUS_PEC | I2C_FUNC_I2C))) {
    fprintf(
      stderr, "Warning: Adapter does "
      "not seem to support PEC\n");
  }
  return 0;
}

int open_device(const char * filename)
{
  int file = open(filename, O_RDWR);
  if (file < 0) {
    fprintf(
      stderr, "Error: Could not open file %s: %s\n", filename, strerror(ENOENT));
    if (errno == EACCES) {
      fprintf(stderr, "Access problem, maybe run as root\n");
    }
  }
  return file;
}

int set_i2c_slave_addr(int file, int address, int force)
{
  if (ioctl(file, force ? I2C_SLAVE_FORCE : I2C_SLAVE, address) < 0) {
    fprintf(
      stderr,
      "Error: Could not set address to 0x%02x: %s\n",
      address, strerror(errno));
    return -errno;
  }
  return 0;
}

SmartBattery::SmartBattery(const char * device_path, unsigned int address)
: address_(address)
{
  if (address < 0x03 || address > 0x77) {
    fprintf(stderr, "Error: device address out of range (0x03-0x77)\n");
    throw address_;
  }
  file_ = open_device(device_path);
  if (file_ < 0) {
    throw "Couldn't open battery device.";
  }
  if (check_smbus_capabilities(file_)) {
    throw "Bad capabilities";
  }
  if (set_i2c_slave_addr(file_, address_, false)) {
    throw "Couldnt set addr";
  }

  if (ioctl(file_, I2C_PEC, 1) < 0) {
    fprintf(stderr, "Error: Could not set PEC: %s\n", strerror(errno));
    close(file_);
    throw "Could not set PEC";
  }
}

bool SmartBattery::cellChemistry(std::string & data) const
{
  data.resize(32);
  unsigned char * buf = (unsigned char *)&data[0];
  int res = i2c_smbus_read_block_data(file_, 0x22, buf);
  if (res < 0) {
    fprintf(stderr, "Error: Read Failed\n");
    return false;
  }
  data.resize(res);
  return true;
}

int SmartBattery::readWord(SBSCommand command) const
{
  int res = i2c_smbus_read_word_data(file_, static_cast<int>(command));
  if (res < 0) {
    fprintf(stderr, "Error: Read failed\n");
  }
  return res;
}

int SmartBattery::voltage() const
{
  return readWord(SBSCommand::Voltage);
}

int16_t SmartBattery::current() const
{
  return readWord(SBSCommand::Current);
}

int SmartBattery::fullChargeCapacity() const
{
  return readWord(SBSCommand::FullChargeCapacity);
}

int SmartBattery::designCapacity() const
{
  return readWord(SBSCommand::DesignCapacity);
}

int SmartBattery::relativeStateOfCharge() const
{
  return readWord(SBSCommand::RelativeStateOfCharge);
}

int SmartBattery::batteryStatus() const
{
  return readWord(SBSCommand::BatteryStatus);
}

int SmartBattery::serialNumber() const
{
  return readWord(SBSCommand::SerialNumber);
}

float SmartBattery::temperature() const
{
  return (readWord(SBSCommand::Temperature) * 0.1) - 273.15;
}

}  // namespace SBS
