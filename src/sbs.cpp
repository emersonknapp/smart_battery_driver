#include <sys/ioctl.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
#include "smart_battery_driver/sbs.hpp"
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
  ChargingCurrent= 0x14,
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

#define MISSING_FUNC_FMT        "Error: Adapter does not have %s capability\n"

int check_smbus_capabilities(int file)
{
  unsigned long funcs;

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

int parse_i2c_bus(const char * i2cbus_arg)
{
  unsigned long i2cbus;
  char * end;
  i2cbus = strtoul(i2cbus_arg, &end, 0);
  if (*end || !*i2cbus_arg) {
    fprintf(stderr, "Error: I2C bus not a number!\n");
    return -1;
  }
  if (i2cbus > 0xFFFFF) {
    fprintf(stderr, "Error: I2C bus out of range!\n");
    return -2;
  }
  return i2cbus;
}

int parse_i2c_address(const char * address_arg)
{
  long address;
  char * end;
  address = strtol(address_arg, &end, 0);
  if (*end || !*address_arg) {
    fprintf(stderr, "Error: device address is not a number\n");
    return -1;
  }
  if (address < 0x03 || address > 0x77) {
    fprintf(stderr, "Error: device address out of range (0x03-0x77)\n");
    return -2;
  }
  return address;
}

int open_i2c_dev(int i2cbus, char * filename, size_t size, bool quiet)
{
  int file;
  snprintf(filename, size, "/dev/i2c/%d", i2cbus);
  filename[size - 1] = '\0';
  file = open(filename, O_RDWR);

  if (file < 0 && (errno == ENOENT || errno == ENOTDIR)) {
    snprintf(filename, size, "/dev/i2c-%d", i2cbus);
    filename[size - 1] = '\0';
    file = open(filename, O_RDWR);
  }

  if (file < 0 && !quiet) {
    if (errno == ENOENT) {
      fprintf(
        stderr, "Error: Could not open file "
        "`/dev/i2c-%d' or `/dev/i2c/%d': %s\n",
        i2cbus, i2cbus, strerror(ENOENT));
    } else {
      fprintf(
        stderr, "Error: Could not open file "
        "`%s': %s\n", filename, strerror(errno));
      if (errno == EACCES) {
        fprintf(stderr, "Run as root?\n");
      }
    }
  }
  return file;
}

int set_i2c_slave_addr(int file, int address, int force)
{
  /* With force, let the user read from/write to the registers
     even when a driver is also running */
  if (ioctl(file, force ? I2C_SLAVE_FORCE : I2C_SLAVE, address) < 0) {
    fprintf(
      stderr,
      "Error: Could not set address to 0x%02x: %s\n",
      address, strerror(errno));
    return -errno;
  }
  return 0;
}


SmartBattery::SmartBattery(unsigned int i2cbus, unsigned int address)
: i2cbus_(i2cbus),
  address_(address)
{
  if (i2cbus_ > 0xFFFFF) {
    fprintf(stderr, "Error: I2C bus out of range!\n");
    throw i2cbus_;
  }
  if (address < 0x03 || address > 0x77) {
    fprintf(stderr, "Error: device address out of range (0x03-0x77)\n");
    throw address_;
  }

  char filename[20];
  file_ = open_i2c_dev(i2cbus_, filename, sizeof(filename), false);
  if (
    file_ < 0 ||
    check_smbus_capabilities(file_) ||
    set_i2c_slave_addr(file_, address_, false))
  {
    throw "Eep!";
  }

  if (ioctl(file_, I2C_PEC, 1) < 0) {
    fprintf(stderr, "Error: Could not set PEC: %s\n", strerror(errno));
    close(file_);
    throw "Could not set PEC";
  }
}
