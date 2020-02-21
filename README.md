# Smart Battery Driver

This is an interface to Smart Battery System (SBS) devices that are available via Linux I2C.
SBS devices communicate via the SMBus protocol, which can use a standard I2C bus.
This package implements the [Smart Battery Data Specification Rev 1.1](http://sbs-forum.org/specs/) to query these SMBus devices and report their state as a `sensor_msgs/BatteryState`


## API

### Node: `smart_battery_driver`
#### Published Topics

* `~battery_state` (sensor_msgs/BatteryState)
  * Collected battery information

#### Subscribed Topics
None.


#### Parameters

For help discovering these values, see following section `I2C Querying`

* `~i2c_bus` (`int`, required)
  * Integer number of the Linux I2C bus where the device can be found
* `~battery_address` (`int`, required)
  * 7-bit SMBus address of the Smart Battery
* `~publish_frequency` (`float`, default: `1`)
  * In Hz, how frequently to query and publish the battery's information


## Tested Hardware

NOTE: any SBS-compliant battery should be compatible with this driver.
The following list shows combinations that have been actually tried.

Combination 1:
* [CP2112 USB to SMBus Bridge Evaluation Kit](https://www.silabs.com/products/development-tools/interface/cp2112ek-evaluation-kit)
* [RRC PMM240 Smart Battery Charger](https://www.rrc-ps.com/en/battery-packs/standard-battery-packs/products/rrc-pmm240/)
* [RRC2054 Smart Battery Pack](https://www.rrc-ps.com/en/battery-packs/standard-battery-packs/products/rrc2054/)
* The ROS 2 robot connects to the CP2112 via USB
* The CP2112 is connected to the PMM240 SCL, SDA, and GND lines via the H1 pins, which provides SMBus communication with the Charger and Battery.


## I2C Querying Help

For the following commands, you need to install the prerequisite `i2c-tools`

```
sudo apt-get install i2c-tools
```


### Finding desired I2C bus

To find out what i2c buses are present on the system, run

```
i2cdetect -l
```

Example output:

```
i2c-3   unknown         DPDDC-B                          N/A
i2c-1   unknown         i915 gmbus dpb                   N/A
i2c-6   unknown         CP2112 SMBus Bridge on hidraw0   N/A
i2c-4   unknown         DPDDC-C                          N/A
i2c-2   unknown         i915 gmbus dpd                   N/A
i2c-0   unknown         i915 gmbus dpc                   N/A
i2c-5   unknown         DPDDC-D                          N/A
```

In the case of Tested Hardware Combination 1, `i2c-6` is the CP2112 bridge, so `~i2c_bus` parameter should be "6".


### Finding I2C Device Addresses

Once you have determined the bus number to use, you can find out what devices addresses are present.

```
i2cdetect -y -r $BUS_NUM
```

If you connect only one device at a time to the bus, this will easily allow you to determine the addresses of your devices.

Example output:

```
0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- -- -- -- -- -- -- -- 0b -- -- -- --
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
70: -- -- -- -- -- -- -- --
```

In this case, I only had the Smart Battery connected, and now I know that its device address is `0x0B`
