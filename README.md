# Sensirion sen5x kernel driver

This repository contains a simple kernel driver (`driver.c`) for the sen5x 
sensor from Sensirion, along with a supporting Makefile and a Python script
(`quick_sensor_check.py`) to verify the presence of the sensor on your I2C bus.

## Files

* `driver.c`: The kernel module source code.

* `Makefile`: Build file for compiling the kernel module.

* `quick_sensor_check.py`: A Python script to verify the sensor is detected
correctly.

## Prerequisites

* Raspberry Pi running Raspbian (or any Debian-based distribution) with kernel
5.15.61.

* I2C interface enabled (use raspi-config to enable I2C if needed).

* Kernel headers installed (check [this repository](https://github.com/RPi-Distro/rpi-source)).

## Detecting the Sensor

To check if the sensor is connected and recognized on the I2C bus:

```bash
i2cdetect -y 1
```

This will scan I2C bus 1 and print a table of detected device addresses.

If `i2cdetect` does not return expected results, the I2C device module may not
be loaded. Load it manually:

```bash
sudo modprobe i2c-dev
```

You can re-run `i2cdetect -y 1` afterward to check again.

## Registering the Sensor

To make the kernel recognize and bind the sensor to your driver, you need to
manually register the device:

```bash
echo sen5x 0x69 | sudo tee /sys/bus/i2c/devices/i2c-1/new_device
```

* `sen5x` is the name expected by the driver.

* `0x69` is the I2C address of the sensor.

* `i2c-1` refers to I2C bus 1 (the sensor has to be connected to this I2C bus).

Ensure that your driver is either built into the kernel or loaded via `insmod`
before registering the device.

## Building and Loading the Driver

To build the kernel module:

```bash
make
```

To install the module:

```bash
make install
```

To uninstall it:

```bash
make uninstall
```

Check `dmesg` for logs related to loading/unloading the module and device
registration.

## Verifying Sensor Detection (Optional)

You can run the provided Python script to verify communication with the sensor:

```bash
# Install required package
pip3 install adafruit-blinka
# Run the script
python3 quick_sensor_check.py
```
