// SPDX-License-Identifier: GPL-2.0-or-later
/* Sensirion sen5x-DIS humidity and temperature sensor driver.
 * The sen5x comes in many different versions, this driver is for the
 * I2C version only.
 *
 * Copyright (C) 2016 Sensirion AG, Switzerland
 * Author: David Frey <david.frey@sensirion.com>
 * Author: Pascal Sachs <pascal.sachs@sensirion.com>
 */

#include <asm/page.h>
#include <linux/crc8.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/jiffies.h>

struct sen5x_platform_data {
	bool blocking_io;
	bool high_precision;
};

/* measurements commands */
static const unsigned char sen5x_cmd_start_measurement[]      = { 0x00, 0x21 };
static const unsigned char sen5x_cmd_stop_measurement[]       = { 0x01, 0x04 };
static const unsigned char sen5x_cmd_start_gas_only_mode[]     = { 0x00, 0x37 };
static const unsigned char sen5x_cmd_read_measured_values[]    = { 0x03, 0xC4 };
/* other commands */
static const unsigned char sen5x_cmd_clear_status_reg[]        = { 0xD2, 0x10 };

/* Delays */
#define sen5x_NEW_MEASUREMENT_INTERVAL_MS  1000  // 1 second
#define sen5x_READ_CMD_WAIT_TIME_US        20000 // 20 ms
#define sen5x_CHANGE_MODE_WAIT_TIME_US     50000 // 50 ms
#define sen5x_STOP_CMD_WAIT_TIME_US        200000 // 200 ms

#define sen5x_CMD_LENGTH                   2
#define sen5x_MEASUREMENT_RESPONSE_LENGTH  24
#define sen5x_CRC8_LEN                     1
#define sen5x_CRC8_POLYNOMIAL              0x31
#define sen5x_CRC8_INIT                    0xFF

DECLARE_CRC8_TABLE(sen5x_crc8_table);

enum {
    pm_1_0 = 0,
    pm_2_5,
    pm_4_0,
    pm_10,
    humidity,
    temperature,
    voc_index,
    nox_index,
};

struct sen5x_data {
	struct i2c_client *client;
	struct mutex i2c_lock; /* lock for sending i2c commands */
	struct mutex data_lock; /* lock for updating driver data */

	u8 mode;
	const unsigned char *command;
	u32 wait_time;			/* in us*/
	unsigned long last_update;	/* last update in periodic mode*/

	struct sen5x_platform_data setup;

	/*
	 * cached values for PM1.0, PM2.5, PM4.0, PM10, compensated ambient humidity
	 * compensated ambient temperature, VOC index, NOx index
	 */
	unsigned int pm_1_0;
    unsigned int pm_2_5;
    unsigned int pm_4_0;
    unsigned int pm_10;
    int humidity;
    int temperature;
    int voc_index;
    int nox_index;
};

// =============================================================
// Utility functions
// =============================================================

static int sen5x_read_from_command(struct i2c_client *client,
				   struct sen5x_data *data,
				   const char *command,
				   char *buf, int length, u32 wait_time)
{
	int ret;

	mutex_lock(&data->i2c_lock);
	ret = i2c_master_send(client, command, sen5x_CMD_LENGTH);

	if (ret != sen5x_CMD_LENGTH) {
		ret = ret < 0 ? ret : -EIO;
		goto out;
	}

	if (wait_time)
		usleep_range(wait_time, wait_time + 1000);

	ret = i2c_master_recv(client, buf, length);
	if (ret != length) {
		ret = ret < 0 ? ret : -EIO;
		goto out;
	}

	ret = 0;
out:
	mutex_unlock(&data->i2c_lock);
	return ret;
}

static struct sen5x_data *sen5x_update_client(struct device *dev)
{
	struct sen5x_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	unsigned long interval_jiffies = \
        msecs_to_jiffies(sen5x_NEW_MEASUREMENT_INTERVAL_MS);
	unsigned char buf[sen5x_MEASUREMENT_RESPONSE_LENGTH];
	int ret = 0;

	mutex_lock(&data->data_lock);
	/*
	 * The sensor has a new measurement every second, so we
     * only read the sensor if the last update was more than 1 second ago.
     * This is to avoid reading the sensor too often and to save power.
	 */
	if (time_after(jiffies, data->last_update + interval_jiffies)) {
		ret = sen5x_read_from_command(client, data, data->command, buf,
					      sizeof(buf), data->wait_time);
		if (ret)
			goto out;

        // Note that temperature is in scale 200, we make it scale 100
		data->pm_1_0 = be16_to_cpup((__be16 *)buf);
        data->pm_2_5 = be16_to_cpup((__be16 *)(buf + 3));
        data->pm_4_0 = be16_to_cpup((__be16 *)(buf + 6));
        data->pm_10 = be16_to_cpup((__be16 *)(buf + 9));
        data->humidity = be16_to_cpup((__be16 *)(buf + 12));
        data->temperature = be16_to_cpup((__be16 *)(buf + 15)) >> 1;
        data->voc_index = be16_to_cpup((__be16 *)(buf + 18));
        data->nox_index = be16_to_cpup((__be16 *)(buf + 21));
		data->last_update = jiffies;
	}

out:
	mutex_unlock(&data->data_lock);
	if (ret)
		return ERR_PTR(ret);

	return data;
}

// =============================================================
// Sensor attributes functions
// =============================================================

/* sysfs attributes */
static ssize_t measurement_input_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct sen5x_data *data = sen5x_update_client(dev);
    u8 index = to_sensor_dev_attr(attr)->index;

	if (IS_ERR(data))
		return PTR_ERR(data);

    switch (index) {
        case pm_1_0:
            return sprintf(buf, "%u\n", data->pm_1_0);
        case pm_2_5:
            return sprintf(buf, "%u\n", data->pm_2_5);
        case pm_4_0:
            return sprintf(buf, "%u\n", data->pm_4_0);
        case pm_10:
            return sprintf(buf, "%u\n", data->pm_10);
        case humidity:
            return sprintf(buf, "%d\n", data->humidity);
        case temperature:
            return sprintf(buf, "%d\n", data->temperature);
        case voc_index:
            return sprintf(buf, "%d\n", data->voc_index);
        case nox_index:
            return sprintf(buf, "%d\n", data->nox_index);
        default:
            return -EINVAL;
    }
}

static ssize_t mode_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf,
				     size_t count)
{
    struct sen5x_data *data = dev_get_drvdata(dev);
    struct i2c_client *client = data->client;
    unsigned int mode;
    int ret;

    ret = kstrtouint(buf, 0, &mode);
    if(ret)
        return ret;

    if (data->mode == mode) {
        dev_info(dev, "Mode is already set to %d\n", mode);
        return count;
    }

    mutex_lock(&data->i2c_lock);
    switch (mode) {
        case 0: // IDLE mode, stop all measurements
            ret = i2c_master_send(client, sen5x_cmd_stop_measurement,
            sen5x_CMD_LENGTH);
            if (ret != sen5x_CMD_LENGTH){
                ret = ret < 0 ? ret : -EIO;
                goto out;
            }
            usleep_range(sen5x_STOP_CMD_WAIT_TIME_US, \
                sen5x_STOP_CMD_WAIT_TIME_US + 1000);
            break;
        case 1: // All measurements mode
            ret = i2c_master_send(client, sen5x_cmd_stop_measurement,
                sen5x_CMD_LENGTH);
            if (ret != sen5x_CMD_LENGTH){
                ret = ret < 0 ? ret : -EIO;
                goto out;
            }
            usleep_range(sen5x_STOP_CMD_WAIT_TIME_US, \
                sen5x_STOP_CMD_WAIT_TIME_US + 1000);
            ret = i2c_master_send(client, sen5x_cmd_start_measurement,
                sen5x_CMD_LENGTH);
            if (ret != sen5x_CMD_LENGTH){
                ret = ret < 0 ? ret : -EIO;
                goto out;
            }
            usleep_range(sen5x_CHANGE_MODE_WAIT_TIME_US, sen5x_CHANGE_MODE_WAIT_TIME_US + 1000);
            break;
        case 2: // Gas only mode
            ret = i2c_master_send(client, sen5x_cmd_stop_measurement,
            sen5x_CMD_LENGTH);
            if (ret != sen5x_CMD_LENGTH){
                ret = ret < 0 ? ret : -EIO;
                goto out;
            }
            usleep_range(sen5x_STOP_CMD_WAIT_TIME_US, \
                sen5x_STOP_CMD_WAIT_TIME_US + 1000);
            ret = i2c_master_send(client, sen5x_cmd_start_gas_only_mode,
                sen5x_CMD_LENGTH);
            if (ret != sen5x_CMD_LENGTH){
                ret = ret < 0 ? ret : -EIO;
                goto out;
            }
            usleep_range(sen5x_CHANGE_MODE_WAIT_TIME_US, sen5x_CHANGE_MODE_WAIT_TIME_US + 1000);
            break;
        default:
            dev_err(dev, "Invalid mode value: %d. Use 0 for IDLE, 1 for all measurements or 2 for gas only.\n", mode);
            return -EINVAL;
    }

    data->mode = (u8) mode;
    ret = count;
out:
    mutex_unlock(&data->i2c_lock);
    return ret;
}

static ssize_t mode_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
    struct sen5x_data *data = dev_get_drvdata(dev);
    if (data->mode == 0) {
        return scnprintf(buf, PAGE_SIZE, "Device in IDLE mode\n");
    }
    if (data->mode == 1) {
        return scnprintf(buf, PAGE_SIZE, "Device in all measurement mode\n");
    } else {
        return scnprintf(buf, PAGE_SIZE, "Device in gas only mode\n");
    }
}


static SENSOR_DEVICE_ATTR_RO(pm_1_0_input, measurement_input, pm_1_0);
static SENSOR_DEVICE_ATTR_RO(pm_2_5_input, measurement_input, pm_2_5);
static SENSOR_DEVICE_ATTR_RO(pm_4_0_input, measurement_input, pm_4_0);
static SENSOR_DEVICE_ATTR_RO(pm_10_input, measurement_input, pm_10);
static SENSOR_DEVICE_ATTR_RO(humidity_input, measurement_input, humidity);
static SENSOR_DEVICE_ATTR_RO(temperature_input, measurement_input, temperature);
static SENSOR_DEVICE_ATTR_RO(voc_index_input, measurement_input, voc_index);
static SENSOR_DEVICE_ATTR_RO(nox_index_input, measurement_input, nox_index);

static SENSOR_DEVICE_ATTR_RW(mode, mode, 0);

static struct attribute *sen5x_attrs[] = {
	&sensor_dev_attr_pm_1_0_input.dev_attr.attr,
	&sensor_dev_attr_pm_2_5_input.dev_attr.attr,
	&sensor_dev_attr_pm_4_0_input.dev_attr.attr,
	&sensor_dev_attr_pm_10_input.dev_attr.attr,
	&sensor_dev_attr_humidity_input.dev_attr.attr,
	&sensor_dev_attr_temperature_input.dev_attr.attr,
	&sensor_dev_attr_voc_index_input.dev_attr.attr,
	&sensor_dev_attr_nox_index_input.dev_attr.attr,
    &sensor_dev_attr_mode.dev_attr.attr,
	NULL
};

ATTRIBUTE_GROUPS(sen5x);

static const struct i2c_device_id sen5x_ids[];

static int sen5x_probe(struct i2c_client *client)
{
	int ret;
	struct sen5x_data *data;
	struct device *hwmon_dev;
	struct i2c_adapter *adap = client->adapter;
	struct device *dev = &client->dev;
	const struct attribute_group **attribute_groups;
    
	/*
    * we require full i2c support since the sen5x uses multi-byte read and
    * writes as well as multi-byte commands which are not supported by
    * the smbus protocol
    */
    printk(KERN_INFO "sen5x driver: Checking i2c functionality...\n");
	if (!i2c_check_functionality(adap, I2C_FUNC_I2C))
		return -ENODEV;

    printk(KERN_INFO "sen5x driver: Clearing status register...\n");
	ret = i2c_master_send(client, sen5x_cmd_clear_status_reg,
			      sen5x_CMD_LENGTH);
	if (ret != sen5x_CMD_LENGTH)
		return ret < 0 ? ret : -ENODEV;
    // Clear the status register needs < 20 ms delay
    usleep_range(20000, 30000);

    printk(KERN_INFO "sen5x driver: Starting measurements mode...\n");
	ret = i2c_master_send(client, sen5x_cmd_start_measurement,
			      sen5x_CMD_LENGTH);
	if (ret != sen5x_CMD_LENGTH)
		return ret < 0 ? ret : -ENODEV;

    printk(KERN_INFO "sen5x driver: Malloc of device data...\n");
	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

    printk(KERN_INFO "sen5x driver: Initializing device data...\n");
	data->setup.blocking_io = false;
	data->setup.high_precision = true;
	data->mode = 1;
	data->last_update = jiffies - msecs_to_jiffies(3000);
	data->client = client;
	crc8_populate_msb(sen5x_crc8_table, sen5x_CRC8_POLYNOMIAL);
	if (client->dev.platform_data)
		data->setup = *(struct sen5x_platform_data *)dev->platform_data;
	data->command = sen5x_cmd_read_measured_values;
    data->wait_time = sen5x_READ_CMD_WAIT_TIME_US;
	mutex_init(&data->i2c_lock);
	mutex_init(&data->data_lock);

    attribute_groups = sen5x_groups;

    printk(KERN_INFO "Registering hwmon device\n");
	hwmon_dev = devm_hwmon_device_register_with_groups(dev,
							   client->name,
							   data,
							   attribute_groups);

	if (IS_ERR(hwmon_dev))
		dev_dbg(dev, "unable to register hwmon device\n");

	return PTR_ERR_OR_ZERO(hwmon_dev);
}

/* device ID table */
static const struct i2c_device_id sen5x_ids[] = {
	{"sen5x", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, sen5x_ids);

static struct i2c_driver sen5x_i2c_driver = {
	.driver.name = "sen5x",
	.probe_new   = sen5x_probe,
	.id_table    = sen5x_ids,
};

module_i2c_driver(sen5x_i2c_driver);

MODULE_AUTHOR("Alberto Martin <martmartalb@gmail.com>");
MODULE_DESCRIPTION("Sensirion sen5x air quality sensor driver");
MODULE_LICENSE("GPL");
