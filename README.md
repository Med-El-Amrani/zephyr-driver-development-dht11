# DHT11 Custom Driver Module for Zephyr RTOS

A custom DHT11 temperature and humidity sensor driver implemented as a Zephyr module for learning purposes.

## Overview 

This project demonstrates how to create a custom sensor driver as an external Zephyr module. The DHT11 is a digital temperature and humidity sensor that uses a single-wire bidirectional protocol.

## Features

- Full Zephyr sensor API implementation.
- Devicetree integration
- Kconfig configuration
- Structured as reusable Zephyr module
- Logging support 


## Enable in prj.conf to use in an application
```conf
# Enable DHT11 driver
CONFIG_DHT11=y

# Enable sensor subsystem
CONFIG_SENSOR=y

# Enable GPIO
CONFIG_GPIO=y

# Optional: Enable logging
CONFIG_LOG=y
CONFIG_SENSOR_LOG_LEVEL_DBG=y
```

## Devicetree Configuration to use in an application

### Example: nRF52840 DK
```dts
/ {
    dht11_sensor: dht11 {
        compatible = "custom,dht11";
        status = "okay";
        data-gpios = <&gpio0 4 GPIO_ACTIVE_HIGH>;
        power-gpios = <&gpio0 5 GPIO_ACTIVE_HIGH>;  /* Optional */
        read-interval = <2000>;  /* 2 seconds minimum between reads */
    };
};
```

### Example: ESP32
```dts
/ {
    dht11_sensor: dht11 {
        compatible = "custom,dht11";
        status = "okay";
        data-gpios = <&gpio0 15 GPIO_ACTIVE_HIGH>;
        read-interval = <2000>;
    };
};
```

### Devicetree Properties

| Property | Type | Required | Default | Description |
|----------|------|----------|---------|-------------|
| `compatible` | string | Yes | - | Must be "custom,dht11" |
| `data-gpios` | phandle-array | Yes | - | GPIO pin for bidirectional data |
| `power-gpios` | phandle-array | No | - | Optional GPIO for power control |
| `read-interval` | int | No | 2000 | Minimum read interval in milliseconds |


## Usage Example : Basic Application 

```c
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

int main(void)
{
    const struct device *dht11 = DEVICE_DT_GET_ANY(custom_dht11);
    struct sensor_value temp, humidity;
    int ret;

    /* Check if device is ready */
    if (!device_is_ready(dht11)) {
        LOG_ERR("DHT11 device not ready");
        return -1;
    }

    LOG_INF("DHT11 sensor initialized successfully");

    while (1) {
        /* Fetch sample from sensor */
        ret = sensor_sample_fetch(dht11);
        if (ret < 0) {
            LOG_ERR("Failed to fetch sample: %d", ret);
            k_sleep(K_SECONDS(2));
            continue;
        }

        /* Get temperature */
        ret = sensor_channel_get(dht11, SENSOR_CHAN_AMBIENT_TEMP, &temp);
        if (ret < 0) {
            LOG_ERR("Failed to get temperature: %d", ret);
            k_sleep(K_SECONDS(2));
            continue;
        }

        /* Get humidity */
        ret = sensor_channel_get(dht11, SENSOR_CHAN_HUMIDITY, &humidity);
        if (ret < 0) {
            LOG_ERR("Failed to get humidity: %d", ret);
            k_sleep(K_SECONDS(2));
            continue;
        }

        /* Print readings */
        LOG_INF("Temperature: %d.%06d°C", temp.val1, temp.val2);
        LOG_INF("Humidity: %d.%06d%%RH", humidity.val1, humidity.val2);

        /* Wait before next reading */
        k_sleep(K_SECONDS(3));
    }

    return 0;
}
```