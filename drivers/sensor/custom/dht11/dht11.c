/* File : dht11.c */
#define DT_DRV_COMPAT custom_dht11

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "dht11.h"

LOG_MODULE_REGISTER(dht11, CONFIG_SENSOR_LOG_LEVEL);

#define DHT11_START_SIGNAL_US 18000
#define DHT11_RESPONSE_TIMEOUT_US 100
#define DHT11_BIT_TIMEOUT_US 100
#define DHT11_DHT11_DATA_BITSDATA_BITS 40


/* Read a single bit from DHT11 */
static int dht11_read_bit(const struct gpio_dt_spec *data_gpio, uint8_t *bit){
    uint32_t start_time, high_time;
    int val;

    /* Wait for low signal */
    start_time = k_cyc_to_us_floor32(k_cycle_get_32());
    do {
        val = gpio_pin_get_dt(data_gpio);
        if(val < 0){
            return val;
        }
        if(k_cyc_to_us_floor32(k_cycle_get_32()) - start_time > DHT11_BIT_TIMEOUT_US){
            return -ETIMEDOUT; /* Timeout waiting for low signal */
        }
    } while (val == 1);

    /* Wait for high signal */
    start_time = k_cyc_to_us_floor32(k_cycle_get_32());
    do {
        val = gpio_pin_get_dt(data_gpio);
        if(val < 0){
            return val;
        }
        if(k_cyc_to_us_floor32(k_cycle_get_32()) - start_time > DHT11_BIT_TIMEOUT_US){
            return -ETIMEDOUT; /* Timeout waiting for high signal */
        }
    } while (val == 0);

    /* Measure duration of high signal */
    start_time = k_cyc_to_us_floor32(k_cycle_get_32());
    do {
        val = gpio_pin_get_dt(data_gpio);
        if(val < 0){
            return val;     
        }
        high_time = k_cyc_to_us_floor32(k_cycle_get_32()) - start_time;
        if(high_time > DHT11_BIT_TIMEOUT_US){
            return -ETIMEDOUT; /* Timeout waiting for end of high signal */
        }
    } while (val == 1);   

    /* Bit value depends on high signal duration (26-28us = 0, ~70us = 1)*/
    *bit = (high_time > 40) ? 1 : 0;
    return 0;
}

/* Read 40 bits of data from DHT11 */
static int dht11_read_data(const struct gpio_dt_spec *data_gpio, uint8_t* data){
    int ret;
    uint8_t bit;

    for(int i = 0; i < DHT11_DATA_BITS; i++){
        ret = dht11_read_bit(data_gpio, &bit);
        if(ret < 0){
            return ret;
        }
        data[i / 8] <<= 1;
        data[i / 8] |= bit;
    }
    return 0;
}

/* Initiate communication with DHT11 */
static int dht11_start_signal(const struct gpio_dt_spec *data_gpio){
    int ret;
    uint32_t start_time;
    int val;

    /* Set data pin as output and pull low for at least 18ms */
    ret = gpio_pin_configure_dt(data_gpio, GPIO_OUTPUT_LOW);
    if(ret < 0){
        return ret;     
    }

    k_busy_wait(DHT11_START_SIGNAL_US);

    /* Set data pin as input and wait for sensor response */
    ret = gpio_pin_configure_dt(data_gpio, GPIO_INPUT | GPIO_PULL_UP);
    if(ret < 0){
        return ret;     
    }

    /* wait for dht11 response (should pull low)*/
    start_time = k_cyc_to_us_floor32(k_cycle_get_32());
    do {
        val = gpio_pin_get_dt(data_gpio);
        if(val < 0){
            return val;         
        }
        if(k_cyc_to_us_floor32(k_cycle_get_32()) - start_time > DHT11_RESPONSE_TIMEOUT_US){
            return -ETIMEDOUT; /* Timeout waiting for response */
        }
    } while (val == 1);

    /* whait for dht1& to pull high */
    start_time = k_cyc_to_us_floor32(k_cycle_get_32());
    do {
        val = gpio_pin_get_dt(data_gpio);
        if(val < 0){
            return val;     
        }
        if(k_cyc_to_us_floor32(k_cycle_get_32()) - start_time > DHT11_RESPONSE_TIMEOUT_US){
            return -ETIMEDOUT; /* Timeout waiting for response */
        }
    } while (val == 0);

    /* Wait for DHT11 to pull low before data transmission */
    start_time = k_cyc_to_us_floor32(k_cycle_get_32());
    do {
        val = gpio_pin_get_dt(data_gpio);
        if(val < 0){
            return val;
        }
        if((k_cyc_to_us_floor32(k_cycle_get_32()) - start_time) > DHT11_RESPONSE_TIMEOUT_US){
            return -ETIMEDOUT; /* Timeout waiting for response */
        }
    } while (val == 1);

    return 0;
}

/* Initialize sensor */
static int dht11_init(const struct device *dev){
    const struct dht11_config *config= dev->config;
    struct dht_data *data = dev->data;
    int ret;

    /* Check if gpio device is ready*/
    if(!gpio_is_ready_dt(&config->data_gpio)){
        LOG_ERR("DHT11 data GPIO device not ready");
        return -ENODEV;
    }

    /* Check power gpio idf sepcified*/
    if(config->power_gpio.port != NULL){
        if(!gpio_is_ready_dt(&config->power_gpio)){
            LOG_ERR("DHT11 power gpio device not reday");
            return -ENODEV;
        }

        ret = gpio_pin_configure_dt(&config->power_gpio, GPIO_OUTPUT_HIGH);
        if(ret < 0){
            LOG_ERR("Failed to configure DHT11 power GPIO");
            return ret;
        }
        k_sleep(K_MSEC(1000)); /* Wait for sensor to power up */
    }

    /* Configure Data GPIO */
    ret = gpio_pin_configure_dt(&config->data_gpio, GPIO_INPUT | GPIO_PULL_UP);
    if(ret < 0){
        LOG_ERR("Failed to configure data gpio");
        return ret;
    } 

    data->last_read_time = sys_timepoint_calc(K_NO_WAIT);

    LOG_INF("DHT11 sensor initialized");
    

    return 0;
}

/* Sensor channel get */
static int dht11_channel_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val){
    struct dht11_data *data = dev->data;

    switch(chan){
        case SENSOR_CHAN_AMBIENT_TEMP:
            val->val1 = data->temperature / 10;
            val->val2 = (data->temperature % 10) * 100000;
            break;
        case SENSOR_CHAN_HUMIDITY:
            val->val1 = data->humidity / 10;
            val->val2 = (data->humidity % 10) * 100000;
            break;
        default:
            return -ENOTSUP;
    }

    return 0;
}

static const struct sensor_driver_api dht11_driver_api = {
    .channel_get = dht11_channel_get,
};