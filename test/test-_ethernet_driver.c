/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include "KSZ8851SNL.h"


/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)
#define SPI0_NODE DT_NODELABEL(my_spi_master)

const struct device *spi_dev;
struct spi_cs_control spim_cs =  {
	.gpio = SPI_CS_GPIOS_DT_SPEC_GET(DT_NODELABEL(reg_my_spi_master)),
	.delay = 0,
};   


// static const struct spi_dt_spec spi = SPI_DT_SPEC_GET(DT_NODELABEL(reg_my_spi_master),SPI_OP_MODE_MASTER,0);
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

static const struct spi_config spi_cfg = {
	.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
	.frequency = 125000,
	.slave = 0,
	.cs = &spim_cs,
};

// Logging
LOG_MODULE_REGISTER(SPI);






int main(void)
{
	// Initiate the external device.
	

	// Defining variables
	int ret;
	uint8_t rx_data[4];
	struct spi_buf buffer_rx[1];
	struct spi_buf_set rx_bufs = {
		.buffers = buffer_rx,
		.count = 1,
	};
	
	// Get the SPI device and check if ready. 
	spi_dev = DEVICE_DT_GET(SPI0_NODE);
	if (!device_is_ready(spi_dev)) {
		LOG_INF("SPI not read "); 
	}
	KSZ_init(spi_dev, &spi_cfg);

	// Check if led is ready. Used for indicating running main.
	if (!gpio_is_ready_dt(&led)) {
		LOG_INF("GPIO not read "); 
	}


	LOG_INF("SPI device and GPIO enabled!");

	
	// uint16_t byteEnabled = BYTES_ENBL_FOUR | BYTES_ENBL_THREE | BYTES_ENBL_TWO | BYTES_ENBL_ONE;
	// uint16_t regAdress = REG_OFFSET_CIDER;
	
	// //printk("Register address: %x \n", regAdress);
	// //printk("Byte enabled: %x \n", byteEnabled);

	// buffer_rx[0].buf = rx_data;
	// buffer_rx[0].len = sizeof(rx_data);
	// KSZ_read(spi_dev, &spi_cfg,  byteEnabled, regAdress, &rx_bufs);

	// LOG_HEXDUMP_INF(buffer_rx[0].buf,buffer_rx[0].len,"Read data: ");

	KSZ_init_QMU(spi_dev, &spi_cfg);

	uint16_t byteEnabled = BYTES_ENBL_FOUR | BYTES_ENBL_THREE | BYTES_ENBL_TWO | BYTES_ENBL_ONE;
	uint16_t regAdress = REG_OFFSET_MARL;
	uint32_t data = 0x01020206;
	KSZ_write(spi_dev, &spi_cfg, byteEnabled, regAdress, data);

	// byteEnabled = BYTES_ENBL_FOUR | BYTES_ENBL_THREE | BYTES_ENBL_TWO | BYTES_ENBL_ONE;
	// uint16_t regAdress = REG_OFFSET_MARL;
	
	//printk("Register address: %x \n", regAdress);
	//printk("Byte enabled: %x \n", byteEnabled);

	buffer_rx[0].buf = rx_data;
	buffer_rx[0].len = sizeof(rx_data);
	KSZ_read(spi_dev, &spi_cfg,  byteEnabled, regAdress, &rx_bufs);

	LOG_HEXDUMP_INF(buffer_rx[0].buf,buffer_rx[0].len,"Read data: ");

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return 0;
	}

	while (1) {
		ret = gpio_pin_toggle_dt(&led);
		if (ret < 0) {
			return 0;
		}
		k_msleep(SLEEP_TIME_MS);
	}
	return 0;
}
