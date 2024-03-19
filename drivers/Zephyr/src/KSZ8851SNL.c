#include "KSZ8851SNL.h"

LOG_MODULE_DECLARE(SPI);

void KSZ_init(const struct device *dev, const struct spi_config *spi_cfg)
{
    LOG_INF("Initiate KSZ: resetting.");

    // Reset pin initiating
    const struct device* reset_pin_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
    if (!reset_pin_dev) {
        LOG_ERR("Cannot get GPIO device");
        return;
    }
    int ret = gpio_pin_configure(reset_pin_dev, RESET_PIN, GPIO_OUTPUT_LOW);
    if (ret < 0) {
        LOG_ERR("Cannot configure GPIO device");
        return;
    }
    
    // Reset device.
    gpio_pin_set(reset_pin_dev, RESET_PIN, 1);
    k_msleep(50);// Reset input must be held low for a minimum of 10 ms after stable supply voltage (3.3V).
    gpio_pin_set(reset_pin_dev, RESET_PIN, 0); // Reset the KSZ8851SNL
    k_msleep(50);
    gpio_pin_set(reset_pin_dev, RESET_PIN, 1);
    k_msleep(20);

    LOG_INF("Initiate KSZ: device is reset.");

    
    // Assert that the device have the correct ID. 
    LOG_INF("Initiate KSZ: Checking ID.");
    uint16_t byteEnabled = BYTES_ENBL_TWO | BYTES_ENBL_ONE;
	uint16_t regAdress = REG_OFFSET_CIDER;
    uint8_t rx_data[4];
	struct spi_buf buffer_rx[1];
	struct spi_buf_set rx_bufs = {
		.buffers = buffer_rx,
		.count = 1,
	};
    buffer_rx[0].buf = rx_data;
	buffer_rx[0].len = sizeof(rx_data);
    KSZ_read(dev, spi_cfg, byteEnabled, regAdress, &rx_bufs);
    if((rx_data[2] != 0x72) | (rx_data[3] != 0x88)){
        LOG_ERR("Incorrect assertion.");
    }
    // zassert(rx_data[2] == 0x73);
    // zassert(rx_data[3] == 0x89);

    // LOG_HEXDUMP_INF(buffer_rx[0].buf,buffer_rx[0].len,"Read data: ");
    // LOG_INF("Reading buffer: %x should be 72", rx_data[2]);

    LOG_INF("Initiate KSZ: Device initiated and in correct operating state. \n ########## ");
}

// void KSZ_init_transmitt(const struct device *dev){
//     /* 
//     1. Read chip ID assert is correct 0x8872;
//     2. Write the QMU MAC address low, medium and high. 
//     3. Enable QMU Transmit Frame Data Pointer
//     4. Enable QMU Transmit flow control/padding/crc and UDP checksum.
//     */
// }

void KSZ_init_QMU(const struct device *dev, const struct spi_config *spi_cfg){
    /*
    The device does not have a default MAC address so one should be provided to it. 
    */

    uint16_t byteEnabled = BYTES_ENBL_FOUR | BYTES_ENBL_THREE | BYTES_ENBL_TWO | BYTES_ENBL_ONE;
	uint16_t regAdress = REG_OFFSET_MARL;
    uint8_t rx_data[4];
	struct spi_buf buffer_rx[1];
	struct spi_buf_set rx_bufs = {
		.buffers = buffer_rx,
		.count = 1,
	};
    buffer_rx[0].buf = rx_data;
	buffer_rx[0].len = sizeof(rx_data);
    KSZ_read(dev, spi_cfg, byteEnabled, regAdress, &rx_bufs);
    LOG_HEXDUMP_INF(buffer_rx[0].buf,buffer_rx[0].len,"Read data: ");
    
}

void KSZ_read(const struct device *dev, const struct spi_config *spi_cfg, uint16_t byteEnabled, uint16_t regAdress, const struct spi_buf_set * rx_bufs){
    /*
    We have to read from the start of each 32-Bit register on the device. From this address start we choose which of the bytes we want to read. 
    */
    LOG_INF("Reading register: %x", regAdress);

    // Variables
    int ret;

    // Create tx message
    uint16_t command = CMD_READ_REG; // Read command
    uint16_t command_phase = ((command & 0x3) << 14) | ((byteEnabled & 0xF) << 10) | ((regAdress & 0xFC) << 2); // Creating tx message. 


    // Construct tx message packet
    uint8_t tx_data[2]; // Save generated message in
    struct spi_buf buffer_tx[1];  // TX message package format. 
    struct spi_buf_set tx_bufs = { // TX message package format. 
		.buffers = buffer_tx,
		.count = 1,
	};
    tx_data[1] = (uint8_t)(command_phase & 0xFF);
	tx_data[0] = (uint8_t)((command_phase >> 8) & 0xFF);
    buffer_tx[0].buf = tx_data;
	buffer_tx[0].len = sizeof(tx_data);

    // Send read message
    LOG_INF("The Message being sent: %x", command_phase);
    ret = spi_transceive(dev, spi_cfg, &tx_bufs, rx_bufs);
	if (ret) { 
		LOG_INF("SPI write status: %d", ret); 
	}

    LOG_INF("Succesfully read register. \n #####");
}

void KSZ_write(const struct device *dev, const struct spi_config *spi_cfg, uint16_t byteEnabled, uint16_t regAdress, uint32_t data){
    LOG_INF("Writing to register: %x", regAdress);

    // Variables
    int ret;

    // Create tx message
    uint16_t command = CMD_WRITE_REG; // Write command
    uint16_t command_phase = ((command & 0x3) << 14) | ((byteEnabled & 0xF) << 10) | ((regAdress & 0xFC) << 2); // Creating tx message. 
    //uint32_t data_phase = // Dependent on byteEnabled, how many databytes are we sending.  

    // Construct tx message packet
    uint8_t tx_data[6]; // Save generated message in
    struct spi_buf buffer_tx[1];  // TX message package format. 
    struct spi_buf_set tx_bufs = { // TX message package format. 
		.buffers = buffer_tx,
		.count = 1,
	};
    tx_data[1] = (uint8_t)(command_phase & 0xFF);
	tx_data[0] = (uint8_t)((command_phase >> 8) & 0xFF);
    tx_data[2] = (uint8_t)(data & 0xFF);
    tx_data[3] = (uint8_t)((data >> 8) & 0xFF);;
    tx_data[4] = (uint8_t)((data >> 16) & 0xFF);;
    tx_data[5] = (uint8_t)((data >> 24) & 0xFF);;
    buffer_tx[0].buf = tx_data;
	buffer_tx[0].len = sizeof(tx_data);

    // Send read message
    LOG_INF("The command being sent: %x", command_phase);
    LOG_INF("The data being sent: %x", data);
    LOG_HEXDUMP_INF(buffer_tx[0].buf,buffer_tx[0].len,"Tx packet being sent: ");
    ret = spi_transceive(dev, spi_cfg, &tx_bufs, NULL); // Can we use NULL for no read buffer?
	if (ret) { 
		LOG_INF("SPI write status: %d", ret); 
	}

    LOG_INF("Succesfully write register. \n #####");
}



// void KSZ_transmitt(const struct device *dev){
//     /*
//     Transmit driver must write each frame data to align with double word boundary. 
//     e.g. Size 65 bytes must be sent as 68 bytes. 
    
    
//     */
// }