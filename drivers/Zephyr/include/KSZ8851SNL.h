/* Driver for Ethernet board.*/
#ifndef KSZ8851SNL_H
#define KSZ8851SNL_H

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
// #include <zephyr/ztest_assert.h>

// Commands
#define CMD_READ_REG 0x0
#define CMD_WRITE_REG 0x1
#define CMD_READ_RX 0x2
#define CMD_WRITE_TX 0x3

// Definme which byte we want to read from the 32bit reg. 
#define BYTES_ENBL_ZERO 0x0
#define BYTES_ENBL_ONE 0x1
#define BYTES_ENBL_TWO 0x2
#define BYTES_ENBL_THREE 0x4
#define BYTES_ENBL_FOUR 0x8

// Registers Base
#define DEVICE_BASE_ADDRESS 0x00

// Register offsets
#define REG_OFFSET_RESERVED1    0x00
#define REG_OFFSET_RESERVED2    0x02
#define REG_OFFSET_RESERVED3    0x04
#define REG_OFFSET_RESERVED4    0x06
#define REG_OFFSET_CCR          0x08
#define REG_OFFSET_RESERVED5    0x0A
#define REG_OFFSET_RESERVED6    0x0C
#define REG_OFFSET_RESERVED7    0x0E
#define REG_OFFSET_MARL         0X10
#define REG_OFFSET_MARM         0X12
#define REG_OFFSET_MARH         0X14
#define REG_OFFSET_RESERVED8    0x16
#define REG_OFFSET_RESERVED9    0x18
#define REG_OFFSET_RESERVED10   0x1A
#define REG_OFFSET_RESERVED11   0x1C
#define REG_OFFSET_RESERVED12   0x1E
#define REG_OFFSET_OBCR         0x20
#define REG_OFFSET_EEPCR        0x22

#define REG_OFFSET_MBIR         0x24
#define REG_OFFSET_GRR          0x26
#define REG_OFFSET_RESERVED13   0x28
#define REG_OFFSET_WFCR         0x2A
#define REG_OFFSET_RESERVED14   0x2C
#define REG_OFFSET_RESERVED15   0x2E
#define REG_OFFSET_WF0CRC0      0x30
#define REG_OFFSET_WF0CRC1      0x32
#define REG_OFFSET_WF0BMO       0x34
#define REG_OFFSET_WF0BM1       0x36
#define REG_OFFSET_WF0BM2       0x38
#define REG_OFFSET_WF0BM3       0x3A
#define REG_OFFSET_RESERVED16   0x3C
#define REG_OFFSET_RESERVED17   0x3E
#define REG_OFFSET_WF1CRC0      0x40
#define REG_OFFSET_WF1CRC1      0x42
#define REG_OFFSET_WF1BMO       0x44
#define REG_OFFSET_WF1BM1       0x46

#define REG_OFFSET_WF1BM2       0x48
#define REG_OFFSET_WF1BM3       0x4A
#define REG_OFFSET_RESERVED18   0x4C
#define REG_OFFSET_RESERVED19   0x4E
#define REG_OFFSET_WF2CRC0      0x50
#define REG_OFFSET_WF2CRC1      0x52
#define REG_OFFSET_WF2BMO       0x54
#define REG_OFFSET_WF2BM1       0x56
#define REG_OFFSET_WF2BM2       0x58
#define REG_OFFSET_WF2BM3       0x5A
#define REG_OFFSET_RESERVED20   0x5C
#define REG_OFFSET_RESERVED21   0x5E
#define REG_OFFSET_WF3CRC0      0x60
#define REG_OFFSET_WF3CRC1      0x62
#define REG_OFFSET_WF3BMO       0x64
#define REG_OFFSET_WF3BM1       0x66

#define REG_OFFSET_WF3BM2       0x68
#define REG_OFFSET_WF3BM3       0x6A
#define REG_OFFSET_RESERVED22   0x6C
#define REG_OFFSET_RESERVED23   0x6E
#define REG_OFFSET_TXCR         0x70
#define REG_OFFSET_TXSR         0x72
#define REG_OFFSET_RXCR1        0x74
#define REG_OFFSET_RXCR2        0x76
#define REG_OFFSET_TXMIR        0x78
#define REG_OFFSET_RESERVED24   0x7A
#define REG_OFFSET_RXFHSR       0x7C
#define REG_OFFSET_RXFHBCR      0x7E
#define REG_OFFSET_TXQCR        0x80
#define REG_OFFSET_RXQCR        0x82
#define REG_OFFSET_TXFDPR       0x84
#define REG_OFFSET_RXFDPR       0x86
#define REG_OFFSET_RESERVED25   0x88
#define REG_OFFSET_RESERVED26   0x8A

#define REG_OFFSET_RXDTTR       0x8C
#define REG_OFFSET_RXDBCTR      0x8E
#define REG_OFFSET_IER          0x90
#define REG_OFFSET_ISR          0x92
#define REG_OFFSET_RESERVED27   0x94
#define REG_OFFSET_RESERVED28   0x96
#define REG_OFFSET_RESERVED29   0x98
#define REG_OFFSET_RESERVED30   0x9A
#define REG_OFFSET_RXFCTSR      0x9C
#define REG_OFFSET_TXNTFSR      0x9E
#define REG_OFFSET_MAHTR0       0xA0
#define REG_OFFSET_MAHTR1       0xA2
#define REG_OFFSET_MAHTR2       0xA4
#define REG_OFFSET_MAHTR3       0xA6
#define REG_OFFSET_RESERVED31   0xA8
#define REG_OFFSET_RESERVED32   0xAA
#define REG_OFFSET_RESERVED33   0xAC
#define REG_OFFSET_RESERVED34   0xAE

#define REG_OFFSET_FCLWR        0xB0
#define REG_OFFSET_FCHWR        0xB2
#define REG_OFFSET_FCOWR        0xB4
#define REG_OFFSET_RESERVED35   0xB6
#define REG_OFFSET_RESERVED36   0xB8
#define REG_OFFSET_RESERVED37   0xBA
#define REG_OFFSET_RESERVED38   0xBC
#define REG_OFFSET_RESERVED39   0xBE
#define REG_OFFSET_CIDER        0xC0
#define REG_OFFSET_RESERVED40   0xC2
#define REG_OFFSET_RESERVED41   0xC4
#define REG_OFFSET_CGCR         0xC6
#define REG_OFFSET_IACR         0xC8
#define REG_OFFSET_RESERVED42   0xCA
#define REG_OFFSET_RESERVED43   0xCC
#define REG_OFFSET_RESERVED44   0xCE
#define REG_OFFSET_IADLR        0xD0
#define REG_OFFSET_IADHR        0xD2
#define REG_OFFSET_PMECR        0xD4
#define REG_OFFSET_GSWUTR       0xD6

#define REG_OFFSET_PHYRR        0xD8
#define REG_OFFSET_RESERVED45   0xDA
#define REG_OFFSET_RESERVED46   0xDC
#define REG_OFFSET_RESERVED47   0xDE
#define REG_OFFSET_RESERVED48   0xE0
#define REG_OFFSET_RESERVED49   0xE2
#define REG_OFFSET_P1MBCR       0xE4
#define REG_OFFSET_P1MBSR       0xE6
#define REG_OFFSET_PHY1ILR      0xE8
#define REG_OFFSET_PHY1IHR      0xEA
#define REG_OFFSET_P1ANAR       0xEC
#define REG_OFFSET_P1ANLPR      0xEE
#define REG_OFFSET_RESERVED50   0xF0
#define REG_OFFSET_RESERVED51   0xF2
#define REG_OFFSET_P1SCLMD      0xF4
#define REG_OFFSET_P1CR         0xF6
#define REG_OFFSET_P1SR         0xF8
#define REG_OFFSET_RESERVED52   0xFA
#define REG_OFFSET_RESERVED53   0xFC
#define REG_OFFSET_RESERVED54   0xFE


// Define PINS
#define RESET_PIN 4



void KSZ_init(const struct device *dev, const struct spi_config *spi_cfg);
void KSZ_init_QMU(const struct device *dev, const struct spi_config *spi_cfg);

void KSZ_init_transmitt(const struct device *dev, const struct spi_config *spi_cfg);
void KSZ_read(const struct device *dev, const struct spi_config *spi_cfg, uint16_t byteEnabled, uint16_t regAdress, const struct spi_buf_set * rx_bufs);
void KSZ_write(const struct device *dev, const struct spi_config *spi_cfg, uint16_t byteEnabled, uint16_t regAdress, uint32_t data);
void KSZ_read_write(const struct device *dev, const struct spi_config *spi_cfg, const struct spi_buf_set * tx_bufs, const struct spi_buf_set * rx_bufs);

void KSZ_transmitt(const struct device *dev);
void KSZ_receive(const struct device *dev);


#endif