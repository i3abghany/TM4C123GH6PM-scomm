#ifndef _SPI_H
#define _SPI_H

#define SSI0_BASE 0x40008000

typedef unsigned int  uint16_t;
typedef unsigned long uint32_t;
typedef unsigned char uint8_t;

#define PTR(x) (*((uint32_t *)(x)))

#define SYS_CTRL_BASE 0x400FE000

#define RCGCSSI  PTR(SYS_CTRL_BASE + 0x61C)
#define RCGCGPIO PTR(SYS_CTRL_BASE + 0x608)

#define PORTA_BASE 0x40004000
#define GPIOAAFSEL PTR(PORTA_BASE + 0x420)
#define GPIOAPUR   PTR(PORTA_BASE + 0x510)
#define GPIOADEN   PTR(PORTA_BASE + 0x51C)
#define GPIOACTL   PTR(PORTA_BASE + 0x52C)

#define SSI0CR0 PTR(SSI0_BASE + 0x000)
#define SSI0CR1 PTR(SSI0_BASE + 0x004)
#define SSI0DR  PTR(SSI0_BASE + 0x008)
#define SSI0SR  PTR(SSI0_BASE + 0x00C)
#define SSI0CC  PTR(SSI0_BASE + 0xFC8)

typedef enum {
    SSI0_MASTER,
    SSI0_SLAVE_OPE,
    SSI0_SLAVE_OPD,
} SSI0_MODE;

typedef enum {
    SYS_CLOCK,
    PIOSC
} CLOCK_SOURCE;

typedef enum {
    FREESCALE_SPI = 0x00,
    TI_SSF = 0x01,
    MICROWIRE = 0x02,
} FRAME_FORMAT;

typedef struct {
    SSI0_MODE mode;
    CLOCK_SOURCE clk_src;
    uint8_t clk_divisor;
    uint8_t data_size;
    FRAME_FORMAT fmt;
    uint8_t clk_polarity;
    uint8_t serial_clk_phase;
    uint8_t serial_clk_rate;
} SSI0_config;


void SSI0_init(SSI0_config *cfg);
void SSI0_transmit(uint8_t dss, uint16_t data);
uint16_t SSI0_recv(uint8_t dss);

#endif
