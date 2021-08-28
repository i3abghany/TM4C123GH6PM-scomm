#ifndef _I2C_H
#define _I2C_H

#define I2C0_BASE 0x40020000

typedef unsigned int  uint16_t;
typedef unsigned long uint32_t;
typedef unsigned char uint8_t;

#define PTR(x) (*((uint32_t *)(x)))

#define SYS_CTRL_BASE 0x400FE000

#define RCGCI2C  PTR(SYS_CTRL_BASE + 0x620)
#define RCGCGPIO PTR(SYS_CTRL_BASE + 0x608)

#define PORTB_BASE 0x40005000
#define GPIOBAFSEL  PTR(PORTB_BASE + 0x420)
#define GPIOBPUR    PTR(PORTB_BASE + 0x510)
#define GPIOBODR    PTR(PORTB_BASE + 0x50C)
#define GPIOBDEN    PTR(PORTB_BASE + 0x51C)
#define GPIOBCTL    PTR(PORTB_BASE + 0x52C)

#define I2C0MCA   PTR(I2C0_BASE + 0x000)
#define I2C0MCS   PTR(I2C0_BASE + 0x004)
#define I2C0MDR   PTR(I2C0_BASE + 0x008)
#define I2C0MTPR  PTR(I2C0_BASE + 0x00C)
#define SSI0CC    PTR(I2C0_BASE + 0xFC8)

#endif
