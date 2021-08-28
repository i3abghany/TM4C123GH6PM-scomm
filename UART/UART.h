#ifndef _UART_H_
#define _UART_H_

typedef unsigned long uint32_t;
#define PTR(x) ((*(volatile uint32_t *)(x)))

#define UART0_BASE 0x4000C000

#define UART0DR   PTR(UART0_BASE + 0x000)
#define UART0RSR  PTR(UART0_BASE + 0x004)
#define UART0RECR PTR(UART0_BASE + 0x004)
#define UART0FR   PTR(UART0_BASE + 0x018)
#define UART0IBRD PTR(UART0_BASE + 0x024)
#define UART0FBRD PTR(UART0_BASE + 0x028)
#define UART0LCRH PTR(UART0_BASE + 0x02C)
#define UART0CTL  PTR(UART0_BASE + 0x030)
#define UART0CC   PTR(UART0_BASE + 0xFC8)
#define UART0IFLS PTR(UART0_BASE + 0x034)
#define UART0IM   PTR(UART0_BASE + 0x038)
#define UART0ICR  PTR(UART0_BASE + 0x044)


#define SYS_CTRL_BASE 0x400FE000
#define RCGCUART  PTR(SYS_CTRL_BASE + 0x618)
#define RCGCGPIO  PTR(SYS_CTRL_BASE + 0x608)

#define PORTA_BASE 0x40004000
#define GPIOAAFSEL PTR(PORTA_BASE + 0x420)
#define GPIOACTL   PTR(PORTA_BASE + 0x52C)
#define GPIOADEN   PTR(PORTA_BASE + 0x51C)

#define NVIC_BASE 0xE000E000
#define NVIC_ISEN0 PTR(NVIC_BASE + 0x100)

void UART0_init();
void UART0_send_byte(unsigned char);
unsigned char UART0_receive_byte();
void UART0_enable();

#endif
