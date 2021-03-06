#ifndef _CAN_MAP_H
#define _CAN_MAP_H

#include "stdint.h"

#define PTR(BASE,OFFSET) (*((volatile uint32_t *)(((uint32_t)(BASE+OFFSET)))))

#define CAN0_BASE_ADDR 0x40040000
#define CAN1_BASE_ADDR 0x40041000

#define CANCTL_OFFSET     0x000
#define CANSTS_OFFSET     0x004
#define CANERR_OFFSET     0x008
#define CANBIT_OFFSET     0x00C
#define CANINT_OFFSET     0x010
#define CANTST_OFFSET     0x014
#define CANIF1CRQ_OFFSET  0x020
#define CANIF1CMSK_OFFSET 0x024
#define CANIF1MSK1_OFFSET 0x028
#define CANIF1MSK2_OFFSET 0x02C
#define CANIF1ARB1_OFFSET 0x030
#define CANIF1ARB2_OFFSET 0x034
#define CANIF1MCTL_OFFSET 0x038
#define CANIF1DA1_OFFSET  0x03C

/*                    SYSCTRL Map                       */

#define SYS_CTRL_ADDR_BASE 0x400FE000
#define RCGC0 PTR(SYS_CTRL_ADDR_BASE, 0x100)
#define RCGC2 PTR(SYS_CTRL_ADDR_BASE, 0x108)

/*                    GPIO Map                          */

/*
 * CAN0 pins are routed to Port B pins 4 and 5
 * CAN1 pins are routed to Port A Pins 0 and 1
 */

#define PORTA_ADDR_BASE 0x40004000
#define PORTB_ADDR_BASE 0x40005000

#define GPIOAFSEL_OFFSET 0x420
#define GPIOPCTL_OFFSET 0x52C

#define GPIOAAFSEL PTR(PORTA_ADDR_BASE, GPIOAFSEL_OFFSET)
#define GPIOBAFSEL PTR(PORTB_ADDR_BASE, GPIOAFSEL_OFFSET)

#define GPIOAPCTL PTR(PORTA_ADDR_BASE, GPIOPCTL_OFFSET)
#define GPIOBPCTL PTR(PORTB_ADDR_BASE, GPIOPCTL_OFFSET)

#endif
