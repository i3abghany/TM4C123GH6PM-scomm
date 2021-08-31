#ifndef _CAN_H
#define _CAN_H

#include "CANMap.h"
#ifndef F_SYS_CLOCK
    #define F_SYS_CLOCK 25000000
#endif

enum CAN {
    CAN0,
    CAN1
};

enum CAN_MODE {
    CAN_MASTER,
    CAN_SLAVE
};

struct CANConfig {
    enum CAN can_num;
    enum CAN_MODE mode;
    uint32_t bit_rate;
};

#define CAN0_RCGC0_OFFSET 24
#define CAN1_RCGC0_OFFSET 25

#define CAN0_RCGC2_OFFSET 1
#define CAN1_RCGC2_OFFSET 0

#define CAN0_AFSEL (0x11 << 4)
#define CAN1_AFSEL (0x11 << 0)

#define CAN0_PCTL ((8 << 16) | (8 << 20))
#define CAN1_PCTL ((8 <<  0) | (8 <<  4))

#define CANCTL_INIT_MASK (1 << 0)
#define CANCTL_CCE_MASK  (1 << 6)

/* The bit time may consist of 4 to 25 time quanta.*/
#define N_TIME_QUANTA 5

#define min(x, y) ((x) < (y)) ? (x) : (y)

void CAN_init(struct CANConfig *cfg);
#endif
