#ifndef _CAN_H
#define _CAN_H

#include <stdbool.h>
#include "CANMap.h"

#ifndef F_SYS_CLOCK
    #define F_SYS_CLOCK 25000000
#endif

enum CAN {
    CAN0,
    CAN1
};

struct CANConfig {
    enum CAN can_num;
    uint32_t bit_rate;
    int8_t prop_time;
};

enum MsgObjectType {
    CAN_MSG_OBJECT_TYPE_RX,
    CAN_MSG_OBJECT_TYPE_RX_REMOTE,
    CAN_MSG_OBJECT_TYPE_TX,
    CAN_MSG_OBJECT_TYPE_TX_REMOTE
};

struct CANMsgObject {
    /* Controller message object ID. */
    uint32_t obj_id;

    /* 11- or 29-bit message identifier (sent in the frame). */
    uint32_t msg_id;

    /* Used by the message handler in acceptance filtering */
    uint32_t msg_id_mask;
    uint32_t flags;
    uint8_t data_len;
    uint8_t *data;
    enum MsgObjectType msg_type;
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

#define CANBIT_SEG2_POS 12
#define CANBIT_SEG1_POS 8
#define CANBIT_SJW_POS 6
#define CANBIT_BRP_POS 0

#define CANBIT_SEG2_MASK (0b00000111)
#define CANBIT_SEG1_MASK (0b00001111)
#define CANBIT_SJW_MASK  (0b00000011)
#define CANBIT_BRP_MASK  (0b00111111)

#define CANIF1CMSK_WRNRD_MASK     (0x00000080)
#define CANIF1CMSK_MASK_MASK      (0x00000040)
#define CANIF1CMSK_ARB_MASK       (0x00000020)
#define CANIF1CMSK_CONTROL_MASK   (0x00000010)
#define CANIF1CMSK_CLRINTPND_MASK (0x00000008)
#define CANIF1CMSK_NEWDAT_MASK    (0x00000004)
#define CANIF1CMSK_DATAA_MASK     (0x00000002)
#define CANIF1CMSK_DATAB_MASK     (0x00000001)

#define CONSTRUCT_CANBIT(seg2, seg1, sjw, brp)         \
    (((seg2 & CANBIT_SEG2_MASK) << CANBIT_SEG2_POS) |  \
    ((seg1 & CANBIT_SEG1_MASK) << CANBIT_SEG1_POS)  |  \
    ((sjw  &  CANBIT_SJW_MASK) <<  CANBIT_SJW_POS)  |  \
    ((brp  &  CANBIT_BRP_MASK) <<  CANBIT_BRP_POS))

/* The bit time may consist of 4 to 25 time quanta. */
#define N_TIME_QUANTA 5

/*              Message Object Flags                */
#define CAN_11_BIT_ID_MASK 0x000007FF

#define min(x, y) ((x) < (y)) ? (x) : (y)

void CAN_disable(enum CAN c);
void CAN_enable(enum CAN c);
bool CAN_init(struct CANConfig *cfg);
bool CAN_config_transmit_message(enum CAN, struct CANMsgObject *msg);

#endif
