#ifndef _CAN_H
#define _CAN_H

#include <stdbool.h>
#include "CANMap.h"

#ifndef F_SYS_CLOCK
    #define F_SYS_CLOCK 25000000
#endif

enum CAN {
    CAN0 = CAN0_BASE_ADDR,
    CAN1 = CAN1_BASE_ADDR
};

typedef struct {
    enum CAN can_num;
    uint32_t bit_rate;
    int8_t prop_time;
} CANConfig;

typedef enum {
    MSG_OBJ_TYPE_RX,
    MSG_OBJ_TYPE_RX_REMOTE,
    MSG_OBJ_TYPE_TX,
    MSG_OBJ_TYPE_TX_REMOTE,
    MSG_OBJ_TYPE_RXTX_REMOTE
} MsgObjectType;

typedef struct {
    /* Controller message object ID. */
    uint32_t obj_id;

    /* 11- or 29-bit message identifier (sent in the frame). */
    uint32_t msg_id;

    /* Used by the message handler in acceptance filtering */
    uint32_t msg_id_mask;
    uint32_t flags;
    uint8_t data_len;
    uint8_t *data;
    MsgObjectType msg_type;
} CANMsgObject;

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

/*              CANBIT fields and flags              */
#define CANBIT_SEG2_POS 12
#define CANBIT_SEG1_POS 8
#define CANBIT_SJW_POS 6
#define CANBIT_BRP_POS 0

#define CANBIT_SEG2_MASK (0b00000111)
#define CANBIT_SEG1_MASK (0b00001111)
#define CANBIT_SJW_MASK  (0b00000011)
#define CANBIT_BRP_MASK  (0b00111111)

/*           CANIF1CMSK fields and flags             */
#define CANIF1CMSK_WRNRD_MASK      0x00000080
#define CANIF1CMSK_MASK_MASK       0x00000040
#define CANIF1CMSK_ARB_MASK        0x00000020
#define CANIF1CMSK_CONTROL_MASK    0x00000010
#define CANIF1CMSK_CLRINTPND_MASK  0x00000008
#define CANIF1CMSK_NEWDAT_MASK     0x00000004
#define CANIF1CMSK_TXRQST_MASK     0x00000004
#define CANIF1CMSK_DATAA_MASK      0x00000002
#define CANIF1CMSK_DATAB_MASK      0x00000001

/* CANIF1CRQ fields and flags */
#define CANIF1CRQ_MNUM_MASK        0x0000003F
#define CANIF1CRQ_BUSY_MASK        0x00008000

/*           CANIF1MSK1 fields and flags             */
#define CANIF1MSK1_ID_MASK         0x000007FF

/*           CANIF1MSK2 fields and flags             */
#define CANIF1MSK2_MSK_MASK       0x00004000
#define CANIF1MSK2_MDIR_MASK       0x00004000
#define CANIF1MSK2_MXTD_MASK       0x00008000

/*           CANIF1MCTL fields and flags             */
#define CANIF1MCTL_DLC_MASK        0x00000007
#define CANIF1MCTL_EOB_MASK        0x00000080
#define CANIF1MCTL_TXRQST_MASK     0x00000100
#define CANIF1MCTL_RMTEN_MASK     0x00000200
#define CANIF1MCTL_RXIE_MASK       0x00000400
#define CANIF1MCTL_TXIE_MASK       0x00000800
#define CANIF1MCTL_UMASK_MASK      0x00001000
#define CANIF1MCTL_INTPND_MASK     0x00002000

/*           CANIF1ARB* fields and flags             */
#define CANIF1ARB1_ID_MASK         0x0000FFFF

#define CANIF1ARB2_ID_MASK         0x00001FFF
#define CANIF1ARB2_DIR_MASK        0x00002000
#define CANIF1ARB2_XTD_MASK        0x00004000
#define CANIF1ARB2_MSGVAL_MASK     0x00008000

#define CONSTRUCT_CANBIT(seg2, seg1, sjw, brp)         \
    (((seg2 & CANBIT_SEG2_MASK) << CANBIT_SEG2_POS) |  \
    ((seg1 & CANBIT_SEG1_MASK) << CANBIT_SEG1_POS)  |  \
    ((sjw  &  CANBIT_SJW_MASK) <<  CANBIT_SJW_POS)  |  \
    ((brp  &  CANBIT_BRP_MASK) <<  CANBIT_BRP_POS))

/* The bit time may consist of 4 to 25 time quanta. */
#define N_TIME_QUANTA 5

/*              Message Object Flags                */

/* Indicates that transmit interrupts enabled. */
#define MSG_OBJ_TX_INT_ENABLED  0x00000001

/* Indicates that receive interrupts enabled. */
#define MSG_OBJ_RX_INT_ENABLED  0x00000002

/* The message object uses the extended ID with 29 bits. */
#define MSG_OBJ_EXTENDED_ID     0x00000004

/* Using the standard ID as an acceptance filter. */
#define MSG_OBJ_USE_ID_FILTER   0x00000008

/*
 * Acceptance filter using the DIR field in the message object. This acceptance
 * filter also implies the usage of the ID filter.
 */
#define MSG_OBJ_USE_DIR_FILTER  (0x00000010 | MSG_OBJ_USE_ID_FILTER)

/* 
 * Use the extened ID as an acceptance filter. This filter implies the usage of
 * the standard ID acceptance filter.
 */
#define MSG_OBJ_USE_EXT_FILTER  (0x00000020 | MSG_OBJ_USE_ID_FILTER)

/* This message object is part of a FIFO structure. */
#define MSG_OBJ_FIFO            0x00000040

/* Indicates new data is available for the message object. */
#define MSG_OBJ_NEW_DATA        0x00000080

/* This message object is part of a FIFO structure. */
#define MSG_OBJ_NO_FLAGS        0x00000000

/* Implies that the message object is a remote frame. */
#define MSG_OBJ_REMOTE_FRAME    0x00000040

#define min(x, y) ((x) < (y)) ? (x) : (y)

void CAN_disable(enum CAN c);
void CAN_enable(enum CAN c);
bool CAN_init(CANConfig *cfg);
bool CAN_config_message(enum CAN, CANMsgObject *msg);

#endif
