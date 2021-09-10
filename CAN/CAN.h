#ifndef _CAN_H
#define _CAN_H

#include "stdbool.h"
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
    uint8_t prop_time;
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
    uint8_t obj_id;

    /* 11- or 29-bit message identifier (sent in the frame). */
    uint32_t msg_id;

    /* Used by the message handler in acceptance filtering */
    uint32_t msg_id_mask;

    /* Message flags used to set various attributes for the message box. */
    uint32_t flags;

    /*
     * The length of the data (if any). The length is masked to leave at most 4
     * bits which is the max-length of the DLC field in the CANIF*MCTL register.
     *
     * Even if the value is > 8, 8-bytes will be used.
     */
    uint8_t data_len;

    /* A pointer to the actual data that will reside inside the message object
     * (if applicable.)
     *
     * Can be left as NULL if the message object shall not contain data.
     */
    uint8_t *data;

    /* The type of the message object. Either transmission or receiving, either
     * remote of data frame, and whether to use automatic transmission of remote
     * requests.
     */
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

/*             CANCTL fields and flags.              */

#define CANCTL_INIT_MASK ((uint32_t)(1 << 0))
#define CANCTL_IE_MASK   ((uint32_t)(1 << 1))
#define CANCTL_SIE_MASK  ((uint32_t)(1 << 2))
#define CANCTL_EIE_MASK  ((uint32_t)(1 << 3))
#define CANCTL_CCE_MASK  ((uint32_t)(1 << 6))

/*              CANBIT fields and flags.             */
#define CANBIT_SEG2_POS 12
#define CANBIT_SEG1_POS  8
#define CANBIT_SJW_POS   6
#define CANBIT_BRP_POS   0

#define CANBIT_SEG2_MASK (uint32_t)(0b00000111)
#define CANBIT_SEG1_MASK (uint32_t)(0b00001111)
#define CANBIT_SJW_MASK  (uint32_t)(0b00000011)
#define CANBIT_BRP_MASK  (uint32_t)(0b00111111)

/*              CANINT fields and flags.             */
#define CANINT_NO_INTR             ((uint32_t)0x00000000)
#define CANINT_STATUS_INTR         ((uint32_t)0x00008000)

/*           CANIF1CMSK fields and flags             */
#define CANIF1CMSK_WRNRD_MASK      ((uint32_t)0x00000080)
#define CANIF1CMSK_MASK_MASK       ((uint32_t)0x00000040)
#define CANIF1CMSK_ARB_MASK        ((uint32_t)0x00000020)
#define CANIF1CMSK_CONTROL_MASK    ((uint32_t)0x00000010)
#define CANIF1CMSK_CLRINTPND_MASK  ((uint32_t)0x00000008)
#define CANIF1CMSK_NEWDAT_MASK     ((uint32_t)0x00000004)
#define CANIF1CMSK_TXRQST_MASK     ((uint32_t)0x00000004)
#define CANIF1CMSK_DATAA_MASK      ((uint32_t)0x00000002)
#define CANIF1CMSK_DATAB_MASK      ((uint32_t)0x00000001)

/* CANIF1CRQ fields and flags */
#define CANIF1CRQ_MNUM_MASK        ((uint32_t)0x0000003F)
#define CANIF1CRQ_BUSY_MASK        ((uint32_t)0x00008000)

/*           CANIF1MSK1 fields and flags             */
#define CANIF1MSK1_MSK_MASK        0x000007FF

/*           CANIF1MSK2 fields and flags             */
#define CANIF1MSK2_MSK_MASK        ((uint32_t)0x00001FFF)
#define CANIF1MSK2_MDIR_MASK       ((uint32_t)0x00004000)
#define CANIF1MSK2_MXTD_MASK       ((uint32_t)0x00008000)

/*           CANIF1MCTL fields and flags             */
#define CANIF1MCTL_DLC_MASK        ((uint32_t)0x00000007)
#define CANIF1MCTL_EOB_MASK        ((uint32_t)0x00000080)
#define CANIF1MCTL_TXRQST_MASK     ((uint32_t)0x00000100)
#define CANIF1MCTL_RMTEN_MASK      ((uint32_t)0x00000200)
#define CANIF1MCTL_RXIE_MASK       ((uint32_t)0x00000400)
#define CANIF1MCTL_TXIE_MASK       ((uint32_t)0x00000800)
#define CANIF1MCTL_UMASK_MASK      ((uint32_t)0x00001000)
#define CANIF1MCTL_INTPND_MASK     ((uint32_t)0x00002000)
#define CANIF1MCTL_MSGLST_MASK     ((uint32_t)0x00004000)

/*           CANIF1ARB* fields and flags             */
#define CANIF1ARB1_ID_MASK         ((uint32_t)0x0000FFFF)

#define CANIF1ARB2_ID_MASK         ((uint32_t)0x00001FFF)
#define CANIF1ARB2_DIR_MASK        ((uint32_t)0x00002000)
#define CANIF1ARB2_XTD_MASK        ((uint32_t)0x00004000)
#define CANIF1ARB2_MSGVAL_MASK     ((uint32_t)0x00008000)

/*            CANSTS fields and flags                */

/* Last Error Code, a change in this field triggers an interrupt. It indicates
 * the last error the CAN bus has encountered
 *
 * Available valeus:
 *
 * 0x0    No Error
 *
 * 0x1    Stuff Error (More than 5 consecutive equal bits occurred in a received
 *        frame)
 *
 * 0x2    Format Error (A fixed format part of the received frame has the wrong
 *        format)
 *
 * 0x3    ACK Error (The message transmitted was not acknowledged by another
 *        node)
 *
 * 0x4    Bit 1 Error (The current particular node lost arbitration, i.e. the
 *        device wanted to send a High level (logical 1) but the monitored bus
 *        value was Low (logical 0).)
 *
 * 0x5    Bit 0 Error (A Bit 0 Error indicates that the device wanted to send a
 *        Low level (logical 0), but the monitored bus value was High (logical 1))
 *
 * 0x6    CRC Error (The CRC field of a received message was different from the
 *        calculated CRC)
 *
 * 0x7    No Event (When the LEC bit shows this value, no CAN bus event was
 *        detected since this value was written to the LEC field)
 */
#define CANSTS_LEC_MASK            ((uint32_t)0x00000007)

/*
 * Transmitted a Message Successfully. This bit must be manually cleared by the
 * software after handling the received message.
 */
#define CANSTS_TXOK_MASK           ((uint32_t)0x00000008)

/*
 * Received a Message Successfully. This bit must be manually cleared by the
 * software.
 */
#define CANSTS_RXOK_MASK           ((uint32_t)0x00000010)

/*
 * Error Passive (If asserted, The CAN module is in the Error Passive state,
 * that is, the receive or transmit error count is greater than 127)
 */
#define CANSTS_EPASS_MASK          ((uint32_t)0x00000020)

/*
 * Warning Status (When asserted at least one of the error counters has reached
 * the error warning limit of 96)
 */
#define CANSTS_EWARN_MASK          ((uint32_t)0x00000040)

/*
 * Bus-Off Status (When asserted, the CAN controller is in bus-off state.)
 */
#define CANSTS_BOFF_MASK           ((uint32_t)0x00000080)

/*                    CAN errors                     */
#define CAN_LEC_NO_ERRORS     ((uint32_t)0x0)
#define CAN_LEC_STUFF_ERROR   ((uint32_t)0x1)
#define CAN_LEC_FMT_ERROR     ((uint32_t)0x2)
#define CAN_LEC_ACK_ERROR     ((uint32_t)0x3)
#define CAN_LEC_BIT1_ERROR    ((uint32_t)0x4)
#define CAN_LEC_BIT0_ERROR    ((uint32_t)0x5)
#define CAN_LEC_CRC_ERROR     ((uint32_t)0x6)
#define CAN_LEC_NO_EVENT      ((uint32_t)0x7)

#define CONSTRUCT_CANBIT(seg2, seg1, sjw, brp)         \
    (((seg2 & CANBIT_SEG2_MASK) << CANBIT_SEG2_POS) |  \
    ((seg1 & CANBIT_SEG1_MASK) << CANBIT_SEG1_POS)  |  \
    ((sjw  &  CANBIT_SJW_MASK) <<  CANBIT_SJW_POS)  |  \
    ((brp  &  CANBIT_BRP_MASK) <<  CANBIT_BRP_POS))

/* The bit time may consist of 4 to 25 time quanta. */
#define N_TIME_QUANTA           (uint8_t)(5)

/*              Message Object Flags                */

/* Indicates that transmit interrupts enabled. */
#define MSG_OBJ_TX_INT_ENABLED  ((uint32_t)0x00000001)

/* Indicates that receive interrupts enabled. */
#define MSG_OBJ_RX_INT_ENABLED  ((uint32_t)0x00000002)

/* The message object uses the extended ID with 29 bits. */
#define MSG_OBJ_EXTENDED_ID     ((uint32_t)0x00000004)

/* Using the standard ID as an acceptance filter. */
#define MSG_OBJ_USE_ID_FILTER   ((uint32_t)0x00000008)

/*
 * Acceptance filter using the DIR field in the message object. This acceptance
 * filter also implies the usage of the ID filter.
 */
#define MSG_OBJ_USE_DIR_FILTER  ((uint32_t)(0x00000010 | MSG_OBJ_USE_ID_FILTER))

/*
 * Use the extened ID as an acceptance filter. This filter implies the usage of
 * the standard ID acceptance filter.
 */
#define MSG_OBJ_USE_EXT_FILTER  ((uint32_t)(0x00000020 | MSG_OBJ_USE_ID_FILTER))

/* This message object is part of a FIFO structure. */
#define MSG_OBJ_FIFO            ((uint32_t)0x00000040)

/* Indicates new data is available for the message object. */
#define MSG_OBJ_NEW_DATA        ((uint32_t)0x00000080)

/* Implies that the message object is a remote frame. */
#define MSG_OBJ_REMOTE_FRAME    ((uint32_t)0x00000100)

/* Implies that the message object is a remote frame. */
#define MSG_OBJ_DATA_LOSS       ((uint32_t)0x00000200)

/* This message object is part of a FIFO structure. */
#define MSG_OBJ_NO_FLAGS        ((uint32_t)0x00000000)

/*                 Interrupt flags                   */
#define CAN_INTR_GLOBAL    CANCTL_IE_MASK
#define CAN_INTR_STATUS    CANCTL_SIE_MASK
#define CAN_INTR_ERROR     CANCTL_EIE_MASK
#define CAN_INTR_ALL       (CAN_INTR_GLOBAL | CAN_INTR_STATUS | CAN_INTR_ERROR)


#define min(x, y) ((x) < (y)) ? (x) : (y)

void CAN_disable(enum CAN c);
void CAN_enable(enum CAN c);
bool CAN_init(CANConfig *cfg);
bool CAN_config_message(enum CAN c, CANMsgObject *msg);
bool CAN_get_message_object(enum CAN c, CANMsgObject *msg);
bool CAN_remove_message_object(enum CAN c, uint8_t obj_id);
bool CAN_interrupt_enable(enum CAN c, uint32_t interrupt_masks);

#endif
