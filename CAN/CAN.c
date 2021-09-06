#include "CAN.h"

static void CAN_GPIO_init(enum CAN c)
{
    /*  The peripheral clock must be enabled using the RCGC0 register */
    uint8_t RCGC0_offset = (c == CAN0) ? CAN0_RCGC0_OFFSET : CAN1_RCGC0_OFFSET;
    RCGC0 |= (1 << RCGC0_offset);

    /* the clock to the appropriate GPIO module must be enabled via the RCGC2 */
    uint8_t RCGC2_offset = (c == CAN0) ? CAN0_RCGC2_OFFSET : CAN1_RCGC2_OFFSET;
    RCGC2 |= (1 << RCGC2_offset);

    /*
     * The GPIO AFSEL bits for the appropriate pins.
     * Configure the PMCn fields in the GPIOPCTL register.
     */
    if (c == CAN0) {
        GPIOBAFSEL |= CAN0_AFSEL;
        GPIOBPCTL  |= CAN0_PCTL;
    } else {
        GPIOAAFSEL |= CAN1_AFSEL;
        GPIOAPCTL  |= CAN1_PCTL;
    }
}

static uint32_t get_canbit(uint32_t bitrate, int8_t cfg_prop_time)
{
    float bit_time = 1.0 / bitrate;

    /*
     * The bit time may consist of 4 to 25 time quanta.
     * bit_time = time_quantum * n
     */
    float time_quantum = bit_time / N_TIME_QUANTA;

    /*
     * time_quantum = (BRP) / fsys
     * BRP = time_quantum * fsys
     */
    uint8_t tBRP = ((uint32_t)(time_quantum * F_SYS_CLOCK) & (0xFF));

    /* tSync = 1 * time_quantum (fixed) */
    uint8_t sync_time = 1;

    /*
     * tProp depends on the physical characteristics of the CAN network.
     * For the sake of simplicity, we'll fix it at 2 time quanta with the
     * ability to override it at configuration time.
     */
    uint8_t prop_time = cfg_prop_time == 0 ? 2 : cfg_prop_time;

    if (cfg_prop_time > N_TIME_QUANTA) {
        return 0;
    }

    /* tPhase1 = tPhase2 =
     *     (N_TIME_QUANTA - ((tProp / time_quantum) + (tSync / time_quantum))
     */
    uint8_t phases_time = N_TIME_QUANTA - (sync_time + prop_time);
    uint8_t phase1_time = phases_time / 2;
    uint8_t phase2_time = phases_time / 2;
    phase2_time += (phases_time % 2 != 0);

    uint8_t tTSeg1 = (prop_time + phase1_time);
    uint8_t tTSeg2 = (phase2_time);

    /* The length of the synchronization jump width is set to the least of 4, Phase1 or Phase2 */
    uint8_t tSJW   = min(min(phase1_time, phase2_time), 4);

    /*
     * The four components TSEG2, TSEG1, SJW, and BRP have to be programmed to a
     * numerical value that is one less than its functional value; so instead of
     * values in the range of [1..n], values in the range of [0..n-1] are
     * programmed. That way, for example, SJW (functional range of [1..4]) is
     * represented by only two bits in the SJW bit field.
     */
    tBRP--;
    tTSeg1--;
    tTSeg2--;
    tSJW--;

    // return (tTSeg2 << 12) | (tTSeg1 << 8) | (tSJW << 6) | (prescaler);
    return CONSTRUCT_CANBIT(tTSeg2, tTSeg1, tSJW, tBRP);
}

void CAN_disable(enum CAN c)
{
    uint32_t CAN_BASE_ADDR = (uint32_t)(c);
    PTR(CAN_BASE_ADDR, CANCTL_OFFSET) |= CANCTL_INIT_MASK;
}

void CAN_enable(enum CAN c)
{
    uint32_t CAN_BASE_ADDR = (uint32_t)(c);
    PTR(CAN_BASE_ADDR, CANCTL_OFFSET) &= ~CANCTL_INIT_MASK;
}

bool CAN_init(CANConfig *cfg)
{
    CAN_GPIO_init(cfg->can_num);

    uint32_t CAN_BASE_ADDR = (uint32_t)(cfg->can_num);

    /* Initialization is started by setting the INIT bit in the CANCTL register */
    CAN_disable(cfg->can_num);

    /* Set the CAN Bit Timing CANBIT register */
    PTR(CAN_BASE_ADDR, CANCTL_OFFSET) |= CANCTL_CCE_MASK;
    uint32_t canbit = get_canbit(cfg->bit_rate, cfg->prop_time);

    if (canbit == 0) {
        return false;
    }

    PTR(CAN_BASE_ADDR, CANBIT_OFFSET) |= canbit;

    /* To leave the initialization state, the INIT bit must be cleared. */
    CAN_enable(cfg->can_num);

    return true;
}

static inline void CAN_config_CANIF1CMSK(uint32_t CAN_BASE_ADDR, uint32_t options)
{
    /* 1. In the CAN IFn Command Mask (CANIFnCMASK) register: */

    /* Set the WRNRD bit to specify a write to the CANIFnCMASK register; */

    /*
     * Conditionally set since it can alter that state of other bits. It's left
     * for the CAN_config_CANIF1CMSK user to ensure consistent F1CMSK register
     * state.
     */
    if (options & CANIF1CMSK_WRNRD_MASK) {
        PTR(CAN_BASE_ADDR, CANIF1CMSK_OFFSET) |= CANIF1CMSK_WRNRD_MASK;
    }

    /*
     * specify whether to transfer the IDMASK, DIR, and MXTD of the message
     * object into the CAN IFn registers using the MASK bit
     */
    if (options & CANIF1CMSK_MASK_MASK) {
        PTR(CAN_BASE_ADDR, CANIF1CMSK_OFFSET) |= CANIF1CMSK_MASK_MASK;
    }

    /*
     * Specify whether to transfer the ID, DIR, XTD, and MSGVAL of the message
     * object into the interface registers using the ARB bit
     */
    if (options & CANIF1CMSK_ARB_MASK) {
        PTR(CAN_BASE_ADDR, CANIF1CMSK_OFFSET) |= CANIF1CMSK_ARB_MASK;
    }

    /*
     * Specify whether to transfer the control bits into the interface registers
     * using the CONTROL bit
     */
    if (options & CANIF1CMSK_CONTROL_MASK) {
        PTR(CAN_BASE_ADDR, CANIF1CMSK_OFFSET) |= CANIF1CMSK_CONTROL_MASK;
    }

    /*
     * Specify whether to clear the INTPND bit in the CANIFnMCTL register using
     * the CLRINTPND bit
     */
    if (options & CANIF1CMSK_CLRINTPND_MASK) {
        PTR(CAN_BASE_ADDR, CANIF1CMSK_OFFSET) |= CANIF1CMSK_CLRINTPND_MASK;
    }

    /*
     * Specify whether to clear the NEWDAT bit in the CANNWDAn register using
     * the NEWDAT bit
     */
    if (options & CANIF1CMSK_NEWDAT_MASK) {
        PTR(CAN_BASE_ADDR, CANIF1CMSK_OFFSET) |= CANIF1CMSK_NEWDAT_MASK;
    }

    /*
     * Specify which bits to transfer using the DATAA and DATAB bits
     */
    if (options & CANIF1CMSK_DATAA_MASK) {
        PTR(CAN_BASE_ADDR, CANIF1CMSK_OFFSET) |= CANIF1CMSK_DATAA_MASK;
    }

    if (options & CANIF1CMSK_DATAB_MASK) {
        PTR(CAN_BASE_ADDR, CANIF1CMSK_OFFSET) |= CANIF1CMSK_DATAB_MASK;
    }
}

static inline bool CAN_is_transmit_type(MsgObjectType type)
{
    return type == CAN_MSG_OBJ_TYPE_TX_REMOTE || type == CAN_MSG_OBJ_TYPE_TX;
}

typedef struct {
    uint32_t *CANIF1MSK1_value;
    uint32_t *CANIF1MSK2_value;
    uint32_t *CANIF1MCTL_value;
    uint32_t *CANIF1ARB1_value;
    uint32_t *CANIF1ARB2_value;
    uint32_t  *CANIF1CRQ_value;
} _CANRegsValues;

static inline bool CAN_config_message_type(uint32_t CAN_BASE_ADDR,
        MsgObjectType type, uint32_t flags, _CANRegsValues *vals)
{
   switch (type) {
    case CAN_MSG_OBJ_TYPE_RX:

        /* Reset the DIR bit to indicate RX. */
        *(vals->CANIF1ARB2_value) &= ~(CANIF1ARB2_DIR_MASK);

        /* Reset the TXRQST bit to indicate RX. */
        *(vals->CANIF1MCTL_value) &= ~(CANIF1MCTL_TXRQST_MASK);

        /*
         * Optionally set the RXIE bit to enable the INTPND bit to be set after
         * a successful receive.
         */
        if (flags & (CAN_MSG_OBJ_FLAG_RX_INT_ENABLED)) {
            *(vals->CANIF1MCTL_value) |= CANIF1MCTL_RXIE_MASK;
        }

        /* Clear the RMTEN bit to leave the TXRQST bit unchanged */
        *(vals->CANIF1MCTL_value) &= ~CANIF1MCTL_RMTEN_MASK;

        break;
    case CAN_MSG_OBJ_TYPE_RX_REMOTE:
        

        break;
    case CAN_MSG_OBJ_TYPE_TX:

        /* Set the DIR bit to indicate TX. */
        *(vals->CANIF1ARB2_value) |= CANIF1ARB2_DIR_MASK;

        /*
         * Optionally set the TXIE bit to enable the INTPND bit to be set after
         * a successful transmission
         */
        if (flags & (CAN_MSG_OBJ_FLAG_TX_INT_ENABLED)) {
            *(vals->CANIF1MCTL_value) |= CANIF1MCTL_TXIE_MASK;
        }

        break;
    case CAN_MSG_OBJ_TYPE_TX_REMOTE:
        /* TODO: Handle remote frames... */

        break;

    default:
        return false;
    }

    /* TODO: Handle FIFO. */
    /* For now, only single-frame messages are supported... */
    *(vals->CANIF1MCTL_value) |= CANIF1MCTL_EOB_MASK;

    return true;
}

static void CAN_memcpy(void *dst, void *src, uint8_t n)
{
    for (uint8_t i = 0; i < n; i++) {
        ((uint8_t *)src)[i] = ((uint8_t *)dst)[i];
    }
}

static void CAN_write_data_to_reg(uint32_t CAN_BASE_ADDR, uint8_t *data, uint8_t len)
{
    /*
     * Since The data registers are laid-out in memory as a contiguous map,
     * starting from CAN*IF1DA1, then we can just memcpy into this starting
     * address.
     */
    CAN_memcpy((void *)(CAN_BASE_ADDR + CANIF1DA1_OFFSET), data, len);
}

bool CAN_config_message(enum CAN c, CANMsgObject *msg)
{
    uint32_t CANIF1MSK1_value = 0;
    uint32_t CANIF1MSK2_value = 0;
    uint32_t CANIF1MCTL_value = 0;
    uint32_t CANIF1ARB1_value = 0;
    uint32_t CANIF1ARB2_value = 0;
    uint32_t CANIF1CRQ_value  = 0;

    uint32_t CAN_BASE_ADDR = (uint32_t)(c);

    /*
     * Since this is a message object configuration, we're writting to the
     * messsage object. This means that the WRNRD bit must be set.
     *
     * WRNRD: Transfer the data in the CANIFn registers to the CAN message
     * object specified by the MNUM field in the CAN Command Request (CANIFnCRQ)
     *
     * Potentially using all the data bytes from the message object, so
     * setting the DATAA and DATAB bits.
     *
     * Transferring control bits from CANIFnMCTL to interface registers.
     */
    CAN_config_CANIF1CMSK(c, CANIF1CMSK_WRNRD_MASK |
                             CANIF1CMSK_DATAA_MASK |
                             CANIF1CMSK_DATAB_MASK |
                             CANIF1CMSK_CONTROL_MASK);

    bool use_extended_bit_id = ((msg->msg_id > CANIF1MSK1_ID_MASK) &&
                          (msg->flags & CAN_MSG_OBJ_FLAG_29_BIT_ID));

    /*
     * In the CANIFnMSK1 register, use the MSK[15:0] bits to specify which of
     * the bits in the 29-bit or 11-bit message identifier are used for
     * acceptance filtering
     *
     * In the CANIFnMSK2 register, use the MSK[12:0] bits to specify which of
     * the bits in the 29-bit or 11-bit message identifier are used for
     * acceptance filtering
     *
     */
    if (msg->flags & (CAN_MSG_OBJ_FLAG_ID_FILTER)) {
        if (use_extended_bit_id) {
            CANIF1MSK1_value |= (msg->msg_id & CANIF1MSK1_ID_MASK);
            CANIF1MSK2_value |= (msg->msg_id >> 16);
        } else {
            CANIF1MSK2_value |= ((msg->msg_id & CANIF1MSK2_MSK_MASK) << 2);
        }
    }

    /*
     * In order for these bits to be used for acceptance filtering, they must be
     * enabled by setting the UMASK bit in the CANIFnMCTL register
     */
    if (msg->flags & (CAN_MSG_OBJ_FLAG_ID_FILTER |
                CAN_MSG_OBJ_FLAG_29_BIT_FILTER)) {
        CANIF1MCTL_value |= CANIF1MCTL_UMASK_MASK;

        /* Send the filtering mask to the message object. */
        CAN_config_CANIF1CMSK(c, CANIF1CMSK_MASK_MASK);
    }

    /*
     * Use the MXTD and MDIR bits to specify whether to use XTD and DIR for
     * acceptance filtering.
     */
    if ((msg->flags & CAN_MSG_OBJ_FLAG_DIR_FILTER)
            == CAN_MSG_OBJ_FLAG_DIR_FILTER) {
        CANIF1MSK2_value |= CANIF1MSK2_MDIR_MASK;
    }

    if ((msg->flags & CAN_MSG_OBJ_FLAG_29_BIT_FILTER)
            == CAN_MSG_OBJ_FLAG_29_BIT_FILTER) {
        CANIF1MSK2_value |= CANIF1MSK2_MXTD_MASK;
    }

    /* For a 29-bit identifier: */
    if (use_extended_bit_id) {
        /*
         * Configure ID[15:0] in the CANIFnARB1 register for bits [15:0] of the
         * message identifier and ID[12:0] in the CANIFnARB2 register for bits
         * [28:16] of the message identifier.
         */
        CANIF1ARB1_value |= (msg->msg_id & CANIF1ARB1_ID_MASK);
        CANIF1ARB2_value |= ((msg->msg_id >> 16) & (CANIF1ARB2_ID_MASK));


        /* Set the XTD bit to indicate an extended identifier; */
        CANIF1ARB2_value |= CANIF1ARB2_XTD_MASK;

        /* Set the MSGVAL bit to indicate that the message object is valid. */
        CANIF1ARB2_value |= CANIF1ARB2_MSGVAL_MASK;
    } else {
        /*
         * For an 11-bit identifier, disregard the CANIFnARB1 register and
         * configure ID[12:2] in the CANIFnARB2 register for bits [10:0] of the
         * message identifier.
         */
        CANIF1ARB2_value |= ((msg->msg_id & (CANIF1ARB2_ID_MASK) << 2));

        /*
         * Set the MSGVAL bit to indicate that the message object is valid.
         */
        CANIF1ARB2_value |= CANIF1ARB2_MSGVAL_MASK;
    }

    /* Configure the DLC[3:0] field to specify the size of the data frame. */
    CANIF1MCTL_value |= (msg->data_len & CANIF1MCTL_DLC_MASK);

    _CANRegsValues current_vals = {
        &CANIF1MSK1_value,
        &CANIF1MSK2_value,
        &CANIF1MCTL_value,
        &CANIF1ARB1_value,
        &CANIF1ARB2_value,
        &CANIF1CRQ_value
    };

    if (!CAN_config_message_type(c, msg->msg_type, msg->flags, &current_vals)) {
        return false;
    }

    /*
     * Load the data to be transmitted into the CAN IFn Data (CANIFnDA1,
     * CANIFnDA2, CANIFnDB1, CANIFnDB2) registers. Byte 0 of the CAN data frame
     * is stored in DATA[7:0] in the CANIFnDA1 register.
     */

    if (CAN_is_transmit_type(msg->msg_type)) {
        CAN_write_data_to_reg(c, msg->data, msg->data_len);
    }

    /*
     * Program the number of the message object to be transmitted in the MNUM
     * field in the CAN IFn Command Request (CANIFnCRQ)
     */
    CANIF1CRQ_value |= (msg->msg_id & CANIF1CRQ_MNUM_MASK);

    /*
     * When everything is properly configured, set the TXRQST bit in the
     * CANIFnMCTL register. Once this bit is set, the message object is
     * available to be transmitted, depending on priority and bus availability.
     */
    if (CAN_is_transmit_type(msg->msg_type)) {
        CANIF1MCTL_value |= CANIF1MCTL_TXRQST_MASK;
    }

    PTR(CAN_BASE_ADDR, CANIF1MSK1_OFFSET) |= CANIF1MSK1_value;
    PTR(CAN_BASE_ADDR, CANIF1MSK2_OFFSET) |= CANIF1MSK2_value;
    PTR(CAN_BASE_ADDR, CANIF1MCTL_OFFSET) |= CANIF1MCTL_value;
    PTR(CAN_BASE_ADDR, CANIF1ARB1_OFFSET) |= CANIF1ARB1_value;
    PTR(CAN_BASE_ADDR, CANIF1ARB2_OFFSET) |= CANIF1ARB2_value;
    PTR(CAN_BASE_ADDR,  CANIF1CRQ_OFFSET) |=  CANIF1CRQ_value;
    return true;
}

