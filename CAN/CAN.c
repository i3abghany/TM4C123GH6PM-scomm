#include "CAN.h"

/*
 * An internal helper to configure the GPIO pins related to a parameter CAN
 * controller.
 *
 * CAN0 can be configured to operate in three pairs of pins, this implementation
 * only uses one pair (RX/TX) in PORT B pins 4 and 5.
 */
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

/*
 * Calculate the CANBIT register value for a particular bit-rate and propagation
 * time.
 *
 * In the CAN specification, bit-time is divided into four parts.
 *
 *  1. Synchronization segment
 *  2. Propagation segment
 *  3. Phase buffer segment 1
 *  4. Phase buffer segment 2
 *
 *  Segment consist of a specific, programmable number of time quanta.
 *
 *  The time quantum is calculated using the following formula.
 *
 *      Time quantum = F_CLK / BRP.
 *
 *  Where F_CLK is the routed clock to the CAN controller.
 *  BRP is the bit-rate pre-scalar.
 *
 *  * Synchronization segment is fixed at one time quantum.
 *  * Propagation segment is parameterized by the physical system delay.
 *  * Phase buffers surround the sample point of the bit.
 *
 *  Propagation segment is provided by the user. It can be defaulted to 2 time
 *  quanta.
 */

static uint32_t get_canbit(uint32_t bitrate, uint8_t n_tq,
                           uint8_t cfg_prop_time)
{
    float bit_time = 1.0f / (float) bitrate;

    /*
     * If the number of time quanta is left as 0, we use the default value
     * N_TIME_QUANTA.
     */
    if (n_tq == 0) {
        n_tq = N_TIME_QUANTA;
    }

    /*
     * The bit time may consist of 4 to 25 time quanta.
     * bit_time = time_quantum * n
     */
    float time_quantum = bit_time / (float)n_tq;

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

    /*
     * With 1 time quantum for Sync, and at least 2 for Phase1 and Phase2, we
     * must have prop_time at least as small as `n_tq` - 3.
     *
     * We return - on (n_tq - 3) > n_tq also to accomodate for the under-flow of
     * (n_tq - 3).
     */
    if (n_tq - 3 > n_tq || cfg_prop_time > n_tq - 3) {
        return 0;
    }

    /* tPhase1 = tPhase2 =
     *     (n_tq - ((tProp / time_quantum) + (tSync / time_quantum))
     */
    uint8_t phases_time = (uint8_t)(n_tq - (sync_time + prop_time));
    uint8_t phase1_time = phases_time / 2;
    uint8_t phase2_time = phases_time / 2;
    phase2_time += (phases_time % 2 != 0) ? 1 : 0;

    uint8_t tTSeg1 = (prop_time + phase1_time);
    uint8_t tTSeg2 = (phase2_time);

    /*
     * The length of the synchronization jump width is set to the least of 4,
     * Phase1 or Phase2
     */
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

    return CONSTRUCT_CANBIT(tTSeg2, tTSeg1, tSJW, tBRP);
}

/*
 * When the INIT bit in the CANCTL register is set, the operation of the CAN is
 * halted.
 *
 * all message transfers to and from the CAN bus are stopped and the CANnTX
 * signal is held High. Entering the initialization state does not change the
 * configuration of the CAN controller, the message objects, or the error
 * counters. However, some configuration registers are only accessible while in
 * the initialization state.
 */
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

/*
 * An API for initializing a CAN controller. Which CAN controller to initialize
 * is a parameter in the `CANConfig` struct. The initialization also takes care
 * of initializing the GPIO pins associated with the selected CAN controller.
 *
 * Before initialization, the CAN controller is disabled by asserting the `INIT`
 * bit in the `CANCTL` register. Entering the initialization state enables
 * wrting to pre-run configuration registers.
 *
 * Return value: Returns true on successful initialization. Returns false if the
 * bit-rate parameters cannot be used to form a conforming bit-time.
 */
bool CAN_init(CANConfig *cfg)
{
    CAN_GPIO_init(cfg->can_num);

    uint32_t CAN_BASE_ADDR = (uint32_t)(cfg->can_num);

    /* Initialization is started by setting the INIT bit in the CANCTL register */
    CAN_disable(cfg->can_num);

    /* Set the CAN Bit Timing CANBIT register */
    PTR(CAN_BASE_ADDR, CANCTL_OFFSET) |= CANCTL_CCE_MASK;
    uint32_t canbit = get_canbit(cfg->bit_rate, cfg->n_time_quanta,
                                 cfg->prop_time);

    if (canbit == 0) {
        return false;
    }

    PTR(CAN_BASE_ADDR, CANBIT_OFFSET) |= canbit;

    /* To leave the initialization state, the INIT bit must be cleared. */
    CAN_enable(cfg->can_num);

    return true;
}

/*
 * An internal utility that asserts the options available in the CANIFCMSK
 * register. The CANIF1CMSK could be, more easily, OR'd with the `options`
 * parameter. But it's left verbose as-is to be like the data-sheet description
 * as much as possible.
 */
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
     * object into the CAN IFn registers (or the other way around) using the
     * MASK bit
     */
    if (options & CANIF1CMSK_MASK_MASK) {
        PTR(CAN_BASE_ADDR, CANIF1CMSK_OFFSET) |= CANIF1CMSK_MASK_MASK;
    }

    /*
     * Specify whether to transfer the ID, DIR, XTD, and MSGVAL of the message
     * object into the interface registers (or the other way around) using the
     * ARB bit
     */
    if (options & CANIF1CMSK_ARB_MASK) {
        PTR(CAN_BASE_ADDR, CANIF1CMSK_OFFSET) |= CANIF1CMSK_ARB_MASK;
    }

    /*
     * Specify whether to transfer the control bits into the interface registers
     * (or the other way around) using the CONTROL bit
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
     * Specify which bits to transfer using the DATAA and DATAB bits to the
     * interface refisters from the message object (or the other way around)
     */
    if (options & CANIF1CMSK_DATAA_MASK) {
        PTR(CAN_BASE_ADDR, CANIF1CMSK_OFFSET) |= CANIF1CMSK_DATAA_MASK;
    }

    if (options & CANIF1CMSK_DATAB_MASK) {
        PTR(CAN_BASE_ADDR, CANIF1CMSK_OFFSET) |= CANIF1CMSK_DATAB_MASK;
    }
}

/*
 * Must we assert the TXRQST bit or not? It must be asserted in case of a
 * transmit-remote message object, a data-transmit message object, or an
 * auto-transmission of a remote frame.
 */
static inline bool CAN_is_transmit_type(MsgObjectType type)
{
    return type == MSG_OBJ_TYPE_TX_REMOTE ||
           type == MSG_OBJ_TYPE_TX        ||
           type == MSG_OBJ_TYPE_RXTX_REMOTE;
}

typedef struct {
    uint32_t *CANIF1MSK1_value;
    uint32_t *CANIF1MSK2_value;
    uint32_t *CANIF1MCTL_value;
    uint32_t *CANIF1ARB1_value;
    uint32_t *CANIF1ARB2_value;
    uint32_t  *CANIF1CRQ_value;
} _CANRegsValues;

/* Internal helper for configuring the CAN controller interface registers
 * according to the message type, since different configurations must be
 * asserted for different message types.
 *
 * Return value: Returns true on success. If a mangled `MsgObjectType` value was
 * supplied, it returns false.
 */
static inline bool CAN_config_message_type(MsgObjectType type, uint32_t flags,
                                           _CANRegsValues *vals)
{
   switch (type) {
    case MSG_OBJ_TYPE_RX:

        /* Reset the DIR bit to indicate RX. */
        *(vals->CANIF1ARB2_value) &= ~(CANIF1ARB2_DIR_MASK);

        /* Reset the TXRQST bit to indicate RX. */
        *(vals->CANIF1MCTL_value) &= ~(CANIF1MCTL_TXRQST_MASK);

        /*
         * Optionally set the RXIE bit to enable the INTPND bit to be set after
         * a successful receive.
         */
        if (flags & (MSG_OBJ_RX_INT_ENABLED)) {
            *(vals->CANIF1MCTL_value) |= CANIF1MCTL_RXIE_MASK;
        }

        /* Clear the RMTEN bit to leave the TXRQST bit unchanged */
        *(vals->CANIF1MCTL_value) &= ~CANIF1MCTL_RMTEN_MASK;

        break;
    case MSG_OBJ_TYPE_RX_REMOTE:

        /* Set the DIR bit to indicate TX. */
        *(vals->CANIF1ARB2_value) |= CANIF1ARB2_DIR_MASK;

        /*
         * Do not change the TXRQST bit of the CANIFnMCTL register at reception
         * of the frame. This means that the software has to handle the frame
         * manually.  That's particularly useful when the receiving controller
         * does not have the data immediately upon the receiving of the remote
         * frame.
         */
        *(vals->CANIF1MCTL_value) &= ~CANIF1MCTL_RMTEN_MASK;

        /*
         * Reset the value of TXRQST for manual resolution by the software.
         * This is done automatically by setting the initial value of CANIF1MCTL
         * to zero.
         */

        /*
         * Enable acceptance filtering using MSK, MXTD, MDIR bits in
         * CANIF*MSK* registers.
         */
        *(vals->CANIF1MCTL_value) |= CANIF1MCTL_UMASK_MASK;

        break;
    case MSG_OBJ_TYPE_TX:

        /* Set the DIR bit to indicate TX. */
        *(vals->CANIF1ARB2_value) |= CANIF1ARB2_DIR_MASK;

        /*
         * Optionally set the TXIE bit to enable the INTPND bit to be set after
         * a successful transmission
         */
        if (flags & MSG_OBJ_TX_INT_ENABLED) {
            *(vals->CANIF1MCTL_value) |= CANIF1MCTL_TXIE_MASK;
        }

        break;
    case MSG_OBJ_TYPE_TX_REMOTE:
        /*
         * Set the TXRQST bit so that the transmission request is fired
         * immediately. We do this outside this function.
         */

        break;

    case MSG_OBJ_TYPE_RXTX_REMOTE:
    /* CAN remote frame receive remote, then transmit message object. */

       /*
        * At the reception of a matching remote frame, the TXRQST bit of this
        * message object is set. The rest of the message object remains
        * unchanged, and the controller automatically transfers the data in the
        * message object as soon as possible
        */

        /*
         * DIR = 1 (direction = transmit); programmed in the CANIFnARB2 register
         */
        *(vals->CANIF1ARB2_value) |= CANIF1ARB2_DIR_MASK;

        /*
         * RMTEN = 1 (set the TXRQST bit of the CANIFnMCTL register at reception
         * of the frame to enable transmission)
         */
        *(vals->CANIF1MCTL_value) |= CANIF1MCTL_RMTEN_MASK;


        /*
         * UMASK = 1 or 0
         *
         * We set it to 1 so that any further ID arbitration is enabled.
         */
        *(vals->CANIF1MCTL_value) |= CANIF1MCTL_UMASK_MASK;
        break;

    default:
        return false;
    }

   /*
    * Is the message a part of a FIFO structure and not the last message in a
    * sequence? If so, do not set the EOB bit in the MCTL register.
    */
   if ((flags & MSG_OBJ_FIFO) == 0) {
        *(vals->CANIF1MCTL_value) |= CANIF1MCTL_EOB_MASK;
   }

    return true;
}

/*
 * Internal helper that sequentially copies `n` bytes from `src` to `dst`.
 */
static inline void CAN_memcpy(void *dst, void *src, uint8_t n)
{
    for (uint8_t i = 0; i < n; i++) {
        ((uint8_t *)dst)[i] = ((uint8_t *)src)[i];
    }
}

/*
 * Internal helper that takes in data and writes it to the data space in the CAN
 * controller register address space. Since The data registers are laid-out in
 * memory as a contiguous map, starting from CAN*IF1DA1, then we can just memcpy
 * into this starting address.
 */
static void CAN_write_data_to_reg(uint32_t CAN_BASE_ADDR, uint8_t *data, uint8_t len)
{
    uint32_t data_reg = (uint64_t)(CAN_BASE_ADDR + CANIF1DA1_OFFSET);
    CAN_memcpy((void *)data_reg, data, len);
}

/*
 * Internal helper to read the `len`-bytes from the data address space of the CAN
 * controller.
 */
static void CAN_read_data_from_reg(uint32_t CAN_BASE_ADDR, uint8_t *buff, uint8_t len)
{
    uint32_t data_reg = (uint64_t)(CAN_BASE_ADDR + CANIF1DA1_OFFSET);
    CAN_memcpy(buff, (void *)data_reg, len);
}

/*
 * An API to configure a message object with the appropriate parameters provided
 * in `msg`.
 *
 * A message object can be configured with acceptance filtering factors so that
 * the CAN controller does not accept any messages but for the ones that pass
 * through the filtering process.
 *
 * Interrupts can also be configured to trigger on events such as message
 * receiving or on a complete transmission.
 *
 * Those options are configured by the API user in the `flags` field in the
 * CANMsgObject struct.
 *
 * Return value: Returns true on successful configuration. Returns false for the
 * following erroneous states:
 *     * The object ID is wrong (i.e. out of th [1, 32] range)
 *     * The message type is a mangled enum value
 */
bool CAN_config_message(enum CAN c, CANMsgObject *msg)
{
    uint32_t CANIF1MSK1_value = 0;
    uint32_t CANIF1MSK2_value = 0;
    uint32_t CANIF1MCTL_value = 0;
    uint32_t CANIF1ARB1_value = 0;
    uint32_t CANIF1ARB2_value = 0;
    uint32_t CANIF1CRQ_value  = 0;

    uint32_t CAN_BASE_ADDR = (uint32_t)(c);

    /* Only allow *correct* message object ids. */
    if (msg->obj_id > 32 || msg->obj_id == 0) {
        return false;
    }

    /*
     * This API is also used to potentially re-configure an already-set-up
     * message object, this means that the message object with `msg->msg_obj` ID
     * may have the `MSGVAL` bit set indicating a valid message. We
     * scrutinizingly reset it up-front.
     */

    CAN_remove_message_object(c, msg->obj_id);

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

    bool use_extended_bit_id = ((msg->msg_id > CANIF1MSK1_MSK_MASK) &&
                          (msg->flags & MSG_OBJ_EXTENDED_ID));

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
    if (msg->flags & (MSG_OBJ_USE_ID_FILTER)) {
        if (use_extended_bit_id) {
            CANIF1MSK1_value |= (msg->msg_id_mask & CANIF1MSK1_MSK_MASK);
            CANIF1MSK2_value |= (msg->msg_id_mask >> 16);
        } else {
            CANIF1MSK2_value |= ((msg->msg_id_mask & CANIF1MSK2_MSK_MASK) << 2);
        }
    }

    /*
     * In order for these bits to be used for acceptance filtering, they must be
     * enabled by setting the UMASK bit in the CANIFnMCTL register
     */
    if (msg->flags & (MSG_OBJ_USE_ID_FILTER | MSG_OBJ_USE_EXT_FILTER)) {
        CANIF1MCTL_value |= CANIF1MCTL_UMASK_MASK;

        /* Send the filtering mask to the message object. */
        CAN_config_CANIF1CMSK(c, CANIF1CMSK_MASK_MASK);
    }

    /*
     * Use the MXTD and MDIR bits to specify whether to use XTD and DIR for
     * acceptance filtering.
     *
     * These specifies extended ID and direction filtering, respectively.
     */
    if ((msg->flags & MSG_OBJ_USE_DIR_FILTER) == MSG_OBJ_USE_DIR_FILTER) {
        CANIF1MSK2_value |= CANIF1MSK2_MDIR_MASK;
    }

    if ((msg->flags & MSG_OBJ_USE_EXT_FILTER) == MSG_OBJ_USE_EXT_FILTER) {
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

    /* Pass the arbitration bits to the message objects */
    CAN_config_CANIF1CMSK(c, CANIF1CMSK_ARB_MASK);

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

    if (!CAN_config_message_type(msg->msg_type, msg->flags, &current_vals)) {
        return false;
    }

    /*
     * Load the data to be transmitted into the CAN IFn Data (CANIFnDA1,
     * CANIFnDA2, CANIFnDB1, CANIFnDB2) registers. Byte 0 of the CAN data frame
     * is stored in DATA[7:0] in the CANIFnDA1 register.
     */

    if (CAN_is_transmit_type(msg->msg_type)) {
        CAN_write_data_to_reg(c, msg->data, (msg->data_len & CANIF1MCTL_DLC_MASK));
    }

    /*
     * Program the number of the message object to be transmitted in the MNUM
     * field in the CANIFn Command Request (CANIFnCRQ) register.
     */
    CANIF1CRQ_value |= (msg->obj_id & CANIF1CRQ_MNUM_MASK);

    /*
     * When everything is properly configured, set the TXRQST bit in the
     * CANIFnMCTL register. Once this bit is set, the message object is
     * available to be transmitted, depending on priority and bus availability.
     */
    if (CAN_is_transmit_type(msg->msg_type)) {
        CANIF1MCTL_value |= CANIF1MCTL_TXRQST_MASK;
    }

    /* Finally, commit the values to the CAN controller registers. */
    PTR(CAN_BASE_ADDR, CANIF1MSK1_OFFSET) |= CANIF1MSK1_value;
    PTR(CAN_BASE_ADDR, CANIF1MSK2_OFFSET) |= CANIF1MSK2_value;
    PTR(CAN_BASE_ADDR, CANIF1ARB1_OFFSET) |= CANIF1ARB1_value;
    PTR(CAN_BASE_ADDR, CANIF1ARB2_OFFSET) |= CANIF1ARB2_value;
    PTR(CAN_BASE_ADDR,  CANIF1CRQ_OFFSET) |=  CANIF1CRQ_value;
    PTR(CAN_BASE_ADDR, CANIF1MCTL_OFFSET) |= CANIF1MCTL_value;

    return true;
}

/*
 * An API to fetch the data out from a message objec, and subsequently populate
 * `msg` with the data.
 *
 * Return value: Returns true on success. When a wrong message object ID is
 * provided in `msg`, it returns false.
 */
bool CAN_get_message_object(enum CAN c, CANMsgObject *msg)
{
    uint32_t CANIF1MSK1_value = 0;
    uint32_t CANIF1MSK2_value = 0;
    uint32_t CANIF1MCTL_value = 0;
    uint32_t CANIF1ARB1_value = 0;
    uint32_t CANIF1ARB2_value = 0;

    uint32_t CAN_BASE_ADDR = (uint32_t)(c);

    /* Only allow *correct* message IDs. */
    if (msg->obj_id > 32 || msg->obj_id == 0) {
        return false;
    }

    /*
     * The CPU first writes 0x007F to the CANIFnCMSK register. Which means we
     * set all the bits in CANIF1CMSK to 1 but for the WRNRD bit, since we're
     * only reading from the message object.
     *
     * That combination transfers the whole received message from the message
     * RAM into the Message Buffer interface registers (CANIFnMSKn, CANIFnARBn,
     * and CANIFnMCTL)
     */
    CAN_config_CANIF1CMSK(CAN_BASE_ADDR, CANIF1CMSK_DATAA_MASK  |
                                         CANIF1CMSK_DATAB_MASK  |
                                         CANIF1CMSK_MASK_MASK   |
                                         CANIF1CMSK_ARB_MASK    |
                                         CANIF1CMSK_TXRQST_MASK |
                                         CANIF1CMSK_CONTROL_MASK);

    /* Write the number of the message object to the CANIFnCRQ register */
    PTR(CAN_BASE_ADDR, CANIF1CRQ_OFFSET) |= (msg->obj_id & CANIF1CRQ_MNUM_MASK);

    CANIF1MSK1_value = PTR(CAN_BASE_ADDR, CANIF1MSK1_OFFSET);
    CANIF1MSK2_value = PTR(CAN_BASE_ADDR, CANIF1MSK2_OFFSET);
    CANIF1MCTL_value = PTR(CAN_BASE_ADDR, CANIF1MCTL_OFFSET);
    CANIF1ARB1_value = PTR(CAN_BASE_ADDR, CANIF1ARB1_OFFSET);
    CANIF1ARB2_value = PTR(CAN_BASE_ADDR, CANIF1ARB2_OFFSET);

    msg->flags = MSG_OBJ_NO_FLAGS;

    bool dir_bit = (CANIF1ARB2_value & CANIF1ARB2_DIR_MASK) != 0;
    bool txrqst_bit = (CANIF1MCTL_value & CANIF1CMSK_TXRQST_MASK) != 0;

    /*
     * For RX remote frames, TXRQST is reset and DIR is set.
     * For TX remote frames, TXRQST is set and DIR is reset.
     *
     * Otherwise, it's not a remote (RX/TX) frame.
     */
    if (dir_bit != txrqst_bit) {
        msg->flags |= MSG_OBJ_REMOTE_FRAME;
    }

    /*
     * The message handler stored a new message into this object when NEWDAT was
     * set; the CPU has lost a message. This bit is only valid for message
     * objects when the DIR bit in the CANIFnARB2 register is clear (receive)
     */
    if (!(CANIF1ARB2_value & CANIF1ARB2_DIR_MASK) &&
        (CANIF1ARB2_value & CANIF1MCTL_MSGLST_MASK)) {
        msg->flags |= MSG_OBJ_DATA_LOSS;
    }

    /*
     * Extract the ID from the message object RAM. If the XTD bit is set,
     * then the arbitration uses the full 29-bit ID.
     */
    if (CANIF1ARB2_value & CANIF1ARB2_XTD_MASK) {
        msg->msg_id = ((CANIF1ARB2_value & CANIF1ARB2_ID_MASK) << 16) |
                       (CANIF1ARB1_value & CANIF1ARB1_ID_MASK);
    } else {
        msg->msg_id = (CANIF1ARB2_value & CANIF1ARB2_ID_MASK) >> 2;
    }

    /* Are interrupts enabled for this message object? */

    /*
     * If the TXIE bit in CANIF1MCTL register was set, then the TX interrupt was
     * initially enabled for this message object.
     */
    if (CANIF1MCTL_value & CANIF1MCTL_TXIE_MASK) {
        msg->flags |= MSG_OBJ_TX_INT_ENABLED;
    }

    /*
     * If the RXIE bit in CANIF1MCTL register was set, then the RX interrupt was
     * initially enabled for this message object.
     */
    if (CANIF1MCTL_value & CANIF1MCTL_RXIE_MASK) {
        msg->flags |= MSG_OBJ_RX_INT_ENABLED;
    }

    /* Is the ID used for acceptance filtering? If so, extract the mask. */
    if (CANIF1MCTL_value & CANIF1MCTL_UMASK_MASK) {
        if (CANIF1ARB2_value & CANIF1ARB2_XTD_MASK) {
            msg->msg_id_mask = ((CANIF1MSK2_value & CANIF1MSK2_MSK_MASK) << 16) |
                               (CANIF1MSK1_value & CANIF1MSK1_MSK_MASK);

            msg->flags |= MSG_OBJ_USE_ID_FILTER;

            /*
             * The Full 29-bit ID was used for acceptance filtering if and only
             * if MXTD bit was set.
             */
            if (CANIF1MSK2_value & CANIF1MSK2_MXTD_MASK) {
                msg->flags |= MSG_OBJ_USE_EXT_FILTER;
            }

        } else {
            msg->msg_id_mask = (CANIF1MSK2_value >> 2) & CANIF1MSK2_MSK_MASK;
            msg->flags |= MSG_OBJ_USE_ID_FILTER;
        }

        /*
        * The direction bit was used for acceptance filtering if and only if
        * MDIR bit was set.
        */
        if (CANIF1MSK2_value & CANIF1MSK2_MDIR_MASK) {
            msg->flags |= MSG_OBJ_USE_DIR_FILTER;
        }
    }

    /*
     * If there are new data since the last read, the NEWDAT bit in CANIF*MCTL
     * register is set.
     *
     * The NEWDAT flag is *not* reset by the controller, it must be reset by the
     * software after extracting the data from the CAN registers.
     */
    if (CANIF1MCTL_value & CANIF1CMSK_NEWDAT_MASK) {
        CAN_read_data_from_reg(CAN_BASE_ADDR, msg->data, msg->data_len);
        PTR(CAN_BASE_ADDR, CANIF1DA1_OFFSET) &= ~CANIF1CMSK_NEWDAT_MASK;
        msg->flags |= MSG_OBJ_NEW_DATA;
    } else {
        /* We do this to indicate that there were no data read. */
        msg->data_len = 0;
    }

    return true;
}

/*
 * An API to clear the contents of a message object. The message object is left
 * in an invalid state. It must be reprogrammed to be used.
 *
 * Return value: Returns true on success. When a wrong message object ID is
 * given, it returns false.
 */
bool CAN_remove_message_object(enum CAN c, uint8_t obj_id)
{
    uint32_t CAN_BASE_ADDR = (uint32_t)c;

    if (obj_id > 32 || obj_id == 0) {
        return false;
    }

    /*
     * Allow writing to the ARB* registers, which contain the MSGVAL bit.  The
     * MSGVAL bit is responsible for identifying whether the message object is
     * valid or not.
     */
    PTR(CAN_BASE_ADDR, CANIF1CMSK_OFFSET) =
        CANIF1CMSK_WRNRD_MASK | CANIF1CMSK_ARB_MASK;

    PTR(CAN_BASE_ADDR, CANIF1ARB2_OFFSET) &= ~CANIF1ARB2_MSGVAL_MASK;

    /*
     * Configure the `MNUM` field of the register `CANIF1CRQ` to the
     * in-consideration message. This indicates, in conjunction with the
     * resetting of the `MSGVAL` field, to an invalid message object.
     */
    PTR(CAN_BASE_ADDR, CANIF1CRQ_OFFSET) |= (obj_id & CANIF1CRQ_MNUM_MASK);

    return true;
}

/*
 * Enable certain interrupts for a specific CAN controller. The available
 * interrupt masks are `CAN_INTR_GLOBAL`, `CAN_INTR_STATUS`, and
 * `CAN_INTR_ERROR`.
 *
 * A utility definition `CAN_INTR_ALL` that comprises the value of the three
 * masks is available to be used by the API user.
 *
 * Return value: Returns true on successful enabling of the specified
 * interrupts. Returns false if undefined interrupts are requested to be
 * enabled.
 */
bool CAN_interrupt_enable(enum CAN c, uint32_t interrupt_masks)
{
    uint32_t CAN_BASE_ADDR = (uint32_t)c;

    if ((interrupt_masks & CAN_INTR_ALL) != 0) {
        return false;
    }

    PTR(CAN_BASE_ADDR, CANCTL_OFFSET) |= interrupt_masks;

    return true;
}
