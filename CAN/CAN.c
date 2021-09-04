#include "CAN.h"

static void CAN_GPIO_init(enum CAN c) {
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
        PTR(GPIOBAFSEL) |= CAN0_AFSEL;
        PTR(GPIOBPCTL)  |= CAN0_PCTL;
    } else {
        PTR(GPIOAAFSEL) |= CAN1_AFSEL;
        PTR(GPIOAPCTL)  |= CAN1_PCTL;
    }
}

static uint32_t get_canbit(uint32_t bitrate, int8_t cfg_prop_time) {
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
    uint8_t phases_time = N_TIME_QUANTA - 3;
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

void CAN_disable(enum CAN c) {
    uint32_t CANCTL = (c == CAN0) ? CAN0CTL : CAN1CTL;
    PTR(CANCTL) |= CANCTL_INIT_MASK;
}

void CAN_enable(enum CAN c) {
    uint32_t CANCTL = (c == CAN0) ? CAN0CTL : CAN1CTL;
    PTR(CANCTL) &= ~CANCTL_INIT_MASK;
}

bool CAN_init(struct CANConfig *cfg) {
    CAN_GPIO_init(cfg->can_num);
    
    uint32_t CANCTL, CANBIT;

    if (cfg->can_num == CAN0) {
        CANCTL = CAN0CTL;
        CANBIT = CAN0BIT;
    } else {
        CANCTL = CAN1CTL;
        CANBIT = CAN1BIT;
    }

    /* Initialization is started by setting the INIT bit in the CANCTL register */
    CAN_disable(cfg->can_num);

    /* Set the CAN Bit Timing CANBIT register */
    PTR(CANCTL) |= CANCTL_CCE_MASK;
    uint32_t canbit = get_canbit(cfg->bit_rate, cfg->prop_time);

    if (canbit == 0) {
        return false;
    }

    PTR(CANBIT) |= canbit;

    /* To leave the initialization state, the INIT bit must be cleared. */
    CAN_enable(cfg->can_num);

    return true;
}

static void CAN_config_CANIF1CMSK(enum CAN c, uint32_t options) {
    /* 1. In the CAN IFn Command Mask (CANIFnCMASK) register: */

    uint32_t CANIF1CMSK;

    if (c == CAN0) {
        CANIF1CMSK = CAN0IF1CMSK;
    } else {
        CANIF1CMSK = CAN1IF1CMSK;
    }

    /* Set the WRNRD bit to specify a write to the CANIFnCMASK register; */

    /*
     * Conditionally set since it can alter that state of other bits. It's left
     * for the CAN_config_CANIF1CMSK user to ensure consistent F1CMSK register
     * state.
     */
    if (options & CANIF1CMSK_WRNRD_MASK) {
        PTR(CANIF1CMSK) |= CANIF1CMSK_WRNRD_MASK;
    }
     
    /* 
     * specify whether to transfer the IDMASK, DIR, and MXTD of the message
     * object into the CAN IFn registers using the MASK bit
     */
    if (options & CANIF1CMSK_MASK_MASK) {
        PTR(CANIF1CMSK) |= CANIF1CMSK_MASK_MASK;
    }

    /*
     * Specify whether to transfer the ID, DIR, XTD, and MSGVAL of the message
     * object into the interface registers using the ARB bit
     */
    if (options & CANIF1CMSK_ARB_MASK) {
        PTR(CANIF1CMSK) |= CANIF1CMSK_ARB_MASK;
    }

    /*
     * Specify whether to transfer the control bits into the interface registers
     * using the CONTROL bit
     */
    if (options & CANIF1CMSK_CONTROL_MASK) {
        PTR(CANIF1CMSK) |= CANIF1CMSK_CONTROL_MASK;
    }

    /*
     * Specify whether to clear the INTPND bit in the CANIFnMCTL register using
     * the CLRINTPND bit
     */
    if (options & CANIF1CMSK_CLRINTPND_MASK) {
        PTR(CANIF1CMSK) |= CANIF1CMSK_CLRINTPND_MASK;
    }

    /*
     * Specify whether to clear the NEWDAT bit in the CANNWDAn register using
     * the NEWDAT bit
     */
    if (options & CANIF1CMSK_NEWDAT_MASK) {
        PTR(CANIF1CMSK) |= CANIF1CMSK_NEWDAT_MASK;
    }

    /*
     * Specify which bits to transfer using the DATAA and DATAB bits
     */
    if (options & CANIF1CMSK_DATAA_MASK) {
        PTR(CANIF1CMSK) |= CANIF1CMSK_DATAA_MASK;
    }

    if (options & CANIF1CMSK_DATAB_MASK) {
        PTR(CANIF1CMSK) |= CANIF1CMSK_DATAB_MASK;
    }
}

bool CAN_config_transmit_message(enum CAN c, struct CANMsgObject *msg)
{
    uint32_t CANIF1MSK1_value;
    uint32_t CANIF1MSK2_value;

    /* TODO: Should we allow the API user to specify those options? */
    CAN_config_CANIF1CMSK(c, CANIF1CMSK_WRNRD_MASK |
                             CANIF1CMSK_ARB_MASK   |
                             CANIF1CMSK_CONTROL_MASK);
    
    bool use_29_bit_id = (msg->msg_id > CAN_11_BIT_ID_MASK);

    if (use_29_bit_id) {
        CANIF1MSK1_value = (msg->msg_id & CAN_11_BIT_ID_MASK);
        CANIF1MSK2_value = (msg->msg_id >> 16);
    } else {
        CANIF1MSK1_value = 0;
        CANIF1MSK2_value = ((msg->msg_id & CAN_11_BIT_ID_MASK) << 2);
    }

}
