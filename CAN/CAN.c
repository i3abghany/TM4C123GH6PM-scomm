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

static uint32_t get_canbit(uint32_t bitrate) {
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
    uint8_t prescaler = (uint32_t)(time_quantum * F_SYS_CLOCK) & (0b00111111);

    /* tSync = 1 * time_quantum (fixed) */
    uint8_t sync_time = 1;

    /*
     * tProp depends on the physical characteristics of the CAN network.
     * For the sake of simplicity, we'll fix it at 2 time quanta.
     */
    uint8_t prop_time = 2;

    /* tPhase1 = tPhase2 = 
     *     (N_TIME_QUANTA - ((tProp / time_quantum) + (tSync / time_quantum))
     */
    uint8_t phases_time = N_TIME_QUANTA - 3;
    uint8_t phase1_time = phases_time / 2;
    uint8_t phase2_time = phases_time / 2;
    phase2_time += (phases_time % 2 != 0);

    uint8_t tTSeg1 = (prop_time + phase1_time) & (0b00001111);
    uint8_t tTSeg2 = (phase2_time) & (0b00000111);

    /* The length of the synchronization jump width is set to the least of 4, Phase1 or Phase2 */
    uint8_t tSJW   = min(min(phase1_time, phase2_time), 4);
    tSJW &= (0b00000011);

    /*
     * The four components TSEG2, TSEG1, SJW, and BRP have to be programmed to a
     * numerical value that is one less than its functional value; so instead of
     * values in the range of [1..n], values in the range of [0..n-1] are
     * programmed. That way, for example, SJW (functional range of [1..4]) is
     * represented by only two bits in the SJW bit field.
     */
    prescaler--;
    tTSeg1--;
    tTSeg2--;
    tSJW--;

    return (tTSeg2 << 12) | (tTSeg1 << 8) | (tSJW << 6) | (prescaler);
}

void CAN_init(struct CANConfig *cfg) {
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
    PTR(CANCTL) |= CANCTL_INIT_MASK;

    /* Set the CAN Bit Timing CANBIT register */
    PTR(CANCTL) |= CANCTL_CCE_MASK;
    PTR(CANBIT) |= get_canbit(cfg->bit_rate);

    /* To leave the initialization state, the INIT bit must be cleared. */
    PTR(CANCTL) &= ~(CANCTL_INIT_MASK);
    PTR(CANCTL) &= ~(CANCTL_CCE_MASK);
}
