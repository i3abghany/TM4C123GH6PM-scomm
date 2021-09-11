#include "CAN.h"

/* Contains the value of the CANSTS register when a status interrupt is fired. */
extern uint32_t g_errflag;

/* The message object ID of the receive message. */
extern const uint8_t g_receive_obj_id;

/* The message object ID of the trasmit message. */
extern const uint8_t g_transmit_obj_id;

/*
 * Indicates to the main thread of execution that a receive interrupt has
 * happened.
 */
extern bool g_received_flag;

/*
 * The ISR for CAN0 interrupts. This must be statically put in the startup code,
 * in the vector table section.
 *
 * We only use one ISR to handle all the possible interrupts, inside the ISR, we
 * determine which interrupt occurred and then serve it in the main thread of
 * execution so as not to make the ISR the performance bound (Although it's not
 * particularly important in this case since the ARM Cortex M3 inside the target
 * MCU can nest interrupts).
 *
 * Any errors are indicated in g_errflag, which can be subsequently handled in
 * the main thread of execution.
 */

void CAN0_interrupt_handler(void)
{
    uint32_t CANSTS_value;
    uint32_t CANINT_value = PTR(CAN0_BASE_ADDR, CANINT_OFFSET);

    /*
     * Status interrupt is caused by a change in the TXOK, RXOK, or LEC bit
     * fields in the CANSTS register. The main thread of execution is
     * responsible for handling any errors indicated by the LEC bits, if any.
     */
    if (CANINT_value == CANINT_STATUS_INTR) {
        CANSTS_value = PTR(CAN0_BASE_ADDR, CANSTS_OFFSET);
        PTR(CAN0_BASE_ADDR, CANSTS_OFFSET) &=
            ~(CANSTS_RXOK_MASK | CANSTS_TXOK_MASK | CANSTS_LEC_MASK);

        g_errflag |= CANSTS_value;
    } else if (CANINT_value == g_receive_obj_id) {
        /*
         * If the value in CANINT is the same the message object ID that we give
         * to our example message, then this is a receive interrupt. No errors
         * are accounted for.
         */
        g_errflag = 0;

        /* The RXOK bit must be cleared manually on serving the interrupt. */
        PTR(CAN0_BASE_ADDR, CANSTS_OFFSET) &= ~CANSTS_RXOK_MASK;

        /*
         * To indicate that the message is delivered and the main thread of
         * execution is responsible for extracting the message.
         */
        g_received_flag = true;
    } else if (CANINT_value == g_transmit_obj_id) {
        /*
         * If the value in CANINT is the same the message object ID that we give
         * to our example message, then this is a transmit interrupt. No errors
         * are accounted for.
         */
        g_errflag = 0;

        /* The TXOK bit must be cleared manually on serving the interrupt. */
        PTR(CAN0_BASE_ADDR, CANSTS_OFFSET) &= ~CANSTS_TXOK_MASK;
    }
}
