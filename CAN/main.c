#include "CAN.h"

volatile uint32_t g_errflag;
volatile const uint8_t g_receive_obj_id = 1;
volatile const uint8_t g_transmit_obj_id = 2;
volatile bool g_received_flag;

const uint32_t transmit_message_id = 1;
const uint32_t receive_message_id  = 2;

uint8_t transmit_data[]  = { 'T', 'e', 's', 't', 'D', 'a', 't', 'a' };
uint8_t receive_buffer[] = { '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0' };

CANMsgObject transmit_message_obj;
CANMsgObject receive_message_obj;

static inline void config_transmit_obj(void)
{
    transmit_message_obj.obj_id = g_transmit_obj_id;
    transmit_message_obj.msg_id_mask = 0;
    transmit_message_obj.msg_id = transmit_message_id;
    transmit_message_obj.data_len = sizeof (transmit_data);
    transmit_message_obj.data = transmit_data;
    transmit_message_obj.flags = MSG_OBJ_TX_INT_ENABLED;
    transmit_message_obj.msg_type = MSG_OBJ_TYPE_TX;
}

static inline void config_receive_obj(void)
{
    receive_message_obj.obj_id = g_receive_obj_id;
    receive_message_obj.msg_id_mask = 0;
    receive_message_obj.msg_id = receive_message_id;
    receive_message_obj.data_len = 0;
    receive_message_obj.data = receive_buffer;
    receive_message_obj.flags = MSG_OBJ_RX_INT_ENABLED;
    receive_message_obj.msg_type = MSG_OBJ_TYPE_RX;
}

static void NVIC_enable_CAN0_interrupt(void)
{
    const uint32_t NVIC_BASE_ADDR = 0xE000E000;
    const uint32_t NVIC_INT_EN_1_OFFSET = 0x104;
    const uint32_t CAN0_interrupt_bit = 39;
    const uint32_t NVIC_INT_EN_CAN0_MASK = (1 << (CAN0_interrupt_bit - 32));

    PTR(NVIC_BASE_ADDR, NVIC_INT_EN_1_OFFSET) |= NVIC_INT_EN_CAN0_MASK;
}

static void handle_bus_off(void)
{
    /*
     * TODO: Proper error handling. For now, we halt for good since bus off state
     * indicates a very unhealthy system.
     */
    g_errflag &= ~CANSTS_BOFF_MASK;
    while (1);
}

static void handle_warning(void)
{
    /* TODO: Proper error handling */
    g_errflag &= ~CANSTS_EWARN_MASK;
}

static void handle_error_passive(void)
{
    /* TODO: Proper error handling */
    g_errflag &= ~CANSTS_EPASS_MASK;
}

static void handle_last_error(void)
{
    /* TODO: Proper error handling */
    g_errflag &= ~CANSTS_LEC_MASK;
}

static void handle_errflag(void)
{
    if (g_errflag & CANSTS_BOFF_MASK) {
        handle_bus_off();
    } else if (g_errflag & CANSTS_EWARN_MASK) {
        handle_warning();
    } else if (g_errflag & CANSTS_EPASS_MASK) {
        handle_error_passive();
    }

    if (g_errflag & CANSTS_LEC_MASK) {
        handle_last_error();
    }
}

int main(void)
{

    return sizeof (uint32_t);

    g_errflag = 0;
    g_received_flag = false;

    CANConfig cfg = {
        .can_num = CAN0,
        .bit_rate = 1000000,

        /* Use the default number of time quanta. */
        .n_time_quanta = 0,

        /* Use the default propagation time. */
        .prop_time = 0,
    };

    CAN_init(&cfg);
    NVIC_enable_CAN0_interrupt();
    CAN_interrupt_enable(CAN0, CAN_INTR_ALL);
    CAN_enable(CAN0);

    config_transmit_obj();
    CAN_config_message(CAN0, &transmit_message_obj);

    config_receive_obj();
    CAN_config_message(CAN0, &receive_message_obj);

    while (true) {
        if (g_received_flag) {
            CAN_get_message_object(CAN0, &receive_message_obj);
            /*
             * Now the newly-fetched data resides in the `data` field of
             * `receive_message_object.
             */

            if (receive_message_obj.flags & MSG_OBJ_DATA_LOSS) {
                /*
                 * The data could not be fetched by the CAN bus and is lost in
                 * the process of transmission/receipt.
                 *
                 * We just halt, for now.
                 */

                while (1);
            }

            g_received_flag = false;
        }

        if (g_errflag) {
            handle_errflag();
        }
    }
}
