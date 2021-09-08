#include "CAN.h"

volatile uint32_t g_errflag;
volatile const uint32_t g_receive_obj_id = 1;
volatile const uint32_t g_transmit_obj_id = 2;
volatile bool g_received_flag;

const uint32_t transmit_message_id = 1;
const uint32_t receive_message_id  = 2;

uint8_t transmit_data[]  = { 'T', 'e', 's', 't', 'D', 'a', 't', 'a' };
uint8_t receive_buffer[] = { '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0' };

CANMsgObject transmit_message_obj;
CANMsgObject receive_message_obj;

static inline void config_transmit_obj()
{
    transmit_message_obj.obj_id = g_transmit_obj_id;
    transmit_message_obj.msg_id_mask = 0;
    transmit_message_obj.msg_id = transmit_message_id;
    transmit_message_obj.data_len = sizeof (transmit_data);
    transmit_message_obj.data = transmit_data;
    transmit_message_obj.flags = MSG_OBJ_TX_INT_ENABLED;
    transmit_message_obj.msg_type = MSG_OBJ_TYPE_TX;
}

static inline void config_receive_obj()
{
    receive_message_obj.obj_id = g_receive_obj_id;
    receive_message_obj.msg_id_mask = 0;
    receive_message_obj.msg_id = receive_message_id;
    receive_message_obj.data_len = 0;
    receive_message_obj.data = receive_buffer;
    receive_message_obj.flags = MSG_OBJ_RX_INT_ENABLED;
    receive_message_obj.msg_type = MSG_OBJ_TYPE_RX;
}

int main()
{
    g_errflag = 0;
    g_received_flag = false;

    CANConfig cfg = {
        .bit_rate = 1000000,
        .can_num = CAN0,
        .prop_time = 2,
    };

    CAN_init(&cfg);
    /* TODO: Enable CAN0 interrupt in the NVIC. */
    CAN_interrupt_enable(CAN0, CAN_INTR_ALL);
    CAN_enable(CAN0);

    config_transmit_obj();
    CAN_config_message(CAN0, &transmit_message_obj);

    config_receive_obj();
    CAN_config_message(CAN0, &receive_message_obj);

    while (1) {
        if (g_received_flag) {
            CAN_get_message_object(CAN0, &receive_message_obj);
            /*
             * Now the newly-fetched data resides in the `data` field of
             * `receive_message_object.
             */

            if (receive_message_obj.flags | MSG_OBJ_DATA_LOSS) {
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
            /* TODO: Handle errors... */
        }
    }
}
