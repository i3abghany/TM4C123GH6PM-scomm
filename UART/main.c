#include "UART.h"

volatile unsigned char recv_byte;
volatile int did_recv;

/* This must be set in the vector table instead of the default handler. This only handles the on-receive interrupt. */
static void UART0_RX_ISR() {    
    /* UARTICR: On a write of 1, the corresponding interrupt is cleared. */
    UART0ICR &= (0x1 << 4);
    
    recv_byte = (unsigned char)(UART0DR & 0xFF);
    did_recv = 1;
}

int main() {
    UART0_init();
    UART0_enable();
    did_recv = 0;
    
    while(1) {
        if (did_recv) {
            /* Capitalize recv if small, otherwise send the successor in the ASCII mapping. */
            UART0_send_byte((recv_byte >= 'a' && recv_byte <= 'z' ? (recv_byte - 'a' + 'A') : (recv_byte + 1)));    
            did_recv = 0;
        }
    }
}
