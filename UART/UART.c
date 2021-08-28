#include "UART.h"

void UART0_init() {
    /* Enable the UART module using the RCGCUART register */
    RCGCUART |= 0x1;

    /* Enable the clock to the appropriate GPIO module via the RCGCGPI register */
    RCGCGPIO |= 0x1;

    /* Set the GPIO AFSEL bits for the appropriate pins to determine which GPIOs to configure, */
    GPIOAAFSEL |= 0x11;

    /* Configure the PMCn fields in the GPIOPCTL register to assign the UART signals to the appropriate pins */
    GPIOACTL |= (0x1 << 0) | (0x1 << 4);

    /* Digital-enable for GPIOA pin 0 and 1. */
    GPIOADEN |=  (0x1 << 0) | (0x1 << 1);

    /* Disable the UART by clearing the UARTEN bit in the UARTCTL register */
    UART0CTL |= 0x1;

    /* Write the integer portion of the BRD to the UARTIBRD register */
    UART0IBRD = 104;

    /* Write the fractional portion of the BRD to the UARTIBRD register */
    UART0IBRD = 11;

    /* Odd parity, FIFO enabled, 8-bit data frames. */
    UART0LCRH |= (0x1 << 1) | (0x1 << 4) | (0x3 << 5);

    /* Use System Clock. */
    UART0CC &= ~(0xF);

    /* Enable UART0, enable transmitting and receiving. */
    UART0CTL |= (0x1) | (0x1 << 8) | (0x1 << 9);
}

void UART0_enable() {
    /* Clear UART0 RX interrupt. */
    UART0ICR &= ~(0x1 << 4);

    /* Enable the UART0 interrupts in the NVIC. */
    NVIC_ISEN0 |= (0x1 << 5);

    /* Enable RX interrupt for UART0. */
    UART0IM &= (0x1 << 4);
}

/* Using polling... */

unsigned char UART0_receive_byte() {
    #define UART0FR_RXFE 4
    while ((UART0FR >> UART0FR_RXFE) & 1);
    return (unsigned char)(UART0DR & 0xFF);
}

void UART0_send_byte(unsigned char c) {
    #define UART0FR_TXFF 5
    while ((UART0FR >> UART0FR_TXFF) & 1);
    UART0DR = c;
}
