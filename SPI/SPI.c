#include "SPI.h"

static void SSI0_config_mode(SSI0_MODE mode) {
    switch (mode) {
    case SSI0_MASTER:
        SSI0CR1 = 0x00000000;
        break;
    case SSI0_SLAVE_OPE:
        SSI0CR1 = 0x00000004;
        break;
    case SSI0_SLAVE_OPD:
        SSI0CR1 = 0x0000000C;
        break;
    default:
        __disable_irq();
        while (1);
    }
}

/* Configure the SSI clock source by writing to the SSICC register */
static void SSI0_init_clock(CLOCK_SOURCE src) {
    switch (src) {
    case SYS_CLOCK:
        SSI0CC &= (0xFFFFFFF0);
        break;
    case PIOSC:
        SSI0CC &= (0xFFFFFFF5);
        break;
    default:
        __disable_irq();
        while (1);
    }
}

void SSI0_init(SSI0_config *cfg) {
    /* Enable the SSI module using the RCGCSSI register */
    RCGCSSI |= 0x01;
    
    /* Enable the clock to the appropriate GPIO module via the RCGCGPIO register */
    RCGCGPIO |= 0x01;

    /* Set the GPIO AFSEL bits for the appropriate pins */
    GPIOAAFSEL |= (0x1111 << 2);

    GPIOACTL |= ((0x10 << 8) | (0x10 << 12) | (0x10 << 16) | (0x10 << 20));

    /* Program the GPIODEN register to enable the pin's digital function */
    GPIOADEN |= (0x1111 << 2);
    GPIOAPUR |= (0x1111 << 2);

    /* Ensure that the SSE bit in the SSICR1 register is clear before making any configuration changes */
    SSI0CR1 &= ~(1 << 2);

    SSI0_config_mode(cfg->mode);

    SSI0_init_clock(cfg->clk_src);

    /* Write the SSICR0 register with the following configuration:
    *   Serial clock rate (SCR)
    *   Desired clock phase/polarity, if using Freescale SPI mode (SPH and SPO)
    *   The protocol mode: Freescale SPI, TI SSF, MICROWIRE (FRF)
    *   The data size (DSS)
    */
    SSI0CR0 |= ((cfg->data_size - 1) & 0xF);
    SSI0CR0 |= (cfg->fmt << 4);
    if (cfg->fmt == FREESCALE_SPI) {
        SSI0CR0 |= ((cfg->clk_polarity & 0x01) << 6);
    }
    SSI0CR0 |= ((cfg->serial_clk_phase & 0x01) << 7);

    SSI0CR0 &= ~(0x0000FF00);
    SSI0CR0 |= (cfg->serial_clk_rate << 8);

    /* Enable the SSI by setting the SSE bit in the SSICR1 register. */
    SSI0CR1 |= (1 << 2);
}

static uint16_t get_data(uint8_t dss, uint16_t data) {
    uint16_t mask = 0;
    for (int i = 0; i < dss; i++) {
        mask |= (1 << i);
    }
    return data & mask;
}

void SSI0_transmit(uint8_t dss, uint16_t data) {
    SSI0DR = get_data(dss, data);
    while (SSI0SR & 0x01);
}

uint16_t SSI0_recv(uint8_t dss) {
    while ((SSI0SR & (1 << 2)) & 0x01);
    return SSI0DR & 0xFFFF;
}
