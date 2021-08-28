#include "i2c.h"

void I2C0_init() {
    /* Enable the I2C clock using the RCGCI2C register in the System Control module */
    RCGCI2C |= 0x01;

    /* Enable the clock to the appropriate GPIO module via the RCGCGPIO register
     * in the System Control module
     *
     * I2C inhabits PORTB pins 2 and 3.
     */
    RCGCGPIO |= (0x02);

    /*
     * In the GPIO module, enable the appropriate pins for their alternate 
     * function using the GPIOAFSEL register
     */
    GPIOBAFSEL |= (0x11 << 2);

    /* Enable the I2CSDA pin for open-drain operation */
    GPIOBODR |= (0x1 << 3);

    /*
     * Configure the PMCn fields in the GPIOPCTL register to assign the I2C 
     * signals to the appropriate pins.
     */
    GPIOBCTL |= ((0x11 << 4) | (0x11 << 8));

    /* Initialize the I2C Master by writing the I2CMCR register with a value of 
     * 0x0000.0010
     */
    I2C0MCA = 0x00000010;

    /*
     * Set the desired SCL clock speed of 100 Kbps by writing the I2CMTPR 
     * register with the correct value. The value written to the I2CMTPR 
     * register represents the number of system clock periods in one SCL clock 
     * period. 
     *
     * Assuming that system clock is 20MHz: 
     *
     * TPR = (System Clock/(2*(SCL_LP + SCL_HP)*SCL_CLK))-1;
     * TPR = (20MHz/(2*(6+4)*100000))-1;
     * TPR = 9
     */

    I2C0MTPR = 0x00000009;
}

void I2C0_send_byte(uint8_t data, uint8_t slave_addr) {
    /* Specify the slave address of the master and that the next operation is a 
     * Transmit by writing the I2CMSA 
     */
    I2C0MCA &= ~(0x1);
    I2C0MCA |= (slave_addr << 1);

    /*
     * Place data (byte) to be transmitted in the data register by writing the
     * I2CMDR register with the desired data
     */
    I2C0MDR |= data;

    /*
     * Initiate a single byte transmit of the data from Master to Slave by 
     * writing the I2CMCS register with a value of 0x0000.0007 
     * (STOP, START, RUN).
     */
    I2C0MCS |= 0x00000007;

    /*
     * Wait until the transmission completes by polling the I2CMCS register's 
     * BUSBSY bit until it has been cleared
     */
    while (I2C0MCS & (1 << 6) != 0);

    if (I2C0MCS & (1 << 2) != 0) {
        if (I2C0MCS & (1 << 4) != 0) {
            /* Arbitration lost... resend (?)
             * This could be done by recursively calling this function again
             * until sending succeeds.
             */ 

            /* back-off delay. */
            for (int i = 0; i < 10000; i++);

            I2C0_send_byte(data, slave_addr);
        } else {
            /* On other errors, send a STOP bit. */
            I2C0MCS |= (1 << 2);

            /* Poll untill the busy bit becomes zero. */
            while ((I2C0MCS & 1) != 0);
        }
    }
}
