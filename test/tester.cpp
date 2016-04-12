//
// TEST FILE: FOR ALL IDEAS AND METHODS TO PLAY AROUND WITH.
//

#include <unistd.h>
#include <signal.h>
#include <iostream>
#include <sstream>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <netdb.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include "mraa.hpp"

#define MAX_BUFFER_LENGTH 6
#define PWM_I2C_ADDR 0x1E

int running = 0;

void
sig_handler(int signo)
{
    if (signo == SIGINT) {
        printf("closing nicely\n");
        running = -1;
    }
}

int main()
{
    float direction = 0;
    int16_t x = 0, y = 0, z = 0;
    uint8_t rx_tx_buf[MAX_BUFFER_LENGTH];

    //! [Interesting]
    mraa::I2c* i2c;
    i2c = new mraa::I2c(6);

    i2c->address(PWM_I2C_ADDR);
    rx_tx_buf[0] = 0xFF;
    rx_tx_buf[1] = 0x02;
    i2c->write(rx_tx_buf, 2);
    //! [Interesting]

    i2c->address(PWM_I2C_ADDR);
    rx_tx_buf[0] = 0xFE;
    rx_tx_buf[1] = 0x01;
    i2c->write(rx_tx_buf, 2);

    signal(SIGINT, sig_handler);

    while (running == 0) {
    }
    delete i2c;

    return MRAA_SUCCESS;
}
