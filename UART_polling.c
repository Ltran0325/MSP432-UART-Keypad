#include "msp.h"
#include <stdint.h>

int main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW |             // Stop watchdog timer
            WDT_A_CTL_HOLD;

    CS->KEY = CS_KEY_VAL;                   // Unlock CS module for register access
    CS->CTL0 = 0;                           // Reset tuning parameters
    CS->CTL0 = CS_CTL0_DCORSEL_3;           // Set DCO to 12MHz (nominal, center of 8-16MHz range)
    CS->CTL1 = CS_CTL1_SELA_2 |             // Select ACLK = REFO
            CS_CTL1_SELS_3 |                // SMCLK = DCO
            CS_CTL1_SELM_3;                 // MCLK = DCO
    CS->KEY = 0;                            // Lock CS module from unintended accesses

    // Configure UART pins
    P3->SEL0 |= BIT2 | BIT3;                // set 2-UART pin as secondary function

    // Configure UART
    EUSCI_A2->CTLW0 |= EUSCI_A_CTLW0_SWRST; // Put eUSCI in reset
    EUSCI_A2->CTLW0 = EUSCI_A_CTLW0_SWRST | // Remain eUSCI in reset
            EUSCI_B_CTLW0_SSEL__SMCLK;      // Configure eUSCI clock source for SMCLK
    // Baud Rate calculation
    // 12000000/(16*9600) = 78.125
    // Fractional portion = 0.125
    // User's Guide Table 21-4: UCBRSx = 0x10
    // UCBRFx = int ( (78.125-78)*16) = 2
    EUSCI_A2->BRW = 78;                     // 12000000/16/9600
    EUSCI_A2->MCTLW = (2 << EUSCI_A_MCTLW_BRF_OFS) |
            EUSCI_A_MCTLW_OS16;

    EUSCI_A2->CTLW0 &= ~EUSCI_A_CTLW0_SWRST; // Initialize eUSCI

    uint8_t receive = 0x30;
    while(1){
        while(!(EUSCI_A2->IFG & EUSCI_A_IFG_TXIFG));    // wait until TX_Buffer empty
        EUSCI_A2->TXBUF = receive;
        while(!(EUSCI_A2->IFG & EUSCI_A_IFG_RXIFG));    // wait until byte ready to read
        receive = EUSCI_A2->RXBUF;
    }
}
