/*******************************************************************************
*                       MSP432 UART Keypad Solution                            *
*                                                                              *
* Author:  Long Tran                                                           *
* Device:  MSP432P401R LaunchPad                                               *
* Program: UART communication using keypad.                                    *
* Demo:                                                                        *
*******************************************************************************/

// Include header file(s) and define constants
#include "msp.h"

// Define digit-bit lookup table
const uint8_t sseg_look_up[17] = {
0b11000000,  // 0
0b11111001,  // 1
0b10100100,  // 2
0b10110000,  // 3
0b10011001,  // 4
0b10010010,  // 5
0b10000010,  // 6
0b11111000,  // 7
0b10000000,  // 8
0b10010000,  // 9
0b10001000,  // A
0b10000011,  // b
0b11000110,  // C
0b10100001,  // d
0b10000110,  // E
0b10001110,  // F
0b11111111,  // Blank Display
};

const uint32_t ASCII_look_up[17] = {"0", "1", "2", "3", "4", "5", "6", "7", "8", "9",
                                   "A", "B", "C", "D", "E", "F"};

// Define prototypes
void init_clock(void);
void init_UART_A2(void);
void init_NVIC(void);
void init_keypad(void);

void wait(int t);
void sseg_display(void);

char out_msg[4] = "BEEF";
char receive_msg[4] = "BEEF";
uint8_t TX_count = 0;
uint8_t RX_count = 0;
#define MAX 4

#define N 100       // debounce loop count
#define d 16        // digit-bit index to clear display

// Define digit-bit lookup table
const int digit_array[17] = {
0b11000000,  // 0
0b11111001,  // 1
0b10100100,  // 2
0b10110000,  // 3
0b10011001,  // 4
0b10010010,  // 5
0b10000010,  // 6
0b11111000,  // 7
0b10000000,  // 8
0b10010000,  // 9
0b10001000,  // A
0b10000011,  // b
0b11000110,  // C
0b10100001,  // d
0b10000110,  // E
0b10001110,  // F
0b11111111,  // Blank Display
};

// Define keypad layout
const int keypad_table[4][9] = {
    {d, 13, 15, d, 0, d, d, d, 14},   // 4x9 multiplixer keypad
    {d, 10,  3, d, 2, d, d, d,  1},   // only columns 1, 2, 4, 8 are valid for single keypress
    {d, 11,  6, d, 5, d, d, d,  4},
    {d, 12,  9, d, 8, d, d, d,  7}};

// Define keypad structure to handle related variables
typedef struct{         // KEYPAD STRUCTURE
    enum{IDLE, PRESS, PROCESS, RELEASE} state; // keypad state variable
    int x;              // x position of pressed key
    int y;              // y position of pressed key
    int display[4];     // array for keeping the last four pressed numbers
    int display_count;  // display array index
    int pulses;         // debouncing pulses
    int k;              // points to active row of keypad
}Keypad;

void keypad_fsm(Keypad *key);    // call keypad FSM as a function
Keypad key = {IDLE, 0, 0, {0,0,0,0}, 0, 0, 0}; // Initalize keypad structure

void main(void)
{

    WDT_A->CTL = WDT_A_CTL_PW |
                 WDT_A_CTL_HOLD;

    init_clock();    // MCLK & SMCLK = 12MHz
    init_UART_A2();  // 9600 baud, 8 bit, no parity
    init_NVIC();
    init_keypad();

    int temp;

    while(1){

        // Display digit-k
        P4->OUT = 0xFF;                    // blank 7-seg display
        P8->OUT = 0xFF & ~(BIT5 >> key.k); // enable k-th keypad row
        P4->OUT = sseg_look_up[key.display[key.k]];      // display k-th char in array

        // scan input key (at row k)
        temp = (P9->IN) & 0x0F;

        // increment k index
        key.k++;
        if (key.k >= 4){key.k = 0;}

        // reduce flickering
        wait(100);

        // Switch keypad debouncing state
        switch (key.state){

            // Wait for input
            case IDLE:
            {
                // go to PRESS state if input detected
                if(temp > 0 ){
                     key.x = temp;      // acknowledge input x position
                     key.y = key.k;     // acknowledge input y position
                     key.state = PRESS;
                     key.pulses = 0;
                 }break;
            }

            // Accept input if N pulses of HIGH detected
            case PRESS:
            {
                if(key.k == key.y && temp == key.x){ // pulse repeat
                    key.pulses++;
                }
                if(key.k == key.y && temp != key.x){
                    key.state = IDLE;                // input fail
                }
                if(key.pulses > N){
                    key.state = PROCESS;             // input success
                }break;
            }

            // Update display array with accepted input
            case PROCESS:
            {
                if(key.x == 1 && key.y == 0){        // If 'D' pressed

                    EUSCI_A2->IE |= EUSCI_A_IE_TXIE;
                    key.display_count = 0;
                    key.pulses = 0;
                    key.state = RELEASE;
                    break;

                }
                // process input into digit display array (decode)
                key.display[key.display_count] = keypad_table[key.y][key.x];

                // decimal->ASCII
                if( key.display[key.display_count] >= 0 && key.display[key.display_count] <= 9){    // 0-9
                    out_msg[key.display_count] = key.display[key.display_count] + 0x30;
                }
                if( key.display[key.display_count] >= 10 && key.display[key.display_count] <= 15){    // 0-9
                    out_msg[key.display_count] = key.display[key.display_count] + 0x37;
                }

                // increment display digit index
                key.display_count++;
                if(key.display_count > 3){key.display_count = 0;}

                key.pulses = 0;
                key.state = RELEASE;
                break;
            }

            // Accept release if N pulses of LOW detected
            case RELEASE:
            {
                if(key.k == key.y && temp == 0){  // release repeat
                    key.pulses++;
                }
                if(key.k == key.y && temp != 0){
                    key.pulses = 0;               // release fail
                }
                if(key.pulses > N){
                    key.state = IDLE;             // release success
                }break;
            }

        }// switch end
    }// while(1) end
}// main end

//-- Functions

void init_clock(void){                      // SMCLK = 12MHz

    CS->KEY = CS_KEY_VAL;                   // Unlock CS module for register access
    CS->CTL0 = 0;                           // Reset tuning parameters
    CS->CTL0 = CS_CTL0_DCORSEL_3;           // Set DCO to 12MHz (nominal, center of 8-16MHz range)
    CS->CTL1 = CS_CTL1_SELA_2 |             // Select ACLK = REFO
               CS_CTL1_SELS_3 |             // SMCLK = DCO
               CS_CTL1_SELM_3;              // MCLK = DCO
    CS->KEY = 0;                            // Lock CS module from unintended accesses

}

void init_UART_A2(void){

    // Configure UART pins
    P3->SEL0 |= BIT2 | BIT3;                // set 2-UART pin as secondary function

    // Configure UART
    EUSCI_A2->CTLW0 |= EUSCI_A_CTLW0_SWRST; // Put eUSCI in reset
    EUSCI_A2->CTLW0 = EUSCI_A_CTLW0_SWRST | // Remain eUSCI in reset
                 EUSCI_A_CTLW0_SSEL__SMCLK; // Configure eUSCI clock source for SMCLK
    // Baud Rate calculation
    // 12000000/(16*9600) = 78.125
    // Fractional portion = 0.125
    // User's Guide Table 21-4: UCBRSx = 0x10
    // UCBRFx = int ( (78.125-78)*16) = 2
    EUSCI_A2->BRW = 78;                     // 12000000/16/9600
    EUSCI_A2->MCTLW = (2 << EUSCI_A_MCTLW_BRF_OFS) |
                      EUSCI_A_MCTLW_OS16;

    EUSCI_A2->CTLW0 &= ~EUSCI_A_CTLW0_SWRST; // Initialize eUSCI

    EUSCI_A2->IFG &= ~(EUSCI_A_IFG_RXIFG);     // Clear eUSCI TX/RX interrupt flag
    EUSCI_A2->IE |= EUSCI_A_IE_RXIE;         // Enable USCI_A2 TX/RX interrupt

}

void init_NVIC(void){

    // Enable eUSCIA2 interrupt in NVIC module
    NVIC->ISER[0] = 1 << ((EUSCIA2_IRQn) & 31);

}

void init_keypad(void){
    P4->DIR = 0xFF;  // P4 is LED output
    P8->DIR = 0xFF;  // P8 is display output
    P9->DIR = 0x00;  // P9 is keypad input
}

void wait(int t){
    while(t >= 0){t--;}
}

//-- Interrupt Handlers

void EUSCIA2_IRQHandler(void)
{
    uint8_t iv_val = EUSCI_A2->IV;

    if( (iv_val & 0x4) && (TX_count < MAX ) )    // if transmit buffer empty and transmit ready
    {
        EUSCI_A2->TXBUF = out_msg[TX_count];
        TX_count++;
        if(TX_count == MAX){
            TX_count = 0;
            EUSCI_A2->IE &= ~EUSCI_A_IE_TXIE;   // disable TXI after array sent
        }
    }


    if( iv_val & 0x2 )    // if receive buffer full (highest priority)
    {
        receive_msg[RX_count] = EUSCI_A2->RXBUF;    // receive ASCI char

        // ASCII -> decimal
        if( receive_msg[RX_count] >= 0x30 && receive_msg[RX_count] <= 0x39){    // 0-9
            key.display[RX_count] = receive_msg[RX_count] - 0x30;
        }
        if( receive_msg[RX_count] >= 0x41 && receive_msg[RX_count] <= 0x46){    // A-F
            key.display[RX_count] = receive_msg[RX_count] - 0x37;
        }
        if( receive_msg[RX_count] >= 0x61 && receive_msg[RX_count] <= 0x66){    // a-f
            key.display[RX_count] = receive_msg[RX_count] - 0x57;
        }

        RX_count++;

        if(RX_count >= 4){
            RX_count = 0;
        }

    }

}


