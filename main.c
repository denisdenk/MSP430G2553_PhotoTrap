#include <msp430.h>
#include <stdint.h>

// UART (For debug) P1.1 P1.2
#define RXD     BIT1
#define TXD     BIT2
// Camera P1.3 P1.4 P1.5
#define FOCUS   BIT3
#define SHUTTER BIT4
#define RELAY   BIT5
// SR501 sensor P1.6
#define SR501   BIT6

// Private function prototypes
void System_Init(void);
void GPIO_Init(void);
void UART_Init(void);
void UART_Print(char * tx_data);
void delay_ms(uint16_t ms);

// Private variable
volatile uint32_t i;
volatile uint16_t count;
uint8_t interrutp_sr501_flag = 0;


// Interrupt vector (Rising front of SR501)
#pragma vector=PORT1_VECTOR
__interrupt void Port1(void) {
    interrutp_sr501_flag = 1;                   // Set flag
    __bic_SR_register_on_exit(LPM4_bits);       // Wake UP
    P1IFG &=~SR501;                             // Clearing interrupt flag
}

// Interrupt vector for Timer (for forming delay)
#pragma vector=TIMER0_A1_VECTOR
__interrupt void Timer0_OVF(void) {
    switch(TA0IV) {
    case TA0IV_TACCR1: break;                   // CCR1 not used
    case TA0IV_TACCR2: break;                   // CCR2 not used
    case TA0IV_TAIFG: if (count)  count--;      // overflow
    break;
    }
}


void main(void) {
    System_Init();
    GPIO_Init();
    UART_Init();
    __enable_interrupt();
    _BIS_SR(LPM4_bits + GIE); // Enter LPM4

    // Start
    UART_Print("Start\r\n");
    for(;;) {
        if(interrutp_sr501_flag) {
            interrutp_sr501_flag = 0;
            UART_Print("Interrupt!\r\n");

            P1OUT |= RELAY;     // Turn ON camera
            delay_ms(1200);
            P1OUT &= ~FOCUS;    // Focus camera
            delay_ms(1000);
            P1OUT &= ~SHUTTER;  // Take photo
            delay_ms(200);
            P1OUT |= SHUTTER;   // Release Shutter and Focus button
            P1OUT |= FOCUS;

            delay_ms(5000);     // Delay time for save img
            P1OUT &= ~RELAY;    // Turn OFF camera
            _BIS_SR(LPM4_bits + GIE); // Enter LPM4 (sleep)
        }
    }
}

void System_Init(void) {
    WDTCTL = WDTPW | WDTHOLD;

    // Set up 1MHz CLK
    BCSCTL1 = CALBC1_1MHZ;                  // Set DCO to 1 MHz
    DCOCTL = CALDCO_1MHZ;
    BCSCTL2 &= ~(DIVS_3);                   // SMCLK = DCO = 1MHz

    // Timer init (for delay)
    TA0CTL = TASSEL_2 | MC_1 | ID_0 | TACTL | TAIE;
    /*  TASSEL_2  =use SMCLK;
        MC_1      =continous to TACCR0;
        ID_0      =prescaler  = 1;
        TACLR     =clean counter TAR;
        TAIE;     =enable timer interrupt;
     */
    TA0CCR0 = 1000;
    count = 0;
}

void GPIO_Init(void) {
    // Pin as Output
    P1DIR |= FOCUS;
    P1DIR |= SHUTTER;
    P1DIR |= RELAY;

    P1OUT |= FOCUS;
    P1OUT |= SHUTTER;
    P1OUT &= ~RELAY;

    // Interrupt pin for SR501
    P1DIR &= ~SR501;                        // HiZ mode
    P1REN |= SR501;
    P1IES &= ~SR501;                        // Rising front
    P1IFG &= ~SR501;                        // Interrupt flag clearing
    P1IE |= SR501;                          // Enable external interrupt
}

void UART_Init(void) {
    P1SEL = RXD + TXD ;                     // Select TX and RX functionality for P1.1 & P1.2
    P1SEL2 = RXD + TXD ;

    UCA0CTL1 |= UCSSEL_2;                   // Have USCI use System Master Clock: AKA core clk 1MHz

    UCA0BR0 = 104;                          // 1MHz 9600, see user manual
    UCA0BR1 = 0;

    UCA0MCTL = UCBRS0;                      // Modulation UCBRSx = 1
    UCA0CTL1 &= ~UCSWRST;                   // Start USCI state machine
}

void UART_Print(char* tx_data) {
    unsigned int i=0;
    while(tx_data[i]) {
        while ((UCA0STAT & UCBUSY));        // Wait if line TX/RX module is busy with data
        UCA0TXBUF = tx_data[i];             // Send out element i of tx_data array on UART bus
        i++;                                // Increment variable for array address
    }
}

void delay_ms(uint16_t ms) {
    count=ms;
    // Stupid delay
    while(count);
}
