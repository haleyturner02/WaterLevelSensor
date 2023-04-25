#include <msp430.h> 
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

int j = 0;

volatile int i, n, sampleCount;
volatile int sample = 0;
volatile int tcnt = 0;
volatile float avg;
volatile float unrounded_avg = 0;
volatile float ADC[9];

volatile unsigned char key;
volatile unsigned char transmit_key;


char packet[] = {0x12, 0x0A, 0x0A, 0x0A, 0x0A};     // 5 byte packet for transmission

volatile unsigned char col_holding, row_holding;
volatile unsigned char pressed_key;
//int lock_state = 0;
int lock_state = 1;

void initI2C_master(){
     UCB1CTLW0 |= UCSWRST;          // SW RESET ON

     UCB1CTLW0 |= UCSSEL_3;         // SMCLK
     UCB1BRW = 10;                  // Prescalar to 10

     UCB1CTLW0 |= UCMODE_3;         // Put into I2C mode
     UCB1CTLW0 |= UCMST;            // Set as MASTER
     UCB1CTLW0 |= UCTR;             // Put into Tx mode

     UCB1CTLW1 |= UCASTP_2;         // Enable automatic stop bit
     UCB1TBCNT = sizeof(packet);    // Transfer byte count

     // Setup ports
     P6DIR |= (BIT6 | BIT5 | BIT4);     // Set P6.6-4 as OUTPUT
     P6OUT &= ~(BIT6 | BIT5 | BIT4);    // Clear P6.6-4

     P4SEL1 &= ~BIT7;            // P4.7 SCL
     P4SEL0 |= BIT7;


     P4SEL1 &= ~BIT6;            // P4.6 SDA
     P4SEL0 |= BIT6;

     PM5CTL0 &= ~LOCKLPM5;       // Turn on I/O
     UCB1CTLW0 &= ~UCSWRST;      // SW RESET OFF

}

void initTimerB0compare(){      // Setup TB0
    TB0CTL |= TBCLR;            // Clear TB0
    TB0CTL |= TBSSEL__ACLK;     // Select SMCLK
    TB0CTL |= MC__UP;           // UP mode
}

void initTimerB1compare() {
    TB1CTL |= TBCLR;            // Clear TB1
    TB1CTL |= TBSSEL__SMCLK;    // Select SMCLK
    TB1CTL |= MC__UP;           // UP mode
    TB1CCR0 = 347985;           // Set CCR0 value (read sensor every 333ms)
    TB1CCTL0 &= ~CCIFG;         // Clear TB1 flag
    TB1CCTL0 |= CCIE;           // Enable TB1 interrupt
}

void configureAdc() {

    SYSCFG2 |= 0x0001 << ADCINCH_8;         // Configure ADC A8 pin (P5.0)
    ADCCTL0 &= ~ADCENC;                     // Disable ADC
    ADCCTL0 |= ADCSHT_2 | ADCON;            // ADC ON, 16 clocks
    ADCCTL1 = ADCSHP;                       // Sampling timer
    ADCCTL2 = ADCRES;                       // 10 bit resolution
    ADCMCTL0 = ADCINCH_8 | ADCSREF_0;       // ADC A8 input select, Vref of AVCC
    ADCIE = ADCIE0;                         // Enable ADC

}

void initLEDs() {
    P2DIR |= (BIT0 | BIT1 | BIT2 | BIT3);
    P2OUT &= ~(BIT0 | BIT1 | BIT2 | BIT3);

    P4DIR |= BIT0;
    P4OUT &= ~BIT0;
}

void columnInput(){
    P3DIR &= ~(BIT0 | BIT1 | BIT2 | BIT3);  // Initialize pins as input
    P3REN |= (BIT0 | BIT1 | BIT2 | BIT3);   // Enable pull up/down resistor
    P3OUT &= ~(BIT0 | BIT1 | BIT2 | BIT3);  // Configure resistors as pull down

    P3DIR |= (BIT4 | BIT5 | BIT6 | BIT7);   // Init pins as outputs
    P3OUT |= (BIT4 | BIT5 | BIT6 | BIT7);   // Set as outputs

    P3IES &= ~(BIT0 | BIT1 | BIT2 | BIT3);  // L-H edge sensitivity
    P3IE |= (BIT0 | BIT1 | BIT2 | BIT3);    // Enable IRQs

    P3IFG &= ~(BIT0 | BIT1 | BIT2 | BIT3);  // Clear the P3 interrupt flags
}

void rowInput(){
    P3DIR &= ~(BIT4 | BIT5 | BIT6 | BIT7);  // Initialize pins as input
    P3REN |= (BIT4 | BIT5 | BIT6 | BIT7);   // Enable pull up/down resistor
    P3OUT &= ~(BIT4 | BIT5 | BIT6 | BIT7);  // Configure resistors as pull down

    P3DIR |= (BIT0 | BIT1 | BIT2 | BIT3);   // Init pins as outputs
    P3OUT |= (BIT0 | BIT1 | BIT2 | BIT3);   // Set as outputs
}


void delay1000() {
    int i;
    for(i = 0; i <= 1000; i++){}
}

void ADC_start() {
    ADCCTL0 |= ADCENC | ADCSC;              // Start sampling and conversion
    __bis_SR_register(LPM3_bits | GIE);     // Enter LPM0, ADC_ISR force exits
}

void ADC_stop() {
    ADCCTL0 &= ~(ADCENC | ADCON);           // Stop sampling
}

void sampleSensor() {           // Shift stack of measured values right one, then record new temperature from sensor using ADC

    if(sampleCount >= 8) {
        ADC[8] = ADC[7];
    }
    if(sampleCount >= 7) {
        ADC[7] = ADC[6];
    }
    if(sampleCount >= 6) {
        ADC[6] = ADC[5];
    }
    if(sampleCount >= 5) {
        ADC[5] = ADC[4];
    }
    if(sampleCount >= 4) {
        ADC[4] = ADC[3];
    }
    if(sampleCount >= 3) {
        ADC[3] = ADC[2];
    }
    if(sampleCount >= 2) {
        ADC[2] = ADC[1];
    }
    if(sampleCount >= 1) {
        ADC[1] = ADC[0];
    }

    delay1000();                    // Wait for ADC ref
    ADC_start();                    // Start sample from ADC
    ADC_stop();                     // Stop sample from ADC

    ADC[0] = ADCMEM0;               // Store ADCMEM0 resistance measurement in first stack ADC[0]

}


int getCharKey(int d) {
    switch(d) {                     // Set transmit key based on digit
        case 1:
            return 0x01;
            break;
        case 2:
            return 0x02;
            break;
        case 3:
            return 0x03;
            break;
        case 4:
            return 0x04;
            break;
        case 5:
            return 0x05;
            break;
        case 6:
            return 0x06;
            break;
        case 7:
            return 0x07;
            break;
        case 8:
            return 0x08;
            break;
        case 9:
            return 0x09;
            break;
        case 0:
            return 0x00;
            break;
        default:
            return 0;
            break;
    }
}

void getWaterLevel() {

    int digit;

    /*if(avg >= 0 && avg <= 100) {                    // No water detected
        key = 0x0B;
    } else if(avg > 100 && avg <= 300) {            // Low water level
        key = 0x0C;
    } else if(avg > 300 && avg <= 360) {            // Mid water level
        key = 0x0D;
    } else if(avg > 360 && avg <= 400) {            // High water level
        key = 0x0E;
    } else if(avg > 400) {                          // Full water level
        key = 0x0F;
    }
    */

    initLEDs();

    if(avg >= 0 && avg <= 10) {
        key = 0x0B;
        P2OUT |= BIT0;
    } else if(avg > 10 && avg <= 200) {
        key = 0x0C;
        P2OUT |= BIT1;
    } else if(avg > 200 && avg <= 325) {
        key = 0x0D;
        P2OUT |= BIT2;
    } else if(avg > 325 && avg <= 400) {
        key = 0x0E;
        P2OUT |= BIT3;
    } else if(avg > 400){
        key = 0x0F;
        P4OUT |= BIT0;
    }

    packet[1] = key;                                // Set second byte in packet to indicate water level

    for(i = 2; i < 5; i++) {                        // Get each digit in average measurement value (3 digit number)
        if(i == 2) {
            digit = avg / 100;
            avg = avg - digit*100;
        } else if(i == 3) {
            digit = avg / 10;
            avg = avg - digit*10;
        } else if(i == 4) {
            digit = avg;
            avg = 0;
        }
        packet[i] = getCharKey(digit);              // Store each digit in the packet
    }
}


void getAverage() {
    if(sampleCount > n - 1  && n > 0) {              // Transmit sensor average if enough samples have been recorded for desired window size

        for(i = 0; i < n; i++) {                     // Calculate moving average for window size n
            avg = avg + ADC[i];
        }
        unrounded_avg = avg / n;
        avg = round(unrounded_avg);
        getWaterLevel();                            // Determine water level and put values into packet
    }

   sample = 0;                                          // Reset sample indicator
}


int main(void) {

    WDTCTL = WDTPW | WDTHOLD;           // Stop watchdog timer

    initI2C_master();                   // Intialize master for I2C transmission
    initTimerB0compare();               // Initialize Timer B0 for getting keypad input

    initTimerB1compare();               // Initialize Timer B1 for sampling temperature sensor every 333ms
    configureAdc();                     // Initialize ADC for receiving input signal from temperature sensor

    initLEDs();

    columnInput();                      // Configure keypad columns to start as inputs

    PM5CTL0 &= ~LOCKLPM5;               // Turn on Digital I/O

    UCB1I2CSA = 0x0068;                 // Set slave address
    UCB1IE |= UCTXIE0;                  // Enable I2C TX interrupt

    __enable_interrupt();

    int i,k;
    while(1){

        UCB1CTLW0 |= UCTXSTT;           // Generate START condition
        for(k=0; k<=20;k++){
            for(i=0;i<=10000;i++){}
        }

    }

    return 0;
}

void displayUnlockPattern() {
    int i;
    initLEDs();
    for(i = 0; i <= 32000; i++) {}
    P2OUT |= (BIT0 | BIT1 | BIT2 | BIT3);
    P4OUT |= BIT0;
    for(i = 0; i <= 32000; i++) {}
    initLEDs();

    packet[0] = 0x14;               // Set first byte in packet to indicate unlocked (Locked: 0x14 - N, 0x12 - Y)
}

int is_unlocked(char k){            // Password: *#06*
    if(lock_state == 0){
        return 1;
    } else {
        if(lock_state == 1){
            if(k == 0x17){
                P2OUT |= BIT0;
                lock_state = 2;
            }
        } else if(lock_state == 2){
            if(k == 0x11){
                P2OUT |= BIT1;
                lock_state = 3;
            }
        } else if(lock_state == 3){
            if(k == 0x13){
                P2OUT |= BIT2;
                lock_state = 4;
            }
        } else if(lock_state == 4) {
            if(k == 0x41) {
                P2OUT |= BIT3;
                lock_state = 5;
            }
        } else if(lock_state == 5) {
            if(k == 0x17) {
                P4OUT |= BIT0;
                lock_state = 0;
                displayUnlockPattern();
            }
        }
        return 0;
    }
}


void keyPressedAction(char pressed_key) {
    if(is_unlocked(pressed_key) == 1) {
        switch(pressed_key) {                   // Get pressed key
            case 0x87:
                key = 0x01;     // 1
                n = 1;
                break;
            case 0x83:
                key = 0x02;     // 2
                n = 2;
                break;
            case 0x81:
                key = 0x03;     // 3
                n = 3;
                break;
            case 0x80:
                key = 0x80;     // A
                break;
            case 0x47:
                key = 0x04;     // 4
                n = 4;
                break;
            case 0x43:
                key = 0x05;     // 5
                n = 5;
                break;
            case 0x41:
                key = 0x06;     // 6
                n = 6;
                break;
            case 0x40:
                key = 0x40;     // B
                break;
            case 0x27:
                key = 0x07;     // 7
                n = 7;
                break;
            case 0x23:
                key = 0x08;     // 8
                n = 8;
                break;
            case 0x21:
                key = 0x09;     // 9
                n = 9;
                break;
            case 0x20:
                key = 0x20;     // C
                break;
            case 0x17:
                key = 0x17;     // *
                break;
            case 0x13:
                key = 0x00;     // 0
                n = 0;
                break;
            case 0x11:
                key = 0x11;     // #
                break;
            case 0x10:
                key = 0x10;     // D
                break;
            default:
                break;
        }

    }

    columnInput();                              // Reset keypad columns to be inputs
    P3IFG &= ~(BIT0 | BIT1 | BIT2 | BIT3);      // Clear the P3 interrupt flags

}

#pragma vector=PORT3_VECTOR
__interrupt void ISR_PORT3(void){           // Enable timer for debouncing
    TB0CCTL0 |= CCIE;                       // Local IRQ enable for CCR0
    TB0CCR0 = 500;                          // Set CCR0 value (period) // old value = 2384
    TB0CCTL0 &= ~CCIFG;                     // Clear CCR0 flag to begin count
}

#pragma vector=TIMER0_B0_VECTOR
__interrupt void ISR_TB0_CCR0(void){

    TB0CCTL0 &= ~CCIE;      // Disable TimerB0
    UCB1IE &= ~UCTXIE0;     // Disable I2C B0 TX interrupt

    col_holding = P3IN;

    rowInput();

    row_holding = P3IN;
    pressed_key = col_holding + row_holding;

    keyPressedAction(pressed_key);

    UCB1IE |= UCTXIE0;      // Enable I2C B0 TX interrupt

}


#pragma vector = TIMER1_B0_VECTOR
__interrupt void ISR_TB1_CCR0(void) {
    TB1CCTL0 &= ~CCIE;                                      // Disable TB1
    sample = 1;                                             // Set sample indicator

    tcnt++;
    if(tcnt == 25){
        sampleCount++;
        sampleSensor();                                     // Sample temperature sensor
        getAverage();                                       // Calculate moving average window

        tcnt = 0;
    }

    TB1CCTL0 &= ~CCIFG;                                     // Clear flag
    TB1CCTL0 |= CCIE;                                       // Enable TB1
}

#pragma vector = ADC_VECTOR
__interrupt void ISR_ADC(void) {
    switch(__even_in_range(ADCIV, ADCIV_ADCIFG)) {
        case ADCIV_ADCIFG:
            ADCIFG &= ~ADCIFG0;                             // Clear ADC flag
            __bic_SR_register_on_exit(LPM3_bits + GIE);     // Exit low power mode
            break;
        default:
            break;
    }
}

#pragma vector=EUSCI_B1_VECTOR
__interrupt void EUSCI_B1_TX_ISR(void){                     // Fill TX buffer with packet values
    if (j == (sizeof(packet)-1)){
        UCB1TXBUF = packet[j];
        j = 0;
    } else {
        UCB1TXBUF = packet[j];
        j++;
    }
}
