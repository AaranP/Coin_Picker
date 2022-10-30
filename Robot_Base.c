#include <XC.h>
#include <sys/attribs.h>
#include <stdio.h>
#include <stdlib.h>
 
// Configuration Bits (somehow XC32 takes care of this)
#pragma config FNOSC = FRCPLL       // Internal Fast RC oscillator (8 MHz) w/ PLL
#pragma config FPLLIDIV = DIV_2     // Divide FRC before PLL (now 4 MHz)
#pragma config FPLLMUL = MUL_20     // PLL Multiply (now 80 MHz)
#pragma config FPLLODIV = DIV_2     // Divide After PLL (now 40 MHz) 
#pragma config FWDTEN = OFF         // Watchdog Timer Disabled
#pragma config FPBDIV = DIV_1       // PBCLK = SYCLK
#pragma config FSOSCEN = OFF        // Turn off secondary oscillator on A4 and B4

// Defines
#define SYSCLK 40000000L
#define FREQ 100000L // We need the ISR for timer 1 every 10 us
#define Baud2BRG(desired_baud)( (SYSCLK / (16*desired_baud))-1)


volatile int ISR_pwm1=250, ISR_pwm2=155, ISR_cnt=0;

// The Interrupt Service Routine for timer 1 is used to generate one or more standard
// hobby servo signals.  The servo signal has a fixed period of 20ms and a pulse width
// between 0.6ms and 2.4ms.
void __ISR(_TIMER_1_VECTOR, IPL5SOFT) Timer1_Handler(void)
{
	
	IFS0CLR=_IFS0_T1IF_MASK; // Clear timer 1 interrupt flag, bit 4 of IFS0

	ISR_cnt++;
	if(ISR_cnt==ISR_pwm1)
	{
		LATAbits.LATA2 = 0;
	}
	if(ISR_cnt==ISR_pwm2)
	{
		LATAbits.LATA3 = 0;
	}
	if(ISR_cnt>=2000)
	{
		ISR_cnt=0; // 2000 * 10us=20ms
		LATAbits.LATA2 = 1;
		LATAbits.LATA3 = 1;
	}
}

void SetupTimer1 (void)
{
	// Explanation here: https://www.youtube.com/watch?v=bu6TTZHnMPY
	__builtin_disable_interrupts();
	PR1 =(SYSCLK/FREQ)-1; // since SYSCLK/FREQ = PS*(PR1+1)
	TMR1 = 0;
	T1CONbits.TCKPS = 0; // 3=1:256 prescale value, 2=1:64 prescale value, 1=1:8 prescale value, 0=1:1 prescale value
	T1CONbits.TCS = 0; // Clock source
	T1CONbits.ON = 1;
	IPC1bits.T1IP = 5;
	IPC1bits.T1IS = 0;
	IFS0bits.T1IF = 0;
	IEC0bits.T1IE = 1;
	
	INTCONbits.MVEC = 1; //Int multi-vector
	__builtin_enable_interrupts();
}

void __ISR(_TIMER_2_VECTOR, IPL5SOFT) Timer2_Handler(void)
{
	if(LATAbits.LATA4 == 1)
		LATAbits.LATA4 = 0;
	else
		LATAbits.LATA4 = 1;
	IFS0bits.T2IF=0; // Clear timer 1 interrupt flag, bit 4 of IFS0


}

void SetupTimer2 (void)
{

	__builtin_disable_interrupts();
	PR2 = SYSCLK/(256*100) -1;
	TMR2 = 0;
	T2CONbits.TCKPS = 3; // 3=1:256 prescale value, 2=1:64 prescale value, 1=1:8 prescale value, 0=1:1 prescale value
	T2CONbits.TCS = 0; // Clock source
	T2CONbits.ON = 0;
	IPC2bits.T2IP = 5;
	IPC2bits.T2IS = 0;
	IFS0bits.T2IF = 0;
	IEC0bits.T2IE = 1;
	
	INTCONbits.MVEC = 1; //Int multi-vector
	__builtin_enable_interrupts();
}

// Use the core timer to wait for 1 ms.
void wait_1ms(void)
{
    unsigned int ui;
    _CP0_SET_COUNT(0); // resets the core timer count

    // get the core timer count
    while ( _CP0_GET_COUNT() < (SYSCLK/(2*1000)) );
}

void waitms(int len)
{
	while(len--) wait_1ms();
}

#define PIN_PERIOD (PORTB&(1<<5))

// GetPeriod() seems to work fine for frequencies between 200Hz and 700kHz.
long int GetPeriod (int n)
{
	int i;
	unsigned int saved_TCNT1a, saved_TCNT1b;
	
    _CP0_SET_COUNT(0); // resets the core timer count
	while (PIN_PERIOD!=0) // Wait for square wave to be 0
	{
		if(_CP0_GET_COUNT() > (SYSCLK/4)) return 0;
	}

    _CP0_SET_COUNT(0); // resets the core timer count
	while (PIN_PERIOD==0) // Wait for square wave to be 1
	{
		if(_CP0_GET_COUNT() > (SYSCLK/4)) return 0;
	}
	
    _CP0_SET_COUNT(0); // resets the core timer count
	for(i=0; i<n; i++) // Measure the time of 'n' periods
	{
		while (PIN_PERIOD!=0) // Wait for square wave to be 0
		{
			if(_CP0_GET_COUNT() > (SYSCLK/4)) return 0;
		}
		while (PIN_PERIOD==0) // Wait for square wave to be 1
		{
			if(_CP0_GET_COUNT() > (SYSCLK/4)) return 0;
		}
	}

	return  _CP0_GET_COUNT();
}
 
void UART2Configure(int baud_rate)
{
    // Peripheral Pin Select
    U2RXRbits.U2RXR = 4;    //SET RX to RB8
    RPB9Rbits.RPB9R = 2;    //SET RB9 to TX

    U2MODE = 0;         // disable autobaud, TX and RX enabled only, 8N1, idle=HIGH
    U2STA = 0x1400;     // enable TX and RX
    U2BRG = Baud2BRG(baud_rate); // U2BRG = (FPb / (16*baud)) - 1
    
    U2MODESET = 0x8000;     // enable UART2
}

void uart_puts(char * s)
{
	while(*s)
	{
		putchar(*s);
		s++;
	}
}

char HexDigit[]="0123456789ABCDEF";
void PrintNumber(long int val, int Base, int digits)
{ 
	int j;
	#define NBITS 32
	char buff[NBITS+1];
	buff[NBITS]=0;

	j=NBITS-1;
	while ( (val>0) | (digits>0) )
	{
		buff[j--]=HexDigit[val%Base];
		val/=Base;
		if(digits!=0) digits--;
	}
	uart_puts(&buff[j+1]);
}

// Good information about ADC in PIC32 found here:
// http://umassamherstm5.org/tech-tutorials/pic32-tutorials/pic32mx220-tutorials/adc
void ADCConf(void)
{
    AD1CON1CLR = 0x8000;    // disable ADC before configuration
    AD1CON1 = 0x00E0;       // internal counter ends sampling and starts conversion (auto-convert), manual sample
    AD1CON2 = 0;            // AD1CON2<15:13> set voltage reference to pins AVSS/AVDD
    AD1CON3 = 0x0f01;       // TAD = 4*TPB, acquisition time = 15*TAD 
    AD1CON1SET=0x8000;      // Enable ADC
}

int ADCRead(char analogPIN)
{
    AD1CHS = analogPIN << 16;    // AD1CHS<16:19> controls which analog pin goes to the ADC
 
    AD1CON1bits.SAMP = 1;        // Begin sampling
    while(AD1CON1bits.SAMP);     // wait until acquisition is done
    while(!AD1CON1bits.DONE);    // wait until conversion done
 
    return ADC1BUF0;             // result stored in ADC1BUF0
}

void ConfigurePins(void)
{
    // Configure pins as analog inputs
    ANSELBbits.ANSB2 = 1;   // set RB2 (AN4, pin 6 of DIP28) as analog pin
    TRISBbits.TRISB2 = 1;   // set RB2 as an input
    ANSELBbits.ANSB3 = 1;   // set RB3 (AN5, pin 7 of DIP28) as analog pin
    TRISBbits.TRISB3 = 1;   // set RB3 as an input
    
	// Configure digital input pin to measure signal period
	ANSELB &= ~(1<<5); // Set RB5 as a digital I/O (pin 14 of DIP28)
    TRISB |= (1<<5);   // configure pin RB5 as input
    CNPUB |= (1<<5);   // Enable pull-up resistor for RB5
    
    // Configure output pins
    //H-bridge 1
	TRISAbits.TRISA0 = 0; // pin  2 of DIP28
	TRISAbits.TRISA1 = 0; // pin  3 of DIP28
	//H-bridge 2
	TRISBbits.TRISB0 = 0; // pin  4 of DIP28
	TRISBbits.TRISB1 = 0; // pin  5 of DIP28
	//servo
	TRISAbits.TRISA2 = 0; // pin  9 of DIP28
	TRISAbits.TRISA3 = 0; // pin 10 of DIP28
	//electromagnet
	TRISBbits.TRISB4 = 0; // pin 11 of DIP28
	//speaker
	TRISAbits.TRISA4 = 0; // pin 12 of DIP28
	
	
	
	
	INTCONbits.MVEC = 1;
}

void pickCoin (void){
	LATBbits.LATB4 = 1;
	waitms(1000);
	LATAbits.LATA4 =1;
	//sweep
	ISR_pwm2=160;
	waitms(200);
	ISR_pwm2=170;
	waitms(200);
	ISR_pwm2=180;
	waitms(200);
	ISR_pwm2=190;
	waitms(200);
	ISR_pwm2=200;
	waitms(200);
	ISR_pwm2=210;
	waitms(200);
	ISR_pwm2=220;
	waitms(200);
	ISR_pwm2=210;
	waitms(200);
	ISR_pwm2=200;
	waitms(200);
	ISR_pwm2=190;
	waitms(200);
	ISR_pwm2=180;
	waitms(200);
	ISR_pwm2=170;
	waitms(200);
	ISR_pwm2=160;
	waitms(200);
	ISR_pwm2=150;
	waitms(200);
	ISR_pwm2=140;
	waitms(200); 
	ISR_pwm2=130;
	waitms(200);
	ISR_pwm2=120;
	waitms(200);
	ISR_pwm2=130;
	waitms(200);
	ISR_pwm2=140;
	waitms(200);
	ISR_pwm2=150;
	waitms(200);
	ISR_pwm2=160;
	waitms(200);
	
	
	//pwm1:90

	ISR_pwm1=250;
	waitms(200);
	ISR_pwm1=240;
	waitms(200);
	ISR_pwm1=230;
	waitms(200);
	ISR_pwm1=220;
	waitms(200);
	ISR_pwm1=210;
	waitms(200);
	ISR_pwm1=200;
	waitms(200);
	ISR_pwm1=190;
	waitms(200);
	ISR_pwm1=180;
	waitms(200);
	ISR_pwm1=170;
	waitms(200);
	ISR_pwm1=160;
	waitms(200);
	ISR_pwm1=150;
	waitms(200);
	ISR_pwm1=140;
	waitms(100);
	ISR_pwm1=130;
	waitms(200);
	ISR_pwm1=120;
	waitms(200);
	ISR_pwm1=110;
	waitms(200);
	ISR_pwm1=100;
	waitms(200);
	ISR_pwm1=90;
	waitms(200);
	
	//pwm2:80
	ISR_pwm2=150;	
	waitms(200);
	ISR_pwm2=140;	
	waitms(200);
	ISR_pwm2=130;	
	waitms(200);
	ISR_pwm2=120;	
	waitms(200);
	ISR_pwm2=110;	
	waitms(200);
	ISR_pwm2=100;	
	waitms(200);
	ISR_pwm2=90;	
	waitms(200);
	ISR_pwm2=80;	
	waitms(200);
	
	
	LATBbits.LATB4 = 0;
	waitms(1000);
		LATAbits.LATA4 =0;
		
	ISR_pwm2=155;
	waitms(1000);
	ISR_pwm1=260;
	waitms(1000);	
}

void goFowards(void){

	//H-bridge 1
	LATAbits.LATA0 = 1;	
	LATAbits.LATA1 = 0;
	
	//H-bridge 2			
	LATBbits.LATB0 = 1;			
	LATBbits.LATB1 = 0;		
	
}

void goBackwards(void){

	//H-bridge 1
	LATAbits.LATA0 = 0;	
	LATAbits.LATA1 = 1;
	
	//H-bridge 2			
	LATBbits.LATB0 = 0;			
	LATBbits.LATB1 = 1;		
	
}

void goRight(void){

	//H-bridge 1
	LATAbits.LATA0 = 1;	
	LATAbits.LATA1 = 0;
	
	//H-bridge 2			
	LATBbits.LATB0 = 0;			
	LATBbits.LATB1 = 1;		
	
}
void goLeft(void){

	//H-bridge 1
	LATAbits.LATA0 = 0;	
	LATAbits.LATA1 = 1;
	
	//H-bridge 2			
	LATBbits.LATB0 = 1;			
	LATBbits.LATB1 = 0;		
	
}

void stop(void){

	//H-bridge 1
	LATAbits.LATA0 = 0;	
	LATAbits.LATA1 = 0;
	
	//H-bridge 2			
	LATBbits.LATB0 = 0;			
	LATBbits.LATB1 = 0;		
	
}
void song(void){
		PR2 = SYSCLK/(256*110) -1;
	T2CONbits.ON = 1;
	waitms(150);
	T2CONbits.ON = 0;
	waitms(100);
	PR2 = SYSCLK/(256*110) -1;
	T2CONbits.ON = 1;
	waitms(150);
	T2CONbits.ON = 0;
	waitms(200);
	PR2 = SYSCLK/(256*110) -1;
	T2CONbits.ON = 1;
	waitms(400);
	T2CONbits.ON = 0;

	PR2 = SYSCLK/(256*87) -1;
	T2CONbits.ON = 1;
	waitms(200);
	T2CONbits.ON = 0;
	
	PR2 = SYSCLK/(256*110) -1;
	T2CONbits.ON = 1;
	waitms(500);
	T2CONbits.ON = 0;
	waitms(100);
	PR2 = SYSCLK/(256*130) -1;
	T2CONbits.ON = 1;
	waitms(1100);
	T2CONbits.ON = 0;
	waitms(100);
	
	PR2 = SYSCLK/(256*65) -1;
	T2CONbits.ON = 1;
	waitms(1100);
	T2CONbits.ON = 0;
	waitms(100);
}

// In order to keep this as nimble as possible, avoid
// using floating point or printf() on any of its forms!
void main(void)
{
	volatile unsigned long t=0;
    int adcval1, adcval2;
    //perimeter threshold voltage
    int thresholdV = 500;
    int remainder;
    //threshold period for metal detector;
    int thresholdP;
    long int v1, v2;
	unsigned long int count, f;
	unsigned char LED_toggle=0;
	int counter = 0;

	CFGCON = 0;
  
    UART2Configure(115200);  // Configure UART2 for a baud rate of 115200
    ConfigurePins();
    SetupTimer1();
    SetupTimer2();
  	stop();
    ADCConf(); // Configure ADC
   /* 
    waitms(500); // Give PuTTY time to start
	uart_puts("\x1b[2J\x1b[1;1H"); // Clear screen using ANSI escape sequence.
	uart_puts("\r\nPIC32 multi I/O example.\r\n");
	uart_puts("Measures the voltage at channels 4 and 5 (pins 6 and 7 of DIP28 package)\r\n");
	uart_puts("Measures period on RB5 (pin 14 of DIP28 package)\r\n");
	uart_puts("Toggles RA0, RA1, RB0, RB1, RA2 (pins 2, 3, 4, 5, 9, of DIP28 package)\r\n");
	uart_puts("Generates Servo PWM signals at RA3, RB4 (pins 10, 11 of DIP28 package)\r\n\r\n");
	*/
	LATBbits.LATB4 = 0;
	
	
	//measure baseline freq 
	waitms(500);
	count=GetPeriod(100);
	f=((SYSCLK/2L)*100L)/count;
	thresholdP = f + 100;

	while( counter < 1)
	{
	
		//gofowards
		goFowards();
		//pickCoin();
    	adcval1 = ADCRead(4); // note that we call pin AN4 (RB2) by it's analog number
		uart_puts("ADC[4]=0x");
		PrintNumber(adcval1, 16, 3);
		uart_puts(", V=");
		v1=(adcval1*3290L)/1023L; // 3.290 is VDD
		
		PrintNumber(v1/1000, 10, 1);
		uart_puts(".");
		PrintNumber(v1%1000, 10, 3);
		uart_puts("V ");

		adcval2=ADCRead(5);
		uart_puts("ADC[5]=0x");
		PrintNumber(adcval2, 16, 3);
		uart_puts(", V=");
		v2=(adcval2*3290L)/1023L; // 3.290 is VDD
		PrintNumber(v2/1000, 10, 1);
		uart_puts(".");
		PrintNumber(v2%1000, 10, 3);
		uart_puts("V ");

		//perimeter code
		if (v1 > thresholdV || v2 > thresholdV){
            if (adcval1%2==0) {
            
                remainder = 1;
                
           }
           else {
            
                remainder = 0;
                
            }
            
            goBackwards();            
            waitms(500);

            if (remainder) {
            	goRight();
                waitms(400);
           }
            else {
            	goRight();
                waitms(600);
                
            }
            
           goFowards();
           //waitms(1000);
        }


		count=GetPeriod(100);
		if(count>0)
		{
			f=((SYSCLK/2L)*100L)/count;
			uart_puts("f=");
			PrintNumber(f, 10, 7);
			uart_puts("Hz, count=");
			PrintNumber(count, 10, 6);
			uart_puts("          \r");
		}
		else
		{
			uart_puts("NO SIGNAL                     \r");
		}
		
		if(f > thresholdP) {
			waitms(100);
			count=GetPeriod(100);
			f=((SYSCLK/2L)*100L)/count;
			if(f > thresholdP){
			
				goBackwards();
				waitms(250);
				stop();
				pickCoin();
				//increment counter;
				counter++;
				//measure baseline freq 
				waitms(500);
				count=GetPeriod(100);
				f=((SYSCLK/2L)*100L)/count;
				thresholdP = f + 120;
			}

		}

		 	

	}
	
	song();
	while(1){
		goRight();
		waitms(1000);
		goLeft();
		waitms(500);
		goRight();
		waitms(2000);
		goLeft();
		waitms(1000);
		goRight();
		waitms(500);
		goLeft();
		waitms(2000);
	}
}

