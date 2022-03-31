// This program converts the analog input from channel 5.
// Channel 5 is configured as single-ended input from PA05 (pin 6).
// External reference A is used from PA03 (pin 4), that should be connected to 3.3V. 

#include "samd20.h"
#include "nvm_data.h"

#define PE 4  // Prescaler as configured in REC_TC3_CTRLA
#define RATE 100000 // in Hz.  We want to interrupt every 10us
#define TC3_RELOAD (((F_CPU/PE)/RATE)-1)

#if (TC3_RELOAD > 255)
#error TC3_RELOAD is greater than 255
#endif

volatile int ISR_pwm1=150, ISR_pwm2=150, ISR_cnt=0;

unsigned char* ARRAY_PORT_PINCFG0 = (unsigned char*)&REG_PORT_PINCFG0;
unsigned char* ARRAY_PORT_PMUX0 = (unsigned char*)&REG_PORT_PMUX0;

void init_Clock48();
void UART3_init(uint32_t baud);
void printString (char * s);
void printNum(uint32_t v, int base, int digits);

void Configure_TC3 (void)
{
    __disable_irq();
    // Configure Clocks
    REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TC2_TC3;
    REG_PM_APBCMASK |= PM_APBCMASK_TC3; // Enable TC3 bus clock
    REG_TC3_CTRLA = 1;              // reset TC3 before configuration
    while (REG_TC3_CTRLA & 1);      // wait till out of reset
    REG_TC3_CTRLA = 0x0204;         // prescaler /64, 8-bit mode, NFRQ. Check page 681 of datasheet
    REG_TC3_COUNT8_PER=TC3_RELOAD;  // TOP count value in 8-bit mode
    REG_TC3_CTRLA |= 2;             // enable TC3
    REG_TC3_INTENSET = 1;           // enable overflow interrupt
    NVIC_EnableIRQ(TC3_IRQn);       // enable TC3 interrupt in NVIC
    __enable_irq();                 // enable interrupt globally

}

void TC3_Handler(void)
{
    REG_TC3_INTFLAG = 1; // clear OVF flag
    
	ISR_cnt++;
	if(ISR_cnt==ISR_pwm1)
	{
		REG_PORT_OUTCLR0 = PORT_PA08;
	}
	if(ISR_cnt==ISR_pwm2)
	{
		REG_PORT_OUTCLR0 = PORT_PA09;
	}
	if(ISR_cnt>=2000)
	{
		ISR_cnt=0; // 2000 * 10us=20ms
		REG_PORT_OUTSET0 = PORT_PA08;
		REG_PORT_OUTSET0 = PORT_PA09;
	}	
}

// This function read the period of the signal connecte to port PA14 (pin 15 of QFP32)
uint32_t GetPeriod (int n)
{
    int i;
    // Configure SysTick
    SysTick->LOAD = 0xffffff; // Reload with max number of clocks (SysTick is 24-bit)
    SysTick->VAL = 0;         // clear current value register
    SysTick->CTRL = 0x5;      // Enable the timer

    while ((REG_PORT_IN0 & PORT_PA15)!=0) // Wait for zero
    {
    	if (SysTick->CTRL & 0x10000) return 0;
    }
    while ((REG_PORT_IN0 & PORT_PA15)==0) // Wait for one
    {
    	if (SysTick->CTRL & 0x10000) return 0;
    }
    SysTick->CTRL = 0; // Stop the timer (Enable = 0)


    // Configure SysTick again
    SysTick->LOAD = 0xffffff;  // Reload with max number of clocks (SysTick is 24-bit)
    SysTick->VAL = 0;          // clear current value register
    SysTick->CTRL = 0x5;       // Enable the timer

    for(i = 0; i < n; i++)
    {
	    while ((REG_PORT_IN0 & PORT_PA15)!=0)
	    {
	    	if (SysTick->CTRL & 0x10000) return 0;
	    }
	    while ((REG_PORT_IN0 & PORT_PA15)==0)
	    {
	    	if (SysTick->CTRL & 0x10000) return 0;
	    }
    }
    SysTick->CTRL = 0; // Stop the timer (Enable = 0)
    
    return (0xffffff-SysTick->VAL);
}

void delayMs(int n)
{
    int i;
    // Configure SysTick
    SysTick->LOAD = (F_CPU/1000L) - 1; // Reload with number of clocks per millisecond
    SysTick->VAL = 0;         // clear current value register
    SysTick->CTRL = 0x5;      // Enable the timer

    for(i = 0; i < n; i++)
    {
        while((SysTick->CTRL & 0x10000) == 0); // wait until the COUNTFLAG is set
    }
    SysTick->CTRL = 0; // Stop the timer (Enable = 0)
}

void ADC_init (void)
{
	PM->APBCMASK.reg |= PM_APBCMASK_ADC; // enable bus clock for ADC
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(ADC_GCLK_ID) |	GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN(0); // GCLK0 to ADC
	
    REG_ADC_SAMPCTRL = 10;       // sampling time 10 clocks
      
    ARRAY_PORT_PINCFG0[3] |= 1; // Use PMUX for PA03
    ARRAY_PORT_PMUX0[1] = 0x10; // PA03 = VREFA
    REG_ADC_REFCTRL = 3;        // Use VREFA
    
    //REG_ADC_REFCTRL = 0x80 | 0x02; // Reference buffer offset compensation is enabled; Reference Selection=VDDANA/2
    REG_ADC_CTRLB |= 0x0700; // clock pre-scaler is: Peripheral clock divided by 512
    
    REG_ADC_INPUTCTRL = 0x1805; // V- = GND; V+ = AIN5
    ARRAY_PORT_PINCFG0[4] |= 1; // Use PMUX for PA04
    ARRAY_PORT_PINCFG0[5] |= 1; // Use PMUX for PA05
    ARRAY_PORT_PMUX0[2] = 0x11; // PA04 = AIN4, PA05 = AIN5 (low nibble is for AIN4, high nibble is for AIN5)
    
    REG_ADC_CALIB = ADC_CALIB_BIAS_CAL(NVM_READ_CAL(ADC_BIASCAL)) |
                    ADC_CALIB_LINEARITY_CAL(NVM_READ_CAL(ADC_LINEARITY));

    REG_ADC_CTRLA = 2;          // enable ADC
}

int ADC_read (unsigned int channel)
{
    int result;

    REG_ADC_INPUTCTRL = 0x1800 | channel; // V- = GND; V+ = channel (either 4 or 5 as configured above)
    
    REG_ADC_SWTRIG = 2;             // start a conversion
    while(!(REG_ADC_INTFLAG & 1));  // wait for conversion complete
    result = REG_ADC_RESULT;        // read conversion result
    
    return result;
}

void ConfigurePins (void)
{
	// Configure input pins
    REG_PORT_DIRCLR0 = PORT_PA15; // Period surbroutine input pin
    ARRAY_PORT_PINCFG0[15] |= 6;  // enable PA15 input buffer with pull
    REG_PORT_OUTSET0 = PORT_PA15; // PA15 pull-up
    
    // Configure output pins
    REG_PORT_DIRSET0 = PORT_PA00; // Configure PA00 as output.  This is pin 1 of the LQFP32 package.
    REG_PORT_DIRSET0 = PORT_PA01; // Configure PA01 as output.  This is pin 2 of the LQFP32 package.
    REG_PORT_DIRSET0 = PORT_PA02; // Configure PA02 as output.  This is pin 3 of the LQFP32 package.
    REG_PORT_DIRSET0 = PORT_PA06; // Configure PA06 as output.  This is pin 7 of the LQFP32 package.
    REG_PORT_DIRSET0 = PORT_PA07; // Configure PA07 as output.  This is pin 8 of the LQFP32 package.
    REG_PORT_DIRSET0 = PORT_PA08; // Configure PA08 as output.  This is pin 11 of the LQFP32 package.
    REG_PORT_DIRSET0 = PORT_PA09; // Configure PA09 as output.  This is pin 12 of the LQFP32 package.

	REG_PORT_DIRSET0 = PORT_PA
}
void coincheck (void)
{
	while (ADC_read(4) <3.5){

	}
}
void move_robot(void)
{
	// motor1 : pin 11 and 12
	// motor2 : pin 13 and 14

}

// In order to keep this as nimble as possible, avoid
// using floating point or printf() on any of its forms!
int main(void)
{
	int adc_value;
	unsigned long v;
	unsigned long int count, f;
	unsigned char LED_toggle=0;
	
	init_Clock48();
    UART3_init(115200);
    ADC_init();
    ConfigurePins();
    Configure_TC3();
    
	delayMs(500);
	printString("\x1b[2J\x1b[1;1H"); // Clear screen using ANSI escape sequence.
	printString("\r\nSAMD20E16 multi I/O example.\r\n");
	printString("Measures the voltage at channels 4 and 5 (pins 5 and 6 of QFP32 package)\r\n");
	printString("Measures period on PA15 (pin 16 of QFP32 package)\r\n");
	printString("Toggles PA00, PA01, PA02, PA06, PA07 (pins 1, 2, 3, 7, 8, of QFP32 package)\r\n");
	printString("Generates PWM servo signals on PA08, PA09 (pins 11, 12 of QFP32 package)\r\n\r\n");
    
    while(1)
    {
    	adc_value=ADC_read(4);
    	v=(adc_value*3300L)/4095; // 3300L=3.3V*1000L
	    printString("AIN4: 0x");
	    printNum(adc_value, 16, 3);
	    printString(" V: ");
	    printNum(v/1000L, 10, 1); // Print the integer part of the voltage.  
	    printString("."); // Print decimal point between integer part and fraction
	    printNum(v%1000L, 10, 3); // Print the fraction part of the voltage 
	    printString("V ");    

    	adc_value=ADC_read(5);
    	v=(adc_value*3300L)/4095; // 3300L=3.3V*1000L
	    printString("AIN5: 0x");
	    printNum(adc_value, 16, 3);
	    printString(" V: ");
	    printNum(v/1000L, 10, 1); // Print the integer part of the voltage.  
	    printString("."); // Print decimal point between integer part and fraction
	    printNum(v%1000L, 10, 3); // Print the fraction part of the voltage 
	    printString("V ");

		count=GetPeriod(40);
		if(count>0)
		{
			f=(F_CPU*40L)/count;
			printString("f=");
			printNum(f, 10, 7);
			printString("Hz, count=");
			printNum(count, 10, 6);
			printString("          \r");
		}
		else
		{
			printString("NO SIGNAL                     \r");
		}

		// Now toggle the pins on/off to see if they are working.
		// First turn all off:
		REG_PORT_OUTCLR0 = PORT_PA00;
		REG_PORT_OUTCLR0 = PORT_PA01;
		REG_PORT_OUTCLR0 = PORT_PA02;
		REG_PORT_OUTCLR0 = PORT_PA06;
		REG_PORT_OUTCLR0 = PORT_PA07;
		// Now turn on one of the outputs per loop cycle to check
		switch (LED_toggle++)
		{
			case 0:
				REG_PORT_OUTSET0 = PORT_PA00;
				break;
			case 1:
				REG_PORT_OUTSET0 = PORT_PA01;
				break;
			case 2:
				REG_PORT_OUTSET0 = PORT_PA02;
				break;
			case 3:
				REG_PORT_OUTSET0 = PORT_PA06;
				break;
			case 4:
				REG_PORT_OUTSET0 = PORT_PA07;
				break;
			default:
				break;
		}
		if(LED_toggle>4) LED_toggle=0;
	        
	    delayMs(200);
	}
}