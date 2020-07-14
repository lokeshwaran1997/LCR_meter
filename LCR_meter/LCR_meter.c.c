// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz



//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include <string.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>

/*BIT BANDED REGIONS OF SINGLE BIT IN FULL WORD(32 BITS)*/
#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define PE5          (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 5*4)))
#define PB4          (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 4*4)))
#define PA5          (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 5*4)))
#define PA6          (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 6*4)))
#define PA7          (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 7*4)))
char getchaar();
unsigned char str[81];    //global variable str
uint8_t argc;
uint8_t pos[81];
unsigned char type[81];

uint8_t minargs;
volatile uint32_t tow=0;
volatile uint32_t tow2=0;

/*CLOCK SETTINGS AND PERIPHERAL SETTINGS*/
// Initialize Hardware
void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use AP (not needed since default configuration -- for clarity)
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable GPIO port A and F peripherals
       SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOF | SYSCTL_RCGC2_GPIOB | SYSCTL_RCGC2_GPIOE | SYSCTL_RCGC2_GPIOC;


    // Configure LED and pushbutton pins
    GPIO_PORTF_DIR_R = 0x02;  // bits 1 is red output, other pins are inputs
    GPIO_PORTF_DR2R_R = 0x02; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R = 0x02;  // enable LEDs and pushbuttons
   // GPIO_PORTF_PUR_R = 0x10;  // enable internal pull-up for push button

    //ENABLE PINS FOR DIGITAL FUNCTIONS;
    GPIO_PORTA_DIR_R = 0xE0; //ENABLE PIN A5,A6,A7 LOW
       GPIO_PORTA_DEN_R = 0xE0; //DEN FOR PIN PA5,A6,A7

      GPIO_PORTB_DIR_R = 0x10; //ENABLE PORT B PIN PB4 LOW
       GPIO_PORTB_DEN_R = 0x10; //DIGITAL ENABLE PB4

       GPIO_PORTE_DIR_R = 0x20; //ENABLE PORTE PIN PE5
       GPIO_PORTE_DEN_R = 0x20; //DIGITAL ENABLE
       GPIO_PORTE_DR2R_R = 0x20;

    // Configure UART0 pins
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other uarts in same status
    GPIO_PORTA_DEN_R |= 3;                           // default, added for clarity
    GPIO_PORTA_AFSEL_R |= 3;                         // default, added for clarity
    GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

    // Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
    UART0_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
    UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module

    PE5=0;
    PB4=0;
    PA5=0;
    PA6=0;
    PA7=0;
    // Configure AN0,AN1 as an analog input
    SYSCTL_RCGCADC_R |= 1;                           // turn on ADC module 0 clocking
    GPIO_PORTE_AFSEL_R |= 0x08;                      // select alternative functions for AN0 (PE3)
    GPIO_PORTE_DEN_R &= ~0x08;                       // turn off digital operation on pin PE3
    GPIO_PORTE_AMSEL_R |= 0x08;                      // turn on analog operation on pin PE3
    ADC0_CC_R = ADC_CC_CS_SYSPLL;                    // select PLL as the time base (not needed, since default value)
    ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN3;                // disable sample sequencer 3 (SS3) for programming
    ADC0_EMUX_R = ADC_EMUX_EM3_PROCESSOR;            // select SS3 bit in ADCPSSI as trigger
    ADC0_SSMUX3_R = 0;                               // set first sample to AN0
    ADC0_SSCTL3_R = ADC_SSCTL3_END0;                 // mark first sample as the end
    ADC0_ACTSS_R |= ADC_ACTSS_ASEN3;                 // enable SS3 for operation

        SYSCTL_RCGCADC_R |= 2;                           // turn on ADC module 1 clocking
        GPIO_PORTE_AFSEL_R |= 0x04;                      // select alternative functions for AN1 (PE2)
        GPIO_PORTE_DEN_R &= ~0x04;                       // turn off digital operation on pin PE2
        GPIO_PORTE_AMSEL_R |= 0x04;                      // turn on analog operation on pin PE2
        ADC1_CC_R = ADC_CC_CS_SYSPLL;                    // select PLL as the time base (not needed, since default value)
        ADC1_ACTSS_R &= ~ADC_ACTSS_ASEN3;                // disable sample sequencer 3 (SS3) for programming
        ADC1_EMUX_R = ADC_EMUX_EM3_PROCESSOR;            // select SS3 bit in ADCPSSI as trigger
        ADC1_SSMUX3_R = 1;                               // set first sample to AN1
        ADC1_SSCTL3_R = ADC_SSCTL3_END0;                 // mark first sample as the end
        ADC1_ACTSS_R |= ADC_ACTSS_ASEN3;                 // enable SS3 for operation


        SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R5;     // turn-on timer
        WTIMER5_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off counter before reconfiguring
        WTIMER5_CFG_R = 4;                               // configure as 32-bit counter (A only)
        WTIMER5_TAMR_R = TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR; // configure for edge time mode, count up
        WTIMER5_CTL_R = TIMER_CTL_TAEVENT_POS;           // measure time from positive edge to positive edge
        WTIMER5_TAV_R = 0;
        WTIMER5_CTL_R |= TIMER_CTL_TAEN;
        NVIC_EN3_R &= ~(1 << (INT_WTIMER5A-16-96));


        //analog comparator
                                SYSCTL_RCGCACMP_R |=0x01;                //TURN ON ANALOG COMP MODULE
                                GPIO_PORTC_AFSEL_R |=0x00000080;
                                GPIO_PORTC_AMSEL_R |=0x80;
                                GPIO_PORTC_DEN_R &= ~0x80;     //PC7-

                                COMP_ACREFCTL_R |= 0x0000020F;
                                COMP_ACCTL0_R |= 0x0000040C;
                                COMP_ACRIS_R = COMP_ACRIS_IN0;
                                COMP_ACINTEN_R = COMP_ACINTEN_IN0;



}
void compIsr()
{
    tow=WTIMER5_TAV_R;
    tow2=tow;
    COMP_ACMIS_R |= COMP_ACMIS_IN0;// clear interrupt
    //COMP_ACMIS_R |=0x01;  
}


int16_t readAdc0Ss3()
{
    ADC0_PSSI_R |= ADC_PSSI_SS3;                     // set start bit
    while (ADC0_ACTSS_R & ADC_ACTSS_BUSY);           // wait until SS3 is not busy
    return ADC0_SSFIFO3_R;                           // get single result from the FIFO
}
int16_t readAdc1Ss3()
{
    ADC1_PSSI_R |= ADC_PSSI_SS3;                     // set start bit
    while (ADC1_ACTSS_R & ADC_ACTSS_BUSY);           // wait until SS3 is not busy
    return ADC1_SSFIFO3_R;                           // get single result from the FIFO
}


// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);
    UART0_DR_R = c;
}
// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{

    uint8_t i;
    for (i = 0; i < strlen(str); i++)
      putcUart0(str[i]);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
     while (UART0_FR_R & UART_FR_RXFE);
           return UART0_DR_R & 0xFF;
}
char getchaar()
{
char b;
uint8_t count=0;

while(1)
{
    b=getcUart0();

       if(b==0x8)                   //backspace
       {
           if(count>0)
           {
               count=count-1;
           }
               else
               {
                  b= getcUart0();
                }
       }
       else if(b==0x0D)                       // carriage return
           {
               count=0;
               break;
           }

       else /*if(b==0x20)*/
               {
               str[count]=tolower(b);
               count=count+1;
               }
}
               /*else
               {
                  getcUart0();
               }*/
         //  }
      // }
    if(count==81)
    {
        count=0;

       //break;
    }
   /* else
    {
        getcUart0();
    }*/
return b;
}
char parsestr();
char parsestr()
{
    uint16_t i;
    uint16_t j=-1;
     uint16_t Length;
    uint8_t flag=0;
    Length=strlen(str);
    for(i=0;i<Length;i++)
    {
     if(isspace(str[i]))
     {
       flag=0;
     }
     if((isalpha(str[i])|| isdigit(str[i])) && (flag==0))
     {
         j=j+1;
         flag=1;
         if((str[i]>='a' && str[i]<='z') || (str[i]>='A' && str[i]<='Z'))
         {
             pos[j]=i;
             type[j]='alpha';
         }
         else if(str[i]>=48 && str[i]<=57)
         {
             pos[j]=i;
             type[j]='n';
         }
     }
    }

    argc=strlen(type);
    putsUart0("\n");
    for(i=0;i<Length;i++)
    {
        if((isspace(str[i]))||(str[i]>=33 && str[i]<=47) || (str[i]>=58 && str[i]<=64) || (str[i]>=91 && str[i]<=96)||(str[i]>=123 && str[i]<=126))
         str[i]='\0';
    }
    for(i=0;i<argc;i++)
    {
        putsUart0("\r\n");
        putsUart0(&str[pos[i]]);
    }

 }

bool iscommand(char* verb,uint8_t minargs)
{

if((strcmp(verb,&str[pos[0]])==0)&& argc>=minargs)
        {
            return true;
        }
else
{
    return false;
}
}
/* bool iscommand1(char* verb,char* verb1,char* verb2)
 {
        if((strcmp(verb,&str[pos[0]])==0) && (strcmp(verb1,&str[pos[1]])==0) && (strcmp(verb2,&str[pos[2]])==0))
        {

                return true;
        }
 }*/

void waitMicrosecond(uint32_t us)
{
    __asm("WMS_LOOP0:   MOV  R1, #6");          // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
    __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
    __asm("             NOP");                  // 5
    __asm("             NOP");                  // 5
    __asm("             B    WMS_LOOP1");       // 5*2 (speculative, so P=1)
    __asm("WMS_DONE1:   SUB  R0, #1");          // 1
    __asm("             CBZ  R0, WMS_DONE0");   // 1
    __asm("             NOP");                  // 1
    __asm("             B    WMS_LOOP0");       // 1*2 (speculative, so P=1)
    __asm("WMS_DONE0:");                        // ---
                                                // 40 clocks/us + error
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main()
{
    char e;
    char d;
    uint8_t i;

          // Initialize hardware
    initHw();
    // Turn on red LED

    RED_LED = 1;
    waitMicrosecond(500000);
     //Turn off red LED
    RED_LED = 0;
    waitMicrosecond(500000);
    while(1)
    {
    putsUart0("\n\r");
    putsUart0("\r\n enter string:");
    d=getchaar();
    putsUart0("\r\n");
    putsUart0("\r\nthe output is...");
    parsestr();

    if(iscommand("measurelr",1)==true)
    {

    PE5 = 1;
    PB4 = 0;
    PA5 = 0;
    PA7 = 0;
    PA6 = 0;
    putsUart0("\r\n");
    putsUart0("measureLr");
    for(i=0;i<81;i++)
    {
        pos[i]=0;
        type[i]=0;
        str[i]='\0';
        argc=0;
    }
   }
/*    if(iscommand("set","measlr","off")==true)
       {
       PE5 = 0;
       putsUart0("\r\n");
       putsUart0("measureLr Off");
       for(i=0;i<81;i++)
       {
           pos[i]=0;
           type[i]=0;
           str[i]='\0';
           argc=0;
       }
      }*/

   if(iscommand("measurec",1)==true)
   {

    PE5 = 0;
    PB4 = 1;
    PA5 = 0;
    PA7 = 0;
    PA6 = 0;
    putsUart0("\r\n");
    putsUart0("measure_C on");
    for(i=0;i<81;i++)
    {
        pos[i]=0;
        type[i]=0;
        str[i]='\0';
        argc=0;
    }
   }
 /*  if(iscommand1("set","measc","off")==true)
          {
          PB4 = 0;
          putsUart0("\r\n");
          putsUart0("measurec Off");
          for(i=0;i<81;i++)
          {
              pos[i]=0;
              type[i]=0;
              str[i]='\0';
              argc=0;
          }
         }*/

 if(iscommand("measurehr",1)==true)
      {
       PA5 = 1;
       PA7 = 0;
       PA6 = 0;
       PE5 = 0;
       PB4 = 0;
       putsUart0("\r\n");
       putsUart0("measure_Hr");
       for(i=0;i<81;i++)
       {
           pos[i]=0;
           type[i]=0;
           str[i]='\0';
           argc=0;
       }
      }
       /*if(iscommand1("set","highside","off")==true)
             {
              PA5 = 0;
              putsUart0("\r\n");
              putsUart0("measure_Hr off");
              for(i=0;i<81;i++)
              {
                  pos[i]=0;
                  type[i]=0;
                  str[i]='\0';
                  argc=0;
              }
      }*/
      if(iscommand("measurelowr",1)==true)
      {
       PA7 = 1;
       PA5 = 0;
       PA6 = 0;
       PE5 = 0;
       PB4 = 0;
       putsUart0("\r\n");
       putsUart0("measureLowR");
       for(i=0;i<81;i++)
       {
           pos[i]=0;
           type[i]=0;
           str[i]='\0';
           argc=0;
       }
      }
 /*     if(iscommand1("set","measlow","off")==true)
            {
             PA7 = 0;
             putsUart0("\r\n");
             putsUart0("measureLowR_off");
             for(i=0;i<81;i++)
             {
                 pos[i]=0;
                 type[i]=0;
                 str[i]='\0';
                 argc=0;
             }
            }*/
      if(iscommand("integrate",1)==true)
          {
           PA6 = 1;
           PB4 = 0;
           PA7 = 1;        //along with lowside R
           PA5 = 0;
           PE5 = 0;
           putsUart0("\r\n");
           putsUart0("IntegrateOn");
           for(i=0;i<81;i++)
           {
               pos[i]=0;
               type[i]=0;
               str[i]='\0';
               argc=0;
           }
          }
     /* if(iscommand1("set","integrate","off")==true)
                {
                 PA6 = 0;
                 putsUart0("\r\n");
                 putsUart0("Integrate_Off");
                 for(i=0;i<81;i++)
                 {
                     pos[i]=0;
                     type[i]=0;
                     str[i]='\0';
                     argc=0;
                 }
                }*/
     if (iscommand("voltage",0)==true)
      {
	      //Vdd=3.3 V
              char temp[20];
              char v2[20];//for sprintf function to print on console 
	      char v1[20];//for sprintf function to print on console 
	      uint32_t dut1;
              uint32_t dut2;
              float voltage;
              float dut_a;
              float dut_b;
              dut2 = readAdc0Ss3();
              dut_b=(dut2 / 4096.0 * 3.3);//4096 level(2^12)->12 bit adc resolution
              putsUart0("\r\n");
              putsUart0("v2: " );
              sprintf(v2, "%3.5f", dut_b);
              putsUart0(v2);
              dut1 = readAdc1Ss3();
              dut_a=(dut1 / 4096.0 * 3.3);
              putsUart0("\r\n");
              putsUart0("v1: " );
              sprintf(v1, "%3.5f", dut_a);
              putsUart0(v1);
              voltage=dut_b-dut_a;
              putsUart0("\r\n");
              putsUart0("Voltage: " );
                       sprintf(temp, "%3.5f", voltage);
                       putsUart0(temp);
                       for(i=0;i<81;i++)
                       {
                           pos[i]=0;
                           type[i]=0;
                           str[i]='\0';
                           argc=0;
                       }
        }
     if(iscommand("reset",0)==true)
         {
          ResetISR();
          putsUart0("\r\n");
          putsUart0("Reset done");
          for(i=0;i<81;i++)
          {
              pos[i]=0;
              type[i]=0;
              str[i]='\0';
              argc=0;
          }
         }



     if(iscommand("resistor",1)==true)
                 {
                     float res=0;
		     char time[10];
		     char ohms[10];
                     RED_LED = 1;
                     waitMicrosecond(500000);
                     RED_LED = 0;
                     PE5 = 0;
                     PB4 = 0;
                     PA5 = 0;
                     PA6 = 0;
                     PA7 = 0;
                     waitMicrosecond(500);
                     //turn on INTEGRATE on and leave it on for while
                     PA6=1;
                     //turn then LOW_R on
                     PA7=1;                     //wait until the charge across Vc drains out through 33 ohm
                     waitMicrosecond(500000);
                     PA7=0;                     //turn off LOW_R to zero
                     PE5=1;
                     WTIMER5_TAV_R = 0;
                     WTIMER5_CTL_R |= TIMER_CTL_TAEN;//timer on
                     NVIC_EN0_R |= 1 << (INT_COMP0-16); //turn on interrupt
                     waitMicrosecond(5000000);//wait for charge across Vc hit the thresholf voltage for Analog comparator
                     NVIC_EN0_R &= ~(1 << (INT_COMP0-16)); //turn off interrupt
                     putsUart0("\r\n");
                     putsUart0("time : " );
                     sprintf(time, "%3.5d", tow);
                     putsUart0(time);
                     if (tow > 0 && tow < 4900)
                     {
                         res = (0.031914893 * tow) - 32.92553191;//equation formed by Linear Regression formula
                     }
                     else if (tow > 4901 && tow < 2700000)           //50k
                     {
                         res = (0.018655195 * tow) + 0.000288295;

                     }
                    else if (tow >2700001 && tow < 35000000)   //100k to 500k
                     {

                         res = (0.013990239 * tow) + 20417.66742;
                     }
                    else if (tow > 35000001 && tow < 58000000)    //600 700
                    {
                        res = (0.007947937828 * tow) + 244349.7554;
                    }
                    else if (tow > 58000001 && tow < 112000000)    // 800 900
                    {
                        res = (0.003799410286 * tow) + 519155.6844;
                    }
                    else
                    {
                        res = tow / 180.361985;
                    }
                     putsUart0("\r\n");
                     putsUart0("ress: " );
                     sprintf(ohms, "%3.5f", res);
                     putsUart0(ohms);
                     putsUart0(" ohms");
                     for(i=0;i<81;i++)
                      {
                         pos[i]=0;
                         type[i]=0;
                         str[i]='\0';
                         argc=0;
                       }
                    }

     if(iscommand("capacitor",1)==true)
     {
         float capp=0;
	 char time1[10];
	 char temp4[10];
         RED_LED = 1;//bit banded definition of single bit
         waitMicrosecond(400000);
         RED_LED = 0;
         PB4 = 1;     // TURN ON MEASURE C
         PE5 = 0;      //meas lr off
         PA6 = 0;    // integrate off
         PA7 = 1;    //lowr on
         PA5 = 0;    //hr off
         waitMicrosecond(3000000);
         PA7 = 0;       //lowr off
         PA5 = 1;       //hr on
         WTIMER5_TAV_R = 0;
         WTIMER5_CTL_R |= TIMER_CTL_TAEN;
         NVIC_EN0_R |= 1 << (INT_COMP0-16); //turn on interrupt
         waitMicrosecond(20000000);
         NVIC_EN0_R &= ~(1 << (INT_COMP0-16)); //turn off interrupt
         putsUart0("\r\n");
         putsUart0("time : " );
         sprintf(time1, "%3.5d", tow);
         putsUart0(time1);
         if (tow <460000000)
         {
         capp = (0.0000001478792212 * tow) + 0.089843214603;
         }
         else
         {
             capp = (0.0000001743285949 * tow) - 12.43381165;
         }
         sprintf(temp4, "%3.5f", capp);
                  putsUart0("\r\n");
                  putsUart0("cap : " );
                  putsUart0(temp4);
                  putsUart0("\r\n");
                  putsUart0(" microfarad" );
                  dut2 = readAdc0Ss3();
                  dut_b=(dut2 / 4096.0 * 3.3);
                  putsUart0("\r\n");
                  putsUart0("v2: " );
                  sprintf(v2, "%3.5f", dut_b);
                  putsUart0(v2);
         for(i=0;i<81;i++)
            {
                 pos[i]=0;
                 type[i]=0;
                 str[i]='\0';
                 argc=0;
             }
     }
     if(iscommand("inductor",1)==true)
     {
      //   tow = 0;
	 float ind=0;
	 char temp5[10];
	 chat time2[10];
         RED_LED = 1;
         waitMicrosecond(400000);
         RED_LED = 0;
         WTIMER5_TAV_R = 0;
         dut2 = readAdc0Ss3();
         dut_b=(dut2 / 4096.0 * 3.3);
         putsUart0("\r\n");
         putsUart0("v2: " );
         sprintf(v2, "%3.5f", dut_b);
         putsUart0(v2);

         PA7 = 0;    //lowr off
         PB4 = 0;    //measurec
         PA6 = 0;      // integrate off
         PE5 = 1;    //meas lr

         PA7 = 1;    // low r on
         WTIMER5_TAV_R = 0;
                WTIMER5_CTL_R |= TIMER_CTL_TAEN;
                NVIC_EN0_R |= 1 << (INT_COMP0-16); //turn on interrupt
                waitMicrosecond(6000000);
                NVIC_EN0_R &= ~(1 << (INT_COMP0-16)); //turn off interrupt
                putsUart0("\r\n");
                putsUart0("time : " );
                sprintf(time2, "%3.5d", tow2);
                putsUart0(time2);
           //     ind = (0.601239531 * tow) - 13.7118928;
               ind = (0.602105263 * tow2) + 0.000001579;
               //   ind = (0.6003 * tow2) - 3.551;
               putsUart0("\r\n");
               putsUart0("inductance : " );
               sprintf(temp5, "%3.5f", ind);
               putsUart0(temp5);
               putsUart0("\r\n");
               putsUart0(" microhenry" );

               tow2 = 0;
                dut2 = readAdc0Ss3();
                                  dut_b=(dut2 / 4096.0 * 3.3);
                                  putsUart0("\r\n");
                                  putsUart0("v2: " );
                                  sprintf(v2, "%3.5f", dut_b);
                                  putsUart0(v2);

                                PE5 = 0;
                                PB4 = 1;
                                  waitMicrosecond(15000000);

         for(i=0;i<81;i++)
                {
                    pos[i]=0;
                    type[i]=0;
                    str[i]='\0';
                    argc=0;
                  }


     }
     if(iscommand("esr",1)==true)
     {
		       char esrRes[20];
]		       float rl;
                       PE5 = 1;    // lr on
                       PA7 = 1;    //lowr on
                       waitMicrosecond(7000000);
                       dut2 = readAdc0Ss3();
                       dut_b=(dut2 / 4096.0 * 3.3);
                       dut1 = readAdc1Ss3();
                       dut_a=(dut1 / 4096.0 * 3.3);
                       rl = (33 * ( dut_a -dut_b)) / dut_b;
                       putsUart0("\r\n");
                       sprintf(esrRes, "%3.5f", rl);
                       putsUart0(esrRes);
                       putsUart0(" ohms");

                       PE5 = 0;
                       PB4 = 1;
                         waitMicrosecond(7000000);

     }
     for(i=0;i<81;i++)
       {
           pos[i]=0;
           type[i]=0;
           str[i]='\0';
           argc=0;
         }


}
return 0;
}







