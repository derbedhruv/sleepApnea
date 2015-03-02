/****************************************
  This has been taken from http://www.protocentral.com/downloads/examples/protoc_afe4400_spo2.ino
  and has been modified.
  
  Author: Dhruv Joshi
  
  The AFE4400 development board is being used with an arduino UNO to be able to 
  develop a reflective pulse oximeter using the NJL5501 sensor.
  
  The following has been taken from the datasheet of this shield:
  * - (via logic level shift) 
  
  Arduino pins  Shield nomenclature  Name (AFE4400)  Usage
  ------------  -------------------  -------------   ------
  D2            *ARD_DRDY            ADC_RDY         ADC conversion completion
  D4            *ARD_PWDN            AFE_PDN         AFE-onle power down
  D7            *ARD_CS0             SPISTE          SPI serial interface enable
  D8            *ARD_PD_ALM          PD_ALM          PD sensor fault indicator
  D9            *ARD_LED_ALM         LED_ALM         LED cable fault indicator
  D10           *ARD_DIAG_END        DIAG_END        completion of diagnostics
  
  D11           *ARD_MOSI            SPI_MOSI        
  D12           *ARD_MISO            SPI_MISO        
  D13           *ARD_SCK             SPI_SCK         
  
  GND           GND
  5V            Vcc (+5V)
  -----------------------------------------
  
  The system uses the LP3878 as a voltage regulator
  The system uses a TPS7A4901 as a voltage regulator
  The system uses a TPS717xx to generate a +3.3V regulated from the +5V input
  The system makes extensive use of the BSS138 NFET to convert logic levels
  No analog pins are used, everything is digital using SPI comm between the AFE4400 and the arduino
  
  The flowchart/algo for the system is the following:
  1. setup SPI comm and settings
  2. 
  
  ****************************************/

#include <string.h>
#include <SPI.h>
#include <math.h>
#include <FIR.h>
//#include "Average.h"
#define BPM_WINDOW 300

// The following are register bits defined as in Table 6 in the AFE4400 datasheet and Section 8.6.2
// This first block are described in Table 2
#define CONTROL0		0x00
#define LED2STC			0x01
#define LED2ENDC		0x02
#define LED2LEDSTC		0x03
#define LED2LEDENDC		0x04
#define ALED2STC		0x05
#define ALED2ENDC		0x06
#define LED1STC			0x07
#define LED1ENDC		0x08
#define LED1LEDSTC		0x09
#define LED1LEDENDC		0x0a
#define ALED1STC		0x0b
#define ALED1ENDC		0x0c
#define LED2CONVST		0x0d
#define LED2CONVEND		0x0e
#define ALED2CONVST		0x0f
#define ALED2CONVEND	        0x10
#define LED1CONVST		0x11
#define LED1CONVEND		0x12
#define ALED1CONVST		0x13
#define ALED1CONVEND      	0x14
#define ADCRSTCNT0		0x15
#define ADCRSTENDCT0    	0x16
#define ADCRSTCNT1		0x17
#define ADCRSTENDCT1    	0x18
#define ADCRSTCNT2		0x19
#define ADCRSTENDCT2    	0x1a
#define ADCRSTCNT3		0x1b
#define ADCRSTENDCT3    	0x1c
#define PRPCOUNT		0x1d

// the rest are defined in detail in Figure 89 onwards (p62), in order.
#define CONTROL1		0x1e
#define SPARE1			0x1f
#define TIAGAIN			0x20
#define TIA_AMB_GAIN    	0x21
#define LEDCNTRL		0x22
#define CONTROL2		0x23
#define SPARE2			0x24
#define SPARE3			0x25
#define SPARE4			0x26
#define SPARE4			0x26
#define RESERVED1		0x27
#define RESERVED2		0x28
#define ALARM			0x29
#define LED2VAL			0x2a
#define ALED2VAL		0x2b
#define LED1VAL			0x2c
#define ALED1VAL		0x2d
#define LED2ABSVAL		0x2e
#define LED1ABSVAL		0x2f
#define DIAG			0x30


#define count 60
int IRheartsignal[count];
int Redheartsignal[count];
int IRdc[count];
int Reddc[count];
double difIRheartsig_dc;
double difREDheartsig_dc;

FIR fir;

// defining the SPI pins which will be used in the arduino
const int SOMI = 12; 
const int SIMO = 11; 
const int SCLK  = 13;
const int SPISTE = 7; 
const int SPIDRDY = 6;

// general declarations
int pin = 3;
int pin2 = 4;
volatile int state = LOW;

// function declarations
void AFE4490Init (void);
void AFE4490Write (uint8_t address, uint32_t data);
uint32_t AFE4490Read (uint8_t address);

void setup()
{
   Serial.begin(9600);    // nice and slow 9600 baud
    
   // all the SPI-related declarations
   SPI.begin(); 
   pinMode (SOMI,INPUT);
   pinMode (SPISTE,OUTPUT);
   pinMode (SCLK, OUTPUT);
   pinMode (SIMO, OUTPUT);
   pinMode (SPIDRDY,INPUT);
   
   pinMode(2, INPUT);
   pinMode(3, OUTPUT);              // ******THIS PIN ISN'T BEING USED!
   
   // an interrupt
   attachInterrupt(0, blink, RISING );
   
   // some simple SPI-related changes
   SPI.setClockDivider (SPI_CLOCK_DIV8);
   SPI.setDataMode (SPI_MODE0);
   SPI.setBitOrder (MSBFIRST);
   
   // FIR filter  
   float coef[FILTERTAPS] = { 0.021, 0.096, 0.146, 0.096, 0.021};
   fir.setCoefficients(coef);

   //declare gain coefficient to scale the output back to normal
   float gain = 1; // set to 1 and input unity to see what this needs to be
   fir.setGain(gain);

   AFE4490Init ();       // initialize the AFE44x0 after reset.
 }
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop() {
  float data = sp02();
  if(data) {
    // Serial.print("sp02:--> ");
    Serial.println(data);
  }
}

void blink() {
  state = HIGH;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void AFE4490Init (void)
{ 
  // Serial.println("AFE4490 Initialization Starts"); 
  
  // CONTROL0 is used for AFE software and count timer reset, diagnostics enable, and SPI read functions.
  // The bits are given as follows
  // bits 23:4 - must be 0
  // bit 3 - software reset. on reset, needs to be 0.
  // bit 2 - diagnostic enable. 0 after reset.
  // bit 1 - Timer counter reset. 0 after reset.
  // bit 0 - SPI read. 0 for disable (after reset).
  AFE4490Write(CONTROL0,0x000000);
  
  // TIAGAIN is the transimpedence amplifier (TIA) gain setting register. It is mentioned to be "reserved for factory use".
  // reset value 0x000000
  AFE4490Write(TIAGAIN,0x000000);	
  
  // Transimpedance Amplifier and Ambient Cancellation Stage Gain Register
  // This register configures the ambient light cancellation amplifier gain, cancellation current, and filter corner frequency
  // 0x000005 = B0000 0000 0000 0000 0000 0101
  // That means, piecewise,
  // 0000         0000                             0            0                        000          000                        00000                                   101
  // ^must be 0   ^ambient DAC value. 0 on reset.  ^must be 0   ^0 default after reset.  ^must be 0   ^stage 2 gain, 0 on reset. ^program CF, here 5 pF (reset default)  ^10 kOhm RF for LEDs
  AFE4490Write(TIA_AMB_GAIN,0x000005);
  

  // the following register is for LED intensity. "LEDCNTRL"
  // 0x0011414 = B0000 0001 0001 0100 0001 0100
  // seeing these bits as packets,
  // B0 0 0 0 0 0            0                 1                                    0001 0100                                            0001 0100
  //  -----------           ---               ---                                   ---------                                            ---------
  //  bits 23:18             17                16                                      15:8                                                  7:0
  //  must be 0    LED current source ON    must be 1      LED1 signal - here 20 (so current is (20/256)*50 mA = 3.9 mA        LED2 signal - same value, same current.
  AFE4490Write(LEDCNTRL,0x0011414);	        
  
  // CONTROL2 controls the LED transmitter, crystal, and the AFE, transmitter, and receiver power modes
  // 
  AFE4490Write(CONTROL2,0x000000);	// LED_RANGE=100mA, LED=50mA 
  
  // CONTROL1 configures the clock alarm pin and timer
  // 0x000702 = B0000 0000 0000 0111 0000 0010
  // explanation is
  // 0000 0000 0000          011                                                                                            1              0000 0010
  // --------------          ---                                                                                            -              ---------
  // bit 23:12 - must be 0   11:9 - clock on alarm pins. In this case, PD_ALM is LED2 convert and LED_ALM is LED1 convert.  timer enabled. ^Must be like this.
  // **************NOTE: THIS IS PROBABLY WRONG, WOULD HAVE TO BE 0X000702 AND NOT 0X010707
  AFE4490Write(CONTROL1,0x010707);	// Timers ON, average 3 samples  
  
  AFE4490Write(PRPCOUNT, 0X001F3F);

  AFE4490Write(LED2STC, 0X001770); //timer control
  AFE4490Write(LED2ENDC,0X001F3E); //timer control
  AFE4490Write(LED2LEDSTC,0X001770); //timer control
  AFE4490Write(LED2LEDENDC,0X001F3F); //timer control
  AFE4490Write(ALED2STC, 0X000000); //timer control
  AFE4490Write(ALED2ENDC, 0X0007CE); //timer control
  AFE4490Write(LED2CONVST,0X000002); //timer control
  AFE4490Write(LED2CONVEND, 0X0007CF); //timer control
  AFE4490Write(ALED2CONVST, 0X0007D2); //timer control
  AFE4490Write(ALED2CONVEND,0X000F9F); //timer control

  AFE4490Write(LED1STC, 0X0007D0); //timer control
  AFE4490Write(LED1ENDC, 0X000F9E); //timer control
  AFE4490Write(LED1LEDSTC, 0X0007D0); //timer control
  AFE4490Write(LED1LEDENDC, 0X000F9F); //timer control
  AFE4490Write(ALED1STC, 0X000FA0); //timer control
  AFE4490Write(ALED1ENDC, 0X00176E); //timer control
  AFE4490Write(LED1CONVST, 0X000FA2); //timer control
  AFE4490Write(LED1CONVEND, 0X00176F); //timer control
  AFE4490Write(ALED1CONVST, 0X001772); //timer control
  AFE4490Write(ALED1CONVEND, 0X001F3F); //timer control

  AFE4490Write(ADCRSTCNT0, 0X000000); //timer control
  AFE4490Write(ADCRSTENDCT0,0X000000); //timer control
  AFE4490Write(ADCRSTCNT1, 0X0007D0); //timer control
  AFE4490Write(ADCRSTENDCT1, 0X0007D0); //timer control
  AFE4490Write(ADCRSTCNT2, 0X000FA0); //timer control
  AFE4490Write(ADCRSTENDCT2, 0X000FA0); //timer control
  AFE4490Write(ADCRSTCNT3, 0X001770); //timer control
  AFE4490Write(ADCRSTENDCT3, 0X001770);

  delay(1);
  // Serial.println("AFE4490 Initialization Done"); 
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void AFE4490Write (uint8_t address, uint32_t data) {
  digitalWrite (SPISTE, LOW); // enable device
  SPI.transfer (address); // send address to device
  SPI.transfer ((data >> 16) & 0xFF); // write top 8 bits
  SPI.transfer ((data >> 8) & 0xFF); // write middle 8 bits
  SPI.transfer (data & 0xFF); // write bottom 8 bits    
  digitalWrite (SPISTE, HIGH); // disable device
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint32_t AFE4490Read (uint8_t address) {       
  uint32_t data=0;
  digitalWrite (SPISTE, LOW); // enable device
  SPI.transfer (address); // send address to device
  //SPI.transfer (data);
  data |= (SPI.transfer (0)<<16); // read top 8 bits data
  data |= (SPI.transfer (0)<<8); // read middle 8 bits  data
  data |= SPI.transfer (0); // read bottom 8 bits data
  digitalWrite (SPISTE, HIGH); // disable device
  	
  return data; // return with 24 bits of read data
}

void enableDRDY()
{
  state = LOW;
  attachInterrupt(0, blink, RISING );  
}

void disableDRDY()
{
  detachInterrupt(0);  
}

signed long AFERead (uint8_t address) {       
  uint32_t data = AFE4490Read (address);
  unsigned long utemp = (unsigned long) (data<<10);
  signed long stemp = (signed long) (utemp);
  stemp = (signed long) (stemp>>10);
  return stemp; // return with 22 bits of read data
}


float sp02(void) {
  // this function finds the actual spo2 and returns it (float)
  long Redvalue, IRvalue, Redhigh, Redlow, IRhigh, IRlow;
  long Redsum = 0, IRsum = 0;
  unsigned long Redac_sq = 0;
  unsigned long IRac_sq = 0;
  static int flag1 = 1;
  long Reddc, IRdc,Reddc_prev,IRdc_prev;
  int samples = 2000;
  long IRac, Redac;

  AFE4490Write(CONTROL0,0x000001);  	  // enable SPI read.
  
  // initialize the max and min values before a loop..
  Redhigh = Redlow = AFERead(LED2VAL);    // 24-bit LED2 value from ADC
  IRhigh = IRlow = AFERead(LED1VAL);      // 24-bit LED2 value from ADC
	   
  return int(Redhigh);
  
/*
  for(int i=1; i<(samples+1); i++) {
    enableDRDY();
    while (state == LOW);
 
    Redvalue = AFERead(LED2VAL);    // this reads in the latest LED2 value
    IRvalue = AFERead(LED1VAL);     // this reads in the latest LED1 value
    
    disableDRDY();          

    // filter the values
    Redvalue = fir.process(Redvalue);  
    IRvalue = fir.process(IRvalue);  
    
    // add them in to the sum within the loop
    Redsum += Redvalue;
    IRsum += IRvalue;
		
    // go through each sample and find the max and min in a particular range..
    if(Redvalue > Redhigh)
      Redhigh = Redvalue;	
    if(Redvalue < Redlow)
      Redlow = Redvalue;	
    if(IRvalue > IRhigh)
      IRhigh = IRvalue;	
    if(IRvalue < IRlow)
      IRlow = IRvalue;	

    if (i<501) {
      continue;
    }
    
    // DC values are taken to be the average.. wut
    Reddc = Redsum/i;
    IRdc = IRsum/i;

    Redac_sq += pow (((long)(Redvalue - Reddc)), 2.0);
    IRac_sq += pow (((long)(IRvalue - IRdc)), 2.0);
  }		
	
  /* 
  // commenting out the 'finger not detected' part during debugging. 
  // THis will have to be recalibrated specifically to our system.
  if((Reddc < 0 && IRdc < 0) ||( Reddc>4000 && IRdc>4000)) {
     Serial.println("Finger not detected.");
     return 0;
  }   
  */

  Redac = sqrt(Redac_sq/(samples-500));
  IRac = sqrt(IRac_sq/(samples-500));

/*
  Serial.print("Reddc: "); 
  Serial.print(Reddc); 
  Serial.print("\t"); 
  Serial.print("Redhigh: ");                 
  Serial.print(Redhigh); 
  Serial.print("\t");
  Serial.print("Redlow: ");                 
  Serial.print(Redlow); 
  Serial.print("\t");
  Serial.print("Redac_sq: ");                 
  Serial.print(Redac_sq); 
  Serial.print("\t");
  Serial.print("Redac :"); 
  Serial.print(Redac); 
  Serial.print("\t");         
  Serial.print("IRdc: "); 
  Serial.print(IRdc); 
  Serial.print("\t");          
  Serial.print("IRhigh: ");                 
  Serial.print(IRhigh); 
  Serial.print("\t");
  Serial.print("IRlow: ");                 
  Serial.print(IRlow); 
  Serial.print("\t");     
  Serial.print("IRac_sq :")  ;
  Serial.print(IRac_sq); 
  Serial.print("\t");            
  Serial.print("IRac :"); 
  Serial.print(IRac); 
  Serial.print("\t"); 
  
  // take the ratio of ratios to find the spo2 value finally..
  float spo2 = (float)((float)Redac/Reddc)/(float)((float)IRac/IRdc);
  return spo2;
  */
}
