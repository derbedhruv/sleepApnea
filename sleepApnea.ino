/*
  MODIFIED FROM SD card datalogger
     
 The circuit:
 * analog sensors on analog ins 0, 1, and 2
 * SD card attached to SPI bus as follows:
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK - pin 13
 ** CS - pin 4
 
 originally created  24 Nov 2010
 by Tom Igoe
 
 This example code is in the public domain.
     
 */

#include <SD.h>

String dataString = "";    // this is going to contain each line that shall be written to the SD card
File dataFile;             // the file shall be made a global variable
const int chipSelect = 4;

void setup()
{
 // Open serial communications and wait for port to open:
 // this shall be removed later yo
  Serial.begin(9600);
  
  // Serial.println("Initializing SD card...");
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(10, OUTPUT);
  
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  // Serial.println("card initialized.");
  delay(500);    // giving some delay because without that it behaves weirdly
  
  // next, we go about initializing a timer interrupt for 100Hz...
  cli();//stop interrupts

//set timer1 interrupt at 1Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 1952;// = (16*10^6) / (8*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  sei();//allow interrupts
  
  delay(500);    // more delays = delayed climax, oh yes
}

void loop()
{
  // do nothing, really.
}

ISR(TIMER1_COMPA_vect){  //timer1 interrupt 8Hz 
  // read three sensors and append to the string:
  for (int analogPin = 14; analogPin < 17; analogPin++) {
    int sensor = analogRead(analogPin);
    dataString += String(sensor);
    if (analogPin < 16) {
      dataString += ","; 
    }
  }

  Serial.println(dataString);

// open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  dataFile = SD.open("log.txt", FILE_WRITE);
  
  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(dataString);
    dataString = "";
    dataFile.close();
  }  
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }   
}
