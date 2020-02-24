/* Author: Vincent K
  This script is to test out the particle detector using a serial connection 
  with an Arduino. 
*/


#include <Wire.h>
#include <LiquidCrystal.h>
#include <SoftwareSerial.h>

SoftwareSerial portOne(2,3); //Instance of software serial used to get around rx/tx pin use

const int Datapoints = 30; // Change variale to fine-tune lowpass filter

float PM10Read[Datapoints];
float PM25Read[Datapoints];
float TotalOutputPM10 = 0;
float TotalOutputPM25 = 0;

int Index, counter, counter2, AverageOutputPM10, AverageOutputPM10 = 0

unsigned volatile int timer1_counter; //volatile used for timer counter

float PM25, PM10 = 0; 

LiquidCrystal lcd(8,9,4,5,6,7); //using pins for RS, enable, 4,5,6,7 for LCD-Arduino

///////////////////////////////////////////////////////////////////////////////////////

ISR(TIMER1_OVF_vect) //timer interrupt 
{
  uint8_t Data = 0; //holds value of byte from sensor
  uint8_t i = 0;
  uint8_t Pkt[10] = {0} //array for entire packet
  uint8_t Check = 0 //checksum
  
  //from data sheet "AA C0 PM25_LOW PM25_HIGH PM10_LOW PM10_HIGH 0 0 CRC AB"
  portOne.begin(9600);
  Data = portOne.read();
  delay(2); //wait until packet goes through, make 5ms if needed
  if(Data == 0xAA) //is head ok?
  {
    Pkt[0] = Data;
    Data = portOne.read();
    
    if (Data = 0xc0) //no. ok?
    {
      Pkt[1] = Data;
      Check = 0;
      for (i=0; i<6; i++) // data recieved and crc calculated
      {
        Pkt[i+2] = portOne.read();
        delay(2); 
        Check += Pkt[i+2]; //Checksum 
      }
      Pkt[8] = portOne.read();
      Pkt[9] = portOne.read();
      if(Check == Pkt[8]) //CRC ok
      {
        portOne.flush();
        //Serial.write(mPkt,10); //comment out unless debugging

        //PM (ug/m3) = (PM_HIGH * 256 + PM_LOW) / 10
        
        PM25 = (uint16_t)Pkt[2] | (uint16_t)(Pkt[3] << 8); //PM25 value
        PM10 = (uint16_t)Pkt[4] | (uint16_t)(Pkt[5] << 8); //PM10 value
        
        //limit to 9999 due to LCD limitations
        if (PM25 > 9999)
          {PM25 = 9999;}
        if (PM10 > 9999)
          {PM10 = 9999;}
        
        return;
       }
     }
   }
   TCNT1 = timer1_counter;
   counter++;
   counter2++;
}


///////////////////////////////////////////////////////////////////////////////////////

void setup() 
{
  noInterrupts(); //disable all interrupts as a precaution until ready
  lcd.begin(16,2); //LCD setup
  TCCR1A = 0; //registers for timer 
  TCCR1B = 0;
  
  int Period = 1; 
  timer1_counter = (65536-(62500*Period)); //
  TCNT1 = timer1_counter; //preload timer
  TCCR1B |= (1 << CS12) ; //256 prescalar
  TIMSK1 |= (1 << TOIE1); //enable timer overflow interrupt
  
  for (int i =0; i < Datapoints; i++)
  {
    PM10Read[i] = 0;
    PM25Read[i] = 0;
  }
  
  //Connection to sensor
  Serial.begin(9600); //can never be too careful 
  portOne.begin(9600);
  PM25 = 0;
  PM10 = 0;
  interrupts(); //enable all interrupts
  
}

void loop() 
{
  lcd.clear(); //reminder: create LCD library for quicker function implementation
  
  if(counter > 3)
  {
    TotalOutputPM10 = TotalOutputPM10 - PM10Read[Index];
    TotalOutputPM25 = TotalOutputPM25 - PM25Read[Index];
    
    //read from the sensors:
    PM10Read[Index] = (PM10/10);
    PM25Read[Index] = (PM25/10);
    
    //add reading to the total:
    TotalOutputPM10 = TotalOutputPM10 + PM10Read[Index];
    TotalOutputPM25 = TotalOutputPM25 + PM25Read[Index];
    
    //next pose in the array
    Index++
    
    //if at end of array
    if(Index >= Datapoints)
    {
      Index = 0;
    }
    
    //calculate the average:
    AverageOutputPM10 = round(TotalOutputPM10/Datapoints);
    AverageOutputPM25 = round(TotalOutputPM25/Datapoints);
    
    if(AverageOutputPM10>999)
    {
      AverageOutputPM10 = 999;
    }
    if(AverageOutputPM25>999)
    {
      AverageOutputPM25 = 999;
    }
    
    delay(1);
    counter = 0;
  }
  
  //display results on LCD after measurement phase
  
  lcd.setCursor(0,0); //set cursor to row 0, column 0.
  lcd.print("PM2.5: ");
  lcd.print(AverageOutputPM25);
  lcd.setCursor(11,0);
  lcd.print("ug/m3");
  
  lcd.setCursor(0,1); 
  lcd.print("PM10: ");
  lcd.print(AverageOutputPM10);
  lcd.setCursor(11,1);
  lcd.print("ug/m3");
  
  
  //uncomment for using serial print over USB to debug
  /* 
  if(counter2 > 2)
  {
    Serial.print(millis());
    Serial.print("PM2.5");
    Serial.print(float(PM25)/10.0);
    Serial.println();
    counter2 = 0
  }
  */
  
  delay(200); // tweak to prevent LCD flickering (Cheap LCD)
}
