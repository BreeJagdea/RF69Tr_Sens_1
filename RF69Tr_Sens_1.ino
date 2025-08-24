/*
 http://arduino-info.wikispaces.com/Nrf24L01-2.4GHz-HowTo
 http://www.airspayce.com/mikem/arduino/RadioHead/index.html
              Arduino      RFM6C9W 
*               GND----------GND  (ground in)
*               3V3----------3.3V (3.3V in)
*  interrupt 0  D2-----------DIO0 (interrupt request out)
*           SS  D10----------NSS  (chip select in)
*          SCK  D13----------SCK  (SPI clock in)
*         MOSI  D11----------MOSI (SPI Data in)
*         MISO  D12----------MISO (SPI Data out)
*         
*         GPLv3
*/
//#define D_HTU21D
//#define  D_SI7021
//#define  D_TMP117
#define D_SHT31x


#include <types.h>
//#include <MemoryFree.h>
//#include <printf.h>
#include "LowPower.h"
#define xstr(s) str(s)
#define str(s) #s
#define sp(x)   Serial.print(x) 
#define spln(x) Serial.println(x) 

#include <SPI.h>
#include <Wire.h>
//***************************************************************************** Funk
#include <RH_RF69.h>
#include <RHReliableDatagram.h>
#define CLIENT_ADDRESS 67
#define SERVER_ADDRESS 76
#define TRIES       0   //Tries 1 will do 
#define BREAK_MS  100   //Der Empfänger ist ca 100 ms in seiner Verarbeitungsloop, 
                        //da macht es keinen Sinn, 50 mal alle 3 ms zu wiederholen
#define SLEEP_PIN  9

//RH_RF69 rf69(SS_PIN, INT_PIN)
RH_RF69 rf69 (10, 2); // UNO
RHReliableDatagram manager(rf69, CLIENT_ADDRESS);

struct transfer {
  int32_t recnr;
  float   temp;
  float   hum;
  float   press;
  int16_t sens_code;
  int32_t cs;
  } ;
transfer sens01;

//********************************************************************************* PowerDownSleep
#include <avr/wdt.h>

//********************************************************************************* Sensor

#ifdef D_SI7021
#include <SI7021.h>        //get it here: https://github.com/LowPowerLab/SI7021 //or here #include "HTU21D.h" https://github.com/sparkfun/SparkFun_HTU21D_Breakout_Arduino_Library.git
SI7021 ws_SI7021;                                                         
#endif

#ifdef D_SHT31x
#include "Adafruit_SHT31.h"
Adafruit_SHT31 sht31x = Adafruit_SHT31();
const uint8_t  sht31x_addr = 0x44;
#endif 

#include "Init_routines.h"
// IIC Devices  27       28     28       29       40       40        41      43         77     77
//              Display  HYT221 ChicCap2 tsl45315 SI7021   HDC1000   Watch   HDC1000    BME280 BMP180
//                                                                           ENS210

//********************************************************************************* Sonstiges
int32_t rec_counter = 0;
int32_t started_loop_at = 0;

 
void setup() //************************************************************************************************************** Setup
{
  Serial.begin(250000);
  Serial.print(F("RF69Tr_Sens V0 Generic Init Begin at:  "));Serial.print(millis());Serial.print("\r\n");   
  setPort(OUTPUT, LOW);
  digitalWrite(8,LOW); //Moteino-LED down Enable Peripherie once more
  //pinMode(A2, OUTPUT);digitalWrite(A2, HIGH);  //req if sensor energy has to be switched 
  //pinMode(A3, OUTPUT);digitalWrite(A3, LOW);  //req if sensor energy has to be switched
  delay(10);
  Wire.begin(); //Enable some IIC, To perform after setPort!
  Sensor_Init();
  Radio_Init();
  sens01.sens_code=12345;
  ADCSRA &= ~(1<<7); // ADCSRA=0; Strom sparen, machen wir auch bei jeden goto sleeep
  //Serial.print(F("RF69 ****>>>>Free Ram: ") ); Serial.println(freeMemory() );
  
  Serial.print(F("RF69Tr_Sens Generic Init Ends   at: ") );Serial.print(millis());Serial.print("\r\n");  
}
  
void loop() { //********************************************************************************************************** Void Loop

  uint32_t started_waiting_at;
  uint32_t wait_millis ;
  //digitalWrite(A2, HIGH);  //req if sensor energy has to be switched
  //digitalWrite(A3, LOW);  //req if sensor energy has to be switched
  //delay(90);    //req if sensor energy has to be switched //SI721 Powerup time 80ms  //besser durchlaufen lassen, 60 to 620 nA  (1E-7, 2.4E-6 Ah day, 7.2e-05 Ah/month) 

#ifdef D_SI7021  
  sens01.temp       =  ws_SI7021.getCelsiusHundredths() +20;                 
  sens01.hum        =  ws_SI7021.getHumidityPercent();                
#endif

#ifdef D_SHT31x 
  sht31x.readBoth(&sens01.temp, &sens01.hum);
#endif
 
  sens01.recnr = rec_counter;
  sens01.cs = sens01.recnr + (long) sens01.temp + (long) sens01.hum + (long) sens01.press + (long) sens01.sens_code; 
  if ( sens01.recnr <= 9) Serial.print("0");if ( sens01.recnr <= 99 ) Serial.print("0");
  Serial.print(sens01.recnr);  Serial.print(" -> ");
  Serial.print(sens01.temp);Serial.print(" *C  "); 
  Serial.print(sens01.hum); Serial.print("%  "); 
  //Serial.print(sens01.press);Serial.print(" hPa <-  ");    
  Serial.print(sens01.sens_code);Serial.print(" ");  Serial.print(sens01.cs);
  Serial.print(", MsgLen = ");  Serial.print(sizeof(sens01));      
  Serial.println(); 
  manager.resetRetransmissions();
  started_waiting_at = millis();
  
  digitalWrite(SLEEP_PIN,HIGH); // HIGH = ON
  manager.sendtoWait ((uint8_t *) &sens01, sizeof(sens01), SERVER_ADDRESS);    // *************************************************SEND*
  rf69.sleep();  //Current consummption withiut sleep 17 mA or 1 mA changin, 
  digitalWrite(SLEEP_PIN,LOW); // LOW = Off
  
  wait_millis =  millis() - started_waiting_at;  // typ 2 bi3 ms fuer sende ohne auf reply zu warten
  
  Serial.print("  TX = ");  if ( rec_counter <= 9) Serial.print("0");if ( rec_counter <= 99 ) Serial.print("0");
  Serial.print(rec_counter);
  Serial.print(", Time used is = "); Serial.print(wait_millis);
  Serial.print(", Retrans = "); Serial.print(manager.retransmissions());
  Serial.println(); 
  rec_counter++;

  digitalWrite(SLEEP_PIN,LOW); //AUS 0
  sp("Go to sleep");
  Serial.flush();

  digitalWrite(SLEEP_PIN,LOW); //AUS 0
  LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);  // 8S 4S 2S 1S 500MS 250MS 120MS 60MS 30MS 15 MS  
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);  // je 8 sec
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);  //  16
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);  //  24
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);  //  32
  
  spln(" ... Sleep done");
  
  
}//--(end main loop )---
//************************************************************************************************************************Main Loop End
/*
ISR(WDT_vect){  // Wenn lowpowerlab übernimmt, benötigt man den isr hier nicht
  delay(2);     // Falls, commeent the lowpowerlb.h
  Serial.println("Woke Up ISR**********************************************************************************");  
}// watchdog interrupt
*/

/*
void getPressure(float *p, float *t ) //***********************************************************************************getPressure
{
  char status; double P=0; double T=0 ;
  
  status = ws_BMP180.startTemperature();
  if (status != 0)
  {
    delay(status); // Wait for the measurement to complete, status is HEX 05, typ
    status = ws_BMP180.getTemperature(T); *t = T;
    if (status != 0)
    {
      status = ws_BMP180.startPressure(3); //0 -Low Res; 3 - High res
      if (status != 0)
      {
        delay(status); // Wait for the measurement to complete, typ 1A
        status = ws_BMP180.getPressure(P,T);
        if (status != 0)
        {
          *p = P; return;
        }
      }
    }        
  }
  *p = 0;  return;
}
*/

  /* 
  WDTCSR = (24);      //change enable and WDE also resets
  WDTCSR = (33);      //prescaler only get rid of WDE & WDCE bit 
  WDTCSR |= (1<<6);   //enable interrupt mode
  ADCSRA &= ~(1<<7); // ADCSRA=0;
  SMCR |= (1<<2);    // power down mode
  SMCR |= 1;         // enable Sleep
  __asm__ __volatile__("sleep");      // we go bed
   ADCSRA |= (1 << ADEN);           // ADC not req, we leav it off !
  */

// EoF
