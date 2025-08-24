void Sensor_Init()   //**************************************************************************** INIT Sensor  
 {
  Serial.print(F(" Sensor_Init begins at: "));Serial.print(millis());Serial.print("\r\n");
#ifdef D_SI7021
  uint16_t si = 0;
  si = ws_SI7021.begin();
  if ( si == 1 ) {                         //1 ist gut  
     Serial.print(F("  SI7021 initalised ok, ID = "));
     Serial.print(ws_SI7021.getDeviceId()); Serial.println();
  } else {
     Serial.print(F("  SI7021 initalised FAILED, ") );  
     Serial.print(si, HEX); Serial.println();
  }
#endif

#ifdef D_SHT31x
Serial.println(F("     SHT31x Begins" ));
  if ( sht31x.begin(sht31x_addr) == true  ) {                         // Return ist true==OK or false wenn IIC start nicht geht 
     Serial.print(F("       SHT31x initialisation ok"));delay(50);
     Serial.print(" Status= ");Serial.println(sht31x.readStatus(), HEX);
  } else { 
     Serial.println(F("       SHT31x initialisation FAILED ") ); 
  }  
  Serial.print(F("       SHT31x Heater State: "));
  if (sht31x.isHeaterEnabled())
    Serial.println(F("       ENABLED"));
  else
    Serial.println(F("       DISABELED"));
Serial.println(F("     SHT31x processed"));
#endif
  Serial.print(F(" Sensor_Init ends   at: ") );Serial.print(millis());Serial.print("\r\n");
}

void Radio_Init() {  //***************************************************************************** INIT Radio  
Serial.print(F(" Radio_Init begins  at: "));Serial.print(millis());Serial.print("\r\n"); 
  
  if (!manager.init())                                
    Serial.println(F("  RF69Tr_Sens RF69 Init Failed") );
  else  
    Serial.println(F("  RF69Tr_Sens RF69 Init OK") );
    
  if (!rf69.setFrequency(868.0))
    Serial.println(F("  setFrequency 868 failed") );

  //RF69W,  valid values are from -18 to +13 , 
  //RF69HW, valid values are from +14 to +20 (H-High Power)
  //RFM69HCW
  int8_t power = +20;  //Works rarely with 0, using 13 we are on the right site for balcony flat;//to high val are not acc'ed.
  Serial.print(F("  Radio_Init Set Power    : "));Serial.print(power); Serial.print("\r\n");
  rf69.setTxPower(power); //Other rfm69
  
  Serial.print(F("  Radio_Init set SleepMode: ") );Serial.print(RH_RF69_OPMODE_MODE_SLEEP); Serial.print("\r\n");
  rf69.setIdleMode(RH_RF69_OPMODE_MODE_SLEEP);

  uint8_t key[] = { 0x71, 0x62, 0x53, 0x44, 0x35, 0x26, 0x17, 0x08,
                    0x81, 0x72, 0x63, 0x54, 0x45, 0x36, 0x27, 0x18};
  rf69.setEncryptionKey(key);
  rf69.setModemConfig(RH_RF69::GFSK_Rb250Fd250);

  manager.setRetries((uint8_t)  TRIES);     // tries
  manager.setTimeout((uint16_t) BREAK_MS);  //ms wait for ACK , 1 ms funzt net, 2 ghet, 3oder4 verm sinnvoll; 
                                     // Am ende wartet man halt 40 oder 50 ms, wenn die störung stört
  Serial.print(F("  Radio_Init set Retries  : ") );Serial.print(manager.retries());
  Serial.print(F(", Each ") );Serial.print(BREAK_MS);Serial.print("ms");
  Serial.print(F(", Notify Time ") );Serial.print(manager.retries()*BREAK_MS);
  Serial.println();

  Serial.print(F(" Radio_Init ends   at: ") );Serial.print(millis());Serial.print("\r\n");
}

void setPort(bool mode, bool hl){ //************************************************************************* setPorts
  Serial.print(F(" setPort begins   at:  "));Serial.print(millis());Serial.print("\r\n"); 
  sp("  Set to: "); spln(hl);
  for (int i = 2; i<=20; i++)
  {  
    pinMode (i, mode);  
    if ( mode == OUTPUT )
        { digitalWrite (i, hl); }
  }
  //All pins as  outputs LOW: 0.35 µA and HIGH: 1.86 µA.
  //All pins as  inputs  LOW (internal pull-ups disabled): 0.35 µA and HIGH (internal pull-ups enabled): 1.25 µA.
  Serial.print(F(" setPort ends     at:  "));Serial.print(millis());Serial.print("\r\n");
}
