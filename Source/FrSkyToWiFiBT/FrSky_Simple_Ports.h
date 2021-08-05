//=================================================================================================  
//================================================================================================= 
//
//                                    F . P O R T   C L A S S
//
//================================================================================================= 
//=================================================================================================

   #define fpStatusLed    99        // not used in this application
   #define InvertfpLed    false

  //    forward declaration of main and utility functions
  
  void      nbdelay(uint32_t);
  void      LogScreenPrintln(String);
  void      Printbyte(byte, bool, char);
  void      PrintFrPeriod(bool);
  void      PrintLoopPeriod();

  class     FrSkyPort 
  { 
    // Data members 
  public: 
    float     fr_lat = 0;    // all moved to public for HUD info display
    float     fr_lon = 0;
    float     fr_bat1_volts;     
    float     fr_bat1_amps;
    uint16_t  fr_bat1_mAh;    
    uint8_t   fr_numsats;   
    uint32_t  fr_rssi;   
    bool      gpsGood = false;
      
  private:
    static const uint8_t timeout_secs = 8;  // FrSky port no traffic timeout
    
    bool      ftgetBaud = true; 
    byte      chr = 0;
    byte      prev_chr = 0;    
    bool      fpInit = false; 
    bool      serGood = false;
    bool      frGood = false;
    bool      frPrev = false; 
    bool      pwmGood = false; 
    bool      pwmPrev = false;

    bool      lonGood = false;
    bool      latGood = false;
    bool      altGood = false;
    bool      hdgGood = false;
    bool      hdopGood = false;
        
    uint32_t  frGood_millis = 0;   
    uint32_t  serGood_millis = 0;  
    uint32_t  pwmGood_millis = 0;       

    bool      parseGood = false;
    uint32_t  goodFrames = 0;
    uint32_t  badFrames = 0;
    uint32_t  packetloss_millis = 0;
    uint8_t   fpLedState = LOW; 
    uint32_t  fpLed_millis = 0;           

    static const uint8_t max_ch = 26;         // 24 + 2 digi ch
    int16_t   pwm_ch[max_ch];                 // PWM Channels
    uint8_t   pwm_rssi = 0;
  
    int16_t   crcin = 0;                      // CRC of inbound frsky frame   
    int16_t   crcout = 0;   
    uint16_t  lth = 0;

    uint8_t   fr_lth = 0;
    uint8_t   fr_type = 0;
    uint8_t   fr_prime = 0;
    
    volatile uint8_t *uartC3;
    
    enum PortMode { rx , tx };
    PortMode mode, modeNow;

    bool      printcrcout = false;
    byte      nb, lb;                      // Nextbyte, Lastbyt of ByteStuff pair     

    uint8_t   prevbyt=0;

    bool      frInvert = false;
    uint32_t  frBaud = 0; 
    uint16_t  fr_idx = 0;
    uint8_t   p_fr = 0;   

    frport_t  frport_type = f_none; 
    
    static const uint16_t  fr_max = 48;           // f.port len, type, 35(24ch control) + CRC = 38 + headroom 
    byte      frbuf[fr_max];
    
    bool latlon_800_flag=false;     // 0800  
    bool lat_800_flag=false;        // 0800
    bool lon_800_flag=false;        // 0800  
    bool ST_5000_flag = false;      // 5000  On demand
    bool AP_5001_flag = false;      // 5001  
    bool GPS_5002_flag = false;     // 5002  
    bool Bat_5003_flag = false;     // 5003  
    bool Home_5004_flag = false;    // 5004  
    bool velyaw_5005_flag = false;  // 5005  
    bool AT_5006_flag = false;      // 5006  
    bool Param_5007_flag = false;   // 5007  3 x each at init
    bool Param_50071_flag = false;  // 5007 id1 
    bool Param_50072_flag = false;  // 5007 id2 
    bool Param_50073_flag = false;  // 5007 id3 
    bool Param_50074_flag = false;  // 5007 id4
    bool Param_50075_flag = false;  // 5007 id5
    bool Param_5008_flag = false;   // 5008 
    bool Param_F101_flag = false;   // f101 
   
    //FrSky Variables
    short ms2bits;
    uint32_t fr_payload;

    // FrSky Passthrough Variables
    // 0x800 GPS
    uint32_t fr_latlong;
    //float fr_lat = 0;    // moved to public for hud info display
    //float fr_lon = 0;

    // 0x5000 Text Msg
    uint32_t fr_textmsg;
    char ct[5];
    char p_ct[5];
    int ct_dups=0;
    uint8_t fr_severity;
    char fr_text[80];
    bool eot=false;

    // 0x5001 AP Status
    uint8_t fr_flight_mode;
    uint8_t fr_simple;

    uint8_t fr_land_complete;
    uint8_t fr_armed;
    uint8_t fr_bat_fs;
    uint8_t fr_ekf_fs;

    // 0x5002 GPS Status
    //uint8_t fr_numsats;     // moved to public for hud info display
    
    uint32_t fr_gps;  
    uint8_t fr_gps_status;
    uint8_t fr_hdop;
    uint32_t fr_gps_alt;
    //float   fr_fgps_alt;
    uint8_t neg;

    //0x5003 Batt
    //float fr_bat1_volts;     // moved to public for hud info display
    //float fr_bat1_amps;
    //uint16_t fr_bat1_mAh;

    // 0x5004 Home
    uint16_t fr_home_dist;
    float    fr_fhome_dist;
    uint32_t fr_home_angle;
    int32_t fr_home_alt;
    float   fr_fhome_alt;
    //float fr_home_angle;
    short fr_pwr;

    // 0x5005 Velocity and yaw
    uint32_t fr_velyaw;
    float fr_yaw;
    float fr_vx;
    float fr_vy;

    // 0x5006 Attitude and range
    float fr_roll;
    float fr_pitch;
    float fr_range;

    // 0x5007 Parameters  - Sent 3x each at init
    uint8_t fr_param_id ;
    uint32_t fr_param_val;
    uint32_t fr_frame_type;
    uint32_t fr_fs_bat_volts;
    uint32_t fr_fs_bat_mAh;
    uint32_t fr_bat1_capacity;
    uint32_t fr_bat2_capacity;

    //0x5008 Batt
    float fr_bat2_volts;
    float fr_bat2_amps;
    uint16_t fr_bat2_mAh;

    //0xF101 RSSI 
    //uint32_t fr_rssi;   // moved to public for hud info display


    // Member function prototypes
  public:  
    void          initialise(frport_t frport_type);   
    void          HandleTraffic();
    void          ReportFrPortOnlineStatus();      
            
  private:  
    pol_t         getPolarity(uint8_t rxpin); 
    uint32_t      getBaud(uint8_t rxpin); 
    uint32_t      getConsistent(uint8_t rxpin);
    uint32_t      SenseUart(uint8_t  rxpin);            
    frport_t      Sense_FrPort(); 
    bool          FPort_Read_A_Frame(uint8_t *buf, frport_t  frport_type);  
    bool          SPort_Read_A_Frame(uint8_t *buf);           
    void          setMode(PortMode mode);
    byte          ReadByte();
    byte          SafeRead();
    void          WriteByte(byte b);   
    void          SafeWrite(byte b, bool isPayload);
    void          ReportOnlineStatus(); 
    bool          ParseFrame(uint8_t *buf, frport_t  frport_type, uint16_t lth); 
    void          Frs_Decode(uint8_t *buf);     
    uint8_t       crcGet(uint8_t *buf, uint8_t lth);
    bool          crcGood(uint8_t *buf, uint8_t lth);                
    bool          BytesToPWM(uint8_t *buf, int16_t *ch, uint8_t lth); 
    void          crcStepIn(uint8_t b);     
    void          crcStepOut(uint8_t b);  
    void          crcStep(int16_t *mycrc, uint8_t b); 
    void          crcEnd(int16_t *mycrc);          
    void          WriteCrc(); 
    void          Print_PWM_Channels(int16_t *ch, uint16_t max_ch); 
    void          PrintBuffer(uint8_t *buf, uint8_t lth);   
     void         PrintFrPeriod(bool LF);      
    void          CheckForTimeout();   
    void          ServiceStatusLed();
    void          BlinkFpLed(uint32_t period);
    bool          Send_WiFi_BT_Frame(); 
    #if (defined btBuiltin)    
      bool          Send_Bluetooth(uint8_t *buf, uint16_t len);   
    #endif         
    bool          Send_UDP(uint8_t *buf, uint16_t len);
    uint32_t      TenToPwr(uint8_t pwr);    
    uint32_t      bit32Extract(uint32_t dword,uint8_t displ, uint8_t lth);
    uint16_t      uint16Extract(uint8_t *buf, int posn);
    uint32_t      uint32Extract(uint8_t *buf, int posn); 
    uint32_t      createMask(uint8_t lo, uint8_t hi);       
        
}; // end of class

    // External member functions
    //===================================================================

    void FrSkyPort::initialise(frport_t  frport_type)  {  
    //set.frport = s_port;
    if (set.frport == f_auto) {
      pol_t pol = (pol_t)getPolarity(fr_rxPin);
      bool ftp = true;
      static int8_t cdown = 30;     
      while ( (pol == no_traffic) && (cdown) ){
        if (ftp) {
          Log.printf("No telem on rx pin:%d. Retrying ", fr_rxPin);
          String s_rxPin=String(fr_rxPin);   // integer to string
          LogScreenPrintln("No telem on rxpin:"+ s_rxPin); 
          ftp = false;
        }
        Log.print(cdown); Log.print(" ");
        pol = (pol_t)getPolarity(fr_rxPin);
        delay(500);      
        if (cdown-- == 1 ) {
          Log.println();
          Log.println("Auto sensing abandoned. Defaulting to S.Port");
          LogScreenPrintln("Default to SPort!");
          pol = idle_low;  
          frport_type = set.frport = s_port;  
          ftp = true;     
        }

      }
      if (!ftp) Log.println();
      
      if (pol == idle_low) {
        frInvert = true;
        Log.printf("Serial port rx pin %d is IDLE_LOW, inverting rx polarity\n", fr_rxPin);
      } else {
        frInvert = false;
        Log.printf("Serial port rx pin %d is IDLE_HIGH, regular rx polarity retained\n", fr_rxPin);        
      }
      
      if (set.frport == f_auto) {
        frBaud = getBaud(fr_rxPin);
        Log.print("Baud rate detected is ");  Log.print(frBaud); Log.println(" b/s"); 
        String s_baud=String(frBaud);   // integer to string. "String" overloaded
        LogScreenPrintln("Telem at "+ s_baud);  
      }
    } 
    
    if (set.frport == s_port) {
        frInvert = true;
        frBaud = 57600;  
        Log.printf("Default S.Port on rxpin:%d is IDLE_LOW, inverting rx polarity\n", fr_rxPin);  
        Log.printf("Default S.Port baud:%d b/s\n", frBaud);           
    } 
    if ( (set.frport == f_port1) && (set.frport == f_port2) ) {  
        frInvert = false;
        frBaud = 115200; 
        Log.printf("Default F.Port on rxpin:%d is IDLE_HIGH, regular rx polarity retained\n", fr_rxPin);   
        Log.printf("Default F.Port baud :%d b/s\n", frBaud);               
    }              

      #if (defined ESP32) || (defined ESP8266) // ESP only
        int8_t frRx;
        int8_t frTx;

        frRx = fr_rxPin;
        frTx = fr_txPin;
        delay(100);
        #if ( (defined ESP8266) || ( (defined ESP32) && (defined ESP32_SoftwareSerial)) )
            frSerial.begin(frBaud, SWSERIAL_8N1, frRx, frTx, frInvert);     // SoftwareSerial
            Log.println("Using SoftwareSerial for F.Port");
        #else  // HardwareSerial
          // waits here for a few seconds timeout if no telemetry present
          frSerial.begin(frBaud, SERIAL_8N1, frRx, frTx, frInvert);                     
        #endif
        delay(10);
      #endif
   
    } // end of member function

    //===================================================================
    void FrSkyPort::HandleTraffic() {  

      if (frport_type == f_none) {
        if (set.frport== f_auto) {     
          frport_type = Sense_FrPort();         
          if (frport_type == s_port) {
            Log.println("SPort detected");         
          } else
          if (frport_type == f_port1) {        
            Log.println("FPort1 detected");   
          } else
          if (frport_type == f_port2) {        
            Log.println("FPort2 detected");   
          }
        } else {
          frport_type = set.frport;
        } 
      }
       
      if (frport_type == s_port) {  
        if ( SPort_Read_A_Frame(&FrSkyPort::frbuf[0]) ) {
          #if (defined Debug_FrPort_Buffer) 
             Log.print("Good FrSky Frame Read: ");
             FrSkyPort::PrintBuffer(frbuf, 10);
          #endif           
          FrSkyPort::Frs_Decode(&frbuf[2]); 
          FrSkyPort::Send_WiFi_BT_Frame();
        }
      }  else
      if ( (frport_type == f_port1) || (frport_type == f_port2) ) { 
        if ( FPort_Read_A_Frame(&FrSkyPort::frbuf[0], frport_type) ) {
          FrSkyPort::Frs_Decode(&frbuf[2]);             
          FrSkyPort::Send_WiFi_BT_Frame();
        }
      }
        
      FrSkyPort::CheckForTimeout();
    }
  
     //===================================================================  
          
    frport_t FrSkyPort::Sense_FrPort() {
      FrSkyPort::setMode(rx);
      lth=frSerial.available();  
      if (lth < 10) {
        if (lth > 0) {
          //Log.printf("lth=%d\n", lth); 
        }
        return f_none;       // prevent 'wait-on-read' blocking
      }
      static int8_t   sp = 0;   
      static int8_t   fp1 = 0; 
      static int8_t   fp2 = 0;  
      uint8_t          b = 0;  
      static uint8_t   prev_b = 0; 
      b = FrSkyPort::ReadByte();
      if ((prev_b == 0x7E) && (b == 0x7E)) {
        fp1++;
        sp--;   
        if (fp1 > 10) return f_port1;  // fport1
      } else
      
      if (b == 0x7E) {
        sp++;
        if (sp > 10) return s_port;  // sport 
      } else
      
      if ( ( (prev_b == 0x0D) || (prev_b == 0x18)|| (prev_b == 0x23) ) && (b == 0xff) )  {       // fp2 control   
        fp2++;        
        if (fp2 > 10) return f_port2;  // fport2
      } else
      
      if ( ( (prev_b == 0x0D) || (prev_b == 0x18)|| (prev_b == 0x23) ) && ( (b == 0xf1) || (b == 0xf1) ) )  {  // fp2 OTA     
        fp2++;      
        if (fp2 > 10) return f_port2;
      } else
      
      if ( (prev_b == 0x08) && ( (b >= 0) && (b < 0x1b) ) )   {     // fp2 downlink  
        fp2++;     
        if (fp2 > 10) return f_port2;
      } 
      
      prev_b = b;
      return f_none;  // non blocking function

    }    
    //===================================================================
 
    bool FrSkyPort::SPort_Read_A_Frame(uint8_t *buf) {

      static uint8_t i = 0;
      byte b;
      
      #if defined Report_Packetloss
       uint32_t period = (Report_Packetloss * 60000);
       if (millis() - packetloss_millis > period) {
         packetloss_millis = millis();
         float packetloss = float(float(badFrames * 100) / float(goodFrames + badFrames));
         Log.printf("S.Port goodFrames:%d   badFrames:%d   frame loss:%.3f%%\n", goodFrames, badFrames, packetloss);
       }      
      #endif  
  
      delay(1);            // I am important!
      
      while (frSerial.available()) {
    
        if (b == 0x7E) {  // end of frame parse
          if (i == 3) {
            memset(&buf[2], 0x00, fr_max-2); // clear the rest
          }
          //Log.printf("0x7E found  i:%d  ", i);   PrintBuffer(buf, 10);
          buf[0] = b;
          i = 1;  
          if (buf[1] == 0x1b) {   // our DIY sensor)
            //Log.printf("0x1B found, buf[2]:%X ", buf[2]);                 
          }
          if (buf[2] == 0x10) {
            if (buf[9] == (0xFF-crcin)){  // Test CRC
              frGood = true;            
              frGood_millis = millis();  
              goodFrames++;               
              crcin = 0;
              return true;              // RETURN
            } else {
              badFrames++;              
              //Log.print(" CRC Bad!: ");          
            }
          }
          crcin = 0;
        }  // end of b == 0x7E
         
        b = SafeRead();
        //Printbyte(b, true, ','); Log.printf(":i[%d] ", i);
        if (b != 0x7E) {  // if next start/stop don't put it in the buffer
          if ((i > 1) && (i < 9))  crcStepIn(b);           
        }
        buf[i] = b;              
        if (i<fr_max-1) i++;          
      }
      return false;     
    }
    //===================================================================
    
    bool FrSkyPort::FPort_Read_A_Frame(uint8_t *buf, frport_t  frport_type) {
      /*
       Master arranges timing of transaction, slave responds
       Master sends downlink frame just after channel (control) frame
       We are a slave, and default to receiving status      
       Slave responds with uplink frame immediately if matching ID received
      */

      lth=frSerial.available();  
      if (lth < 10) {
        if (lth > 0) {
          //Log.printf("lth=%d\n", lth); 
        }
        return false;       // prevent 'wait-on-read' blocking
      }
          
      #if defined Report_Packetloss
        uint32_t period = (Report_Packetloss * 60000);
        if (millis() - packetloss_millis > period) {
          packetloss_millis = millis();
          float packetloss = float(float(badFrames * 100) / float(goodFrames + badFrames));
          Log.printf("F.Port goodFrames:%d   badFrames:%d   frame loss:%.3f%%\n", goodFrames, badFrames, packetloss);
        }
      #endif

      // ======================= F.Port1 ==========================
      
      if (frport_type == f_port1) {           // find start of frame 
        while (!(chr==0x7E)) {     // find first 0x7E, should be start, but could be previous stop
          chr = ReadByte();
        }
        buf[0] = chr;
        chr = ReadByte();    // could be start 0x7E or len 0x08, 0x0D, 0x18, 0x20, 0x23  
        while (chr == 0x7E) {           // if start 0x7E, then the first one was a stop so ignore it
          *buf = chr;
          chr = ReadByte();
        }
        fr_lth = *(buf+1) = chr;                  // lth
        fr_type = *(buf+2) = ReadByte();          // frame_type  
        
        if ((fr_lth == 0x08) || (fr_lth == 0x19) ) {  // downlink or control frame
          frGood = true;            
          frGood_millis = millis();  
  
          switch(fr_type){
            case 0x00:      // F.Port v1.0 Control Frame (RC Channels)
              parseGood = ParseFrame(buf, frport_type, fr_lth);
              if (parseGood) {
                #if defined Derive_PWM_Channesl           
                  pwmGood = BytesToPWM(buf+3, &pwm_ch[0], fr_lth);
                  if (pwmGood) {
                    #if defined Debug_PWM_Channels
                      Print_PWM_Channels(&pwm_ch[0], num_of_channels);
                    #endif  
                    pwmGood_millis = millis();
                  }
                #endif  
                #if defined Support_SBUS_Out 
                #endif
                return true;    
              }  
              return false;
  
            case 0x01:      // F.Port v1.0 downlink frame from master  -  match on our sensor byte ( range 0x0~0x1B or 0x1E (FC) )              
              parseGood = ParseFrame(buf, frport_type, fr_lth); 
              if (parseGood) {    
                fr_prime = buf[3];                         // if prime == 0x00, reply prime = 0x00
                                                           // if prime == 0x10, reply prime = 0x10 (slave ready)                                                       
                return true;  
              }
              return false;  
              
            case 0x0D:   // mavlite downlink frame from master, match on sensor id 0x0D                          
              parseGood = ParseFrame(buf,  frport_type, fr_lth); 
              if (parseGood) {
                fr_prime = buf[3];                         // should be 0x30                                                                                                              
                // Mavlite   do something
                return false;      
              }
              return false;
                    
            default: 
              //   Log.printf("Unknown frame type = %X\n", fr_type);  
              return false;     
          }       // end of switch   
        } else {  // end of length ok
          badFrames++; // frame length error due to fail length test
          //Log.printf("Bad FPort frame length = %X\n", fr_lth); 
          return false; 
        }     
      }          // end of FPort1

      // ======================= F.Port2 ==========================
      
      if (frport_type == f_port2) {                // find start of frame
        bool ctl = false;
        bool ota = false; 
        bool dlink = false;
        while ( (!(ctl)) && (!(ota)) && (!(dlink)) ) {     // find valid lth + type combo
          prev_chr = chr;
          chr = ReadByte();
          ctl = (((prev_chr == 0x0D) || (prev_chr == 0x18)|| (prev_chr == 0x23)) && (chr == 0xFF)); 
          ota = (((prev_chr == 0x0D) || (prev_chr == 0x18)|| (prev_chr == 0x23)) && (chr == 0xF1)); 
          dlink = ((prev_chr == 0x08)  && (chr == 0x1B));         
        }  
            
        *(buf) = 0;                     // not used for fp2
        fr_lth = *(buf+1) = prev_chr;   // lth 
        fr_type = *(buf+2) = chr;       // frame_type 
                                        
        if ((fr_lth == 0x08) || (fr_lth == 0x0d) || (fr_lth == 0x18) || (fr_lth == 0x20) ) {  // 
          frGood = true;            
          frGood_millis = millis();  
          //Log.printf("fr_lth:%d   fr_type:%X\n", fr_lth, fr_type);   
    
          switch(fr_type){
            
            case 0x0D:   // mavlite downlink frame from master, match on sensor id 0x0D            
               
              parseGood = ParseFrame(buf+1,  frport_type, fr_lth); 
              if (parseGood) {
                fr_prime = buf[3];                         // should be 0x30                                                                                                              
                return false;  // ignore mavlite for now
              }
              return false; 
                    
            case 0x1B:   // F.Port v2.3.7 downlink frame from master, match on sensor id 0x0~0x1B or 0x1E (FC)                     
              parseGood = ParseFrame(buf,  frport_type, fr_lth); // + CRC
              //printf("fr:type:%X parseGood:%d=================<\n", fr_type, parseGood);
              if (parseGood) {
                fr_prime = buf[3];                         // if prime == 0x00, reply prime = 0x00
                                                           // if prime == 0x10, reply prime = 0x10 (slave ready)                                                                                                                 
               return true; 
              }
              return false;

            case 0xff:      // F.Port v2.3.7 Control Frame (RC Channels)  
              parseGood = ParseFrame(buf,  frport_type, fr_lth+1); // + CRC
              //printf("fr:type:%X parseGood:%d\n", fr_type, parseGood);              
              if (parseGood) {
                #if defined Derive_PWM_Channesl           
                  pwmGood = BytesToPWM(buf+3, &pwm_ch[0], fr_lth);
                  if (pwmGood) {
                    #if defined Debug_PWM_Channels
                      Print_PWM_Channels(&pwm_ch[0], num_of_channels);
                    #endif  
                    pwmGood_millis = millis();
                  }
                #endif  
                #if defined Support_SBUS_Out      
                #endif
                return true;     
              }
 
              return false;

            case 0xf0:      // OTA start frame
              return false;
            case 0xf1:      // OTA data frame
              return false;
            case 0xf2:      // OTA end frame
              return false;    
            default: 
           //   Log.printf("Unknown frame type = %X\n", fr_type);  
              break;           
          }       // end of switch   
        } else {  // end of length ok
          badFrames++; // due to fail length test
          return false;
          // Log.printf("Bad FPort frame length = %X\n", fr_lth);  
        }     
      }          // end of FPort2

      // No start/stop 

    }

    //===================================================================   
 
    bool FrSkyPort::ParseFrame(uint8_t *buf, frport_t  frport_type, uint16_t lth) {
      //Log.printf("buf[0]:%X  buf[1]:%X  buf[2]:%X\n", buf[0], buf[1], buf[2]);
      uint8_t crc_lo, crc_hi;
         
      int i = 0;
      for (i = 3 ; i < lth+2 ; i++) {        
        chr = SafeRead();          // f_port2 ignores start byte[0]
        //if (*(buf+2) == 0x1b) Log.printf("i:%d  chr:%X\n", i, chr);
        *(buf+i) = chr;
      }
      
       chr = SafeRead();           // this is the crc byte
       *(buf+i) = chr;      
       
      #if (defined Debug_FrPort_Buffer) 
        PrintBuffer(buf, lth+4);
      #endif 

      bool mycrcGood =  0; 
      if (frport_type == f_port1) {
        mycrcGood =  crcGood(buf+1, lth+1); // CRC range set here, include len
      } else
      if (frport_type == f_port2) {
        mycrcGood =  crcGood(buf+2, lth);  // CRC range set here, exclude len
      //    Log.printf("mycrcGood:%d\n", mycrcGood);       
      }
      
      if (mycrcGood) {
        goodFrames++;
      } else {
        badFrames++; // due to crc
      }
      #if defined Debug_CRC
        Log.printf("mycrcGood=%d\n\n", mycrcGood);  
      #endif  
      return mycrcGood;   
   
    }  


    //===================================================================
      
    void FrSkyPort::setMode(PortMode mode) {   
    
    
    }  // end of member function    
    
    //===================================================================

    byte FrSkyPort::ReadByte() {
    byte b;
      FrSkyPort::setMode(rx);
      if (lth == 0) {
        while (lth==0) {
          FrSkyPort::CheckForTimeout();
          lth=frSerial.available();
        }
     //    Log.printf("\nlen=%3d\n",len); 
      } 

      // Data is available
      FrSkyPort::serGood = true;            // We have a good serial connection!
      FrSkyPort::serGood_millis = millis();
      
      b = frSerial.read();
      lth--;
      
      #if (defined Debug_FrPort_Stream)  
        Printbyte(b, false, '<');
      #endif 
      delay(0); // yield to rtos for wifi & bt to get a sniff      
      return b;
    }

    //===================================================================

    byte FrSkyPort::SafeRead() {
      byte b;  

      FrSkyPort::setMode(rx);

      b = FrSkyPort::ReadByte();     
      
      //  if 0x7D is received it should be omitted, and the next byte should 
      //  be XOR or ADD with 0x20
      //  0x5D => 0x7D, 0x5E => 0x7E
    
      if (b == 0x7D) {
        b = FrSkyPort::ReadByte();
        b ^= 0x20;
      }
      #if (defined Debug_FrPort_Safe_Read)  
        Printbyte(b, false, '<');
      #endif 
      delay(0); // yield to rtos for wifi & bt to get a sniff 
      return b;
    } // end of member function
    //===================================================================

    void FrSkyPort::SafeWrite(byte b, bool isPayload) {
    #if (not defined inhibit_FPort)
      FrSkyPort::setMode(tx);
        
      //  B Y T E   S T U F F   S.Port and F.Port1, not F.Port2
      //  Byte in frame has value 0x7E is changed into 2 bytes: 0x7D 0x5E
      //  Byte in frame has value 0x7D is changed into 2 bytes: 0x7D 0x5D
     
      if ( (isPayload) && ( (frport_type == s_port) || (frport_type == f_port1) ) )  {
        if (b == 0x7E) {
          FrSkyPort::WriteByte(0x7D);           
          FrSkyPort::WriteByte(0x5E);    
        } else if (b == 0x7D) {
          FrSkyPort::WriteByte(0x7D);                   
          FrSkyPort::WriteByte(0x5D);          
        } else {   
        FrSkyPort::WriteByte(b);       
        }
      } else {
        FrSkyPort::WriteByte(b);         
      }
     if (isPayload) {  // Add crcout
       FrSkyPort::crcStepOut(b);
     }
      delay(0); // yield to rtos for wifi & bt to get a sniff    
    #endif      
    }
    //===========================
    void FrSkyPort::WriteByte(byte b) {

      if ( (set.fr_io == fr_bt) || (set.fr_io == fr_wifi_bt) ) {          // BT
      //  frSerial.write(b); 
      }

      if (wifiSuGood) { 
        if ( (set.fr_io == fr_wifi) || (set.fr_io == fr_wifi_bt) ) {       // WiFi
            frs_udp_object.write(b);                                 
        }
      }

      #if (defined Debug_FrPort_Stream) || (defined Debug_FrPort_Stream_Out) 
        Printbyte(b, false, '>');
      #endif         
    }
    //===================================================================

    void FrSkyPort::crcStepIn(uint8_t b) {
       crcin += b;          // add in new byte
       crcin += crcin >> 8;   // add in high byte overflow if any
       crcin &= 0xff;  // mask all but low byte, constrain to 8 bits
       #if defined Debug_CRC       
         Log.printf("AddIn %3d %2X\tcrcin_now=%3d %2X\n", b, b, crcin, crcin);
       #endif  
    }  
    //===================================================================

    void FrSkyPort::crcStepOut(uint8_t b) {
       crcout += b;          // add in new byte
       crcout += crcout >> 8;   // add in high byte overflow if any
       crcout &= 0xff;  // mask all but low byte, constrain to 8 bits
       #if defined Debug_CRC       
         Log.printf("AddIn %3d %2X\tcrcout_now=%3d %2X\n", b, b, crcout, crcout);
       #endif  
    }   
    
    //=======================================================================  
    
    void FrSkyPort::crcStep(int16_t *mycrc, uint8_t b) {
       *mycrc += b;             // add in new byte
       *mycrc += *mycrc >> 8;   // add in high byte carry if any
       *mycrc &= 0xff;          // mask all but low byte, constrain to 8 bits

      #if defined Debug_CRC
         Log.printf("CRC Step: b=%3X %3d\  crc=%3X %3d\n", b, b, *mycrc, *mycrc);
       #endif
    }
    //=========================================================================
    
    void FrSkyPort::crcEnd(int16_t *mycrc)  {
      *mycrc = 0xFF - *mycrc;                  // final 2s complement
      #if defined Debug_CRC
        Log.printf("crcEnd=%3X %3d\n", *mycrc, *mycrc );
      #endif  
    } 
    //=======================================================================   
       
    uint8_t FrSkyPort::crcGet(uint8_t *buf, uint8_t lth)  {

      int16_t mycrc = 0;
      for (int i = 0; i < lth; i++) {
        crcStep(&mycrc, *buf++);
      }
      FrSkyPort::crcEnd(&mycrc);
      return mycrc;
    }
    //=======================================================================  
    bool FrSkyPort::crcGood(uint8_t *buf, uint8_t lth)  {
      
      uint8_t mycrc = FrSkyPort::crcGet(buf, lth);   
      uint8_t fpcrc = *(buf+lth);
      #if defined Debug_CRC    
        Log.printf("mycrc=%3X %3d  fpcrc=%3X\ %3d\n", mycrc, mycrc, fpcrc, fpcrc );
      #endif
    return (mycrc == fpcrc);

   }  
     //=======================================================================    
         
    bool FrSkyPort::BytesToPWM(uint8_t *buf, int16_t *ch, uint8_t lth) {  // PWM Channels

      //uint8_t fpcrc = *(buf+lth);  // may be useful
     uint8_t num_of_channels = 0;
     if (lth == 0x0D) {
       num_of_channels = 8;
     } else
     if ((lth == 0x18) || (lth == 0x19)){
       num_of_channels = 16;
     } else     
     if (lth == 0x23) {
       num_of_channels = 24;
     } 
     Log.printf("lth=%d  num_of_channels=%d\n", lth, num_of_channels);
     *ch  = ((*buf|*(buf+1)<< 8) & 0x07FF);
     *(ch+1)  = ((*(buf+1)>>3|*(buf+2)<<5) & 0x07FF);
     *(ch+2)  = ((*(buf+2)>>6|*(buf+3)<<2|*(buf+4)<<10) & 0x07FF);
     *(ch+3)  = ((*(buf+4)>>1|*(buf+5)<<7) & 0x07FF);
     *(ch+4)  = ((*(buf+5)>>4|*(buf+6)<<4) & 0x07FF);
     *(ch+5)  = ((*(buf+6)>>7|*(buf+7)<<1|*(buf+8)<<9) & 0x07FF);
     *(ch+6)  = ((*(buf+8)>>2|*(buf+9)<<6) & 0x07FF);
     *(ch+7)  = ((*(buf+9)>>5|*(buf+10)<<3) & 0x07FF); 
     
     if ((num_of_channels == 16) || (num_of_channels == 24))  {
       *(ch+8)  = ((*(buf+11)|*(buf+12)<< 8) & 0x07FF);
       *(ch+9)  = ((*(buf+12)>>3|*(buf+13)<<5) & 0x07FF);
       *(ch+10) = ((*(buf+13)>>6|*(buf+14)<<2|*(buf+15)<<10) & 0x07FF);
       *(ch+11) = ((*(buf+15)>>1|*(buf+16)<<7) & 0x07FF);
       *(ch+12) = ((*(buf+16)>>4|*(buf+17)<<4) & 0x07FF);
       *(ch+13) = ((*(buf+17)>>7|*(buf+18)<<1|*(buf+19)<<9) & 0x07FF);
       *(ch+14) = ((*(buf+19)>>2|*(buf+20)<<6) & 0x07FF);
       *(ch+15) = ((*(buf+20)>>5|*(buf+21)<<3) & 0x07FF);
     }  

     if (num_of_channels == 24)  {
       *(ch+16)  = ((*(buf+22)|*(buf+23)<< 8) & 0x07FF);
       *(ch+17)  = ((*(buf+23)>>3|*(buf+24)<<5) & 0x07FF);
       *(ch+18) = ((*(buf+24)>>6|*(buf+25)<<2|*(buf+26)<<10) & 0x07FF);
       *(ch+19) = ((*(buf+26)>>1|*(buf+27)<<7) & 0x07FF);
       *(ch+20) = ((*(buf+27)>>4|*(buf+28)<<4) & 0x07FF);
       *(ch+21) = ((*(buf+28)>>7|*(buf+29)<<1|*(buf+30)<<9) & 0x07FF);
       *(ch+22) = ((*(buf+30)>>2|*(buf+31)<<6) & 0x07FF);
       *(ch+23) = ((*(buf+31)>>5|*(buf+32)<<3) & 0x07FF);
     } 

     // remap values to regular uS range
     for (int i = 0 ; i < num_of_channels ; i++) {
       if (*(ch+i) > 0) {
         *(ch+i) = map(*(ch+i), 172, 1811, 885, 2115);      // new F.Port uS limits
       }
     }

      // flags digi ch 1
      if (*(buf+22) & (1<<0)) {
       ch[16] = 1;
      }
      else{
       ch[16] = 0;
      }
      // flags digi ch 2
      if (*(buf+22) & (1<<1)) {
       ch[17] = 1;
      }
      else{
       ch[17] = 0;
     }

     FrSkyPort::pwm_rssi = *(buf+23);

     if (*ch > 0) {
      return true;
     } else {
      return false;
     }
    }  
     
    

    //===================================================================    
    bool FrSkyPort::Send_WiFi_BT_Frame() {

    bool msgSent = false;    
    
    if ((set.fr_io == fr_bt) || (set.fr_io == fr_wifi) || (set.fr_io == fr_wifi_bt) ) {
      
      #if (defined btBuiltin)
        if ((set.fr_io == fr_bt) || (set.fr_io == fr_wifi_bt))  {  // Bluetooth

          msgSent = Send_Bluetooth(&frbuf[0], 10);

          #ifdef  Debug_Send_BT_Frs
            if (msgSent) {
              Log.println("Sent by Bluetooth:");  
              PrintBuffer(&frbuf[0], 10);
            }
          #endif
        }
      #endif
    
      if ((set.fr_io == fr_wifi) || (set.fr_io == fr_wifi_bt)) { //  WiFi
      
        if (wifiSuGood) {       

        
           // UDP     
  
            udp_read_port = set.udp_localPort;  
            udp_send_port = set.udp_remotePort;                       
       
            #if defined UDP_Broadcast                     // broadcast to remote udp clients 
              UDP_remoteIP[3] = 255; 
              msgSent = Send_UDP(&frbuf[0], 10);  // to downlink
              msgSent = msgSent; // stop stupid compiler warnings                       
            #else
                   
              for (int i = 1 ; i < max_clients ; i++) {   // send to each individual remote udp ip. Not to FC side client ip [0] 
                if (udpremoteip[i][0] != 0) {
                  //Log.printf("Non-zero ip i=%d  ip=%s\n", i, udpremoteip[i].toString().c_str());
                  UDP_remoteIP =  udpremoteip[i];         // target the remote IP 
                                   
                  msgSent = Send_UDP(&frbuf[0], 10);    
                      
                  msgSent = msgSent; // stop stupid compiler warnings    

                  #if (defined Debug_Frs_Down) || (defined Debug_Send_UDP_Frs)
                    Log.print("Sent by WiFi UDP: msgSent="); Log.println(msgSent);
                    FrSkyPort::PrintBuffer(&frbuf[0], 10);
                  #endif                          
                }
             }  
           #endif  
  
                                                                   
        }  
      }
     }
     return msgSent;
    }  // end of Send_WiFi_BT_Frame()
 
    //===================================================================      

    #if (defined btBuiltin)
      bool FrSkyPort::Send_Bluetooth(uint8_t *buf, uint16_t len) {

        bool msgSent = false;   
       // Log.printf("BT has client : %u\n", SerialBT.hasClient()); 
        if (SerialBT.hasClient()) {
          size_t sent = SerialBT.write(buf,len);
          if (sent == len) {
            msgSent = true;
            link_status.packets_sent++;
          }
        }
        return msgSent;
      }
    #endif


   //===================================================================   

    bool FrSkyPort::Send_UDP(uint8_t *buf, uint16_t len) {
      if (!wifiSuGood) return false;     
      bool msgSent = false;

      frs_udp_object.beginPacket(UDP_remoteIP, udp_send_port);  
    
      size_t sent = frs_udp_object.write(buf,len);
      
      #if (defined Debug_FrPort_Buffer) 
        Log.print("Send_UDP buffer: ");
        FrSkyPort::PrintBuffer(buf, 10);
      #endif   
      if (sent == len) {
        msgSent = true;
        link_status.packets_sent++;
        frs_udp_object.flush();
      }

      bool endOK = frs_udp_object.endPacket();
      //   if (!endOK) Log.printf("msgSent=%d   endOK=%d\n", msgSent, endOK);
      return msgSent;
    }

    //===================================================================  

    void FrSkyPort::Frs_Decode(uint8_t *buf) {
    // Do the sensor packets according to appID
        static uint16_t prev_appID;
        
        uint16_t appID = uint16Extract(buf, 1 );
        fr_payload = uint32Extract(buf, 3);
        //Log.printf("appID:%4X\n", appID);
        switch(appID) {

                 case 0x800:                      // Latitude and Longitude
                 
                   fr_latlong= uint32Extract(buf, 3); 
                   ms2bits = fr_latlong >> 30;
                   fr_latlong = fr_latlong & 0x3fffffff; // remove ms2bits
                   #if defined Debug_All     
                     Log.print(" ms2bits=");
                     Log.println(ms2bits);
                   #endif   
                   switch(ms2bits) {
                     case 0:   // Latitude Positive
                       fr_lat = fr_latlong;     // lon always comes first   
                       cur.lat = (float)(fr_lat / 6E5);     // lon always comes first                                           
                       #if defined Debug_All || defined Debug_FrSky_Messages  
                         PrintFrPeriod(0);               
                         Log.print(" FrSky 0x800 latitude=");
                         Log.println(cur.lat,7);
                       #endif
                       latGood=true;
                       break;
                     case 1:   // Latitude Negative 
                       fr_lat = fr_latlong;                         
                       cur.lat = (float)(0-(fr_lat / 6E5)); 
                       #if defined Debug_All || defined Debug_FrSky_Messages
                         PrintFrPeriod(0);           
                         Log.print(" FrSky 0x800 latitude=");
                         Log.println(cur.lat,7);  
                       #endif   

                       if (!(cur.lat==0.000000) && !(cur.lon==0.000000)){
                         latGood=true;                       
                       }
                       break;
                     case 2:   // Longitude Positive
                       fr_lon = fr_latlong;    
                       cur.lon = (float)(fr_lon / 6E5);                                         
                       #if defined Debug_All || defined Debug_FrSky_Messages   
                         PrintFrPeriod(0);
                         Log.print(" FrSky 0x800 longitude=");
                         Log.println(cur.lon,7); 
                       #endif                       
                       lonGood=true;                   
                       break;
                     case 3:   // Longitude Negative
                       fr_lon = fr_latlong; 
                       cur.lon = (float)(0-(fr_lon / 6E5));                         
                       #if defined Debug_All    
                         PrintFrPeriod(0);                    
                         Log.print(" FrSky 0x800 longitude=");
                         Log.println(cur.lon,7); 
                       #endif                   
                       lonGood=true;                     
                       break;
                    }
                    break;
  
                 //   Mavlink Passthrough Protocol below    
                 //===================================================================
                 case 0xF000:   
                   break;
                 case 0xF101:                        // RSSI  
                   fr_rssi = fr_payload;
                   #if defined Debug_FrSky
                     PrintFrPeriod(0);
                     Log.print(" FrSky F101: RSSI=");
                     Log.println(fr_rssi);
                   #endif  
                   break;
                 case 0xF103:   
                   break;
                 case 0xF104:   
                   break;
                 case 0xF105:   
                   break;                 
                  case 0x5002:
                  
                  // GPS Status &  fr_gps_alt
                    fr_gps = uint32Extract(buf, 3);
                    fr_numsats = bit32Extract(fr_gps, 0, 4);
                    fr_gps_status = bit32Extract(fr_gps, 4, 2) + bit32Extract(fr_gps, 14, 2);
                    fr_hdop = bit32Extract(fr_gps, 7, 7) * TenToPwr(bit32Extract(fr_gps, 6, 1));  
                    fr_gps_alt = bit32Extract(fr_gps,24,7) * TenToPwr(bit32Extract(fr_gps,22,2)); //-- dm                                    
                    cur.alt  = (float)(fr_gps_alt) / 10;
                    neg = bit32Extract(fr_gps, 31, 1);
                    if (neg==1) cur.alt = 0 - cur.alt;
                    hdopGood=(fr_hdop>=3) && (fr_numsats>10);
                    #if defined Debug_All || defined Debug_FrSky_Messages 
                      PrintFrPeriod(0);
                      Log.print(" FrSky 0x5002 Num sats=");
                      Log.print(fr_numsats);
                      Log.print(" gpsStatus=");
                      Log.print(fr_gps_status);                
                      Log.print(" HDOP=");
                      Log.print(fr_hdop);                    
                      Log.print(" fr_gps_alt=");
                      Log.print(cur.alt, 1);
                      Log.print(" hdopGood=");
                      Log.print(hdopGood);                      
                      Log.print(" neg=");
                      Log.println(neg);   
                    #endif

                    break;
                    
                  case 0x5003:                         // Battery 1 Hz
                   FrSkyPort::fr_bat1_volts = (float)(bit32Extract(fr_payload,0,9));  // dv -> V
                   //Log.printf("mantissa:%d  10exponent:%d mutiplier:%d \n", bit32Extract(fr_payload,10,7), bit32Extract(fr_payload,9,1), TenToPwr(bit32Extract(fr_payload,9,1)) );
                   FrSkyPort::fr_bat1_amps = (bit32Extract(fr_payload,10,7) * TenToPwr(bit32Extract(fr_payload,9,1) ) );  // rounded to nearest whole A
                   FrSkyPort::fr_bat1_mAh = bit32Extract(fr_payload,17,15);
                   FrSkyPort::fr_bat1_amps *= 10;  // prep_number() divided by 10 to get A, but we want dA for consistency
                   #if defined Debug_FrSky
                     PrintFrPeriod(0);
                     Log.print(" FrSky 5003: Battery Volts=");
                     Log.print(fr_bat1_volts, 1);
                     Log.print("  Battery Amps=");
                     Log.print((float)fr_bat1_amps, 0);
                     Log.print("  Battery mAh=");
                     Log.println(fr_bat1_mAh); 
                   #endif       
                   break;                          
                   
                  case 0x5004:                         // Home
                    fr_home_dist = bit32Extract(fr_payload,2,10) * TenToPwr(bit32Extract(fr_payload,0,2));                   
                    fr_fhome_dist = (float)fr_home_dist * 0.1;  // Not used here 
                    cur.alt = bit32Extract(fr_payload,14,10) * TenToPwr(bit32Extract(fr_payload,12,2)); // decimetres
                    if (bit32Extract(fr_payload,24,1) == 1) 
                      cur.alt = cur.alt * -1;
                    altGood=true; 
                    #if defined Debug_All || defined Debug_FrSky_Messages
                      PrintFrPeriod(0);
                      Log.print(" FrSky 0x5004 Dist to home=");
                      Log.print(fr_fhome_dist, 1);             
                      Log.print(" Rel Alt=");
                      Log.println(cur.alt,1);
                    #endif
                    break;
                      
                  case 0x5005:                      
                  // Vert and Horiz Velocity and Yaw angle (Heading)     
                    fr_velyaw = fr_home_dist = bit32Extract(fr_payload, 16, 11);
                    cur.hdg = fr_velyaw * 0.1F;
      
                    hdgGood=true;
                    #if defined Debug_All || defined Debug_FrSky_Messages
                      PrintFrPeriod(0);
                      Log.print(" FrSky 0x5005 Heading=");
                      Log.println(cur.hdg,1);
                    #endif
                    break;   
                 case 0x5006:                         // Roll, Pitch and Range - Max Hz      
                   fr_roll = bit32Extract(fr_payload,0,11);        
                   fr_roll = (fr_roll - 900) * 0.2;             //  -- roll [0,1800] ==> [-180,180] 
                   fr_pitch = bit32Extract(fr_payload,11,10);   
                   fr_pitch = (fr_pitch - 450) * 0.2;           //  -- pitch [0,900] ==> [-90,90]
                   fr_range = bit32Extract(fr_payload,22,10) * TenToPwr(bit32Extract(fr_payload,21,1));

                   AT_5006_flag = true;
                   #if defined Debug_FrSky_Messages
                     PrintFrPeriod(0);                  
                     //if (appID == prev_appID) break; 
                     Log.print(" Frsky 5006: Range=");
                     Log.print(fr_range,2);
                     Log.print(" Roll=");
                     Log.print(fr_roll);
                     Log.print("deg   Pitch=");
                     Log.print(fr_pitch);   
                     Log.println("deg");   
                                 
                   #endif
                   break;                                         
                 case 0x5007:                         // Parameters
                   fr_param_id = bit32Extract(fr_payload,24,4);
                   fr_param_val = bit32Extract(fr_payload,0,24);
                   if (fr_param_id == 1) {
                     fr_frame_type = fr_param_val;
                     Param_50071_flag = true;
                     #if defined Debug_FrSky_Messages
                       PrintFrPeriod(0);
                       Log.print("Frsky 5007: Frame_type=");
                       Log.println(fr_frame_type);
                     #endif  
                   }
                   else if (fr_param_id == 2) {
                     fr_fs_bat_volts = fr_param_val;
                     Param_50072_flag = true;
                     #if defined Debug_FrSky_Messages
                       PrintFrPeriod(0);
                       Log.print("Frsky 5007: Bat failsafe volts=");
                       Log.println(fr_fs_bat_volts);
                     #endif  
                   }
                   else if (fr_param_id == 3) {
                     fr_fs_bat_mAh = fr_param_val;
                     Param_50073_flag = true;
                     #if defined Debug_FrSky_Messages
                       PrintFrPeriod(0);
                       Log.print("Frsky 5007: Bat failsafe mAh=");
                       Log.println(fr_fs_bat_mAh);  
                     #endif         
                   }
                   else if (fr_param_id== 4) {
                     fr_bat1_capacity = fr_param_val;
                     Param_50074_flag = true;
                     #if defined Debug_FrSky_Messages
                       PrintFrPeriod(0);
                       Log.print("Frsky 5007: Bat1 capacity=");
                       Log.println(fr_bat1_capacity);
                     #endif  
                   }         
                   else if (fr_param_id == 5) {
                     fr_bat2_capacity = fr_param_val;
                     Param_50075_flag = true;
                     #if defined Debug_FrSky_Messages
                       PrintFrPeriod(0);    
                       Log.print("Frsky 5007: Bat2 capacity=");
                       Log.println(fr_bat2_capacity); 
                     #endif  
                   }
                   
                   Param_5007_flag = true;

                   break;    
                 case 0x5008:                         // Battery 2
                   fr_bat2_volts = bit32Extract(fr_payload,0,9);
                   fr_bat2_amps = bit32Extract(fr_payload,10,7)  * TenToPwr(bit32Extract(fr_payload,9,1));
                   fr_bat2_mAh = bit32Extract(fr_payload,17,15);
                   #if defined Debug_FrSky_Messages
                     PrintFrPeriod(0);
                     Log.print("FrSky 5008: Battery2 Volts=");
                     Log.print(fr_bat2_volts, 1);
                     Log.print("  Battery2 Amps=");
                     Log.print(fr_bat2_amps, 1);
                     Log.print("  Battery2 mAh=");
                     Log.println(fr_bat2_mAh); 
                   #endif                    
                   Param_5008_flag = true;  
                   break;                   
                   
        }

        gpsGood = hdopGood & lonGood & latGood & altGood & hdgGood ; 
        prev_appID = appID;
    }

    //=======================================================================================================================   

    pol_t FrSkyPort::getPolarity(uint8_t rxpin) {
      uint16_t hi = 0;
      uint16_t lo = 0;  
      const uint16_t mx = 1000;
      while ((lo <= mx) && (hi <= mx)) {

        if (digitalRead(rxpin) == 0) { 
          lo++;
        } else 
        if (digitalRead(rxpin) == 1) {  
          hi++;
        }

        if (lo > mx) {
          if (hi == 0) {         
            return no_traffic;
          } else {
            return idle_low; 
          }
        }
        if (hi > mx) {
          if (lo == 0) {
            return no_traffic;
          } else {
            return idle_high; 
          }
        }
        delayMicroseconds(200);
       //Log.printf("rxpin:%d  %d\t\t%d\n",rxpin, lo, hi);  
     } 
    } 
    //===================================================================      
    uint32_t FrSkyPort::getBaud(uint8_t rxpin) {
      Log.print("autoBaud - sensing rxpin "); Log.println(rxpin);
    // Log.printf("autoBaud - sensing rxPin %2d \n", rxpin );
      uint8_t i = 0;
      uint8_t col = 0;
      pinMode(rxpin, INPUT);       
      //digitalWrite (rxpin, HIGH); // pull up enabled for noise reduction ?

      uint32_t gb_baud = getConsistent(rxpin);
      while (gb_baud == 0) {
        if(ftgetBaud) {
          ftgetBaud = false;
      }

        i++;
        if ((i % 5) == 0) {
          Log.print(".");
          col++; 
        }
        if (col > 60) {
          Log.println(); 
            Log.print("No telemetry found on pin "); Log.println(rxpin);
            // Log.printf("No telemetry found on pin %2d\n", rxpin); 
          col = 0;
          i = 0;
        }
        gb_baud = getConsistent(rxpin);
      } 
      if (!ftgetBaud) {
        Log.println();
      }

      //Log.print("Telem found at "); Log.print(gb_baud);  Log.println(" b/s");
      //LogScreenPrintln("Telem found at " + String(gb_baud));

      return(gb_baud);      
    }
    //===================================================================   
    uint32_t FrSkyPort::getConsistent(uint8_t rxpin) {
      uint32_t t_baud[5];

      while (true) {  
        t_baud[0] = SenseUart(rxpin);
        delay(10);
        t_baud[1] = SenseUart(rxpin);
        delay(10);
        t_baud[2] = SenseUart(rxpin);
        delay(10);
        t_baud[3] = SenseUart(rxpin);
        delay(10);
        t_baud[4] = SenseUart(rxpin);
        #if defined Debug_All || defined Debug_Baud
          Log.print("  t_baud[0]="); Log.print(t_baud[0]);
          Log.print("  t_baud[1]="); Log.print(t_baud[1]);
          Log.print("  t_baud[2]="); Log.print(t_baud[2]);
          Log.print("  t_baud[3]="); Log.println(t_baud[3]);
        #endif  
        if (t_baud[0] == t_baud[1]) {
          if (t_baud[1] == t_baud[2]) {
            if (t_baud[2] == t_baud[3]) { 
              if (t_baud[3] == t_baud[4]) {   
                #if defined Debug_All || defined Debug_Baud    
                  Log.print("Consistent baud found="); Log.println(t_baud[3]); 
                #endif   
                return t_baud[3]; 
              }          
            }
          }
        }
      }
    }
    //===================================================================   
    uint32_t FrSkyPort::SenseUart(uint8_t  rxpin) {

    uint32_t pw = 999999;  //  Pulse width in uS
    uint32_t min_pw = 999999;
    uint32_t su_baud = 0;
    const uint32_t su_timeout = 5000; // uS !  Default timeout 1000mS!

      #if defined Debug_All || defined Debug_Baud
        Log.printf("rxpin:%d  rxInvert:%d\n", rxpin, frInvert);  
      #endif  

      if (frInvert) {
        while(digitalRead(rxpin) == 0){ };  // idle_low, wait for high bit (low pulse) to start
      } else {
        while(digitalRead(rxpin) == 1){ };  // idle_high, wait for high bit (high pulse) to start  
      }

      for (int i = 0; i < 10; i++) {

        if (frInvert) {               //  Returns the length of the pulse in uS
          pw = pulseIn(rxpin,HIGH, su_timeout);     
        } else {
          pw = pulseIn(rxpin,LOW, su_timeout);    
        }    

        if (pw !=0) {
          min_pw = (pw < min_pw) ? pw : min_pw;  // Choose the lowest
          //Log.printf("i:%d  pw:%d  min_pw:%d\n", i, pw, min_pw);      
        } 
      } 
      #if defined Debug_All || defined Debug_Baud
        Log.printf("pw:%d  min_pw:%d\n", pw, min_pw);
      #endif

      switch(min_pw) {   
        case 1:     
        su_baud = 921600;
          break;
        case 2:     
        su_baud = 460800;
          break;     
        case 4 ... 11:     
        su_baud = 115200;
          break;
        case 12 ... 19:  
        su_baud = 57600;
          break;
        case 20 ... 28:  
        su_baud = 38400;
          break; 
        case 29 ... 39:  
        su_baud = 28800;
         break;
        case 40 ... 59:  
        su_baud = 19200;
          break;
        case 60 ... 79:  
        su_baud = 14400;
          break;
        case 80 ... 149:  
        su_baud = 9600;
          break;
        case 150 ... 299:  
        su_baud = 4800;
          break;
        case 300 ... 599:  
        su_baud = 2400;
          break;
        case 600 ... 1199:  
        su_baud = 1200;  
          break;                        
        default:  
        su_baud = 0;    // no signal        
      }
      return su_baud;
    }  
    //===================================================================   
    uint32_t FrSkyPort::TenToPwr(uint8_t pwr) {
      uint32_t ttp = 1;
      for (int i = 1 ; i<=pwr ; i++) {
        ttp*=10;
      }
      return ttp;
    }  
    //===================================================================   

    uint32_t FrSkyPort::bit32Extract(uint32_t dword,uint8_t displ, uint8_t lth) {
      uint32_t r = (dword & FrSkyPort::createMask(displ,(displ+lth-1))) >> displ;
      //  Log.print(" Result=");
      // Log.println(r);
      return r;
    }
    //===================================================================      
    uint32_t FrSkyPort::createMask(uint8_t lo, uint8_t hi) {
      uint32_t r = 0;
      for (unsigned i=lo; i<=hi; i++)
         r |= 1 << i;
      //  Log.print(" Mask 0x=");
      //  Log.println(r, HEX);      
      return r;
    }
      
    //========================================================

    uint32_t FrSkyPort::uint32Extract(uint8_t *buf, int posn){
  
      //  The number starts at byte "posn" of the received packet and is four bytes long.
      //  GPS payload fields are little-endian, i.e they need an end-to-end byte swap
    
      byte b1 = buf[posn+3];
      byte b2 = buf[posn+2];
      byte b3 = buf[posn+1];
      byte b4 = buf[posn]; 
   
      unsigned long highWord = b1 << 8 | b2;
      unsigned long lowWord  = b3 << 8 | b4;
    
        // Now combine the four bytes into an unsigned 32bit integer

      uint32_t myvar = highWord << 16 | lowWord;
      return myvar;
    }

    //========================================================
    uint16_t FrSkyPort::uint16Extract(uint8_t *buf, int posn){
  
      //  The number starts at byte "posn" of the received packet and is two bytes long
      //  GPS payload fields are little-endian, i.e they need an end-to-end byte swap

       byte b1 = buf[posn+1];
       byte b2 = buf[posn];  
    
        // Now convert the 2 bytes into an unsigned 16bit integer
    
        uint16_t myvar = b1 << 8 | b2;
        return myvar;
    }
      //=====================================================   


    void  FrSkyPort::Print_PWM_Channels(int16_t *ch, uint16_t num_of_channels) {
      for (int i = 0 ; i < num_of_channels ; i++ ) {
        Log.printf("%2d:%4d  ", i+1, *(ch+i)); 
      }
      Log.printf("  rssi:%d\n", FrSkyPort::pwm_rssi );
    }
    //===================================================================      

    void FrSkyPort::PrintBuffer(uint8_t *buf, uint8_t length) {
    // Log.printf("length=%d\t", length);
     for (int i = 0 ; i < length ; i++) {  
       if (*(buf+i) <= 0xf) Log.print("0");
         Log.print(*(buf+i),HEX);
         Log.write(" ");
       }
       Log.println();
    }
   
    //===================================================================
    void FrSkyPort::CheckForTimeout() {
      
     if ((millis() - serGood_millis) > (FrSkyPort::timeout_secs * 1000)) {
       FrSkyPort::serGood = false;
       FrSkyPort::frGood = false; 
       FrSkyPort::pwmGood = false;           
     }    

     if ((millis() - frGood_millis) > (FrSkyPort::timeout_secs * 1000)) {
       FrSkyPort::frGood = false;
       FrSkyPort::pwmGood = false;  
     }
     
     if ((millis() - pwmGood_millis) > (FrSkyPort::timeout_secs * 1000)) {
       FrSkyPort::pwmGood = false;
     }

     
     FrSkyPort::ServiceStatusLed(); 
     FrSkyPort::ReportOnlineStatus();
    }   
    //===================================================================     

    void FrSkyPort::ServiceStatusLed() {
      if (fpStatusLed == 99) return;
      
      if (FrSkyPort::pwmGood) {
          if (InvertfpLed) {
           FrSkyPort::fpLedState = LOW;
          } else {
           FrSkyPort::fpLedState = HIGH;
          }
          digitalWrite(fpStatusLed, fpLedState); 
      }
        else {
          if (serGood) {
            FrSkyPort::BlinkFpLed(500);
          }
        }
      digitalWrite(fpStatusLed, fpLedState); 
    }
    //===================================================================   
    void FrSkyPort::BlinkFpLed(uint32_t period) {
         if (millis() - FrSkyPort::fpLed_millis >= period) {    // blink period
            FrSkyPort::fpLed_millis = millis();
            if (FrSkyPort::fpLedState == LOW) {
              FrSkyPort::fpLedState = HIGH; }   
            else {
              FrSkyPort::fpLedState = LOW;  } 
          }
    }  
    //===================================================================  
    
    void FrSkyPort::ReportOnlineStatus() {
  
       if (FrSkyPort::frGood != FrSkyPort::frPrev) {  // report on change of status
         FrSkyPort::frPrev = FrSkyPort::frGood;
         if (frGood) {
           Log.println("FrPort read good!");
           LogScreenPrintln("FrPort read ok");         
         } else {
          Log.println("FrPort read timeout!");
          LogScreenPrintln("FrPort timeout");         
         }
       }
       
       if (FrSkyPort::pwmGood != FrSkyPort::pwmPrev) {  // report on change of status
         FrSkyPort::pwmPrev = FrSkyPort::pwmGood;
         if (pwmGood) {
           Log.println("RC PWM good");
           LogScreenPrintln("RC PWM good");         
         } else {
          Log.println("RC PWM timeout");
          LogScreenPrintln("RC PWM timeout");         
         }
       }
    } 
     //===================================================================  

     void FrSkyPort::PrintFrPeriod(bool LF) {
           
      now_millis = millis();
      now_micros = micros(); 
      uint32_t period = now_millis - prev_fr_millis;
      if (period < 10) {
        period = now_micros - prev_fr_micros;
        Log.printf(" FrPeriod uS=%d", period);
      } else {
        Log.printf(" FrPeriod mS=%d", period);
      }
      if (period < 100) {
        Log.print("\t");
      }       
      if (LF) {
        Log.print("\t\n");
      } else {
       Log.print("\t");
      }
    
      prev_fr_millis = now_millis;
      prev_fr_micros = now_micros;
    }                                    
