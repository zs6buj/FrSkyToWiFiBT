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
    float     fr_bat_volts;     
    float     fr_bat_amps;
    uint16_t  fr_bat_mAh;    
    uint8_t   fr_numsats;   
    uint32_t  fr_rssi;    
      
  private:
    static const uint8_t timeout_secs = 5;
    
    byte      chr = 0;
    byte      prev_chr = 0;    
    bool      fpInit = false; 
    bool      serGood = false;
    bool      frGood = false;
    bool      frPrev = false; 
    bool      pwmGood = false; 
    bool      pwmPrev = false;
    uint32_t  frGood_millis = 0;   
    uint32_t  serGood_millis = 0;  
    uint32_t  pwmGood_millis = 0;       

    bool      parseGood = false;
    uint32_t  goodFrames = 0;
    uint32_t  badFrames = 0;
    uint32_t  packetloss_millis = 0;
    uint8_t   fpLedState = LOW; 
    uint32_t  fpLed_millis = 0;
             
    #define   frameSize  38                   // len, type, 35(24ch control) + CRC = 38
    byte      fpbuf[frameSize];

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
    
    static const uint16_t  fr_max = 48;           // frport frame buffer  35 + 2 + headroom 
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
    uint8_t fr_gpsStatus;
    uint8_t fr_hdop;
    uint8_t fr_vdop;
    uint32_t fr_gps_alt;
    float   fr_fgps_alt;
    uint8_t neg;

    //0x5003 Batt
    //float fr_bat_volts;     // moved to public for hud info display
    //float fr_bat_amps;
    //uint16_t fr_bat_mAh;

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


    // Member function declarations
  public:  
    void          initialise();   
    void          HandleTraffic();
    void          ReportFrPortOnlineStatus();      
            
  private:  
    frport_t      Sense_FrPort(); 
    bool          FPort_Read_A_Frame(uint8_t *buf, frport_t  frport_type);  
    bool          SPort_Read_A_Frame(uint8_t *buf);           
    void          setMode(PortMode mode);
    byte          ReadByte();
    byte          SafeRead();
    void          WriteByte(byte b);   
    void          SafeWrite(byte b, bool isPayload);
    void          ReportOnlineStatus(); 
    bool          ParseFrame(uint8_t *buf, uint16_t lth);
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
    void          CheckForTimeout();   
    void          ServiceStatusLed();
    void          BlinkFpLed(uint32_t period);
    bool          Send_Frsky_Frame(); 
    #if (defined btBuiltin)    
      bool          Send_Bluetooth(uint8_t *buf, uint16_t len);   
    #endif  
    bool          Send_TCP(uint8_t *buf, uint16_t len);           
    bool          Send_UDP(uint8_t *buf, uint16_t len);
    void          DecodeFrsky();
    uint32_t      bit32Unpack(uint32_t dword,uint8_t displ, uint8_t lth);
    uint16_t      byt16Unpack(uint8_t posn);
    uint32_t      byt32Unpack(uint8_t posn); 
    uint32_t      createMask(uint8_t lo, uint8_t hi);       
            
}; // end of class

    // External member functions
    //===================================================================

    void FrSkyPort::initialise()  {  
    
    if ( (set.frport == f_port1) || (set.frport == f_port2) ) {  // check for fport2 later
      frBaud = 115200; 
      frInvert = false;   // F.Port is normally idle high (not inverted)
      Log.printf("FPort selected, expecting regular fport idle high, baud:%d", frBaud);     
    } else 
    if (set.frport == s_port) {
      frBaud = 57600;
      #if defined Inverted_Inverted_FrSky_Receiver_Pad // note S.Port is normally idle low (inverted)
        frInvert = false;  
        Log.printf("SPort selected, expecting inverted inverted/idle high, baud:%d", frBaud); 
      #else
        frInvert = true;  
          Log.printf("SPort selected, now expecting regular sport inverted/idle low, baud:%d", frBaud);           
      #endif  
    }
    
    #if (defined ESP32) || (defined ESP8266) // ESP only
      int8_t frRx;
      int8_t frTx;

      frRx = fr_rxPin;
      frTx = fr_txPin;

      #if ( (defined ESP8266) || ( (defined ESP32) && (defined ESP32_SoftwareSerial)) )
 
          Log.printf(" and is 1-wire simplex on tx pin = %d\n", frTx);

          nbdelay(100);
          frSerial.begin(frBaud, SWSERIAL_8N1, frRx, frTx, frInvert);     // SoftwareSerial
          nbdelay(100);
          Log.println("Using SoftwareSerial for F.Port");

      #else  // HardwareSerial
          frSerial.begin(frBaud, SERIAL_8N1, frRx, frTx, frInvert);                     
          Log.printf(" simplex on tx pin = %d\n", frTx);

   
      #endif
    #endif
    
      
    } // end of member function

    //===================================================================
    void FrSkyPort::HandleTraffic() {  
       
      if (frport_type == f_none) {   // detect FrPort Type
        if (set.frport == s_port) {
          frport_type = s_port;      //  convenient to use eeprom setting because sport has different baud rate
        } else {
       frport_type = Sense_FrPort();            
        }  
      } else {
         
      static bool ft = true;
      if (ft) {
         if (frport_type == s_port) {
           Log.println("SPort detected");         
         } else
         Log.printf("FrPort V%d detected\n",  frport_type); 
        ft = false; 
      }

      if ( (frport_type == f_port1) || (frport_type == f_port2) ) { 
        if ( FPort_Read_A_Frame(&FrSkyPort::frbuf[0], frport_type) ) {
          FrSkyPort::DecodeFrsky();     
          FrSkyPort::Send_Frsky_Frame();
        }
 
      } else  
      if (set.frport == s_port) {  
        if ( SPort_Read_A_Frame(&FrSkyPort::frbuf[0]) ) {
          FrSkyPort::DecodeFrsky();
          FrSkyPort::Send_Frsky_Frame();
        }
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
    
    bool FrSkyPort::FPort_Read_A_Frame(uint8_t *buf, frport_t  frport_type) {
      /*
       Master arranges timing of transaction, slave responds
       Master sends downlink frame just after channel (control) frame
       We are a slave, and default to receiving status      
       Slave responds with uplink frame immediately if matching ID received
      */
      
      FrSkyPort::setMode(rx);
      lth=frSerial.available();  
      if (lth < 10) {
        if (lth > 0) {
          //Log.printf("lth=%d\n", lth); 
        }
        return false;       // prevent 'wait-on-read' blocking
      }
          
      #if defined Report_Packetloss
        uint32_t period = (Report_Packetloss * 60000);
        if (millis() - FrSkyPort::packetloss_millis > period) {
          FrSkyPort::packetloss_millis = millis();
          float packetloss = float(float(FrSkyPort::badFrames * 100) / float(FrSkyPort::goodFrames + FrSkyPort::badFrames));
          Log.printf("goodFrames:%d   badFrames:%d   frame loss:%.3f%%\n", FrSkyPort::goodFrames, FrSkyPort::badFrames, packetloss);
        }
      #endif
  
      #if defined Debug_FrPort_Serial_Loop
        while(true) {            // loop here forever - debugging only
          FrSkyPort::chr = FrSkyPort::ReadByte();  
        }
      #endif  


      if (frport_type == f_port1) {           // find start of frame 
        while (!(FrSkyPort::chr==0x7E)) {     // find first 0x7E, should be start, but could be previous stop
          FrSkyPort::chr = FrSkyPort::ReadByte();
        }
        FrSkyPort::fpbuf[0] = FrSkyPort::chr;
        FrSkyPort::chr = FrSkyPort::ReadByte();    // could be start 0x7E or len 0x08, 0x0D, 0x18, 0x20, 0x23  
        while (FrSkyPort::chr == 0x7E) {           // if start 0x7E, then the first one was a stop so ignore it
          *buf = FrSkyPort::chr;
          FrSkyPort::chr = FrSkyPort::ReadByte();
        }
        fr_lth = *(buf+1) = FrSkyPort::chr;                  // lth
        fr_type = *(buf+2) = FrSkyPort::ReadByte();          // frame_type  
        
        if ((fr_lth == 0x08) || (fr_lth == 0x19) ) {  // downlink or control frame
          FrSkyPort::frGood = true;            
          FrSkyPort::frGood_millis = millis();  
  
          switch(fr_type){
            case 0x00:      // F.Port v1.0 Control Frame (RC Channels)
              FrSkyPort::parseGood = FrSkyPort::ParseFrame(buf, fr_lth);
              if (FrSkyPort::parseGood) {
                #if defined Derive_PWM_Channesl           
                  FrSkyPort::pwmGood = FrSkyPort::BytesToPWM(buf+3, &FrSkyPort::pwm_ch[0], fr_lth);
                  if (FrSkyPort::pwmGood) {
                    #if defined Debug_PWM_Channels
                      FrSkyPort::Print_PWM_Channels(&FrSkyPort::pwm_ch[0], num_of_channels);
                    #endif  
                    FrSkyPort::pwmGood_millis = millis();
                  }
                #endif  
                #if defined Support_SBUS_Out 
                #endif
                return true;    
              }  
              return false;
  
            case 0x01:      // F.Port v1.0 downlink frame from master  -  match on our sensor byte ( range 0x0~0x1B or 0x1E (FC) )              
              FrSkyPort::parseGood = FrSkyPort::ParseFrame(buf, fr_lth); 
              if (FrSkyPort::parseGood) {    
                fr_prime = buf[3];                         // if prime == 0x00, reply prime = 0x00
                                                           // if prime == 0x10, reply prime = 0x10 (slave ready)                                                       
                return true;  
              }
              return false;  
              
            case 0x0D:   // mavlite downlink frame from master, match on sensor id 0x0D                          
              FrSkyPort::parseGood = FrSkyPort::ParseFrame(buf, fr_lth); 
              if (FrSkyPort::parseGood) {
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
          badFrames++; // due to fail length test
          // Log.printf("Bad FPort frame length = %X\n", fr_lth); 
          return false; 
        }     
      }          // end of FPort1
      
      if (frport_type == f_port2) {                // find start of frame
        bool ctl = false;
        bool ota = false; 
        bool dlink = false;
        while ( (!(ctl)) && (!(ota)) && (!(dlink)) ) {     // find valid lth + type combo
          prev_chr = chr;
          FrSkyPort::chr = FrSkyPort::ReadByte();
          ctl = (((prev_chr == 0x0D) || (prev_chr == 0x18)|| (prev_chr == 0x23)) && (chr == 0xFF)); 
          ota = (((prev_chr == 0x0D) || (prev_chr == 0x18)|| (prev_chr == 0x23)) && (chr == 0xF1)); 
          dlink = ((prev_chr == 0x08)  && (chr == 0x1B));         
        }      
        *(buf) = 0;                     // not used for fp2
        fr_lth = *(buf+1) = prev_chr;   // lth
        fr_type = *(buf+2) = chr;       // frame_type 
          
        if ((fr_lth == 0x08) || (fr_lth == 0x0d) || (fr_lth == 0x18) || (fr_lth == 0x20) ) {  // 
          FrSkyPort::frGood = true;            
          FrSkyPort::frGood_millis = millis();  
          switch(fr_type){
            
            case 0x0D:   // mavlite downlink frame from master, match on sensor id 0x0D            
               
              FrSkyPort::parseGood = FrSkyPort::ParseFrame(buf, fr_lth); 
              if (FrSkyPort::parseGood) {
                fr_prime = buf[3];                         // should be 0x30                                                                                                              
                return false;  // ignore mavlit for now
              }
              return false; 
                    
            case 0x1B:   // F.Port v2.3.7 downlink frame from master, match on sensor id 0x0~0x1B or 0x1E (FC)                       
              FrSkyPort::parseGood = FrSkyPort::ParseFrame(buf, fr_lth); 
              if (FrSkyPort::parseGood) {
                fr_prime = buf[3];                         // if prime == 0x00, reply prime = 0x00
                                                           // if prime == 0x10, reply prime = 0x10 (slave ready)                                                                                                                 
               return true; 
              }
              return false;

            case 0xff:      // F.Port v2.3.7 Control Frame (RC Channels)  
              FrSkyPort::parseGood = FrSkyPort::ParseFrame(buf, fr_lth);             
              if (FrSkyPort::parseGood) {
                #if defined Derive_PWM_Channesl           
                  FrSkyPort::pwmGood = FrSkyPort::BytesToPWM(buf+3, &FrSkyPort::pwm_ch[0], fr_lth);
                  if (FrSkyPort::pwmGood) {
                    #if defined Debug_PWM_Channels
                      FrSkyPort::Print_PWM_Channels(&FrSkyPort::pwm_ch[0], num_of_channels);
                    #endif  
                    FrSkyPort::pwmGood_millis = millis();
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
    
    bool FrSkyPort::SPort_Read_A_Frame(uint8_t *buf) {

      static uint8_t i = 0;
      byte b;
      
      while (Serial1.available()) {
    
        if (b == 0x7E) {  // end of frame parse
          buf[0] = b;
          i = 1;  
        
          if (buf[2] == 0x10) {
            if (buf[9] == (0xFF-crcin)){
              goodFrames++;
              //Log.print(" CRC Good: ");
              crcin = 0;
              memset(&buf[2], 0x00, 1);
              return true;              // RETURN
            } else {
              badFrames++;              
              //Log.print(" CRC Bad!: "); 
             memset(&buf[2], 0x00, 8);            
            }
          }
          crcin = 0;
        }  // end of b == 0x7E
         
        b = SafeRead();
        if (b != 0x7E) {  // if next start/stop don't put it in the buffer
            if ((i > 1) && (i < 9))  crcStepIn(b);   
            buf[i] = b;              
            if (i<fr_max-1) i++;          
        }
      }
      return false;     
    }
    //===================================================================   
 
    bool FrSkyPort::ParseFrame(uint8_t *buf, uint16_t lth) {
      //Log.printf("buf[0]:%X  buf[1]:%X  buf[2]:%X\n", buf[0], buf[1], buf[2]);
      int i = 0;
      for (i = 3 ; i < lth+2 ; i++) {         // payload after start byte[0], lth[1] and type[2], 
        chr = FrSkyPort::SafeRead();          // f_port2 ignores start byte[0]
      //  if (*(buf+2) == 0x1b) Log.printf("i:%d  chr:%X\n", i, chr);
        *(buf+i) = chr;
      }
      
       chr = FrSkyPort::SafeRead();           // this is the crc byte
       *(buf+i) = chr;

       // FrSkyPort::crcEnd(&crcin);        
       
      #if (defined Debug_FPort_Buffer) 
        FrSkyPort::PrintBuffer(buf, lth);
      #endif 

      bool mycrcGood =  0; 
      if (frport_type == f_port1) {
        mycrcGood =  FrSkyPort::crcGood(buf+1, lth+1); 

      }
      if (frport_type == f_port2) {
        mycrcGood =  FrSkyPort::crcGood(buf+2, lth);  
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
    bool FrSkyPort::Send_Frsky_Frame() {

    bool msgSent = false;    
    
    if ((set.fr_io == fr_bt) || (set.fr_io == fr_wifi) || (set.fr_io == fr_wifi_bt) ) {
      
      #if (defined btBuiltin)
        if ((set.fr_io == fr_bt) || (set.fr_io == fr_wifi_bt))  {  // Bluetooth

          msgSent = Send_Bluetooth(&frbuf[0], 10);

          #ifdef  Debug_Frs_Down
            if (msgSent) {
              Log.println("Sent by Bluetooth:");
              PrintMavBuffer(&frbuf);
            }
          #endif
        }
      #endif
    
      if ((set.fr_io == fr_wifi) || (set.fr_io == fr_wifi_bt)) { //  WiFi
      
        if (wifiSuGood) {       
          if (set.wfproto == tcp)  { // TCP      
            for (int i = 1 ; i < max_clients ; ++i) {       // send to each active client. Not to inbound client [0] (FC side)
              if (NULL != tcp_client[i]) {        // if active client
                active_client_idx = i;
                           
                prev_millis = millis();
                           
                msgSent = Send_TCP(&frbuf[0], 10);  // to downlink
                
                if ((millis() -  prev_millis) > 4) {
                  PrintFrPeriod(false);  Log.println(" TCP Write timeout =====================================>");  
                }
              
                #ifdef  Debug_Frs_Down
                  if (msgSent) {
                    Log.printf("Sent to client %d by WiFi TCP\n", active_client_idx+1);  
                  }
                #endif
              }
            }    
          }
        
          if (set.wfproto == udp)  { // UDP     
  
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
                    PrintMavBuffer(&frbuf);
                  #endif                          
                }
             }  
           #endif  
  
          }                                                                     
        }  
      }
     }
     return msgSent;
    }  // end of Send_FrSky_Frame()
 
    //===================================================================      

    #if (defined btBuiltin)
      bool FrSkyPort::Send_Bluetooth(uint8_t *buf, uint16_t len) {

        bool msgSent = false;   
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

      bool FrSkyPort::Send_TCP(uint8_t *buf, uint16_t len) {
      if ( (!wifiSuGood) || ((!inbound_clientGood) && (!outbound_clientGood)) ) return false; 
        bool msgSent = false;
    
        size_t sent =  tcp_client[active_client_idx]->write(buf,len);  

        if (sent == len) {
          msgSent = true;
          link_status.packets_sent++;
        }

        return msgSent;
      }

    //===================================================================   

    bool FrSkyPort::Send_UDP(uint8_t *buf, uint16_t len) {
      if (!wifiSuGood) return false;  

      // 2 possible udp objects, STA [0]  and  AP [1] 
        
      bool msgSent = false;

      udp_object[active_object_idx]->beginPacket(UDP_remoteIP, udp_send_port);  
    
      size_t sent = udp_object[active_object_idx]->write(buf,len);

      if (sent == len) {
        msgSent = true;
        link_status.packets_sent++;
        udp_object[active_object_idx]->flush();
      }

      bool endOK = udp_object[active_object_idx]->endPacket();
      //   if (!endOK) Log.printf("msgSent=%d   endOK=%d\n", msgSent, endOK);
      return msgSent;
    }
    //===================================================================   

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
    void FrSkyPort::DecodeFrsky() {
    // Do the sensor packets according to fr_payload type. We are expecting Mavlink Passthrough only

    static uint16_t prev_appID = 0;
       
    uint16_t fr_appID = FrSkyPort::byt16Unpack(3);
    fr_payload = FrSkyPort::byt32Unpack(5);
    //   Log.print(" fr_appID=");
    //   Log.println(fr_appID, HEX);

      
    switch(fr_appID) {
 
      case 0x800:                      // Latitude and Longitude
                   fr_latlong = FrSkyPort::byt32Unpack(5);
                   ms2bits = fr_latlong >> 30;
                   fr_latlong = fr_latlong & 0x3fffffff; // remove ms2bits
             
                   // Log.printf(" ms2bits:%d   latlong:%X\n", ms2bits, fr_latlong);

                   if (ms2bits==0) {
                     fr_lat = fr_latlong / 6E5;     // Only ever update lon and lat in pairs. Lon always comes first
                     if (!(FrSkyPort::fr_lon==0)) lat_800_flag = true;  }
                     else
                       if (ms2bits==1) {
                         fr_lat = 0-(fr_latlong / 6E5); 
                         if (!(FrSkyPort::fr_lat==0)) lat_800_flag = true;   }
                         else 
                           if (ms2bits==2) {
                             FrSkyPort::fr_lon = fr_latlong / 6E5;
                             if (!(FrSkyPort::fr_lon==0)) lon_800_flag = true; } 
                             else
                               if (ms2bits==3) {
                                 FrSkyPort::fr_lon = 0-(fr_latlong / 6E5);
                                 if (!(FrSkyPort::fr_lon==0)) lon_800_flag = true;  }
                                 
                   latlon_800_flag = lat_800_flag && lon_800_flag; 
                   
                   if (latlon_800_flag) {
                     #if defined Print_Decoded_FrSky
                       Log.print("FrSky: latitude=");
                       Log.print(FrSkyPort::fr_lat,7);
                       Log.print(" longitude=");
                       Log.println(FrSkyPort::fr_lon,7);
                     #endif          
                   }           
                   break;

                 //===================================================================
                 case 0xF000:   
                   break;
                 case 0xF101:                        // RSSI  
                   fr_rssi = fr_payload;
                   #if defined Print_Decoded_FrSky
                     Log.print("FrSky F101: RSSI=");
                     Log.println(fr_rssi);
                   #endif  
                   break;
                 case 0xF103:   
                   break;
                 case 0xF104:   
                   break;
                 case 0xF105:   
                   break;
                 //===================================================================
                 //   Mavlink Passthrough Protocol below  
                 
                 case 0x5000:                         // Text Message
                   ct[0] = frbuf[8];
                   ct[1] = frbuf[7];
                   ct[2] = frbuf[6];
                   ct[3] = frbuf[5];
                   ct[4] = 0;  // terminate string

                   if (ct[0] == 0 || ct[1] == 0 || ct[2] == 0 || ct[3] == 0) 
                 //    fr_severity = (bit32Unpack(fr_payload,15,1) * 4) + (bit32Unpack(fr_payload,23,1) * 2) + (bit32Unpack(fr_payload,30,1) * 1);
                     fr_severity = (bit32Unpack(fr_payload,23,1) * 4) + (bit32Unpack(fr_payload,15,1) * 2) + (bit32Unpack(fr_payload,7,1) * 1); 
                   if (strcmp(ct,p_ct)==0){     //  If this one = previous one it's a duplicate
                     ct_dups++;
                     break;
                     }
                     
                   // This is the last (3rd) duplicate of (usually) 4 chunks

                   for ( int i=0; i<5 ;i++ ) {  // Copy current 4 byte chunk to previous
                     p_ct[i]=ct[i];
                    }
                   
                   eot=false; 
 
                   if (ct[1] >= 0x80) { // It should always be this one, first of last 3 bytes
                 //    b1 = ct[1]; 
                     ct[1]=0;
                     ct[2]=0;
                     ct[3]=0;
                     eot=true;
                   }
                   #if defined Print_Decoded_FrSky
                     Log.print("TXT=");  
                     Log.print(ct);
                     Log.print(" ");
                   #endif  
                   strcat(fr_text, ct);  // Concatenate ct onto the back of fr_text
                   ct_dups=0;
                 
                   if (!eot) break; // Drop through when end-of-text found
                   #if defined Print_Decoded_FrSky
                     Log.print("Frsky 5000: Severity=");  
                     Log.print(fr_severity);
                     Log.print(" Msg : ");  
                     Log.println(fr_text);
                   #endif  

                   ST_5000_flag = true;  // Trigger Status Text encode in EncodeMavlink()

                   break; 
                   
                 case 0x5001:                         // AP Status 2 Hz
                   fr_flight_mode = bit32Unpack(fr_payload,0,5);
                   fr_simple = bit32Unpack(fr_payload,5,2);
                   fr_land_complete = bit32Unpack(fr_payload,7,1);
                   fr_armed = bit32Unpack(fr_payload,8,1);
                   fr_bat_fs = bit32Unpack(fr_payload,9,1);
                   fr_ekf_fs = bit32Unpack(fr_payload,10,2);                   

                   AP_5001_flag = true;   
                   #if defined Print_Decoded_FrSky
                     Log.print("FrSky 5001: Flight_mode=");
                     Log.print(fr_flight_mode);
                     Log.print(" simple_mode=");
                     Log.print(fr_simple);
                     Log.print(" land_complete=");
                     Log.print(fr_land_complete);
                     Log.print(" armed=");
                     Log.print(fr_armed);
                     Log.print(" battery_failsafe=");
                     Log.print(fr_bat_fs);    
                     Log.print(" EKF_failsafe=");
                     Log.println(fr_ekf_fs);  
                   #endif  
                                                    
                   break;                   
                 case 0x5002:                         // GPS Status & Alt msl 1 Hz
                   fr_numsats = bit32Unpack(fr_payload, 0, 4);
                   fr_gpsStatus = bit32Unpack(fr_payload, 4, 2) + bit32Unpack(fr_payload, 14, 2);
                   fr_hdop = bit32Unpack(fr_payload, 7, 7) * (10^bit32Unpack(fr_payload, 6, 1));
               //    fr_vdop = bit32Unpack(fr_payload, 15, 7) * (10^bit32Unpack(fr_payload, 14, 1)); 
               
                  fr_gps_alt = bit32Unpack(fr_payload,24,7) * (10^bit32Unpack(fr_gps_alt,22,2)); //-- dm
                  fr_gps_alt*=10;
                  if (bit32Unpack(fr_gps_alt,31,1) == 1)
                     fr_gps_alt = fr_gps_alt * -1;
                  
                //   fr_gps_alt = bit32Unpack(fr_payload, 16, 16);  //  For use with modified arducopter 3.5.5
                   #if defined Print_Decoded_FrSky
                     Log.print("FrSky 5002: Num sats=");
                     Log.print(fr_numsats);
                     Log.print(" gpsStatus=");
                     Log.print(fr_gpsStatus);                 
                     Log.print(" HDOP=");
                     Log.print(fr_hdop);   
                     Log.print(" alt amsl=");
                     Log.println(fr_gps_alt); 
                   #endif  

                   GPS_5002_flag=true;
                   break;
                 case 0x5003:                         // Battery 1 Hz
  
                   fr_bat_volts = (bit32Unpack(fr_payload,0,9)) / 10;  // dv -> V
                   fr_bat_amps = bit32Unpack(fr_payload,10,7) * (10^bit32Unpack(fr_payload,9,1));
                   fr_bat_mAh = bit32Unpack(fr_payload,17,15);

                   Bat_5003_flag = true;
                   #if defined Print_Decoded_FrSky
                     Log.print("FrSky 5003: Battery Volts=");
                     Log.print(fr_bat_volts, 1);
                     Log.print("  Battery Amps=");
                     Log.print(fr_bat_amps, 1);
                     Log.print("  Battery mAh=");
                     Log.println(fr_bat_mAh); 
                   #endif       
                   break;                          
                 case 0x5004:                         // Home 2 Hz
                   fr_home_dist = bit32Unpack(fr_payload,2,10) * (10^bit32Unpack(fr_payload,0,2));
                   fr_fhome_dist = (float)fr_home_dist * 0.1;   // metres
                   fr_home_alt = bit32Unpack(fr_payload,14,10) * (10^bit32Unpack(fr_payload,12,2));
                   fr_fhome_alt = (float)(fr_home_alt) * 0.01;  // metres
                   if (bit32Unpack(fr_payload,24,1) == 1) 
                     fr_fhome_alt *=  -1;
                   fr_home_angle = bit32Unpack(fr_payload, 25,  7) * 3;
                   
                   Home_5004_flag = true;
                   #if defined Print_Decoded_FrSky
                     Log.print("FrSky 5004: Dist to home=");
                     Log.print(fr_fhome_dist, 1);              
                     Log.print(" home_alt=");
                     Log.print(fr_fhome_alt, 1);    // This is correct but fluctuates slightly. Can be negative
                     Log.print(" home_angle="); 
                     Log.println(fr_home_angle);    // degrees
                   #endif  
               
                   break;                        
                 case 0x5005:                    // Vert and Horiz Velocity and Yaw angle (Heading) 2 Hz
                   fr_vx = bit32Unpack(fr_payload,1,7) * (10^bit32Unpack(fr_payload,0,1));
                   if (bit32Unpack(fr_payload,8,1) == 1)
                     fr_vx *= -1;
                   fr_vy = bit32Unpack(fr_payload,10,7) * (10^bit32Unpack(fr_payload,9,1));
                   fr_yaw = bit32Unpack(fr_payload,17,11) * 0.2;
    
                   velyaw_5005_flag = true;        
                   #if defined Print_Decoded_FrSky
                     Log.print("FrSky 5005: v_veloc=");
                     Log.print(fr_vx,1);        // m/s
                     Log.print(" h_veloc=");
                     Log.print(fr_vy,1);        // m/s       
                     Log.print(" yaw_angle="); 
                     Log.println(fr_yaw,1);    // degrees
                   #endif

                   
                   break; 
                 case 0x5006:                         // Roll, Pitch and Range - Max Hz      
                   fr_roll = bit32Unpack(fr_payload,0,11);        
                   fr_roll = (fr_roll - 900) * 0.2;             //  -- roll [0,1800] ==> [-180,180] 
                   fr_pitch = bit32Unpack(fr_payload,11,10);   
                   fr_pitch = (fr_pitch - 450) * 0.2;           //  -- pitch [0,900] ==> [-90,90]
                   fr_range = bit32Unpack(fr_payload,22,10) * (10^bit32Unpack(fr_payload,21,1));

                   AT_5006_flag = true;
                   #if defined Print_Decoded_FrSky
                   if (fr_appID == prev_appID) break;
                   
                     Log.print("Frsky 5006: Range=");
                     Log.print(fr_range,2);
                     Log.print(" Roll=");
                     Log.print(fr_roll);
                     Log.print("deg   Pitch=");
                     Log.print(fr_pitch);   
                     Log.println("deg");   
                                 
                   #endif
                   break;                                         
                 case 0x5007:                         // Parameters
                   fr_param_id = bit32Unpack(fr_payload,24,4);
                   fr_param_val = bit32Unpack(fr_payload,0,24);
                   if (fr_param_id == 1) {
                     fr_frame_type = fr_param_val;
                     Param_50071_flag = true;
                     #if defined Print_Decoded_FrSky
                       Log.print("Frsky 5007: Frame_type=");
                       Log.println(fr_frame_type);
                     #endif  
                   }
                   else if (fr_param_id == 2) {
                     fr_fs_bat_volts = fr_param_val;
                     Param_50072_flag = true;
                     #if defined Print_Decoded_FrSky
                       Log.print("Frsky 5007: Bat failsafe volts=");
                       Log.println(fr_fs_bat_volts);
                     #endif  
                   }
                   else if (fr_param_id == 3) {
                     fr_fs_bat_mAh = fr_param_val;
                     Param_50073_flag = true;
                     #if defined Print_Decoded_FrSky
                       Log.print("Frsky 5007: Bat failsafe mAh=");
                       Log.println(fr_fs_bat_mAh);  
                     #endif         
                   }
                   else if (fr_param_id== 4) {
                     fr_bat1_capacity = fr_param_val;
                     Param_50074_flag = true;
                     #if defined Print_Decoded_FrSky
                       Log.print("Frsky 5007: Bat1 capacity=");
                       Log.println(fr_bat1_capacity);
                     #endif  
                   }         
                   else if (fr_param_id == 5) {
                     fr_bat2_capacity = fr_param_val;
                     Param_50075_flag = true;
                     #if defined Print_Decoded_FrSky
                       Log.print("Frsky 5007: Bat2 capacity=");
                       Log.println(fr_bat2_capacity); 
                     #endif  
                   }
                   
                   Param_5007_flag = true;

                   break;    
                 case 0x5008:                         // Battery 2
                   fr_bat2_volts = bit32Unpack(fr_payload,0,9);
                   fr_bat2_amps = bit32Unpack(fr_payload,10,7)  * (10^bit32Unpack(fr_payload,9,1));
                   fr_bat2_mAh = bit32Unpack(fr_payload,17,15);
                   #if defined Print_Decoded_FrSky
                     Log.print("FrSky 5008: Battery2 Volts=");
                     Log.print(fr_bat_volts, 1);
                     Log.print("  Battery2 Amps=");
                     Log.print(fr_bat_amps, 1);
                     Log.print("  Battery2 mAh=");
                     Log.println(fr_bat_mAh); 
                   #endif                    
                   Param_5008_flag = true;  
                   break;       
      }
      prev_appID = fr_appID;
    }

    //===================================================================   

    uint32_t FrSkyPort::bit32Unpack(uint32_t dword,uint8_t displ, uint8_t lth) {
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
    //===================================================================    
    uint16_t FrSkyPort::byt16Unpack(uint8_t posn){
  
    //  The number starts at byte "posn" of the received packet and is two bytes long
    //  GPS payload fields are little-endian, i.e they need an end-to-end byte swap

      byte b1 = frbuf[posn+1];
      byte b2 = frbuf[posn];  
    
        // Now convert the 2 bytes into an unsigned 16bit integer
    
        uint16_t myvar = b1 << 8 | b2;
        return myvar;
    }
    //===================================================================    

    uint32_t FrSkyPort::byt32Unpack(uint8_t posn){
  
    //  The number starts at byte "posn" of the received packet and is four bytes long.
    //  GPS payload fields are little-endian, i.e they need an end-to-end byte swap
    
      byte b1 = frbuf[posn+3];
      byte b2 = frbuf[posn+2];
      byte b3 = frbuf[posn+1];
      byte b4 = frbuf[posn]; 
   
      unsigned long highWord = b1 << 8 | b2;
      unsigned long lowWord  = b3 << 8 | b4;
    
      // Now combine the four bytes into an unsigned 32bit integer

       uint32_t myvar = highWord << 16 | lowWord;
       return myvar;
    }
    //===================================================================               
