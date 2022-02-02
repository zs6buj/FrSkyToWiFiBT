//=================================================================================================  
//================================================================================================= 
//
//                                       U T I L I T I E S
//
//================================================================================================= 
//=================================================================================================
 

//=================================================================================================   
//                             W I F I   S U P P O R T   -   ESP Only 
//=================================================================================================  


  void IRAM_ATTR gotWifiButton(){        // "Start WiFi" Button
    if (millis() - debnceTimr < delaytm) return;
    debnceTimr = millis();  
    wifiButton = true;
    
  }
  //==========================================
  void SenseWiFiPin() {
    #if defined Start_WiFi
      if (!wifiSuDone) {
        SetupWiFi();
        return;
      }
      return;
    #else
      if ((wifiButton) && (!wifiSuDone)) {
        wifiButton = false;
        SetupWiFi();
        } 
    #endif      
  }
  
  //===============================       H a n d l e   W i F i   E v e n t s 
  
    void CheckStaLinkStatus();  // Forward declaration 
    
    void ServiceWiFiRoutines() {
      
      SenseWiFiPin();   // check optional start-wifi pin
      
      if ((set.wfmode == sta) && wifiSuGood && wifiSuDone)  { // in sta mode check for disconnect from AP
        CheckStaLinkStatus();    // Handle disconnect/reconnect from AP
      }
      

      // Report stations connected to/from our AP
      AP_sta_count = WiFi.softAPgetStationNum();
      
      if (AP_sta_count > AP_prev_sta_count) {  
        AP_prev_sta_count = AP_sta_count;
        Log.printf("Remote STA %d connected to our AP\n", AP_sta_count);  
        snprintf(snprintf_buf, snp_max, "STA %d connected", AP_sta_count);                
        LogScreenPrintln(snprintf_buf); 
                           
      } else 
      if (AP_sta_count < AP_prev_sta_count) {      // a device has disconnected from the AP
        AP_prev_sta_count = AP_sta_count;
        Log.println("A STA disconnected from our AP");     // back in listening mode
        LogScreenPrintln("A STA disconnected"); 
      }
   }  

    

   //===================     H a n d l e   S T A   D i s c o n n e c t  /  R e c o n n e c t
  
   void CheckStaLinkStatus() {

      if ( !WiFi.isConnected() )   { 
        if (!wifiDisconnected) StartWiFiTimer();   // start wifi retry interrupt timer 
        wifiDisconnected = true;
 
        #if (defined ESP32)
          if (xSemaphoreTake(wifiTimerSemaphore, 0) == pdTRUE)  { 
            uint32_t isrCount = 0, isrTime = 0;   
            portENTER_CRITICAL(&timerMux);
           //  do something here if necessary   
            portEXIT_CRITICAL(&timerMux);
            RestartWiFiSta();
          } 
        #endif  

        #if (defined ESP8266)
          if ( (esp8266_wifi_retry_millis > 0) && ( (millis() - esp8266_wifi_retry_millis) > 5000) ) {
            RestartWiFiSta();
            esp8266_wifi_retry_millis = millis();  // restart timer
          }
        #endif           

      } else {
        if (wifiDisconnected) {
          Log.println("Wifi link restored");
          LogScreenPrintln("Wifi link restored"); 
          wifiDisconnected = false;
          
          #if (defined ESP32) 
            if (timer) {
              timerEnd(timer);
            }  
          #endif
          #if (defined ESP8266)        
            esp8266_wifi_retry_millis = 0;  // stop timer
          #endif    
        }
       }        
   }
   //==================================================  
  
   void StartWiFiTimer() {
    #if (defined ESP32)
      timerAlarmEnable(timer);
    #endif

    #if (defined ESP8266)
      esp8266_wifi_retry_millis = millis();
    #endif
    
   }
   //==================================================
   void RestartWiFiSta() { 
     Log.println("WiFi link lost - retrying");
     LogScreenPrintln("Wifilink lost- retry"); 
     WiFi.disconnect(false); 
     WiFi.mode(WIFI_STA);
     WiFi.begin(set.staSSID, set.staPw);
     nbdelay(250);
   }
   //=================================================================================================
   #if (defined ESP32)     
   void IRAM_ATTR onTimer(){         // interrupt for periodic wifi retry 

     portENTER_CRITICAL_ISR(&timerMux);
     // do something here if needed 
     portEXIT_CRITICAL_ISR(&timerMux);
     // Give a semaphore that we can check in the loop
     xSemaphoreGiveFromISR(wifiTimerSemaphore, NULL);  

   }
   #endif
 
   //==================================================
   void SetupWiFi() { 
    
    for (int i = 0 ; i < max_clients ; i++) {   // initialise udp remote ip table
         udpremoteip[i][0] = 0;   
         udpremoteip[i][1] = 0; 
         udpremoteip[i][2] = 0;   
         udpremoteip[i][3] = 0;         
    }

    udpremoteip[0][3] = 255;              // initialise the only outbound udp remote client ip for broadcast
    udpremoteip[1][3] = 255;              // initialise the first inbound udp remote client ip for broadcast
   
    bool apFailover = false;              // used when STA fails to connect
    
    #if (defined ESP32)
      // set up wifi retry interrupt 
      wifiTimerSemaphore = xSemaphoreCreateBinary();  // Create wifi retry semaphore
      timer = timerBegin(3, 80, true);                // use timer 0 (0 thru 3) set 80 divider for prescaler
      timerAttachInterrupt(timer, &onTimer, true);
      timerAlarmWrite(timer, 5 * 1E6, true);          // uS, repeat  (semaphore every 5 seconds when alarm enabled)
    #endif

    #if (defined ESP8266)             // ESP8266 - not using interrup times, just use millis()
      esp8266_wifi_retry_millis = 0;
    #endif
    
    apFailover = byte(EEPROM.read(0));              //  Read first eeprom byte
    #if defined Debug_Eeprom
      Log.print("Read EEPROM apFailover = "); Log.println(apFailover); 
    #endif

    //=====================================  S T A T I O N ========================================== 

   if ((set.wfmode == sta) || (set.wfmode == sta_ap) )  {  // STA mode or AP_STA mode or STA failover to AP mode
     if (!apFailover) {   
     
      uint8_t retry = 0;
      WiFi.disconnect(true);   // To circumvent "wifi: Set status to INIT" error bug
      nbdelay(500);
      
      if (WiFi.mode(WIFI_STA)) {
         Log.println("WiFi mode set to STA sucessfully");  
      } else {
        Log.println("WiFi mode set to STA failed!");  
      }
      
      Log.print("Trying to connect to ");  
      Log.print(set.staSSID); 
      LogScreenPrintln("WiFi trying ..");    
      nbdelay(500);
      
      WiFi.begin(set.staSSID, set.staPw);
      while (WiFi.status() != WL_CONNECTED){
        retry++;
        if (retry > 20) {
          Log.println();
          Log.println("Failed to connect in STA mode");
          LogScreenPrintln("No connect STA Mode");
          if (set.wfmode == sta_ap) {  // STA failover to AP mode
            apFailover = true;         
            Log.println("Failover to AP. Rebooting ....");
            LogScreenPrintln("Failover to AP");  
            apFailover = 1;                // set STA failover to AP flag
            EEPROM.write(0, apFailover);   // (addr, val)  
            EEPROM.commit();
            #if defined Debug_Eeprom
              Log.print("Write EEPROM apFailover = "); Log.println(apFailover); 
            #endif          
            delay(1000);
            ESP.restart();                 // esp32 and esp8266
          }  
          
          break;
        }
        nbdelay(500);
        Log.print(".");
      }
      
      if (WiFi.status() == WL_CONNECTED) {

        if (set.wfmode == sta_ap) {   // in sta failover to ap mode we had successful sta connect
          set.wfmode = sta;           // so set correct mode      
        }
        
        localIP = WiFi.localIP();  
   
        UDP_remoteIP = localIP;    // Initially broadcast on the subnet we are attached to. patch by Stefan Arbes. 
        UDP_remoteIP[3] = 255;     // patch by Stefan Arbes  
                               
        Log.println();
        Log.println("WiFi connected!");
        Log.print("Local IP address: ");
        Log.println(localIP);
 
        wifi_rssi = WiFi.RSSI();
        Log.print("WiFi RSSI:");
        Log.print(wifi_rssi);
        Log.println(" dBm");

        LogScreenPrintln("Connected!");
        LogScreenPrintln(localIP.toString());

         // UDP

        udp_read_port = set.udp_localPort; 
        udp_send_port = set.udp_remotePort;                          
      
        Log.printf("Begin UDP using Frs UDP object  read port:%d  send port:%d\n", udp_read_port, udp_send_port);                                                             
        frs_udp_object.begin(udp_read_port);  
                  
        UDP_remoteIP = localIP;        
        UDP_remoteIP[3] = 255;           // broadcast until we know which ip to target     

        if ((set.fr_io == fr_wifi) || (set.fr_io == fr_wifi_bt) ) {  // if fr wifi            
            udpremoteip[1] = UDP_remoteIP;                             // [1] IPs reserved for downlink side              
        }    
 
        Log.printf("UDP for STA started, local %s   remote %s\n", localIP.toString().c_str(), 
            UDP_remoteIP.toString().c_str());
        snprintf(snprintf_buf, snp_max, "Local port=%d", set.udp_localPort);     
        LogScreenPrintln(snprintf_buf);

        wifiSuGood = true;
        
      } 
    }  else {   // if apFailover clear apFailover flag
        apFailover = 0;
        EEPROM.write(0, apFailover);   // (addr, val)  
        EEPROM.commit();
        #if defined Debug_Eeprom
          Log.print("Clear EEPROM apFailover = "); Log.println(apFailover); 
        #endif  
      }

   }
 
   //===============================  A C C E S S   P O I N T ===============================
     
   if ((set.wfmode == ap) || (set.wfmode == sta_ap)) { // AP mode or STA failover to AP mode
    if (!wifiSuGood) {  // not already setup in STA above  
    
      Log.printf("WiFi mode set to WIFI_AP %s\n", WiFi.mode(WIFI_AP) ? "" : "Failed!"); 
      
      Start_Access_Point();
      
      if (set.wfmode == sta_ap) {  // in sta_ap mode we had successful failover to ap
        set.wfmode = ap;           // so set correct mode      
      }
      
    }
      
    wifiSuGood = true;  
    
   }        // end of AP ===========================================================================         

   #if defined Debug_SRAM
     Log.printf("==============>Free Heap after WiFi setup = %d\n", ESP.getFreeHeap());
   #endif

   #if defined webSupport
     if (wifiSuGood) {
       WebServerSetup();  
       Log.print("Web support active on http://"); 
       Log.println(localIP.toString().c_str());
       LogScreenPrintln("webSupprt active");  
     }  else {
       Log.println("No web support possible"); 
       LogScreenPrintln("No web support!");  
     }
   #endif

   wifiSuDone = true;
    
  } 
  //=================================================================================================  
  void Start_Access_Point() {
    
      WiFi.softAP(set.apSSID, set.apPw, set.channel);
      
      localIP = WiFi.softAPIP();   

      Log.print("AP IP address: ");
      Log.print (localIP); 
      snprintf(snprintf_buf, snp_max, "AP IP = %s", localIP.toString().c_str());        
      LogScreenPrintln(snprintf_buf);  
      
      Log.print("  SSID: ");
      Log.println(String(set.apSSID));
      LogScreenPrintln("WiFi AP SSID =");
      snprintf(snprintf_buf, snp_max, "%s", set.apSSID);        
      LogScreenPrintln(snprintf_buf);  
     
      // UDP 
            
      udp_read_port = set.udp_localPort;  
      udp_send_port = set.udp_remotePort;                    
          
      Log.printf("Begin UDP using Frs UDP object  read port:%d  send port:%d\n", set.udp_localPort, set.udp_remotePort);                    
      frs_udp_object.begin(set.udp_localPort);          // local port for Frs out                   
                         
      UDP_remoteIP = WiFi.softAPIP();
      UDP_remoteIP[3] = 255;           // broadcast until we know which ip to target       

      // Now initialise the first entry of the udp targeted ip table 
      // FC side uses udpremoteip[0] and GCS side uses udpremoteip[1]


      udpremoteip[0] = UDP_remoteIP;  // we never use FC side udpremoteip[0]
      udpremoteip[1] = UDP_remoteIP;          
       
      Log.printf("UDP for Frs started, local %s   remote %s\n", WiFi.softAPIP().toString().c_str(), 
          UDP_remoteIP.toString().c_str());         
          snprintf(snprintf_buf, snp_max, "UDP port = %d", set.udp_localPort);        
      LogScreenPrintln(snprintf_buf);                 
  }
  
  //=================================================================================================  
   void PrintRemoteIP() {
    if (FtRemIP)  {
      FtRemIP = false;
      Log.print("UDP client identified, remote IP: "); Log.print(UDP_remoteIP);
      Log.print(", remote port: "); Log.println(udp_send_port);
      LogScreenPrintln("UDP client connected");
      LogScreenPrintln("Remote IP =");
      snprintf(snprintf_buf, snp_max, "%s", UDP_remoteIP.toString().c_str());        
      LogScreenPrintln(snprintf_buf);        
      snprintf(snprintf_buf, snp_max, "Remote port = %d", set.udp_remotePort);        
      LogScreenPrintln(snprintf_buf);    
     }
  }
  

//=================================================================================================   
//                             E N D   O F   W I F I   S U P P O R T   -   ESP Only - for now
//=================================================================================================

void ServiceStatusLeds() {
  if (FrsStatusLed != 99) {
    ServiceFrsStatusLed();
  }
}
void ServiceFrsStatusLed() {
  
  if (frsGood) {
      if (InvertFrsLed) {
       frsLedState = LOW;
      } else {
       frsLedState = HIGH;
      }
      digitalWrite(FrsStatusLed, frsLedState); 
  }
    else {
      BlinkFrsLed(500);
    }
  digitalWrite(FrsStatusLed, frsLedState); 
}


void BlinkFrsLed(uint32_t period) {
  uint32_t cMillis = millis();
     if (cMillis - frs_led_millis >= period) {    // blink period
        frs_led_millis = cMillis;
        if (frsLedState == LOW) {
          frsLedState = HIGH; }   
        else {
          frsLedState = LOW;  } 
      }
}

//================================================================================================= 
//=================================================================================================  

void Printbyte(byte b, bool LF, char delimiter) {
  if ((b == 0x7E) && (LF)) {
    Log.println();
  }
  
  if (b == 0x7E)  {//             || (b == 0x10)  || (b == 0x32)) {
    Log.println();
  } 
  if (b<=0xf) Log.print("0");
  Log.print(b,HEX);
  Log.write(delimiter);
}


//=================================================================================================  
void PrintPeriod(bool LF) {
  now_millis=millis();
  now_micros=micros();

  uint32_t period = now_millis - prev_millis;
  if (period < 10) {
    period = now_micros - prev_micros;
    Log.printf(" Period uS=%d", period);
  } else {
    Log.printf(" Period mS=%d", period);
  }

  if (LF) {
    Log.print("\t\n");
  } else {
   Log.print("\t");
  }
    
  prev_millis=now_millis;
  prev_micros=now_micros;
}

//=================================================================================================  
void PrintLoopPeriod() {
  now_millis=millis();
  now_micros=micros();

  uint32_t period = now_millis - prev_lp_millis;
  if (period < 10) {
    period = now_micros - prev_lp_micros;
    Log.printf("Loop Period uS=%d\n", period);
  } else {
    Log.printf("Loop Period mS=%d\n", period);
    if (period > 5000) Log.println("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
  }
    
  prev_lp_millis=now_millis;
  prev_lp_micros=now_micros;
}

//=================================================================================================   
//                           D I S P L A Y   S U P P O R T   -   ESP Only - for now
//================================================================================================= 

  #if defined displaySupport  
    void HandleDisplayButtons() {

      if (Pinfo != 99)  {   // if digital pin for info display enumerated
        infoButton = !digitalRead(Pinfo);  // low == pressed
      }
        
      if ((infoButton) && (!infoPressBusy)) { 
        infoPressBusy = true; 
        infoNewPress = true;          
        info_debounce_millis = millis();   

        info_millis = millis();                       
        if (show_log) {
          show_log = false; 
          info_millis = millis() + db_period;  
        } else {
          show_log = true;    
        }
      }
        
      if(millis() - info_debounce_millis > db_period) { 
        infoPressBusy = false; 
        infoButton = false; // for slow decay touch buttons
      }

      if (millis() - last_log_millis > 15000) { // after 15 seconds default to flight info screen
        last_log_millis = millis();             // and enable toggle button again
        show_log = false;
      }

      
      if (show_log) {
        if (infoNewPress) {     
          PaintLogScreen(row, show_last_row);  // one time     
          infoNewPress = false; 
          last_log_millis = millis();
        }
      } else {            // else show flight info
        DisplayFlightInfo();             
      }
      
      //Log.printf("busy=%d  new=%d log=%d  bounce=%d  info=%d\n", infoPressBusy, infoNewPress, show_log, info_debounce_millis, info_millis); 
      
     #if ((defined ESP32) || (defined ESP8266))   // Teensy does not have touch pins          
      if ( (Tup != 99) && (Tdn != 99) ) {         // if ESP touch pin-pair enumerated
        if (upButton) {
          Scroll_Display(up);
        }
        if (dnButton) {
          Scroll_Display(down);
        }     
      } else
      #endif
      
      if ( (Pup != 99) && (Pdn != 99) ) {   // if digital pin-pair enumerated
        upButton = !digitalRead(Pup);       // low == pressed
        if (upButton) {                 
          Scroll_Display(up);
        }
        dnButton = !digitalRead(Pdn);
        if (dnButton) {
          Scroll_Display(down);
        }        
      }        
    }

    //===================================
    #if ((defined ESP32) || (defined ESP8266)) 
    void IRAM_ATTR gotButtonUp(){
      upButton = true;
    }

    void IRAM_ATTR gotButtonDn(){
      dnButton = true;  
    }
    
    void IRAM_ATTR gotButtonInfo(){
      infoButton = true;
    }
    #endif 
    //===================================
   
    void Scroll_Display(scroll_t up_dn) {
      
      if (millis() - scroll_millis < 300) return;
      show_log = true;    
      scroll_millis = millis(); 
      
      if (up_dn == up) {
         scroll_row--;
         scroll_row = constrain(scroll_row, SCR_H_CH, row);
         upButton = false; 
         PaintLogScreen(scroll_row, show_last_row);   // paint down to scroll_row
      }
      if (up_dn == down) {
          scroll_row++; 
          scroll_row = constrain(scroll_row, SCR_H_CH, row);       
          dnButton = false; 
          PaintLogScreen(scroll_row, show_last_row);  // paint down to scroll_row      
      }   
    }

    //===================================
    void PaintLogScreen(uint8_t new_row, last_row_t last_row_action) { 
      if (display_mode != logg) { 
          SetupLogDisplayStyle();
          display_mode = logg; 
      }  
     
        #if (defined ST7789_Display) || (defined SSD1331_Display) ||  (defined ILI9341_Display)   
        //  hardware SPI pins defined in config.h 
          display.fillScreen(SCR_BACKGROUND);                 
        #elif (defined SSD1306_Display) 
          display.clearDisplay();
        #endif  
        display.setCursor(0,0);  
        int8_t first_row = (last_row_action==omit_last_row) ? (new_row - SCR_H_CH +1) : (new_row - SCR_H_CH); 
        int8_t last_row = (last_row_action==omit_last_row) ? new_row : (new_row );        
        for (int i = first_row ; i < last_row; i++) { // drop first line, display rest of old lines & leave space for new line          
          display.println(ScreenRow[i].x);
        }
   
        #if (defined SSD1306_Display)
          display.display();
        #endif 
    }
    
  #endif  // end of defined displaySupport
    
    //===================================
    void LogScreenPrintln(String S) {
    #if defined displaySupport   
    
      if (display_mode != logg) {
          SetupLogDisplayStyle();
          display_mode = logg; 
      }   
      if (row >= SCR_H_CH) {                 // if the new line exceeds the page lth, re-display existing lines
        PaintLogScreen(row, omit_last_row);
      }
      uint16_t lth = strlen(S.c_str());           // store the new line a char at a time
      if (lth > max_col-1) {
        Log.printf("Display width of %d exceeded for |%s|\n", SCR_W_CH, S.c_str());  // SCR_W_CH = max_col-1
        lth = max_col-1;  // prevent array overflow
      }

      for (int i=0 ; i < lth ; i++ ) {
        ScreenRow[row].x[col] = S[i];
        col++;
      } 

      for (col=col ; col < max_col; col++) {    //  padd out the new line to eol
        ScreenRow[row].x[col] = '\0';
      } 

      display.println(ScreenRow[row].x);        // display the new line, which is always the last line
      #if (defined SSD1306_Display)
        display.display();
      #endif  

      col = 0;
      row++;
      if (row > max_row-1) {
        Log.println("Display rows exceeded!");
        row = max_row-1;  // prevent array overflow
      }
      last_log_millis = millis();             // and enable toggle button again
      show_log = true;          
    #endif       
    } // ready for next line

    //===================================
   
    void LogScreenPrint(String S) {
    #if defined displaySupport  

      if (display_mode != logg) {
          SetupLogDisplayStyle();
          display_mode = logg; 
      }   

     // scroll_row = row; 
      if (row >= SCR_H_CH) {              // if the new line exceeds the page lth, re-display existing lines
        PaintLogScreen(row, omit_last_row);
      }
      display.print(S);                         // the new line
      #if (defined SSD1306_Display)
        display.display();
      #endif 
       
      uint8_t lth = strlen(S.c_str());          // store the line a char at a time
      if (lth > SCR_W_CH) {
        Log.printf("Display width of %d exceeded for |%s|\n", SCR_W_CH, S.c_str());  // SCR_W_CH = max_col-1
        lth = max_col-1;  // prevent array overflow
      }  

      for (int i=0 ; i < lth ; i++ ) {
        ScreenRow[row].x[col] = S[i];
        col++;
      } 
      for (col=col ; col < max_col; col++) {  //  padd out to eol
        ScreenRow[row].x[col] = '\0';
      }
      
      if (col > max_col-1) {   // only if columns exceeded, increment row
        col = 0;
        row++;
      }
      last_log_millis = millis();             // and enable toggle button again
      show_log = true;
    #endif    
    } // ready for next line
    
    //===================================
    #if defined displaySupport  
    
    void DisplayFlightInfo() {
      uint16_t xx, yy; 
      if (display_mode != flight_info) {
          SetupInfoDisplayStyle();
          display_mode = flight_info; 
      }
      

      #if  (defined ILI9341_Display)


        if (millis() - info_millis > 200) {    // refresh rate
          info_millis = millis();  

          // artificial horizon
          draw_horizon(ap_roll, ap_pitch, SCR_W_PX, SCR_H_PX);

          display.setTextSize(2);    // 26 ch wide x 15 ch deep
          
          // sats visible
          xx = 0;
          yy = 0 ;          
          display.setCursor(xx, yy);  
          snprintf(snprintf_buf, snp_max, "Sats:%d", fr_numsats); 
          display.fillRect(xx +(4*CHAR_W_PX), yy, 2 * CHAR_W_PX, CHAR_H_PX, ILI9341_BLUE); // clear the previous line               
          display.println(snprintf_buf);  

          // heading (yaw)
          xx = 9 * CHAR_W_PX;
          yy = 0 ;          
          display.setCursor(xx, yy);  
          snprintf(snprintf_buf, snp_max, "Hdg:%.0f%", fr_yaw / 10);
          display.fillRect(xx+(4*CHAR_W_PX), yy, 4 * CHAR_W_PX, CHAR_H_PX, ILI9341_BLUE); // clear the previous line                                
          display.println(snprintf_buf);

          // Radio RSSI
          xx = 17 * CHAR_W_PX;
          yy = 0 ;          
          display.setCursor(xx, yy);  
          snprintf(snprintf_buf, snp_max, "RSSI:%ld%%", ap_rssi); 
          display.fillRect(xx+(4*CHAR_W_PX), yy, 4 * CHAR_W_PX, CHAR_H_PX, ILI9341_BLUE); // clear the previous line               
          display.println(snprintf_buf);
              
          // distance to home
          xx = 0;
          yy = 13.5 * CHAR_H_PX;        
          display.setCursor(xx, yy); 
          snprintf(snprintf_buf, snp_max, "Home:%d", pt_home_dist);    // m 
          display.fillRect(xx+(5*CHAR_W_PX), yy, (4*CHAR_W_PX), CHAR_H_PX, ILI9341_BLUE); // clear the previous line   
          display.println(snprintf_buf); 

          // arrow to home
          xx = 14 * CHAR_W_PX;
          yy = 13.5 * CHAR_H_PX;   
          draw_home_arrow(xx, yy, pt_home_angle, SCR_W_PX, SCR_H_PX);
   
          // altitude above home
          xx = 18 * CHAR_W_PX;
          yy = 14 * CHAR_W_PX;        
          display.setCursor(xx, yy); 
          snprintf(snprintf_buf, snp_max, "Alt:%d", ap33_alt_ag / 1000);    // mm => m 
          display.fillRect(xx+(4*CHAR_W_PX), yy, (4*CHAR_W_PX), CHAR_H_PX, ILI9341_BLUE); // clear the previous line   
          display.println(snprintf_buf); 

          // voltage
          xx = 0;
          yy = 16 * CHAR_W_PX;        
          display.setCursor(xx, yy); 
          snprintf(snprintf_buf, snp_max, "V:%.1fV", (float)fr_bat1_volts);     
          display.fillRect(xx+(2*CHAR_W_PX), yy, (6*CHAR_W_PX), CHAR_H_PX, ILI9341_BLUE); // clear the previous line   
          display.println(snprintf_buf); 
          
          // current
          xx = 9 * CHAR_W_PX;
          yy = 16 * CHAR_W_PX;        
          display.setCursor(xx, yy); 
          snprintf(snprintf_buf, snp_max, "A:%.0f", (float)fr_bat1_amps/100);     
          display.fillRect(xx+(2*CHAR_W_PX), yy, (6*CHAR_W_PX), CHAR_H_PX, ILI9341_BLUE); // clear the previous line   
          display.println(snprintf_buf); 
          
          // Ah consumed
          xx = 18 * CHAR_W_PX;
          yy = 16 * CHAR_W_PX;        
          display.setCursor(xx, yy); 
          snprintf(snprintf_buf, snp_max, "Ah:%.1f", (float)fr_ba1t_mAh / 1000);     
          display.fillRect(xx+(3*CHAR_W_PX), yy, (5*CHAR_W_PX), CHAR_H_PX, ILI9341_BLUE); // clear the previous line   
          display.println(snprintf_buf);           
          
          // latitude and logitude
          display.setTextSize(1);          
          xx = 0;
          yy = 18 * CHAR_H_PX;
          display.setCursor(xx,yy);       
          snprintf(snprintf_buf, snp_max, "Lat:%.7f", cur.lat);
          display.fillRect(xx, yy, (15*CHAR_W_PX), CHAR_H_PX, ILI9341_BLUE); // clear the previous line        
          display.println(snprintf_buf);  
          xx = 18 * CHAR_W_PX;   
          yy = 18 * CHAR_H_PX;  
          display.setCursor(xx, yy);    
          snprintf(snprintf_buf, snp_max, "Lon:%.7f", cur.lon);
          display.fillRect(xx, yy, 21 * CHAR_W_PX, CHAR_H_PX, ILI9341_BLUE); // clear the previous line            
          display.println(snprintf_buf);  
          display.setTextSize(2);    // 26 ch wide x 15 ch deep
          display_mode = flight_info;
        }
      #else
        if (millis() - info_millis > 2000) {    // refresh rate
          info_millis = millis();  
          // Latitude
          xx = 0;
          yy = 0;
          display.setCursor(xx,yy);       
          snprintf(snprintf_buf, snp_max, "Lat %.7f", cur.lat);
          display.fillRect(xx+(4*CHAR_W_PX), yy, 11 * CHAR_W_PX, CHAR_H_PX, SCR_BACKGROUND); // clear the previous data           
          display.println(snprintf_buf);  

          // Longitude
          xx = 0;
          yy = 1.8 * CHAR_H_PX;    
          display.setCursor(xx, yy);                 
          snprintf(snprintf_buf, snp_max, "Lon %.7f", cur.lon);
          display.fillRect(xx+(4*CHAR_W_PX), yy, 11 * CHAR_W_PX, CHAR_H_PX, SCR_BACKGROUND);        
          display.println(snprintf_buf); 

          // Volts, Amps and Ah 
          xx = 0;
          yy = 3.6 * CHAR_H_PX;      
          display.setCursor(xx, yy);               
          snprintf(snprintf_buf, snp_max, "%.0fV %.1fA %.1fAh", myFrPort.fr_bat1_volts * 0.1F, myFrPort.fr_bat1_amps * 0.1F, myFrPort.fr_bat1_mAh * 0.001F);     
          display.fillRect(xx, yy, SCR_W_PX, CHAR_H_PX, SCR_BACKGROUND); // clear the whole line  
          display.println(snprintf_buf); 

          // Number of Sats and RSSI
          xx = 0;
          yy = 5.4 * CHAR_H_PX;      
          display.setCursor(xx, yy);            
          snprintf(snprintf_buf, snp_max, "Sats %d RSSI %ld%%", myFrPort.fr_numsats, myFrPort.fr_rssi); 
          display.fillRect(xx+(5*CHAR_W_PX), yy, 3 * CHAR_W_PX, CHAR_H_PX, SCR_BACKGROUND);  
          display.fillRect(xx+(12*CHAR_W_PX), yy, 4 * CHAR_W_PX, CHAR_H_PX, SCR_BACKGROUND);   // blank rssi     
          display.println(snprintf_buf);       

           
          #if (defined SSD1306_Display)
            display.display();
          #endif 
  
        }
      #endif    
    } 
    #endif    
    
    //===================================
    #if defined displaySupport  
    
    void SetupLogDisplayStyle() {
       
      #if (defined ST7789_Display)      // LILYGO® TTGO T-Display ESP32 1.14" ST7789 Colour LCD
        #if (SCR_ORIENT == 0)           // portrait
          display.setRotation(0);       // or 4 
          display.setTextFont(0);       // Original Adafruit font 0, try 0 thru 6 
        #elif (SCR_ORIENT == 1)         // landscape
          display.setRotation(3);       // or 1 
          display.setTextFont(1);        
        #endif   
         
      display.setTextSize(TEXT_SIZE);
      display.fillScreen(SCR_BACKGROUND);
      display.setTextColor(TFT_SKYBLUE);    
            
      //display.setTextColor(TFT_WHITE);
      //display.setTextColor(TFT_BLUE);  
      //display.setTextColor(TFT_GREEN, TFT_BLACK);
    
      #elif (defined SSD1306_Display)            // all  boards with SSD1306 OLED display
        display.clearDisplay(); 
        display.setTextColor(WHITE);  
        display.setTextSize(TEXT_SIZE);  
 
      #elif (defined SSD1331_Display)            // T2 board with SSD1331 colour TFT display
        //  software SPI pins defined in config.h 
        display.fillScreen(BLACK);
        display.setCursor(0,0);
        display.setTextSize(TEXT_SIZE);
        #define SCR_BACKGROUND BLACK  
        
      #elif (defined ILI9341_Display)           // ILI9341 2.8" COLOUR TFT SPI 240x320 V1.2  
        //  hardware SPI pins defined in config.h 
        display.fillScreen(ILI9341_BLUE);    
        display.setCursor(0,0);
        display.setTextSize(TEXT_SIZE);    // setup in config.h  
        #if (SCR_ORIENT == 0)              // portrait
          display.setRotation(2);          // portrait pins at the top rotation      
        #elif (SCR_ORIENT == 1)            // landscape
          display.setRotation(3);          // landscape pins on the left    
        #endif 
        #define SCR_BACKGROUND ILI9341_BLUE 
      #endif

    }
   #endif
    //===================================
    #if defined displaySupport  
    
    void SetupInfoDisplayStyle() {
    
      #if (defined ST7789_Display)      // LILYGO® TTGO T-Display ESP32 1.14" ST7789 Colour LCD
        #if (SCR_ORIENT == 0)           // portrait
          display.setRotation(0);       // or 4 
          display.setTextSize(1);
          display.setTextFont(0);       // Original Adafruit font 0, try 0 thru 6 
        #elif (SCR_ORIENT == 1)         // landscape
          display.setRotation(3);       // or 1
          display.setTextSize(2);  
          display.setTextFont(1);        
        #endif    
      
      display.fillScreen(SCR_BACKGROUND);
      display.setTextColor(TFT_SKYBLUE);    
            
      //display.setTextColor(TFT_WHITE);
      //display.setTextColor(TFT_BLUE);  
      //display.setTextColor(TFT_GREEN, TFT_BLACK);
    
    #elif (defined SSD1306_Display)            // all  boards with SSD1306 OLED display
      display.clearDisplay(); 
      display.setTextColor(WHITE);  
      display.setTextSize(1);  
 
    #elif (defined SSD1331_Display)            // T2 board with SSD1331 colour TFT display
      //  SPI pins defined in config.h 
      display.fillScreen(BLACK);
      display.setTextColor(WHITE);  
      display.setTextSize(1);
      #define SCR_BACKGROUND BLACK  
      
    #elif (defined ILI9341_Display)            // ILI9341 2.8" COLOUR TFT SPI 240x320 V1.2  
      //  SPI pins defined in config.h 
      display.fillScreen(ILI9341_BLUE);
      display.setRotation(3);          // landscape pins on the left  
      display.setTextSize(2);     
      display.setCursor(0,0);
      #define SCR_BACKGROUND ILI9341_BLUE      
    #endif
   }
   #endif    


//================================================================================================= 
#if (defined ESP32)
void WiFiEventHandler(WiFiEvent_t event)  {
    Log.printf("[WiFi-event] event: %d ", event);

    switch (event) {
        case SYSTEM_EVENT_WIFI_READY: 
            Log.println("WiFi interface ready");
            break;
        case SYSTEM_EVENT_SCAN_DONE:
            Log.println("Completed scan for access points");
            break;
        case SYSTEM_EVENT_STA_START:
            Log.println("WiFi client started");
            break;
        case SYSTEM_EVENT_STA_STOP:
            Log.println("WiFi client stopped");
            break;
        case SYSTEM_EVENT_STA_CONNECTED:
            Log.println("Connected to access point");
            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            Log.println("Disconnected from WiFi access point");
            break;
        case SYSTEM_EVENT_STA_AUTHMODE_CHANGE:
            Log.println("Authentication mode of access point has changed");
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
            Log.print("Obtained IP address: ");
            Log.println(WiFi.localIP());
            break;
        case SYSTEM_EVENT_STA_LOST_IP:
            Log.println("Lost IP address and IP address is reset to 0");
            break;
        case SYSTEM_EVENT_STA_WPS_ER_SUCCESS:
            Log.println("WiFi Protected Setup (WPS): succeeded in enrollee mode");
            break;
        case SYSTEM_EVENT_STA_WPS_ER_FAILED:
            Log.println("WiFi Protected Setup (WPS): failed in enrollee mode");
            break;
        case SYSTEM_EVENT_STA_WPS_ER_TIMEOUT:
            Log.println("WiFi Protected Setup (WPS): timeout in enrollee mode");
            break;
        case SYSTEM_EVENT_STA_WPS_ER_PIN:
            Log.println("WiFi Protected Setup (WPS): pin code in enrollee mode");
            break;
        case SYSTEM_EVENT_AP_START:
            Log.println("WiFi access point started");
            break;
        case SYSTEM_EVENT_AP_STOP:
            Log.println("WiFi access point  stopped");
            break;
        case SYSTEM_EVENT_AP_STACONNECTED:
            Log.println("Client connected");
            break;
        case SYSTEM_EVENT_AP_STADISCONNECTED:
            Log.println("Client disconnected");
            break;
        case SYSTEM_EVENT_AP_STAIPASSIGNED:
            Log.println("Assigned IP address to client");
            break;
        case SYSTEM_EVENT_AP_PROBEREQRECVED:
            Log.println("Received probe request");
            break;
        case SYSTEM_EVENT_GOT_IP6:
            Log.println("IPv6 is preferred");
            break;
        case SYSTEM_EVENT_ETH_START:
            Log.println("Ethernet started");
            break;
        case SYSTEM_EVENT_ETH_STOP:
            Log.println("Ethernet stopped");
            break;
        case SYSTEM_EVENT_ETH_CONNECTED:
            Log.println("Ethernet connected");
            break;
        case SYSTEM_EVENT_ETH_DISCONNECTED:
            Log.println("Ethernet disconnected");
            break;
        case SYSTEM_EVENT_ETH_GOT_IP:
            Log.println("Obtained IP address");
            break;
        default: break;
    }
}
#endif
/* AVAILABLE EVENTS:
0  SYSTEM_EVENT_WIFI_READY               < ESP32 WiFi ready
1  SYSTEM_EVENT_SCAN_DONE                < ESP32 finish scanning AP
2  SYSTEM_EVENT_STA_START                < ESP32 station start
3  SYSTEM_EVENT_STA_STOP                 < ESP32 station stop
4  SYSTEM_EVENT_STA_CONNECTED            < ESP32 station connected to AP
5  SYSTEM_EVENT_STA_DISCONNECTED         < ESP32 station disconnected from AP
6  SYSTEM_EVENT_STA_AUTHMODE_CHANGE      < the auth mode of AP connected by ESP32 station changed
7  SYSTEM_EVENT_STA_GOT_IP               < ESP32 station got IP from connected AP
8  SYSTEM_EVENT_STA_LOST_IP              < ESP32 station lost IP and the IP is reset to 0
9  SYSTEM_EVENT_STA_WPS_ER_SUCCESS       < ESP32 station wps succeeds in enrollee mode
10 SYSTEM_EVENT_STA_WPS_ER_FAILED        < ESP32 station wps fails in enrollee mode
11 SYSTEM_EVENT_STA_WPS_ER_TIMEOUT       < ESP32 station wps timeout in enrollee mode
12 SYSTEM_EVENT_STA_WPS_ER_PIN           < ESP32 station wps pin code in enrollee mode
13 SYSTEM_EVENT_AP_START                 < ESP32 soft-AP start
14 SYSTEM_EVENT_AP_STOP                  < ESP32 soft-AP stop
15 SYSTEM_EVENT_AP_STACONNECTED          < a station connected to ESP32 soft-AP
16 SYSTEM_EVENT_AP_STADISCONNECTED       < a station disconnected from ESP32 soft-AP
17 SYSTEM_EVENT_AP_STAIPASSIGNED         < ESP32 soft-AP assign an IP to a connected station
18 SYSTEM_EVENT_AP_PROBEREQRECVED        < Receive probe request packet in soft-AP interface
19 SYSTEM_EVENT_GOT_IP6                  < ESP32 station or ap or ethernet interface v6IP addr is preferred
20 SYSTEM_EVENT_ETH_START                < ESP32 ethernet start
21 SYSTEM_EVENT_ETH_STOP                 < ESP32 ethernet stop
22 SYSTEM_EVENT_ETH_CONNECTED            < ESP32 ethernet phy link up
23 SYSTEM_EVENT_ETH_DISCONNECTED         < ESP32 ethernet phy link down
24 SYSTEM_EVENT_ETH_GOT_IP               < ESP32 ethernet got IP from connected AP
25 SYSTEM_EVENT_MAX
*/

    //===================================================================
    void Free_Bluetooth_RAM() {
    #if (defined btBuiltin)  
      if (!btDisabled) {  // if not already disabled           
        esp_bt_controller_disable();
        esp_bt_controller_deinit();
        esp_bt_mem_release(ESP_BT_MODE_BTDM);
        btActive = false;
        btDisabled = true;
        Log.println("Bluetooth disabled to free up SRAM for web support. PROCEED TO REBOOT for BT to be restored"); 
        LogScreenPrintln("Bluetooth disabled");               
        #if ((defined ESP32) || (defined ESP8266)) && (defined Debug_SRAM)
          Log.printf("==============>Free Heap after bluetooth disabled = %d\n", ESP.getFreeHeap());
        #endif
      }
    #endif
    }
    //=================================================================== 
        String wifiStatusText(uint16_t wifi_status) {
       switch(wifi_status) {  
        case 0:
          return "WL_IDLE_STATUS";  
        case 1:
          return "WL_NO_SSID_AVAIL"; 
        case 2:
          return "WL_SCAN_COMPLETED";                
        case 3:
          return "WL_CONNECTED";                       
        case 4:
          return "WL_CONNECT_FAILED";            
        case 5:
          return "WL_CONNECTION_LOST";       
        case 6:
          return "WL_DISCONNECTED";  
        case 255:
          return "WL_NO_SHIELD";               
        default:
          return "UNKNOWN";     
       }
    }
    //=================================================================== 
    #if  (defined ILI9341_Display)
    uint32_t draw_horizon(float roll, float pitch, int16_t width, int16_t height) {
      int16_t x0, y0, x1, y1, xc, yc, ycp, lth, tick_lean;
      static int16_t px0, py0, px1, py1, pycp, ptick_lean; 
      uint8_t tick_height = 5;  
      float roll_slope = 0;      // [-180,180]
      float pitch_offset = 0;     //  [-90,90]
      const float AngleToRad = PI / 180;
      
      xc = width / 2;    // centre point / pivot / origin
      yc = height / 2;   // 

      display.drawLine(0 + (width * 0.2), yc, width - (width * 0.2), yc, ILI9341_WHITE);   // static reference horizon
      display.drawLine(xc, yc+10, xc, yc-10, ILI9341_WHITE);
      
      roll_slope = tan(roll * AngleToRad);             // roll slope 
      pitch_offset = (sin(pitch * AngleToRad)) * yc;   // pitch offset  
      tick_lean = roll_slope * tick_height;

      lth = (xc * 0.8) * cos(roll * AngleToRad);
      x0 = (xc - lth);
      x1 = (xc + lth);

      y0 = yc - ((xc - x0) * roll_slope);
      y1 = yc + ((x1 - xc) * roll_slope);   
  
      y0 += pitch_offset;
      y1 += pitch_offset;
      ycp = yc + pitch_offset;
     
      static bool ft = true;
      if (!ft) {
        display.drawLine(px0, py0, px1, py1, ILI9341_BLUE);   // Erase old horizon line 
        display.drawLine(xc-ptick_lean, pycp+tick_height, xc+ptick_lean, pycp-tick_height, ILI9341_BLUE); 
        display.drawLine(xc-ptick_lean+1, pycp+tick_height, xc+ptick_lean+1, pycp-tick_height, ILI9341_BLUE);
        display.drawLine(px0-ptick_lean+1, py0+tick_height, px0+ptick_lean+1, py0-tick_height, ILI9341_BLUE);
        display.drawLine(px1-ptick_lean+1, py1+tick_height, px1+ptick_lean+1, py1-tick_height, ILI9341_BLUE);        
        display.drawLine(px0+1, py0, px1+1, py1,ILI9341_BLUE);
      }  
      ft = false;
      display.drawLine(x0, y0, x1, y1, ILI9341_WHITE);      // Horizon line over the top
      display.drawLine(xc-tick_lean, ycp+tick_height, xc+tick_lean, ycp-tick_height, ILI9341_WHITE);
      display.drawLine(xc-tick_lean+1, ycp+tick_height, xc+tick_lean+1, ycp-tick_height, ILI9341_WHITE);
      display.drawLine(x0-tick_lean+1, y0+tick_height, x0+tick_lean+1, y0-tick_height, ILI9341_WHITE);
      display.drawLine(x1-tick_lean+1, y1+tick_height, x1+tick_lean+1, y1-tick_height, ILI9341_WHITE); 
      display.drawLine(x0+1, y0, x1+1, y1, ILI9341_WHITE);
      px0 = x0;
      py0 = y0;
      px1 = x1;
      py1 = y1;    
      pycp = ycp;
      ptick_lean = tick_lean;
    
      return micros();   
    }
    #endif
    //=================================================================== 
    #if  (defined ILI9341_Display)
    void draw_home_arrow(int16_t x, int16_t y, int16_t arrow_angle, int16_t width, int16_t height) {
      int16_t x0, y0, x1, y1, x2, y2, x3, y3;
      static int16_t px0, py0, px1, py1, px2, py2, px3, py3;
      int16_t opp, adj, hyp, opp90, adj90, hyp90;
      const int16_t al = 40;  // arrow length  
      static bool ft = true;
      const float AngleToRad = PI / 180;
      
      int16_t home_angle = 0 - arrow_angle - 25;    // direction of rotation
    
      home_angle = (home_angle < 0) ? home_angle + 360 : home_angle;
      home_angle = (home_angle > 360) ? home_angle - 360 : home_angle;   
      //Log.printf("home_angle=%d \n", pt_home_angle);         
      hyp = al / 2;        
      opp = hyp * sin(home_angle * AngleToRad);
      adj = hyp * cos(home_angle * AngleToRad);
      
      hyp90 = al / 5; 
      opp90 = hyp90 * sin((home_angle + 90) * AngleToRad);
      adj90 = hyp90 * cos((home_angle + 90) * AngleToRad); 
      
      x0 = x + adj; 
      y0 = y - opp; 
      
      x1 = x - adj;
      y1 = y + opp;
 
      x2 = x1 + adj90;
      y2 = y1 - opp90;

      x3 = x1 - adj90;
      y3 = y1 + opp90;
           
      if (!ft) {
       //display.drawTriangle(px0, py0, px2, py2, px3, py3, ILI9341_BLUE);   
       display.fillTriangle(px0, py0, px2, py2, px3, py3, ILI9341_BLUE);                 
      }
      ft = false;     

      //display.drawTriangle(x0, y0, x2, y2, x3, y3, ILI9341_RED); 
      display.fillTriangle(x0, y0, x2, y2, x3, y3, ILI9341_RED);         
             
      px0 = x0; py0 = y0; px1 = x1; py1 = y1; px2 = x2; py2 = y2; px3 = x3; py3 = y3;

    }
    #endif

    //================================================================================================= 
    void nbdelay(uint32_t delaymS) { // non-blocking delay
    uint32_t start;
      start = millis();
  
      while (millis() - start < delaymS) {     
        yield();
      }
    } 
        
