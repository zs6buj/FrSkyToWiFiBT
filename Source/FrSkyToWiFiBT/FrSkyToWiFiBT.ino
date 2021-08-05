/*
=====================================================================================================================
     
     FrSkyToWiFi

     Author: Eric Stockenstrom
       
     First code February 2021
    
     License and Disclaimer

 
  This software is provided under the GNU v2.0 License. All relevant restrictions apply. In case there is a conflict,
  the GNU v2.0 License is overriding. This software is provided as-is in the hope that it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
  General Public License for more details. In no event will the authors and/or contributors be held liable for any 
  damages arising from the use of this software.

  Permission is granted to anyone to use this software for any purpose, including commercial 
  applications, and to alter it and redistribute it freely, subject to the following restrictions:

  1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software.
  2. If you use this software in a product, an acknowledgment in the product documentation would be appreciated.
  3. Altered versions must be plainly marked as such, and must not be misrepresented as being the original software.

  By downloading this software you are agreeing to the terms specified in this page and the spirit thereof.
    
 
*/
//    =====================================================================================================================
  
#include "global_variables.h"
#include "config.h"                      // ESP_IDF libs included here
#include "FrSky_Simple_Ports.h"

//=================================================================================================   
//                     F O R W A R D    D E C L A R A T I O N S
//=================================================================================================


void main_loop();
void ServiceWiFiRoutines();
void CheckStaLinkStatus(); 
void StartWiFiTimer();
void RestartWiFiSta();
void Start_Access_Point();
void ServiceStatusLeds();
void BlinkFrsLed(uint32_t);
void DisplayRemoteIP();
void WebServerSetup();
void RecoverSettingsFromFlash();
void PrintPeriod(bool);
void PrintLoopPeriod();
void SetupWiFi(); 
void handleLoginPage();
void handleSettingsPage();
void handleSettingsReturn();
void handleReboot();
void handleOtaPage();
uint8_t EEPROMRead8(uint16_t);
void ReadSettingsFromEEPROM();
uint32_t EEPROMRead32(uint16_t);
uint16_t EEPROMRead16(uint16_t);
void RefreshHTMLButtons();
void RawSettingsToStruct();
void WriteSettingsToEEPROM();
void EEPROMReadString(uint16_t, char*);
void EEPROMWrite16(uint16_t, uint16_t);
void EEPROMWrite8(uint16_t, uint8_t);
void EEPROMWriteString(uint16_t, char*);
void EEPROMWrite32(uint16_t, uint32_t);

void PrintRemoteIP();
void LogScreenPrint(String);
#if (defined ESP32) || (defined ESP8266)
  void IRAM_ATTR gotButtonDn();
  void IRAM_ATTR gotButtonUp();
  void IRAM_ATTR gotButtonInfo();  
  void IRAM_ATTR gotWifiButton();
  String wifiStatusText(uint16_t);    
#endif
#if (defined ESP32)
  void IRAM_ATTR onTimer();                         // wifi retry timer
  esp_err_t event_handler(system_event_t *event);   // wifi events handler
#endif  
#if defined displaySupport
  void PaintLogScreen(uint8_t, last_row_t);
  void Scroll_Display(scroll_t);
  void SetupLogDisplayStyle(); 

  void HandleDisplayButtons();  
#endif


  FrSkyPort myFrPort;   // instantiate FrSky Port object   



//=================================================================================================
//=================================================================================================   
//                                      S   E   T   U   P 
//=================================================================================================
//=================================================================================================
void setup()  {
 
  Log.begin(115200);
  delay(2500);
  Log.println();
  pgm_path = __FILE__;  // ESP8266 __FILE__ macro returns pgm_name and no path
  pgm_name = pgm_path.substring(pgm_path.lastIndexOf("\\")+1);  
  pgm_name = pgm_name.substring(0, pgm_name.lastIndexOf('.'));  // remove the extension
  Log.print("Starting "); Log.print(pgm_name);
  Log.printf(" version:%d.%02d.%02d\n", MAJOR_VERSION,  MINOR_VERSION, PATCH_LEVEL);
    
  #if (defined Debug_WiFi)
   WiFi.onEvent(WiFiEventHandler);   
  #endif  
 
  #if (defined Debug_SRAM)
    Log.printf("Free Heap just after startup = %d\n", ESP.getFreeHeap());
  #endif  
//=================================================================================================   
//                                   S E T U P   D I S P L A Y
//=================================================================================================
  #if (defined displaySupport) 

    #if (defined ESP32)
    
      if (Tinfo != 99)  {   // enable info touch pin-pair
        touchAttachInterrupt(digitalPinToInterrupt(Tinfo), gotButtonInfo, threshold); 
       } else
      if (Pinfo != 99)  {   // enable info digital pin
        pinMode(Pinfo, INPUT_PULLUP);   
      }  
       
      if ( (Tup != 99) && (Tdn != 99) ) {   // enable touch pin-pair
        touchAttachInterrupt(digitalPinToInterrupt(Tup), gotButtonUp, threshold);
        touchAttachInterrupt(digitalPinToInterrupt(Tdn), gotButtonDn, threshold);   
      } else
      if ( (Pup != 99) && (Pdn != 99) ) {   // enable digital pin-pair
        pinMode(Pup, INPUT_PULLUP);
        pinMode(Pdn, INPUT_PULLUP);          
      }

    #endif  

    #if (defined ESP8266)        
      if ( (Pup != 99) && (Pdn != 99) ) { // enable digital pin pair
        pinMode(Pup, INPUT_PULLUP);
        pinMode(Pdn, INPUT_PULLUP);
      }

    #endif 

    #if (defined ST7789_Display)               // LILYGO® TTGO T-Display ESP32 1.14" ST7789 Colour LCD (135 x 240)
      display.init(); 
      #define SCR_BACKGROUND TFT_BLACK
      
    #elif (defined SSD1306_Display)            // all  boards with SSD1306 OLED display (128 x 64)
      #if not defined TEENSY3X                 // Teensy uses default SCA and SCL in teensy "pins_arduino.h"
         Wire.begin(SDA, SCL);
      #endif   
      display.begin(SSD1306_SWITCHCAPVCC, i2cAddr);  
      #define SCR_BACKGROUND BLACK   
      
    #elif (defined SSD1331_Display)            // T2 board with SSD1331 colour TFT display (96 x 64)
        //  uses software SPI pins defined in config.h 
        display.begin();
        #define SCR_BACKGROUND BLACK  
        
    #elif (defined ILI9341_Display)    // ILI9341 2.8" COLOUR TFT SPI 240x320 V1.2 (320 x 240)
        //  uses hardware SPI pins defined in config.h 
        display.begin();
        #define SCR_BACKGROUND ILI9341_BLUE
    #endif
   
    #if (SCR_ORIENT == 0)              // portrait
        Log.print("Display Setup: Portrait ");         
    #elif (SCR_ORIENT == 1)            // landscape   
        Log.println("Display support activated: Landscape "); 
    #endif

    int eol= 0;
    for (eol = 0 ; eol < SCR_W_CH ; eol++) {
      clear_line[eol] = ' ';
    }
    clear_line[eol] = 0x00;

    Log.printf("%dx%d  TEXT_SIZE=%d  CHAR_W_PX=%d  CHAR_H_PX=%d  SCR_H_CH=%d  SCR_W_CH=%d\n", SCR_H_PX, SCR_W_PX, TEXT_SIZE, CHAR_W_PX, CHAR_H_PX, SCR_H_CH, SCR_W_CH);
    
    LogScreenPrintln("Starting .... ");
  #endif
  /*
  display.setFont(Dialog_plain_8);     //  col=24 x row 8  on 128x64 display
  display.setFont(Dialog_plain_16);    //  col=13 x row=4  on 128x64 display
  */
  #if ((defined ESP32) || (defined ESP8266)) && (defined Debug_SRAM)
    Log.printf("==============>Free Heap after OLED setup = %d\n", ESP.getFreeHeap());
  #endif
  
//=================================================================================================   
//                             S E T U P   E E P R O M
//=================================================================================================

  #if defined ESP32    
    if (!EEPROM.begin(EEPROM_SIZE))  {
      Log.println("Fatal error!  EEPROM failed to initialise.");
      LogScreenPrintln("EEPROM fatal error!");
      while (true) delay(100);  // wait here forever 
     } else {
      Log.println("EEPROM initialised successfully");
      LogScreenPrintln("EEPROM good"); 
     }
  #endif       
    
  #if defined ESP8266
    EEPROM.begin(EEPROM_SIZE);
    Log.println("EEPROM initialised successfully");
    LogScreenPrintln("EEPROM good"); 
  #endif

  RawSettingsToStruct();      // So that we can use them regardless of webSupport
  
  #if (defined webSupport) 
    RecoverSettingsFromFlash(); 
  #endif

  #if ((defined ESP32) || (defined ESP8266)) && (defined Debug_SRAM)
    Log.printf("==============>Free Heap after EEPROM setup = %d\n", ESP.getFreeHeap());
  #endif

  // =============================================
  
  #if (defined ESP32) //  ESP32 Board
    Log.print("ESP32 / Variant is ");
    LogScreenPrintln("ESP32 Variant is");
    #if (ESP32_Variant == 1)
      Log.println("Dev Module");
      LogScreenPrintln("Dev Module");
    #endif
    #if (ESP32_Variant == 2)
      Log.println("Wemos® LOLIN ESP32-WROOM-32");
      LogScreenPrintln("Wemos® LOLIN");
    #endif
    #if (ESP32_Variant == 3)
      Log.println("Dragonlink V3 slim with internal ESP32");
      LogScreenPrintln("Dragonlink V3 ESP32");
    #endif
    #if (ESP32_Variant == 4)
      Log.println("Heltec Wifi Kit 32");
      LogScreenPrintln("Heltec Wifi Kit 32");
    #endif
    #if (ESP32_Variant == 5)
      Log.println("LILYGO® TTGO T-Display ESP32 1.14 inch ST7789 Colour LCD");
      LogScreenPrintln("TTGO T-Display ESP32");
    #endif    
    #if (ESP32_Variant == 6)
      Log.println("LILYGO® TTGO T2 ESP32 OLED SD");
      LogScreenPrintln("TTGO T2 ESP32 SD");
    #endif 
    #if (ESP32_Variant == 7)
      Log.println("Dev Module with ILI9341 2.8in COLOUR TFT SPI");
      LogScreenPrintln("Dev module + TFT");
    #endif                   
  #elif (defined ESP8266) 
    Log.print("ESP8266 / Variant is ");
    LogScreenPrintln("ESP8266 / Variant is");  
    #if (ESP8266_Variant == 1)
      Log.println("Lonlin Node MCU 12F");
      LogScreenPrintln("Node MCU 12");
    #endif 
    #if (ESP8266_Variant == 2)
      Log.println("ESP-F - RFD900X TX-MOD");
      LogScreenPrintln("RFD900X TX-MOD");
    #endif   
    #if (ESP8266_Variant == 3)
      Log.println("Wemos® ESP-12F D1 Mini");
      LogScreenPrintln("Wemos ESP-12F D1 Mini");
    #endif       
  #endif


  if (set.fr_io == fr_bt)     {          // 1
    Log.println("FrSky BT Out");
    LogScreenPrintln("Frs BT Out");
  } else
  if (set.fr_io == fr_wifi)  {           // 2
    Log.println("FrSky WiFi Out");
    LogScreenPrintln("Frs WiFi Out");
  } else   
  if (set.fr_io == fr_wifi_bt)    {  // 3
    Log.println("FrSky WiFi+BT Out");
    LogScreenPrintln("Frs WiFi+BT Out");
  } 

  if ( (set.fr_io == fr_wifi) ||  (set.fr_io == fr_wifi_bt) || (set.web_support)) {
   if (set.wfmode == ap)  {
     Log.println("WiFi mode is AP");
     LogScreenPrintln("WiFi-mode is AP");
   } else
   if (set.wfmode == sta)  {
     Log.println("WiFi mode is STA");
     LogScreenPrintln("WiFi-mode is STA");
   } else
   if (set.wfmode == sta_ap)  {
     Log.println("WiFi mode is STA>AP");
     LogScreenPrintln("WiFi-mode=STA>AP");
   } 
    
   Log.println("Protocol is IP-Targeted UDP");
   LogScreenPrintln("Protocol = UDP");

  }


//=================================================================================================   
//                                S E T U P   W I F I  --  E S P only
//=================================================================================================

    if ( (set.fr_io == fr_wifi) || (set.fr_io == fr_wifi_bt) || (set.web_support)) {     
      #if (not defined Start_WiFi)
        if (startWiFiPin != 99) { 
          pinMode(startWiFiPin, INPUT_PULLUP);
          attachInterrupt(digitalPinToInterrupt(startWiFiPin),gotWifiButton, FALLING);   // For optional Start_WiFi button       
        }
      #endif       
    }

//=================================================================================================   
//                                   S E T U P   B L U E T O O T H
//=================================================================================================

// Slave advertises hostname for pairing, master connects to slavename

  #if (defined btBuiltin)   
    if ((set.fr_io == fr_bt) || (set.fr_io == fr_wifi_bt)) { 

      if (set.btmode == 1)   {               // master
        Log.printf("Bluetooth master mode host %s is trying to connect to slave %s. This can take up to 30s\n", set.host, set.btConnectToSlave); 
        LogScreenPrintln("BT connecting ......");   
        SerialBT.begin(set.host, true);      
        // Connect(address) is relatively fast (10 secs max), connect(name) is slow (30 secs max) as it 
        // needs to resolve the name to an address first.
        // Set CoreDebugLevel to Info to view devices bluetooth addresses and device names
        bool bt_connected;
        bt_connected = SerialBT.connect(set.btConnectToSlave);
        if(bt_connected) {
          Log.println("Bluetooth done");  
          LogScreenPrintln("BT done");
        }          
      } else {                               // slave, passive                           
        SerialBT.begin(set.btConnectToSlave);        
        Log.printf("Bluetooth slave mode, slave name for pairing is %s\n", set.btConnectToSlave);               
      } 
      btActive = true;    
    }
  #else
    Log.println("No Bluetooth options selected, BT support not compiled in");
    btActive = false;
  #endif
  
  #if ((defined ESP32) || (defined ESP8266)) && (defined Debug_SRAM)
    Log.printf("==============>Free Heap after Bluetooth setup = %d\n", ESP.getFreeHeap());
  #endif

//=================================================================================================   
//                                    S E T U P   S E R I A L
//=================================================================================================  
            
  myFrPort.initialise(set.frport);
  
  #if ((defined ESP32) || (defined ESP8266)) && (defined Debug_SRAM)
    Log.printf("==============>Free Heap after Serial UART setup = %d\n", ESP.getFreeHeap());
  #endif

//=================================================================================================   
//                                    S E T U P   O T H E R
//=================================================================================================   
  frsGood = false;

  pinMode(FrsStatusLed, OUTPUT); 

}

  
//================================================================================================= 
//================================================================================================= 
//                                   M  A  I  N    L  O  O  P
//================================================================================================= 
//================================================================================================= 

void loop() {
                             
 #if ((defined ESP32) || (defined ESP8266)) 
   ServiceWiFiRoutines();
 #endif

 //====================
  
  #if defined Debug_Loop_Period
    PrintLoopPeriod();
  #endif

  
 //====================        H a n d l e   F r S k y   P o r t   T r a f f i c
 
  myFrPort.HandleTraffic();  

  //===============   Check For Display Button Touch / Press
  
  #if defined displaySupport
    HandleDisplayButtons();
  #endif
     
  
  //====================   Service The Leds
  
  ServiceStatusLeds();
  
  //====================   Service The Web Server
  #if defined webSupport     //  esp32 and esp8266 only
    if (wifiSuGood) {
      server.handleClient();
    }
  #endif
 
}
//================================================================================================= 
//                               E N D   O F    M  A  I  N    L  O  O  P
//================================================================================================= 


  
