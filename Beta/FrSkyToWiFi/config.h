//================================================================================================= 
//================================================================================================= 
//
//                                    C O N F I G U R A T I O N 
// 
#define MAJOR_VERSION      0
#define MINOR_VERSION      0
#define PATCH_LEVEL        3
/*
=================================================================================================== 
                                M o s t    R e c e n t   C h a n g e s
=================================================================================================== 

Complete change log and debugging options are at the bottom of this tab
                                          
 v0.0.2               Added HUD display, bug fixes
 V0.0.3  2021-03-04   Hud rssi blank fix
         2021-03-09   Serial-in, allways auto detect polarity, speed and protocol.
                      Other small fixes.
                                                                 
*/
//===========================================================================================
//
//                      PLEASE SELECT YOUR DEFAULT OPTIONS BELOW BEFORE COMPILING
//
//              Most of the settings below are saved to EEPROM the first time mav2pt runs
//              Use the web interface to change them, or Reset_Web_Defaults below
//
//===========================================================================================

                                       // Most of the settings below are saved to EEPROM the first time mav2pt runs
                                       // They should be change through the web interface
//#define Reset_Web_Defaults           // Reset settings in eeprom. Do this if you suspect eeprom settings are corrupt USE SPARINGLY


// Choose only one setting for FrSky Port Type 
//#define FrSky_Port_Type 0   // No FrSky Port support needed. Now I'm a "Mavlink Switch"
//#define FrSky_Port_Type 1   // F.Port v1
//#define FrSky_Port_Type 2   // F.Port v2 FrSky ISRM/ACCESS capable transmitters and receivers only
#define FrSky_Port_Type 3   // S.Port / legacy
//#define FrSky_Port_Type 4   // AUTO detect



#define webSupport                  // ESP only. Enable wifi web support, including OTA firmware updating. Browse to IP.
#define webPassword      "admin"    // Web password, change me!

#define displaySupport                 // Enable if you have a display attached - choose display type where board variant is defined 


//=================================================================================================
//                D E F A U L T   F R S K Y   S / F P O R T   I / O    S E T T I N G S    
//=================================================================================================
// Choose only one of these default FrSky S/Port I/O channels
// How does FrSky telemetry leave this board
// Select one only

//#define FrSky_IO     1     // BT
//#define FrSky_IO     2     // WiFi
#define FrSky_IO     3     // WiFi+BT

#if (not defined FrSky_IO) 
  #define FrSky_IO  0 
#endif

#define Derive_PWM_Channels   // from fport control channel. just an interesting option at this juncture

// NOTE: The Bluetooth class library uses a lot of SRAM application memory. During Compile/Flash
//       you may need to select Tools/Partition Scheme: "Minimal SPIFFS (1.9MB APP ...) or similar


//================================================================================================= 
//=================================================================================================                             
//                          S E L E C T   E S P   B O A R D   V A R I A N T   
//=================================================================================================
//================================================================================================= 
//#define ESP32_Variant     1    //  ESP32 Dev Board - Use Partition Scheme: "Minimal SPIFFS(1.9MB APP...)"
//#define ESP32_Variant     2    //  Wemos® LOLIN ESP32-WROOM-32_OLED_Dual_26p
//#define ESP32_Variant     3    //  Dragonlink V3 slim with internal ESP32 - contributed by Noircogi - Select ESP32 Dev Board in IDE
//#define ESP32_Variant     4    //  Heltec Wifi Kit 32 - Use Partition Scheme: "Minimal SPIFFS(Large APPS with OTA)" - contributed by Noircogi select Heltec wifi kit
#define ESP32_Variant     5    //  LILYGO® TTGO T-Display ESP32 1.14" ST7789 Colour LCD (135 x 240) - Select TTGO_T1 in IDE
//#define ESP32_Variant     6    //  LILYGO® TTGO T2 SD SSD1331 TFT Colour 26pin - 16Ch x 8 lines (96 x 64)- Select ESP32 Dev Board in IDE
//#define ESP32_Variant     7    // ESP32 Dev Board with ILI9341 2.8" COLOUR TFT SPI 240x320 V1.2  select Dev Board in IDE

//#define ESP8266_Variant   1   // NodeMCU ESP 12F - choose "NodeMCU 1.0(ESP-12E)" board in the IDE
#define ESP8266_Variant   2   // ESP-12E, ESP-F barebones boards. RFD900X TX-MOD, QLRS et al - use Generic ESP8266 on IDE
//#define ESP8266_Variant   3   // ESP-12F - Wemos® LOLIN D1 Mini

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ 
//=================================================================================================
//                      D E F A U L T   B L U E T O O T H   S E T T I N G S   
//=================================================================================================
//#define BT_Mode  1           // Master Mode - active, initiate connection with slave (name)
#define BT_Mode  2           // Slave Mode - passive, advertise our hostname & wait for master to connect to us
#define BT_ConnectToSlave     "Crossfire 0277"  // Example

//=================================================================================================
//                            D E F A U L T   W I F I   S E T T I N G S   
//=================================================================================================

#define Start_WiFi                              // Start WiFi at startup, override startWiFi pin

#define HostName             "Frs2BT"           // This translator's host name
#define APssid               "FrSkyToWiFi"      // The AP SSID that we advertise         ====>
#define APpw                 "password"         // Change me! Must be >= 8 chars
#define APchannel            9                  // The wifi channel to use for our AP
#define STAssid              "OmegaOffice"      // Target AP to connect to (in STA mode) <====
#define STApw                "Navara@98"         // Target AP password (in STA mode). Must be >= 8 chars      

// Choose one default mode for ESP only - AP means advertise as an access point (hotspot). STA means connect to a known host
//#define WiFi_Mode   1  //AP            
//#define WiFi_Mode   2  // STA
#define WiFi_Mode   3  // (STA>AP) STA failover to AP 

// Choose one default protocol - for ESP32 only
//#define Mav_WiFi_Protocol 1    // TCP/IP
#define Mav_WiFi_Protocol 2    // UDP 

//#define UDP_Broadcast      // Comment out (default) if you want to track and target remote udp client ips
// NOTE; UDP is not a connection based protocol. To communicate with > 1 client at a time, we must broadcast on the subnet  

int16_t  TCP_localPort = 5760;     
uint16_t  TCP_remotePort = 5760;    
uint16_t  UDP_localPort = 14551;    // readPort 
uint16_t  UDP_remotePort = 14556;   // sendPort

//=================================================================================================
//                            R  S  S  I    O  P  T  I  O  N  S  
//=================================================================================================

#define Rssi_Pacemaker   200  // mS. RSSI 0xF101 frame sent with this period regardless of rate of RSSI arrival
#define RSSI_Override     70  // Only if (RSSI == 0) force it to value, but only sent when Mavlink good.)                                                                                                            
//#define Rssi_In_Percent     // Un-comment if RSSI is already in %, not relative to (254->100%)
//#define QLRS                // QLRS Longe Range System uses remote rssi field - PR from giocomo892 april 2020)

// RSSI_Source is automatic. Order of precedence: 
//      First:  #109 SiK style RADIO_STATUS
//      Second: #65 RC_CHANNELS
//      Third:  #35 RC_CHANNELS_RAW

//=================================================================================================
//                            O T H E R   U S E R   O P T I O N S  
//=================================================================================================

//defined PitLab                      // Uncomment me to force PitLab OSD stack

//#define Battery_mAh_Source  1       // Get battery mAh from the FC - note both rx and tx lines must be connected      
//#define Battery_mAh_Source  2       // Define bat1_capacity and bat2_capacity below and use those 
const uint16_t bat1_capacity = 5200;       
const uint16_t bat2_capacity = 0;
#define Battery_mAh_Source  3         // Define battery mAh in the LUA script on the Taranis/Horus - Recommended



// Status_Text messages place a huge burden on the meagre 4 byte FrSky telemetry payload bandwidth
// The practice has been to send them 3 times to ensure that they arrive unscathed at the receiver
//  but that makes the bandwidth limitation worse and may crowd out other message types. Try without
//  sending 3 times, but if status_text gets distorted, un-comment the next line
//#define Send_status_Text_3_Times

//#define Send_Sensor_Health_Messages

//#define Request_Missions_From_FC    // Un-comment if you need mission waypoint from FC - NOT NECESSARY RIGHT NOW

//#define Data_Streams_Enabled        // Requests data streams from FC. Requires both rx and tx lines to FC. Rather set SRn in Mission Planner

//================================== Set your time zone here ======================================
// Only for SD / TF Card adapter option
// Date and time determines the TLog file name only
//const float Time_Zone = 10.5;    // Adelaide, Australia
const float Time_Zone = 2.0;    // Jo'burg
bool daylightSaving = false;

//=================================================================================================
//                        E X P E R I M E N T A L    O P T I O N S  
//    Don't change anything here unless you are confident you know the outcome

//#define Support_MavLite

//=================================================================================================   
//                              Auto Determine Target Platform
//================================================================================================= 
//
//                Don't change anything here
//
#if defined (__MK20DX128__) || defined(__MK20DX256__)
  #define TEENSY3X   
      
#elif defined ESP32
  #define Target_Board   3      // Espressif ESP32 Dev Module

#elif defined ESP8266
  #define Target_Board   4      // Espressif ESP8266
  
#else
  #error "Unsupported board type!"
#endif

//=================================================================================================   
//                              CHECK #MACRO OPTIONS LOGIC
//================================================================================================= 

#if (not defined ESP32) && (not defined ESP8266)
  #if defined webSupport
    #undef webSupport
    //    #error webSupport only available on ESP32 or ESP8266
  #endif
#endif
  
#if defined ESP32
  #include <iostream> 
  #include <sstream> 
  #include <driver/uart.h>  // In Arduino ESP32 install repo 
  //C:\Users\<YourName>\AppData\Local\Arduino15\packages\esp32\hardware\esp32\1.0.4\tools\sdk\include\driver

#endif
  

  #if (defined ESP32)
    #ifndef WiFi_Mode 
      #error Please define WiFi_Mode
    #endif
  #endif  

  #if (defined ESP32)
    #ifndef Mav_WiFi_Protocol
      #error Please define Mav_WiFi_Protocol
    #endif
  #endif

  #if (defined ESP32)         
    #ifndef ESP32_Variant 
      #error Please define an ESP32 board variant
    #endif
  #endif

  #if (defined ESP8266)
    #ifndef ESP8266_Variant
         #error Please define an ESP8266 board variant
    #endif
 
  #endif         

  #if (defined ESP32) && (FrSky_IO == 1 || FrSky_IO == 3)
    #define btBuiltin   //  for these features we need bluetooth support compiled in
  #endif

  #if (not defined FrSky_Port_Type)
    #error define FrSky_Port_Type, None or SPort_Version or FPort_Version
  #endif    

 
//=================================================================================================   
//                          P L A T F O R M   D E P E N D E N T   S E T U P S
//================================================================================================= 

  
#if defined TEENSY3X               // Teensy3x
  #define Teensy_One_Wire          // default half-duplex
  #define FrsStatusLed  13
  #define InvertFrsLed false   
  #define BufStatusLed  14        
  #define mav_rxPin      9  
  #define mav_txPin     10
  #define frPort_Serial    1      // Teensy F/SPort port1=pin1, port3=pin8. The default is Serial 1, but 3 is possible 

  #if (frPort_Serial == 1)
    #define fr_rxPin       0      // FPort rx - optional
    #define fr_txPin       1      // FPort tx - Use me in single wire mode 
    #define GC_Mav_rxPin   7    
    #define GC_Mav_txPin   8   
  #elif (frPort_Serial == 3)
    #define fr_txPin       7      // Optional FPort rx  
    #define fr_txPin       8      // Optional FPort tx - use me in single wire mode 
  #endif  
   // #define displaySupport       // un-comment for displaySupport
    #if (defined displaySupport)   // Display type defined with # define displaySupport 
      #define SSD1306_Display      // I2C OLED display type   
      #define SCR_ORIENT   1       // 1 Landscape or 0 Portrait 
      /* Below please choose either Touch pin-pair or Digital pin-pair for display scrolling
       *  Pin == 99 means the pin-pair is not used
       */ 
      #define Pinfo         99        // Digital pin to trigger information display              
      #define Pup           11        // Digital pin to scroll the display up
      #define Pdn           12        // Board Button 2 to scroll the display down   
      // SDA                18        // I2C OLED - default in teensy "pins_arduino.h"
      // SCL                19        // I2C OLED - default in teensy "pins_arduino.h"
      #define i2cAddr      0x3C       // I2C OLED board
    #endif  
 
#elif defined ESP32                 // ESP32 Platform
  
  //========================================================================= 
  //   N O T E:  G P I O 1 2  is a bootstrap pin on the ESP32 - avoid GPIO12 high on bootup

  #if (ESP32_Variant == 1)          // ESP32 Dev Module
    #define FrsStatusLed  02        // Onboard LED
    #define InvertFrsLed false      
    #define BufStatusLed  27        // untested pin      
    #define fr_rxPin      16        // FPort- Not used in 1-wire mode DON'T use 12!
    #define fr_txPin      17        // FPorttx - Use me in single wire mode
    #define startWiFiPin   5        // 5 Trigger WiFi startup  
    
    #if (defined displaySupport)   // Display type defined with board variant
      #define SSD1306_Display         // OLED display type    
      /* Below please choose either Touch pin-pair or Digital pin-pair for display scrolling
       *  Pin == 99 means the pin-pair is not used
       */ 
      #define Pinfo         99        //    Digital pin to toggle information/log page              
      #define Pup           99        // 35 Board Button 1 to scroll the display up
      #define Pdn           99        //  0 Board Button 2 to scroll the display down  
      #define Tinfo         15        // 15 Touch pin to toggle information/log page       
      #define Tup           33        // 33 Touch pin to scroll the display up
      #define Tdn           32        // 32 Touch pin to scroll the display down   
      #define SDA           21        // I2C OLED board
      #define SCL           22        // I2C OLED board
      #define i2cAddr      0x3C       // I2C OLED board
    #endif   
    /*  
      SPI/CS                       Pin 05   For optional TF/SD Card Adapter
      SPI/MOSI                     Pin 23   For optional TF/SD Card Adapter
      SPI/MISO                     Pin 19   For optional TF/SD Card Adapter
      SPI/SCK                      Pin 18   For optional TF/SD Card Adapter  
    */

  #endif
  
  //========================================================================= 
  #if (ESP32_Variant == 2)          // Wemos® LOLIN ESP32-WROOM-32_OLED_Dual_26p
    #define FrsStatusLed  15        // No Onboard LED
    #define InvertFrsLed false     
    #define BufStatusLed  99        // None    
    #define fr_rxPin      25        // FPort- Not used in single wire mode DON'T use 12!
    #define fr_txPin      26        // FPorttx - Use me in single wire mode
    #define startWiFiPin  18        // Trigger WiFi startup
    
    #if (defined displaySupport)   // Display type defined with # define displaySupport 
      #define SSD1306_Display      // OLED display type   
      #define SCR_ORIENT   1       // 1 Landscape or 0 Portrait 
      /* Below please choose either Touch pin-pair or Digital pin-pair for display scrolling
       *  Pin == 99 means the pin-pair is not used
       */ 
      #define Pinfo         99        //    Digital pin to trigger information display              
      #define Pup           99        // 35 Board Button 1 to scroll the display up
      #define Pdn           99        //  0 Board Button 2 to scroll the display down   
      #define Tinfo         99        // 02 Touch pin to toggle information/log page       
      #define Tup           99        // 33 Touch pin to scroll the display up
      #define Tdn           99        // 32 Touch pin to scroll the display down   
      #define Tinfo         99        //    Touch pin to toggle information/log page
      #define SDA           05        // I2C OLED board
      #define SCL           04        // I2C OLED board
      #define i2cAddr      0x3C       // I2C OLED board
    #endif  
  #endif
  
  //=========================================================================  
  #if (ESP32_Variant == 3)          // Dragonlink V3 slim with internal ESP32
    #define FrsStatusLed  18        // Blue LED
    #define InvertFrsLed false    
    #define BufStatusLed  19        // Green LED        
    #define fr_rxPin      16        // FPort- Not used in single wire mode DON'T use 12!
    #define fr_txPin      17        // FPorttx - Use me in single wire mode
    #define startWiFiPin  99        // Trigger WiFi startup  
    
    #if (defined displaySupport)   // Display type defined with # define displaySupport   
      #define SSD1306_Display      // OLED display type 
      #define SCR_ORIENT   0       // 1 Landscape or 0 Portrait       
      /* Below please choose either Touch pin-pair or Digital pin-pair for display scrolling
       *  Pin == 99 means the pin-pair is not used
       */ 
      #define Pinfo         99        //    Digital pin to toggle information/log page         
      #define Pup           99        // 35 Board Button 1 to scroll the display up
      #define Pdn           99        //  0 Board Button 2 to scroll the display down 
      #define Tinfo         99        //    Touch pin to toggle information/log page         
      #define Tup           99        // 33 Touch pin to scroll the display up
      #define Tdn           99        // 32 Touch pin to scroll the display down   
      #define SDA           05        // I2C OLED board
      #define SCL           04        // I2C OLED board 
      #define i2cAddr      0x3C       // I2C OLED board
    #endif  
  #endif
  
  //=========================================================================   
  #if (ESP32_Variant == 4)          // Heltec Wifi Kit 32 (NOTE! 8MB) 
    #define FrsStatusLed    25        // Onboard LED
    #define InvertFrsLed   false     
    #define BufStatusLed    99        // none  
    #define fr_rxPin        27        // FPort rx - (NOTE: DON'T use pin 12! boot fails if pulled high)
    #define fr_txPin        17        // FPort tx - Use me in single wire mode
    #define startWiFiPin    18        // Trigger WiFi startup 
    #if !defined displaySupport       // I2C OLED board is built into Heltec WiFi Kit 32
      #define displaySupport
    #endif  
    #define SSD1306_Display         // OLED display type  
    #define SCR_ORIENT   1          // 1 Landscape or 0 Portrait 
    /* Below please choose either Touch pin-pair or Digital pin-pair for display scrolling
     *  Pin == 99 means the pin-pair is not used
     */ 
    #define Pup           99        // Board Button to scroll the display up
    #define Pdn           99        // Board Button to scroll the display down
    #define Pinfo         99        // 02 Digital pin to toggle information/log page
    #define Tup           33        // 33 Touch pin to scroll the display up
    #define Tdn           32        // 32 Touch pin to scroll the display down 
    #define Tinfo         02        // 02 Touch pin to toggle information/log page
    #define SDA           04        // I2C OLED board 
    #define SCL           15        // I2C OLED board
    #define i2cAddr      0x3C       // I2C OLED board
    #define OLED_RESET    16        // RESET here so no reset lower down    

    /*  
      SPI/CS               05   For optional TF/SD Card Adapter
      SPI/MOSI             23   For optional TF/SD Card Adapter
      SPI/MISO             19   For optional TF/SD Card Adapter
      SPI/SCK              18   For optional TF/SD Card Adapter  
    */

  #endif    
  
  //=========================================================================   
  #if (ESP32_Variant == 5)          // LILYGO® TTGO T-Display ESP32 1.14" ST7789 Colour LCD(135 x 240), IDE board = "ESP32 Dev Module"
                                    // Remember to select the T_Display board in User_Setup_Select.h in TFT_eSPI library
    #define FrsStatusLed  25        // Add your own LED with around 1K series resistor
    #define InvertFrsLed false     
    #define BufStatusLed  99        // none
    uint8_t fr_rxPin =    13;       // F/SPort rx
    #define fr_txPin      15        
    //#define fr_rxPin    27        // alternate
    //#define fr_txPin    17       
    #define startWiFiPin  99        // 99=none. No input pin available (non touch!) Could use touch with a bit of messy work.
     
    #if !defined displaySupport    // I2C TFT board is built into TTGO T-Display
      #define displaySupport
    #endif    
    #define ST7789_Display          // TFT display type - if you have a display you must define which type
    #define SCR_ORIENT   1          // 1 Landscape or 0 Portrait    
    
    /* Below please choose either Touch pin-pair or Digital pin-pair for display scrolling
     *  Pin == 99 means the pin-pair is not used
     */ 
    #define Pinfo          2        //    Digital pin to toggle information/log page     
    #define Pup            0        //  0 Board Button 1 to scroll the display up
    #define Pdn           35        // 35 Board Button 2 to scroll the display down 
    #define Tinfo         99        //    Touch pin to toggle information/log page       
    #define Tup           99        // 33 Touch pin to scroll the display up
    #define Tdn           99        // 32 Touch pin to scroll the display down   
    
    #define SCR_ORIENT     1        // 1 Landscape or 0 Portrait
 
    #define SDA           21        // I2C TFT board 
    #define SCL           22        // I2C TFT board
    #define i2cAddr      0x3C       // I2C TFT board
  #endif
 
   //=========================================================================   
  #if (ESP32_Variant == 6)          // LILYGO® TTGO T2 SD SSD1331 TFT Colour 26pin  IDE board = "ESP32 Dev Module"
    #define FrsStatusLed   5        // BoardLED
    #define InvertFrsLed true     
    #define BufStatusLed  99        // none
    #define fr_rxPin      17        // FPort rx - possible 22 / 23
    #define fr_txPin      18        // FPort tx - Use me in single wire mode
    #define startWiFiPin  99        // 99=none. No input pin available (non touch!) Could use touch with a bit of messy work.

    #if !defined sdBuiltin         // SD reader is built into TTGO T2
      #define sdBuiltin
    #endif
    #if !defined displaySupport      // I2C OLED board is built into TTGO T2
      #define displaySupport
    #endif
    #define SSD1331_Display         // colour TFT display type see here https://github.com/emard/ulx3s/issues/8
                                    // software graphic display clear very slow
    #define SCLK          14          // SPI pins for SSD1331 
    #define MOSI          13
    #define CS            15
    #define DC            16
    #define RST            4
    #define MISO          12          // apparently not used by Adafruit    
    /*    Below please choose either Touch pin-pair or Digital pin-pair for display scrolling
     *    Pin == 99 means the pin-pair is not used
     */ 
    #define Pinfo         25        // Digital pin to toggle information/log page        
    #define Pup           99        // Board Button 1 to scroll the display up
    #define Pdn           99        // Board Button 2 to scroll the display down    
    #define Tinfo         99        // Touch pin to toggle information/log page    
    #define Tup           32        // Touch pin to scroll the display up
    #define Tdn            2        // Touch pin to scroll the display down      
      
  #endif  

   //=========================================================================  
  #if (ESP32_Variant == 7)          // ESP32 Dev Module with ILI9341 2.8" colour TFT SPI 240x320
    #define FrsStatusLed  02        // Onboard LED
    #define InvertFrsLed false      
    #define BufStatusLed  27        // untested pin      
    #define fr_rxPin      16        // FPort- Not used in 1-wire mode DON'T use 12!
    #define fr_txPin      17        // FPort tx - Use me in single wire mode
    #define startWiFiPin   5        // 5 Trigger WiFi startup  
    
    #if !defined displaySupport      // I2C OLED board is built into TTGO T2
      #define displaySupport
    #endif
    #define ILI9341_Display         // ILI9341 2.8" COLOUR TFT SPI 240x320 V1.2  
    #define SCLK          18        // blue wire on my test setup
    #define MOSI          23        // yellow wire
    #define CS            25        // white wire
    #define DC            22        // green wire
    #define RST           21        // brown wire
    // LED=3.3V,  Vcc=5v,  Gnd 
    // MISO                not used by Adafruit     
    
    /* Below please choose either Touch pin-pair or Digital pin-pair for display scrolling
     *  Pin == 99 means the pin-pair is not used
     */ 
    #define Pinfo         99        //    Digital pin to toggle information/log page              
    #define Pup           99        // 35 Board Button 1 to scroll the display up
    #define Pdn           99        //  0 Board Button 2 to scroll the display down  
    #define Tinfo         15        // 15 Touch pin to toggle information/log page       
    #define Tup           33        // 33 Touch pin to scroll the display up
    #define Tdn           32        // 32 Touch pin to scroll the display down   

  #endif   
   
#elif defined ESP8266                    // ESP8266 Platform

  
  //========================================================================= 
  #if (ESP8266_Variant == 1)        // NodeMCU 12F board - Dev board with usb etc
  
    #define FrsStatusLed  D4        // D4/GPIO2 Board LED - Mav Status LED inverted logic - use 99 while debug
    #define InvertFrsLed true      
    #define BufStatusLed  99        // None     
   //                     D4        // TXD1 - Serial1 debug log out SHARED WITH BOARD LED                         
    #define fr_rxPin      D9        // FPort- Not used in single wire mode
    #define fr_txPin      D10       // FPorttx - Use me in single wire mode
    #define startWiFiPin  99        // 99=none or D3/D7 - Trigger WiFi startup 
                                    // NOTE: There are not enough pins for wifi pin and display scrolling

    #if (defined displaySupport)   // Display type defined with # define displaySupport   
      /* Below please choose Digital pin-pair for display scrolling
       *  Pin == 99 means the pin-pair is not used
       */ 
      #define Pinfo       99        // Digital pin to toggle information/log page           
      #define Pup         D3        // D3 Board Button 1 to scroll the display up
      #define Pdn         D7        // D7 Board Button 2 to scroll the display down    
      #define SCL         D1        // I2C OLED board   
      #define SDA         D2        // I2C OLED board
      #define i2cAddr    0x3C       // I2C OLED board
    #endif
        
  #endif
  
  //=========================================================================   
  #if (ESP8266_Variant == 2)   // ESP-12E, ESP-F barebones boards. RFD900X TX-MOD, QLRS et al - use Generic ESP8266 on IDE
    //                         GPIO as per node mcu
    static const uint8_t D0   = 16;   // SCL - optional
    static const uint8_t D1   = 5;    // SDA - optional
    static const uint8_t D2   = 4;    // FPorttx - Use me in single wire mode
    static const uint8_t D3   = 0;    // Flash
    static const uint8_t D4   = 2;    // BoardLED & TXD1 optional debug out
    static const uint8_t D5   = 14;   // FPortrx (unused in half-duplex)
    static const uint8_t D6   = 12;   // P2-3 exposed dual row of pins
    static const uint8_t D7   = 13;   // CTS
    static const uint8_t D8   = 15;   // RTS
    static const uint8_t D9   = 3;    // RXD0 mavlink and flashing
    static const uint8_t D10  = 1;    // TXD0 mavlink and flashing
    
    #define FrsStatusLed  D4        // D4 Board LED - Mav Status LED inverted logic - use 99 while debug
    #define InvertFrsLed true    
    #define BufStatusLed  99        // None
    //                    D4        // TXD1 - Serial1 default debug log out SHARED WITH BOARD LED                            
    #define fr_rxPin      D9        // FPort- Not used in single wire mode
    #define fr_txPin      D10       // FPort(half-duplex) inverted - Use me in single wire mode
    #define startWiFiPin  D6        // Trigger WiFi startup 
                                    // NOTE: There may not be enough pins for wifi pin AND display scrolling  
    #if (defined displaySupport)    // Display type defined with # define displaySupport 
      #define SSD1306_Display  
      /* Below please choose Digital pin-pair for display scrolling
       *  Pin == 99 means the pin-pair is not used
       */ 
      #define Pinfo         99        // Digital pin to toggle information/log page          
      #define Pup           99        // D3 Board Button 1 to scroll the display up
      #define Pdn           99        // D7 Board Button 2 to scroll the display down    
      #define SCL           D0        // I2C OLED board   
      #define SDA           D1        // I2C OLED board
      #define i2cAddr      0x3C       // I2C OLED board
    #endif 
  #endif // end of this ESP8266 variant 
  
  //========================================================================= 
  #if (ESP8266_Variant == 3)   // ESP-12F, Wemos® LOLIN D1 Mini - use Generic ESP8266 on Arduino IDE
    //  Pin Map as per C:\Users\<user>\AppData\Local\Arduino15\packages\esp8266\hardware\esp8266\2.7.1\variants\d1_mini\pins_arduino.h

    #define LED_BUILTIN 2

    static const uint8_t D0   = 16;  // WiFi trigger
    static const uint8_t D1   = 5;   // SCL - optional
    static const uint8_t D2   = 4;   // SDA - optional
    static const uint8_t D3   = 0;   // Flash - reserved
    static const uint8_t D4   = 2;   // LED_BUILTIN & TXD1 optional debug out
    static const uint8_t D5   = 14;  // FPortrx (unused in half-duplex)
    static const uint8_t D6   = 12;  // FPorttx - Use me in single wire mode
    static const uint8_t D7   = 13;  // CTS 
    static const uint8_t D8   = 15;  // RTS
    static const uint8_t D9   = 3;   // RXD0
    static const uint8_t D10  = 1;   // TCD0 
    
    #define FrsStatusLed  D4        // D4 Board LED - Mav Status LED inverted logic - use 99 while debug
    #define InvertFrsLed true    
    #define BufStatusLed  99        // None
    //                    D4        // TXD1 - Serial1 default debug log out SHARED WITH LED_BUILTIN BOARD LED                           
    #define fr_rxPin      D9        // FPort- Not used in single wire mode
    #define fr_txPin      D10        // FPort- inverted - Use me in single wire mode
    #define startWiFiPin  D16       // Trigger WiFi startup 
      
    #if (defined displaySupport)   // Display type defined with # define displaySupport   
      /* Below please choose Digital pin-pair for display scrolling
       *  Pin == 99 means the pin-pair is not used
       */ 
      #define Pinfo         99        // Digital pin to toggle information/log page           
      #define Pup           99        // D3 Board Button 1 to scroll the display up
      #define Pdn           99        // D7 Board Button 2 to scroll the display down    
      #define SCL           D1        // I2C OLED board   
      #define SDA           D2        // I2C OLED board
      #define i2cAddr      0x3C       // I2C OLED board
    #endif 
 
  #endif // end of this ESP8266 variant 
  
#endif  // end of all ESP8266 variants

  //=================================================================================================   
  //                            E E P R O M    S U P P O R T   -   ESP Only - for now
  //================================================================================================= 

  #if (defined ESP32) || (defined ESP8266)
    #include <EEPROM.h>       // To store AP_Failover_Flag and webSupport Settings
    #define EEPROM_SIZE 166   // 1 + 165 bytes = 166, addr range 0 thru 165 
  #endif
    

  //=================================================================================================   
  //                      D I S P L A Y   S U P P O R T    E S P  O N L Y - for now
  //=================================================================================================  

  #if defined displaySupport
    #if (!( (defined SSD1306_Display) || (defined SSD1331_Display) || (defined ST7789_Display) || (defined ILI9341_Display) ))
      #error please define a display type in your board variant configuration, or disable displaySupport
    #endif   

    #if not defined SD_Libs_Loaded    // by SD block
      #include <SPI.h>                // for SD card and/or Display
    #endif  

    typedef enum display_mode_set { logg = 1 , flight_info = 2 } display_mode_t;
    
    display_mode_t      display_mode;     
        
    bool infoPressBusy = false;
    bool infoNewPress = false;         
    bool infoPressed = false;
    bool show_log = true;    
    uint32_t info_debounce_millis = 0; 
    uint32_t  info_millis = 0; 
    uint32_t  last_log_millis = 0;
    const uint16_t db_period = 1000; // debounce period mS

/*
    // Generic colour definitions
    #define BLACK           0x0000
    #define BLUE            0x001F
    #define RED             0xF800
    #define GREEN           0x07E0
    #define CYAN            0x07FF
    #define MAGENTA         0xF81F
    #define YELLOW          0xFFE0
    #define WHITE           0xFFFF 
*/
    //==========================================================
    
    #if (defined ST7789_Display)      // TTGO T_Display 1.14 TFT display 135 x 240 SPI
      #include <TFT_eSPI.h>           // Remember to select the T_Display board in User_Setup_Select.h in TFT_eSPI library (135 x 240) 
      TFT_eSPI display = TFT_eSPI();
      #define SCR_W_PX      240       // OLED display width, in pixels
      #define SCR_H_PX      135       // OLED display height, in pixels

      #if (SCR_ORIENT == 0)           // portrait
        #define SCR_H_CH     20       // characters not pixels
        #define SCR_W_CH     11       // ?
        #define CHAR_W_PX    12       // pixels    
        #define CHAR_H_PX    12       // pixels 
        #define TEXT_SIZE     1                
      #elif (SCR_ORIENT == 1)         // landscape
        #define SCR_H_CH      8       // characters not pixels 
        #define SCR_W_CH     20  
        #define CHAR_W_PX    12       // 12 x 20 = 240  
        #define CHAR_H_PX    16       // 16 x 8 =  128
        #define TEXT_SIZE     2                       
      #endif 
      
    //==========================================================
    
    #elif (defined SSD1306_Display)    // SSD1306 OLED display     (128 x 64) 
      #include <Adafruit_GFX.h>
      #include <Adafruit_SSD1306.h> 
      #define SCR_W_PX 128     // OLED display width, in pixels
      #define SCR_H_PX 64      // OLED display height, in pixels
      #define SCR_ORIENT  1    // 0 portrait  1 landscape
      #define TEXT_SIZE   1
      #define SCR_H_CH    8    // characters not pixels 
      #define SCR_W_CH   21   
      #define CHAR_W_PX   6    // 6 x 21 = 126  
      #define CHAR_H_PX   8    // 8 x 8 = 64    
      #ifndef OLED_RESET
        #define OLED_RESET    -1 // Reset pin # (or -1 if sharing Arduino reset pin)
      #endif  
      Adafruit_SSD1306 display(SCR_W_PX, SCR_H_PX, &Wire, OLED_RESET); 
          
    //==========================================================  
    #elif (defined SSD1331_Display)    // SSD1331 0.95" TTGO T2 colour TFT display (96 x 64)
      #include <Adafruit_GFX.h>
      #include <Adafruit_SSD1331.h>
      #define SCR_W_PX    96     // OLED display width, in pixels
      #define SCR_H_PX    64     // OLED display height, in pixels
      #define SCR_ORIENT   1     // 0 portrait  1 landscape
      #define TEXT_SIZE    1
      #define SCR_H_CH     8     // characters not pixels 
      #define SCR_W_CH    16        
      #define CHAR_W_PX    6     // 6 x 16 = 96   
      #define CHAR_H_PX    8     // 8 x 8  = 64 
      Adafruit_SSD1331 display = Adafruit_SSD1331(CS, DC, MOSI, SCLK, RST);  
      
    //========================================================== 
    #elif  (defined ILI9341_Display)    // ILI9341 2.8" COLOUR TFT SPI 240x320 V1.2  
      #include "Adafruit_GFX.h"   
      #include "Adafruit_ILI9341.h"
      // Uses hardware SPI
      
      Adafruit_ILI9341 display = Adafruit_ILI9341(CS, DC, RST);    // LED=3.3V,  Vcc=5v,  Gnd 
         
      #define SCR_ORIENT   1            // 0 for portrait or 1 for landscape
      #define TEXT_SIZE    2            // default, may be changed on the fly
        
      #if (SCR_ORIENT == 0)      // portrait
        #define SCR_H_PX  320
        #define SCR_W_PX  240  
      #elif (SCR_ORIENT == 1)    // landscape
        #define SCR_H_PX  240
        #define SCR_W_PX  320
      #endif 

      #if (TEXT_SIZE == 1) 
        #define CHAR_W_PX    6     // 40 / 53 ch wide   portrait / landscape
      #elif  (TEXT_SIZE == 2)       
        #define CHAR_W_PX    12    // 20 / 26 ch wide  
      #elif  (TEXT_SIZE == 3)  
        #define CHAR_W_PX    18    // 13 / 17 ch wide
      #elif  (TEXT_SIZE == 4)  
        #define CHAR_W_PX    24    // 10 / 13 ch wide
      #elif  (TEXT_SIZE == 5)  
        #define CHAR_W_PX    30    // 10 / 8 ch wide
      #endif
      
     #define CHAR_H_PX   (CHAR_W_PX) + ( (CHAR_W_PX) / 3)    // so vertical spacing is h spacing plus 1/3
     #define SCR_W_CH   (SCR_W_PX) / (CHAR_W_PX)     
     #define SCR_H_CH  (SCR_H_PX) / (CHAR_H_PX)  
      
    #endif   
    //==========================================================   

    #if (!(defined SCR_ORIENT) )
      #error please define a desired screen orientation,  0 portrait or 1 landscape for your board variant or display type
    #endif 

    char clear_line[SCR_W_CH+1];
    
    // allocate space for screen buffer   
    #define max_col   SCR_W_CH+1  // +1 for terminating line 0x00        
    #define max_row   64
  
    static const uint16_t threshold = 40;
    volatile bool upButton = false;
    volatile bool dnButton = false;
    volatile bool infoButton = false;
    
    #if (not defined Tup) 
      #define Tup         99
    #endif

    #if (not defined Tdn) 
      #define Tdn         99
    #endif

    #if (not defined Pup) 
      #define Tup         99
    #endif

    #if (not defined Pdn) 
      #define Tdn         99
    #endif

    typedef enum scroll_set { non = 0, up = 1, down = 2 } scroll_t;
    scroll_t up_down = non; 

    typedef enum last_row_set { omit_last_row = 0, show_last_row = 1} last_row_t;
    last_row_t last_row_action;   

    struct row_t {
      char x[max_col];
      };
  
     row_t ScreenRow[max_row]; 
     
    uint8_t   row = 0;
    uint8_t   col = 0;
    uint8_t   scroll_row = 0;
    uint32_t  scroll_millis =0 ;

  #endif   // end of Display_Support

  //=================================================================================================   
  //                     B L U E T O O T H   S U P P O R T -  E S P 3 2  O n l y
  //================================================================================================= 

  #if ((defined ESP32) && (defined btBuiltin))

    #include "BluetoothSerial.h"
    #include "esp_bt.h"
    #if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
      #error Bluetooth is not enabled! Please run `idf.py menuconfig` in IDF workspace 
    #endif

    BluetoothSerial SerialBT;

  #endif  

  //=================================================================================================   
  //                       W I F I   S U P P O R T - ESP32 and ES8266 Only
  //=================================================================================================  
    
    uint16_t  udp_read_port;
    uint16_t  udp_send_port;
             
    bool      FtRemIP = true;
    bool      wifiDisconnected = false;
    int16_t   wifi_rssi;
    uint16_t  wifi_status = 0xfe;   // 0xfe = unused yet
    uint8_t   AP_sta_count = 0;
    uint8_t   AP_prev_sta_count = 0;
    
  #if ( (defined ESP32) || (defined ESP8266) )

  #if (defined ESP32)
    // timer for wifi retry interrupt
    hw_timer_t * timer = NULL;
    volatile SemaphoreHandle_t wifiTimerSemaphore;
    portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
  #endif

  #if (defined ESP8266)
    uint32_t esp8266_wifi_retry_millis = 0;
  #endif
  
  volatile bool  wifiButton = false;
    
    // Define link variables
    struct linkStatus {
      uint32_t    packets_received;
      uint32_t    packets_lost;
      uint32_t    packets_sent;
    };
    bool          hb_heard_from = false;
    uint8_t       hb_system_id = 0;
    uint8_t       hb_comp_id = 0;
    uint8_t       hb_seq_expected = 0;
    uint32_t      hb_last_heartbeat = 0;
    linkStatus    link_status;
  
    #if defined ESP32 
      #include <WiFi.h>  // includes UDP class
      #if defined webSupport
        #include <WebServer.h> 
        #include <Update.h> 
        WebServer server(80);
        
      #endif      
      #include <WiFiAP.h>  
    #endif

    #if defined ESP8266
      #include <ESP8266WiFi.h>   // Includes AP class
      #if defined webSupport
        #include <ESP8266WebServer.h>    
        ESP8266WebServer server(80);  
        #include <WiFiUdp.h>       
      #endif      
    #endif
    
   //====================       W i F i   O b j e c t s 
   
    #define max_clients    6
    uint8_t active_client_idx = 0;  // for TCP 
    uint8_t active_object_idx = 0;  // for UDP

    WiFiClient *tcp_client[max_clients] = {NULL}; // pointers to TCP client objects 
    
    WiFiServer TCPserver(TCP_localPort);          // dummy TCP local port(changes on TCPserver.begin() ).

    IPAddress UDP_remoteIP(192, 168, 1, 255);     // default to broadcast unless (not defined UDP_Broadcast)               
    IPAddress udpremoteip[max_clients];           // table of remote UDP client IPs
                   
    WiFiUDP *udp_object[2] = {NULL};              // pointers to UDP objects for STA and AP modes
    WiFiUDP frs_udp_object;  
                 
    IPAddress localIP;                            // tcp and udp
    IPAddress TCP_remoteIP(192,168,4,1);          // when we connect to a server in tcp client mode, put the server IP here 
    
  #endif  // end of ESP32 and ESP8266
  
//=================================================================================================   
//                                 S E R I A L
//=================================================================================================

#if (defined ESP32)  
  #define Log                 Serial         // USB
  #define mvSerial            Serial2        // RXD2 and TXD2

  #if defined ESP32_SoftwareSerial
    #include <SoftwareSerial.h>
    SoftwareSerial frSerial; 
  #else     // default HW Serial
    #define frSerial          Serial1     
  #endif   
    
#elif (defined ESP8266)
  #define Log                 Serial1        //  D4   TXD1 debug out  - no RXD1 !
  #define mvSerial            Serial         //  RXD0 and TXD0
  #include <SoftwareSerial.h>
  SoftwareSerial frSerial;  
#endif 

#if (defined TEENSY3X)      //  Teensy 3.1
  #define Log                 Serial         // USB  
  #define mvSerial            Serial2   
  #if (frPort_Serial == 1) 
    #define frSerial          Serial1        // F.Port 
  #elif (frPort_Serial == 3)
    #define frSerial          Serial3        // F.Port 
 #else
    #error frPort_Serial can only be 1 or 3. Please correct.
  #endif 
#endif 


//=================================================================================================   
//                                 D E B U G G I N G   O P T I O N S
//=================================================================================================

//#define Frs_Debug_All
//#define Debug_BT    
//#define Debug_Read_UDP_GCS  
//#define Debug_Send_UDP_GCS
//#define Debug_Read_UDP_FC  
//#define Debug_Send_UDP_FC  
//#define Frs_Debug_Servo
//#define Frs_Debug_Rssi        // 0xF101         
//#define Frs_Debug_RC
//#define Frs_Debug_Params       //0x5007
//#define Frs_Debug_APStatus    // 0x5001
//#define Debug_Batteries       // 0x5003
//#define Frs_Debug_Home        // 0x5004
//#define Frs_Debug_LatLon      // 0x800
//#define Frs_Debug_VelYaw      // 0x5005
//#define Frs_Debug_GPS_status  // 0x5002
//#define Frs_Debug_Hud         // 0x50F2
//#define Frs_Debug_AttiRange   // 0x5006
//#define Frs_Debug_StatusText  // 0x5000
//#define Frs_Debug_Mission   
//#define Debug_Eeprom 
//#define Debug_WiFi
//#define Debug_Loop_Period
//#define Debug_SRAM
//#define Debug_Web_Settings
//#define Frs_Debug_Payload

#define Report_Packetloss   2     // F.Port packet loss every n minutes

//#define CRC_Test_Case
//#define Debug_CRC
//#define Debug_FrPort_Serial_Loop
//#define Debug_FrPort_Switching
//#define Frs_Debug_Period
 //#define Support_SBUS_Out 

//#define Debug_Send_UDP

//#define Debug_Baud
//#define Debug_FrPort_Stream
//#define Debug_FPort_Buffer 
//#define Debug_FrSky_Messages
//#define Debug_PWM_Channels
//=================================================================================================   
//                                   C H A N G E   L O G
//=================================================================================================
/*

Change log:
                                                                                                                                                                                                                                                                     
*/
