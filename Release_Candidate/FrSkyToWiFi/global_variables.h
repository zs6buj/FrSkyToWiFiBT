//=================================================================================================  
//================================================================================================= 
//
//                                 G L O B A L   V A R I A B L E S
//
//================================================================================================= 
//=================================================================================================

uint8_t   clm = 0;                    // Columns for Printbyte();
String    pgm_path;
String    pgm_name;

const uint8_t snp_max = 32;
char          snprintf_buf[snp_max];       // for use with snprintf() formatting of display line

volatile uint32_t debnceTimr;              // for pin interrupts
volatile uint32_t delaytm = 100;


bool      frsLedState = LOW;  
bool      frsGood = false;
bool      spGood = false;    // Good F.Port serial read
bool      spPrev = false;

bool      wifiSuGood = false;
bool      wifiSuDone = false;
bool      inbound_clientGood = false;
bool      outbound_clientGood = false;
bool      clientPrev = true;
bool      btActive = false;
bool      btDisabled = false;

uint32_t  frs_led_millis = 0;
uint32_t  now_millis = 0;
uint32_t  now_micros = 0;
uint32_t  prev_millis = 0;
uint32_t  prev_micros = 0;
uint32_t  prev_lp_millis = 0;
uint32_t  prev_lp_micros = 0;
uint32_t  prev_fr_millis = 0;
uint32_t  prev_fr_micros = 0;
uint32_t  free_heap_millis = 0;

  bool lonGood = false;
  bool latGood = false;
  bool altGood = false;
  bool hdgGood = false;
  bool hdopGood = false;
  
    // 3D Location vectors
    struct Location {
     float lat; //long
     float lon;
     float alt;
     float hdg;
    };

    struct Location hom     = {
      0,0,0,0};   // home location

    struct Location cur      = {
      0,0,0,0};   // current location
//=================================================================================================   
//                S E T T I N G S   A N D   O P T I O N S   S T R U C T U R E
//=================================================================================================

    typedef enum polarity_set { idle_low = 0, idle_high = 1, no_traffic = 2 } pol_t;  
    typedef enum frport_type_set { f_none = 0, f_port1 = 1, f_port2 = 2, s_port = 3, f_auto = 4} frport_t; 
    typedef enum fr_io_set { fr_bt = 1 , fr_wifi = 2, fr_wifi_bt = 3, fr_none = 0} fr_io_t;     
    typedef enum wfmode_set { ap = 1 , sta = 2, sta_ap = 3 } wfmode_t; // sta_ap means sta failover to ap
    typedef enum wf_proto_set { tcp = 1 , udp = 2, } wf_proto_t; 
    typedef enum btmode_set { master = 1, slave = 2 } btmode_t;
           
    typedef struct  {
      byte          validity_check;  // 0xdc      
      frport_t      frport;            // position 164 in EEPROM    
      fr_io_t       fr_io;  
      wfmode_t      wfmode;
      wf_proto_t    wfproto;
      uint8_t       channel;
      char          apSSID[30];
      char          apPw[20];
      char          staSSID[30];
      char          staPw[20];
      char          host[20];     
      uint16_t      tcp_localPort;
      uint16_t      tcp_remotePort;     // 162 thru 163 in EEPROM   
      uint16_t      udp_localPort;
      uint16_t      udp_remotePort;
      btmode_t      btmode;
      char          btConnectToSlave[20];  // 140 - 159 in EEPROM - 160 byte unused
      
      // NOT saved in EEPROM

      bool          web_support;                                 
 
      char*         fr_io1;       // bt
      char*         fr_io2;       // wifi
      char*         fr_io3;       // wifi_bt    
      char*         wfmode1;      // ap 
      char*         wfmode2;      // sta
      char*         wfmode3;      // sta_ap         
      char*         wfproto1; // tcp
      char*         wfproto2; // udp       
      char*         btmode1;      // master
      char*         btmode2;      // slave 
      char*         rssioverride; //rssi override        
      char*         frport1;      // F.Port1    
      char*         frport2;      // F.Port2  
      char*         frport3;      // S.Port 
      char*         frport4;      // Auto                 
      } settings_struct_t;
      
    settings_struct_t set;  

  //=================================================================================================
