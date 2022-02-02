//================================================================================================= 
//================================================================================================= 
//
//                                      W E B   S U P P O R T  
// 
//================================================================================================= 
//================================================================================================= 
#if defined webSupport

/*
In the ESP32, constant data is automatically stored in FLASH memory and can be accessed directly 
from FLASH memory without first copying it to RAM. So, there is no need to use the PROGMEM keyword
*/

 static const String styleLogin =  // Stored in FLASH not SRAM Heap - see above
    "<style>h1{background:#3498db;color:#fff;border-radius:5px;height:34px;font-family:sans-serif;}"
    "#file-input,input{width:100%;height:44px;border-radius:4px;margin:10px auto;font-size:15px}"
    "input{background:#f1f1f1;border:0;padding:0 15px}body{background:#3498db;font-family:sans-serif;font-size:14px;color:#777}"
    "form{background:#fff;max-width:258px;margin:75px auto;padding:30px;border-radius:5px;text-align:center}"
    ".btn{background:#3498db;color:#fff;cursor:pointer} .big{ width: 1em; height: 1em;}"
    "::placeholder {color: white; opacity: 1; /* Firefox */}"
    "</style>";
   
 static const String styleSettings =
    "<style>"
    "h{color:#fff;font-family:sans-serif;}"
    "h3{background:#3498db;color:#fff;border-radius:5px;height:22px;font-family:sans-serif;}"
    "input{background:#f1f1f1;border:1;margin:8px auto;font-size:14px}"
    "body{background:#3498db;font-family:arial;font-size:10px;color:black}"
    "#bar,#prgbar{background-color:#f1f1f1;border-radius:10px}#bar{background-color:#3498db;width:0%;height:10px}"
    "form{background:#fff;max-width:440px;margin:30px auto;padding:30px;border-radius:10px;text-align:left;font-size:16px}"
    ".big{ width: 1em; height: 1em;} .bold {font-weight: bold;}"
    "</style>";

 static const String styleOTA =
    "<style>#file-input,input{width:100%;height:44px;border-radius:4px;margin:10px auto;font-size:15px}"
    "input{background:#f1f1f1;border:0;padding:0}"
    "body{background:#3498db;font-family:sans-serif;font-size:14px;color:#777}"
    "#file-input{padding:0;border:1px solid #ddd;line-height:44px;text-align:left;display:block;cursor:pointer}"
    "#bar,#prgbar{background-color:#f1f1f1;border-radius:10px}#bar{background-color:#3498db;width:0%;height:10px}"
    "form{background:#fff;margin:75px auto;padding:30px;text-align:center;max-width:450px;border-radius:10px;}"       
    ".btn{background:#3498db;color:#fff;cursor:pointer; width: 80px;} .big{ width: 1em; height: 1em;}</style>"  
    "<script>function backtoLogin() {window.close(); window.open('/');} </script>";

   
 static const String otaIndex = styleOTA +  
    "<script src='https://ajax.googleapis.com/ajax/libs/jquery/3.4.1/jquery.min.js'></script>"
    "<form method='POST' action='#' enctype='multipart/form-data' id='upload_form'>"
    "<input type='file' name='update' id='file' onchange='sub(this)' style=display:none>"
    "<label id='file-input' for='file' class=btn > Choose file...</label><br><br>"
    "<center><input type='submit' onclick='backtoLogin()' class=btn value='Cancel'> &nbsp &nbsp &nbsp &nbsp "  
    "<input type='submit' class=btn value='Update'></center>"
    "<br><br>"
    "<div id='prg' align='left'></div>"
    "<br><left><div id='prgbar'><div id='bar'></div></div><br><br>"
    "<p id='rebootmsg'></p><br><br>"
    "<center><input type='submit' onclick='window.close()' class=btn value='Close'></center></form>"
    "<script>"
    "function sub(obj){"
    "var fileName = obj.value.split('\\\\');"
    "document.getElementById('file-input').innerHTML = '   '+ fileName[fileName.length-1];"
    "};"
    "$('form').submit(function(e){"
    "e.preventDefault();"
    "var form = $('#upload_form')[0];"
    "var data = new FormData(form);"
    "$.ajax({"
    "url: '/update',"
    "type: 'POST',"
    "data: data,"
    "contentType: false,"
    "processData:false,"
    "xhr: function() {"
    "var xhr = new window.XMLHttpRequest();"
    "xhr.upload.addEventListener('progress', function(evt) {"
    "if (evt.lengthComputable) {"
    "var per = evt.loaded / evt.total;"
    "$('#prg').html('progress: ' + Math.round(per*100) + '%');"
    "$('#bar').css('width',Math.round(per*100) + '%');"
    "if (per == 1.0) {document.getElementById('rebootmsg').innerHTML = 'Rebooting .....'}"
    "}"
    "}, false);"
    "return xhr;"
    "},"
    "success:function(d, s) {console.log('success!')},"
    "error: function (a, b, c) {}"
    "});"
    "});"
    "</script>";

  String settingsPage;
  String loginPage;
  char   temp[128]; 
    
  //===========================================================================================
  void WebServerSetup() {
      
            //===========================================================
            //                 HANDLE INCOMING HTML 
            //===========================================================


  server.on("/", handleLoginPage);                          // root
  server.on("/settingsIndex", handleSettingsPage);             
  server.on("/settingsReturnIndex", handleSettingsReturn);  // save settings and reboot
  server.on("/rebootIndex", handleReboot);                  // reboot only
  server.on("/otaIndex", handleOtaPage); 
  
  server.begin();
   
  #if defined Debug_SRAM
    Log.printf("==============>Free Heap after WebServer setup = %d\n", ESP.getFreeHeap());
  #endif
  
  }
  
//=================================================================================================   
//                            Recover WiFi Settings From Flash
//=================================================================================================

void RecoverSettingsFromFlash() {
  
  set.validity_check = EEPROMRead8(1);         // apFailover is in 0

  #if defined Reset_Web_Defaults
    set.validity_check = 0;                    // reset
  #endif

  if (set.validity_check != 0xdc) {            // eeprom does not contain previously stored setings
                                               // so write default settings to eeprom
    Log.println("NOTE! ALL SETTINGS IN EEPROM SET TO COMPILE_TIME DEFAULTS!");                                           
    WriteSettingsToEEPROM();
    }  
      
  ReadSettingsFromEEPROM();                           

}

//=================================================================================
int32_t String_long(String S) {
  const char* c;
  c = S.c_str();
  return strtol(c, NULL, 0);
}   

//=================================================================================
  void ComposeLoginPage() {

    loginPage  =  styleLogin;
    loginPage += "<form name=loginForm>";
    sprintf(temp,  "<h1>%s Login</h1>", set.host);
    loginPage += temp;
    loginPage += "<br><input type='radio' class='big' name='_nextFn' value='set' checked> Settings &nbsp &nbsp &nbsp";          
    loginPage += "<input type='radio' class='big' name='_nextFn' value='ota' > Update Firmware<br> <br>";
    loginPage += "<input name=userid class=btn placeholder='User ID' size='10' color:#fff;> ";
    loginPage += "<input name=pwd class=btn placeholder=Password type=Password> <br> <br>";
    loginPage += "<input type=submit onclick=check(this.form) class=btn value=Login></form>";
    loginPage += "<script>";
    loginPage += "function check(form) {";
    sprintf(temp, "if(form.userid.value=='admin' && form.pwd.value=='%s')", webPassword);
    loginPage += temp;
    loginPage += "{{if(form._nextFn.value=='ota'){window.close(); window.open('/otaIndex')}}";
    loginPage += "{if(form._nextFn.value=='set'){window.close(); window.open('/settingsIndex')}}}";
    loginPage += "else";
    loginPage += "{alert('Error Password or Username')}";
    loginPage += "}";
    loginPage += "</script>";
  }
  //=================================================================================
  void ComposeSettingsPage() {
   
  settingsPage  = styleSettings;

  settingsPage += "<!DOCTYPE html><html><body><h>Mavlink To Passthru</h><form action='' ";   
  settingsPage += "autocomplete='on'> <center> <b><h3>MavToPassthru Translator Setup</h3> </b></center> <style>text-align:left</style>";
  
    settingsPage += "<br>FrSky Port Type: &nbsp ";    
    sprintf(temp, "<input type='radio' class='big' name='_frport' value='FPort1' %s> FPort1 &nbsp ", set.frport1);
    settingsPage += temp;
    sprintf(temp, "<input type='radio' class='big' name='_frport' value='FPort2' %s> FPort2 &nbsp ", set.frport2);
    settingsPage += temp;  
    sprintf(temp, "<input type='radio' class='big' name='_frport' value='SPort' %s> SPort &nbsp ", set.frport3);
    settingsPage += temp;    
    sprintf(temp, "<input type='radio' class='big' name='_frport' value='Auto' %s> Auto <br>", set.frport4);
    settingsPage += temp;    

  settingsPage += "Frs Downlink &nbsp &nbsp";  
  #if (defined btBuiltin)  
    sprintf(temp, "<input type='radio' class='big' name='_set.fr_io' value='BT' %s> BT &nbsp &nbsp", set.fr_io1);
    settingsPage += temp; 
  #endif      
  sprintf(temp, "<input type='radio' class='big' name='_set.fr_io' value='WiFi' %s> WiFi &nbsp &nbsp", set.fr_io2);
  settingsPage += temp; 
  #if (defined btBuiltin)  
    sprintf(temp, "<input type='radio' class='big' name='_set.fr_io' value='WiFi+BT' %s> WiFi+BT &nbsp &nbsp", set.fr_io3);
    settingsPage += temp; 
  #endif  
  settingsPage += "<br>WiFi Mode: &nbsp &nbsp ";
  sprintf(temp, "<input type='radio' class='big' name='_wfmode' value='AP' %s> AP &nbsp &nbsp", set.wfmode1);
  settingsPage += temp;
  sprintf(temp, "<input type='radio' class='big' name='_wfmode' value='STA' %s> STA &nbsp &nbsp", set.wfmode2);
  settingsPage += temp;    
  sprintf(temp, "<input type='radio' class='big' name='_wfmode' value='STA>AP' %s> STA>AP &nbsp <br>", set.wfmode3);
  settingsPage += temp;
  //sprintf(temp, "<input type='radio' class='big' name='_wfmode' value='AP_STA' %s> AP_STA &nbsp <br>", set.wfmode4);
  //settingsPage += temp;
  settingsPage += "WiFi Protocol: &nbsp &nbsp ";
  sprintf(temp, "<input type='radio' class='big' name='_wfproto' value='TCP' %s> TCP &nbsp &nbsp", set.wfproto1);
  settingsPage += temp;
  sprintf(temp, "<input type='radio' class='big' name='_wfproto' value='UDP' %s> UDP &nbsp <br>", set.wfproto2);
  settingsPage += temp; 
  sprintf(temp, "WiFi Channel: <input type='text' name='_channel' value='%d' size='1' maxlength='2'> <br>", set.channel);
  settingsPage += temp;
  sprintf(temp, "AP SSID: <input type='text' name='_apSSID' value='%s' size='30' maxlength='30'> <br>", set.apSSID);
  settingsPage+= temp;
  sprintf(temp, "AP Password: <input type='password' name='_apPw' value='%s' size='20' minlength='8' required> <br>", set.apPw);
  settingsPage += temp;
  sprintf(temp, "STA SSID: <input type='text' name='_staSSID' value='%s' size='30'> <br>", set.staSSID);
  settingsPage += temp;
  sprintf(temp, "STA Password: <input type='password' name='_staPw' value='%s' size='20' minlength='8' required> <br>", set.staPw);
  settingsPage += temp;
  sprintf(temp, "Host Name: <input type='text' name='_host' value='%s' size='20'> <br>", set.host);
  settingsPage += temp;
  sprintf(temp, "TCP Ports: local  <input type='text' name='_tcp_localPort' value='%d' size='2' minlength='2' required maxlength='5'>", set.tcp_localPort);
  settingsPage += temp;
  sprintf(temp, " &nbsp &nbsp remote <input type='text' name='_tcp_remotePort' value='%d' size='2' minlength='2' required maxlength='5'> <br>", set.tcp_remotePort);
  settingsPage += temp;
  sprintf(temp, "UDP Ports: local  <input type='text' name='_udp_localPort' value='%d' size='2' minlength='2' required maxlength='5'> ", set.udp_localPort);
  settingsPage += temp;
  sprintf(temp, " &nbsp &nbsp remote <input type='text' name='_udp_remotePort' value='%d' size='2' minlength='2' required maxlength='5'> <br>", set.udp_remotePort);
  settingsPage += temp;
  settingsPage += "Bluetooth Mode: &nbsp &nbsp ";
  sprintf(temp, "<input type='radio' class='big' name='_btmode' value='Master' %s> Master &nbsp &nbsp &nbsp &nbsp ", set.btmode1);
  settingsPage += temp;
  sprintf(temp, "<input type='radio' class='big' name='_btmode' value='Slave' %s> Slave &nbsp &nbsp <br>", set.btmode2);
  settingsPage += temp;
  sprintf(temp, "Slave Name: <input type='text' name='_btConnectToSlave' value='%s' size='22' maxlength='22'>  <br><br><center>", set.btConnectToSlave);
  settingsPage += temp;  
  settingsPage += "<b><input type='submit' onclick='closeWin()' formaction='/rebootIndex' class=btn value='Cancel'> </b>&nbsp &nbsp &nbsp &nbsp";
  settingsPage += "&nbsp &nbsp &nbsp &nbsp<b><input type='submit' formaction='/settingsReturnIndex' class=btn value='Save & Reboot'> </b><br><br>";
  settingsPage += "<p><font size='1' color='black'><strong>";
  settingsPage += pgm_name + ".  Compiled for "; 
  #if defined ESP32 
    settingsPage += "ESP32";
  #elif defined ESP8266
    settingsPage += "ESP8266";
  #endif   
  settingsPage += "</strong></p></center> </form> </body>";

  settingsPage += "<script>";
  settingsPage += "var myWindow";
  settingsPage += "function closeWin() {";
  settingsPage += "myWindow.close() }";
  settingsPage += "</script>";
}
//=================================================================================

void ReadSettingsFromEEPROM() {
byte b;
    
    b = EEPROMRead8(164);                         // fr-port type 164
    if (b == 0) {
      set.frport = f_none;
    } else if (b == 1) {
      set.frport = f_port1;
    } else if (b == 2) {
      set.frport = f_port2;      
    }  else if (b == 3) {
      set.frport = s_port;
    }  else if (b == 4) {
      set.frport = f_auto;
    }  
        
    b = EEPROMRead8(165);                         // set.fr_io 165

    if (b == 1) {
      set.fr_io = fr_bt;
    }  else 
    if (b == 2) {
      set.fr_io = fr_wifi;
    }  else 
     if (b == 3) {
      set.fr_io = fr_wifi_bt;
    } 

    b = EEPROMRead8(6);     // wifi mode
    if (b == 1) {
      set.wfmode = ap;
    } else if (b == 2) {
      set.wfmode = sta;
    } else if (b == 3) {
      set.wfmode = sta_ap;
    } 
    b = EEPROMRead8(7);     // wifi protocol
    if (b == 1) {
      set.wfproto = tcp;
    } else if (b == 2) {
      set.wfproto = udp;
    }  
    set.channel = EEPROMRead8(12);               //  12
    EEPROMReadString(13, set.apSSID);            //  13 thru 42
    EEPROMReadString(43, set.apPw);               //  4 thru 62

    if(strlen(set.apPw) < 8) {
        // esp fail passphrase too short
      Log.println("AP Password < 8 chars long! Will fail to authenticate");
      LogScreenPrintln("AP PW < 8 chars long");   
    }
    
    EEPROMReadString(63, set.staSSID);           // 63 thru 92 
    EEPROMReadString(93, set.staPw);             // 93 thru 112  

    if(strlen(set.staPw) < 8) {
        // esp fail passphrase too short
      Log.println("STA Password < 8 chars long! Will fail to authenticate");
      LogScreenPrintln("STA PW < 8 chars long");   
    }
    
    EEPROMReadString(113, set.host);             // 113 thru 132   
    set.tcp_localPort = EEPROMRead16(133);       // 133 thru 134 
    set.tcp_remotePort = EEPROMRead16(162);      // 162 thru 163     
    set.udp_localPort = EEPROMRead16(135);       // 135 thru 136 
    set.udp_remotePort = EEPROMRead16(137);      // 137 thru 138 
    b = EEPROMRead8(139);                        // 139
    if (b == 1) {
      set.btmode = master;
    } else if (b == 2) {
      set.btmode = slave;
    } 
    EEPROMReadString(140, set.btConnectToSlave);  // 140 thru 159 - 160 unused for now, 161 sport sd above 

    RefreshHTMLButtons();     
   
    #if defined Debug_Web_Settings
      Log.println();
      Log.println("Debug Read WiFi Settings from EEPROM: ");
      Log.print("validity_check = "); Log.println(set.validity_check, HEX);
      Log.print("fr_port_type = "); Log.println(set.frport);      
      Log.print("set.fr_io = "); Log.println(set.fr_io);                      
      Log.print("wifi mode = "); Log.println(set.wfmode);
      Log.print("wifi protocol = "); Log.println(set.wfproto);     
      Log.print("wifi channel = "); Log.println(set.channel);  
      Log.print("apSSID = "); Log.println(set.apSSID);
      Log.print("apPw = "); Log.println(set.apPw);
      Log.print("staSSID = "); Log.println(set.staSSID);
      Log.print("staPw = "); Log.println(set.staPw); 
      Log.print("Host = "); Log.println(set.host);           
      Log.print("tcp_localPort = "); Log.println(set.tcp_localPort);
      Log.print("tcp_remotePort = "); Log.println(set.tcp_remotePort);      
      Log.print("udp_localPort = "); Log.println(set.udp_localPort);
      Log.print("udp_remotePort = "); Log.println(set.udp_remotePort); 
      Log.print("bt mode = "); Log.println(set.btmode); 
      Log.print("SlaveConnect To Name = "); Log.println(set.btConnectToSlave);
      Log.println();          
    #endif  

    Log.println("EEPROM settings read and adopted");  
                
  }
//=================================================================================
void WriteSettingsToEEPROM() {
      set.validity_check = 0xdc;                                     
      EEPROMWrite8(1, set.validity_check);           // apFailover is in 0 
      EEPROMWrite8(164, set.frport);                 //  164  came late to the party        
      EEPROMWrite8(165, set.fr_io);                  //  165  came late to the party       
      EEPROMWrite8(6, set.wfmode);                   //  6
      EEPROMWrite8(7, set.wfproto);                  //  7     
      EEPROMWrite8(12, set.channel);                 // 12
      EEPROMWriteString(13, set.apSSID);             // 13 thru 42 
      EEPROMWriteString(43, set.apPw);               // 43 thru 62      
      EEPROMWriteString(63, set.staSSID);            // 63 thru 92 
      EEPROMWriteString(93, set.staPw);              // 93 thru 112 
      EEPROMWriteString(113, set.host);              // 113 thru 132 
      EEPROMWrite16(133, set.tcp_localPort);         // 133 thru 134       
      EEPROMWrite16(162, set.tcp_remotePort);        // 162 thru 163       
      EEPROMWrite16(135, set.udp_localPort);         // 135 thru 136
      EEPROMWrite16(137, set.udp_remotePort);        // 137 thru 138
      EEPROMWrite8(139, set.btmode);                 // 139     
      EEPROMWriteString(140, set.btConnectToSlave);  // 140 thru 159 - 160 unused for now      
      EEPROM.commit();  
      RefreshHTMLButtons();
                                                                  
      #if defined Debug_Web_Settings
        Log.println();
        Log.println("Debug Write WiFi Settings to EEPROM: ");
        Log.print("validity_check = "); Log.println(set.validity_check, HEX);
        Log.print("fr_port_type = "); Log.println(set.frport);        
        Log.print("set.fr_io = "); Log.println(set.fr_io);                       
        Log.print("wifi mode = "); Log.println(set.wfmode);
        Log.print("wifi protocol = "); Log.println(set.wfproto);     
        Log.print("wifi channel = "); Log.println(set.channel);  
        Log.print("apSSID = "); Log.println(set.apSSID);
        Log.print("apPw = "); Log.println(set.apPw);
        Log.print("staSSID = "); Log.println(set.staSSID);
        Log.print("staPw = "); Log.println(set.staPw); 
        Log.print("Host = "); Log.println(set.host);           
        Log.print("tcp_localPort = "); Log.println(set.tcp_localPort);
        Log.print("tcp_remotePort = "); Log.println(set.tcp_remotePort);        
        Log.print("udp_localPort = "); Log.println(set.udp_localPort);
        Log.print("udp_remotePort = "); Log.println(set.udp_remotePort); 
        Log.print("bt mode = "); Log.println(set.btmode); 
        Log.print("Master to Slave Name = "); Log.println(set.btConnectToSlave);       
        Log.println();             
      #endif 
}  
//=================================================================================
void ReadSettingsFromForm() {
  String S;
  
    S = server.arg("_frport");
    if (S == "None") {
      set.frport = f_none;
   } else 
   if (S == "FPort1") {
      set.frport = f_port1;
   }  else 
   if (S == "FPort2") {
      set.frport = f_port2;
   }  else 
   if (S == "SPort") {
     set.frport = s_port; 
   } else 
   if (S == "Auto") {
     set.frport = f_auto; 
   }
   
  S = server.arg("_set.fr_io");      
  if (S == "BT") {
    set.fr_io = fr_bt;
  } 
  S = server.arg("_set.fr_io");      
  if (S == "WiFi") {
    set.fr_io = fr_wifi;
  } 
  S = server.arg("_set.fr_io");      
  if (S == "WiFi+BT") {
    set.fr_io = fr_wifi_bt;
  }   

  
  S = server.arg("_wfmode");
  if (S == "AP") {
    set.wfmode = ap;
  } else 
  if (S == "STA") {
    set.wfmode = sta;
  } else 
  if (S == "STA>AP") {
    set.wfmode = sta_ap;
  } 
   
  S = server.arg("_wfproto");
  if (S == "TCP") {
    set.wfproto = tcp;
  } else 
  if (S == "UDP") {
    set.wfproto = udp;
  }   
  set.channel = String_long(server.arg("_channel")); 
  S = server.arg("_apSSID");
  strcpy(set.apSSID, S.c_str());
  S = server.arg("_apPw");
  strcpy(set.apPw, S.c_str());

  if(strlen(set.apPw) < 8) {
        // esp fail passphrase too short
      Log.println("AP Password < 8 chars long! Will fail to authenticate");
      LogScreenPrintln("AP PW < 8 chars long");   
    }

  S = server.arg("_staSSID");
  strcpy(set.staSSID, S.c_str());
  S = server.arg("_staPw");
  strcpy(set.staPw, S.c_str());  

  if(strlen(set.staPw) < 8) {
        // esp fail passphrase too short
      Log.println("STA Password < 8 chars long! Will fail to authenticate");
      LogScreenPrintln("STA PW < 8 chars long");   
    }

  S = server.arg("_host");
  strcpy(set.host, S.c_str());    
  set.tcp_localPort = String_long(server.arg("_tcp_localPort"));
  set.tcp_remotePort = String_long(server.arg("_tcp_remotePort"));  
  set.udp_localPort = String_long(server.arg("_udp_localPort"));
  set.udp_remotePort = String_long(server.arg("_udp_remotePort")); 
  S = server.arg("_btmode");
  if (S == "Master") {
    set.btmode = master;
  } else 
  if (S == "Slave") {
    set.btmode = slave;
  } 
  
  S = server.arg("_btConnectToSlave");
  strcpy(set.btConnectToSlave, S.c_str());    // strcpy() copies the C string including the terminating null character
  
      #if defined Debug_Web_Settings
        Log.println();
        Log.println("Debug Read WiFi Settings from Form: ");
        Log.print("validity_check = "); Log.println(set.validity_check, HEX); 
        Log.print("fr_port_type = "); Log.println(set.frport);       
        Log.print("set.fr_io = "); Log.println(set.fr_io);                                
        Log.print("wifi mode = "); Log.println(set.wfmode);
        Log.print("wifi protocol = "); Log.println(set.wfproto);     
        Log.print("wifi channel = "); Log.println(set.channel);  
        Log.print("apSSID = "); Log.println(set.apSSID);
        Log.print("apPw = "); Log.println(set.apPw);
        Log.print("staSSID = "); Log.println(set.staSSID);
        Log.print("staPw = "); Log.println(set.staPw); 
        Log.print("Host = "); Log.println(set.host);           
        Log.print("tcp_localPort = "); Log.println(set.tcp_localPort);
        Log.print("tcp_remotePort = "); Log.println(set.tcp_remotePort);        
        Log.print("udp_localPort = "); Log.println(set.udp_localPort);
        Log.print("udp_remotePort = "); Log.println(set.udp_remotePort); 
        Log.print("bt mode = "); Log.println(set.btmode); 
        Log.print("Master to Slave Name = "); Log.println(set.btConnectToSlave);      
        Log.println();      
      #endif 
 }
//=================================================================================
void RefreshHTMLButtons() {

  if (set.frport == f_port1) {
    set.frport1 = "checked";
    set.frport2 = "";
    set.frport3 = ""; 
    set.frport4 = "";          
  }  else 
  if (set.frport == f_port2) {
    set.frport1 = "";
    set.frport2 = "checked";
    set.frport3 = "";   
    set.frport4 = "";         
  }  else
   if (set.frport == s_port) {
    set.frport1 = "";
    set.frport2 = "";
    set.frport3 = "checked"; 
    set.frport4 = "";          
  } else
   if (set.frport == f_auto) {
    set.frport1 = "";
    set.frport2 = "";
    set.frport3 = ""; 
    set.frport4 = "checked";     
  }  
   
  if (set.fr_io == fr_bt) {
    set.fr_io1 = "checked";
    set.fr_io2 = "";
    set.fr_io3 = "";     
  } else 
  if (set.fr_io == fr_wifi) {
    set.fr_io1 = "";
    set.fr_io2 = "checked";
    set.fr_io3 = "";     
  }  else 
  if (set.fr_io == fr_wifi_bt) {
    set.fr_io1 = "";
    set.fr_io2 = "";
    set.fr_io3 = "checked";     
  }   

  if (set.wfmode == ap) {
    set.wfmode1 = "checked";
    set.wfmode2 = "";
    set.wfmode3 = ""; 
  } else 
  if (set.wfmode == sta) {
    set.wfmode1 = "";
    set.wfmode2 = "checked";
    set.wfmode3 = ""; 
  } else
  if (set.wfmode == sta_ap) {
    set.wfmode1 = "";
    set.wfmode2 = "";
    set.wfmode3 = "checked";    
  } 

  if (set.wfproto == tcp) {
    set.wfproto1 = "checked";
    set.wfproto2 = "";
  } else 
  if (set.wfproto == udp) {
    set.wfproto1 = "";
    set.wfproto2 = "checked";
  }

  if (set.btmode == master) {
    set.btmode1 = "checked";
    set.btmode2 = "";
  } else 
  if (set.btmode == slave) {
    set.btmode1 = "";
    set.btmode2 = "checked";
  }  
 
}
 //===========================================================================================
 void handleLoginPage() {

  #if ((defined ESP32) || (defined ESP8266)) && (defined Debug_SRAM)
    Log.printf("==============>Free Heap before handleLoginPage = %d\n", ESP.getFreeHeap());
  #endif  

  ComposeLoginPage();
  server.send(200, "text/html", loginPage); 
 }
 //===========================================================================================
 void handleSettingsPage() {

  Free_Bluetooth_RAM();   // Disables BT and required a reboot to reinstate BT

  ComposeSettingsPage();
  server.send(200, "text/html", settingsPage); 
 }
 //===========================================================================================
 void handleSettingsReturn() {
  ReadSettingsFromForm();

  WriteSettingsToEEPROM();
  
  String s = "<a href='/'> Rebooting........  Back to login screen</a>";
  server.send(200, "text/html", styleLogin+s);
  Log.println("Rebooting to adopt new settings ....\n");  
  delay(3000);
  ESP.restart();                 // esp32 and esp8266   
 }
 
 //===========================================================================================
 void handleReboot() {
  String s = "<a href='/'> Rebooting........  Back to login screen</a>";
  server.send(200, "text/html", styleLogin+s);
  Log.println("Rebooting ......");  
  delay(3000);
  ESP.restart();                 // esp32 and esp8266   
 }
 
 //===========================================================================================
 void handleOtaPage() {

  //Free_Bluetooth_RAM();   // Disables BT and required a reboot to reinstate BT
  
  server.sendHeader("Connection", "close");
  server.send(200, "text/html", otaIndex);
  /*handle upload of firmware binary file */
  server.on("/update", HTTP_POST, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
    ESP.restart();
  }, []() {
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      uint32_t uploadSize;
 //   Log.setDebugOutput(true);
      #if (defined ESP32) 
        uploadSize = UPDATE_SIZE_UNKNOWN;
      #elif (defined ESP8266) 
        WiFiUDP::stopAll();
        uint32_t maxSketchSpace = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
        uploadSize = maxSketchSpace;
      #endif

      Log.printf("Update: %s\n", upload.filename.c_str());    
      if (!Update.begin(uploadSize)) { //start with max available size
        Update.printError(Log);
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      /* flashing firmware to ESP OTA space */
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        Update.printError(Log);
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) { //true to show the percentage progress bar
        Log.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
        delay(2000);
      } else {
        Update.printError(Log);
      }
   // Log.setDebugOutput(false);
    }
    #if (defined ESP8266) // ESP8266 
      yield();
    #endif  
  });
 }
  //===========================================================================================
  //                           E E P R O M  Routines - ESP Only
  //===========================================================================================


  //This function will write a 4 byte (32bit) uint32_t to the eeprom at
  //the specified address to address + 3.
  void EEPROMWrite32(uint16_t address, uint32_t value)
      {
      //Decomposition from a long to 4 bytes by using bitshift.
      //One = Most significant -> Four = Least significant byte
      byte four = (value & 0xFF);
      byte three = ((value >> 8) & 0xFF);
      byte two = ((value >> 16) & 0xFF);
      byte one = ((value >> 24) & 0xFF);

      //Write the 4 bytes into the eeprom memory.
      EEPROM.write(address, four);
      EEPROM.write(address + 1, three);
      EEPROM.write(address + 2, two);
      EEPROM.write(address + 3, one);
      }

  //===========================================================================================
  //This function will read a 4 byte (32bits) uint32_t from the eeprom at
  //the specified address to address + 3.         
  uint32_t EEPROMRead32(uint16_t address)
      {
      //Read the 4 bytes from the eeprom memory.
      uint32_t four = EEPROM.read(address);
      uint32_t three = EEPROM.read(address + 1);
      uint32_t two = EEPROM.read(address + 2);
      uint32_t one = EEPROM.read(address + 3);

      //Return the recomposed long by using bitshift.
      return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
      }
  //===========================================================================================
  //This function will read a 2 byte (16bits) uint16_t from the eeprom at
  //the specified address to address + 1.     
  uint16_t EEPROMRead16(uint16_t address)
      {
      //Read the 2 bytes from the eeprom memory.
      uint32_t two = EEPROM.read(address);
      uint32_t one = EEPROM.read(address + 1);

      //Return the recomposed uint by using bitshift.
      uint16_t n = ((two << 0) & 0xFF) + ((one << 8)& 0xFFFF);
      return n;
      }
  //===========================================================================================
  //This function will read a byte (8bits) from the eeprom at
  //the specified address.    
  uint8_t EEPROMRead8(uint16_t address)   {
      uint8_t one = EEPROM.read(address);
      return one;
  }
  //===========================================================================================
  // This function will read a char array (string) from EEPROM at 
  //the specified address. 

  /*
  void EEPROMReadString(uint16_t address, char strptr[]) {
    String s;
    s = EEPROM.readString(address);  
    strcpy(strptr, s.c_str()); 
  }
  */

  void EEPROMReadString(uint16_t address, char *strptr) {
  char s[30];
        for (int i = 0 ; i < 30 ; i++) {  // safety limit
          s[i] = EEPROM.read(address+i);
          if (s[i] == 0x00) {             // eo string
            strcpy(strptr, s);  
            break;    
          }
        }
   } 

  //===========================================================================================
  //This function will write a 2 byte (16bits) uint16_t to the eeprom at
  //the specified address to address + 1.
  void EEPROMWrite16(uint16_t address, uint16_t value)
      {
      //Decomposition from an int to 2 bytes by using bitshift.
      //One = Most significant -> Four = Least significant byte
      byte two = (value & 0xFF);
      byte one = ((value >> 8) & 0xFF);

      //Write the 2 bytes into the eeprom memory.
      EEPROM.write(address, two);
      EEPROM.write(address + 1, one);
      } 
  //===========================================================================================
  //This function will write 1 byte (8bits) uint16_t to the eeprom at
  //the specified address.
  void EEPROMWrite8(uint16_t address, uint8_t value)
      {
      //Write the byte into the eeprom memory.
      EEPROM.write(address, value);
      }   
  //===========================================================================================
  // This function will write a char array (string) to EEPROM at 
  //the specified address.    
  void EEPROMWriteString(uint16_t address, char* strptr) {
        for (int i = 0 ; i <= 30 ; i++) {    // constraint
          EEPROM.write(address+i, strptr[i]);
          if (strptr[i] == 0x00) {
            break;       // eo string
          }
        }   
   }

  /*
  Also available in class library
  EEPROMwriteString (int address, const char* value)
  */

  //=================================================================================
#endif               // End of webSupport - ESP only
 //=================================================================================
 
void RawSettingsToStruct() {
  
  
  #if (FrSky_Port_Type == 0)
    set.frport = f_none;
  #elif (FrSky_Port_Type == 1)
    set.frport = f_port1;
  #elif (FrSky_Port_Type == 2)
    set.frport = f_port2;   
   #elif (FrSky_Port_Type == 3)
    set.frport = s_port;   
  #elif (FrSky_Port_Type == 4)
    set.frport = f_auto;    
  #endif
  
  if (FrSky_IO == 1) {
    set.fr_io = fr_bt;
  } else 
  if (FrSky_IO == 2) {
    set.fr_io = fr_wifi;
  }  else
  if (FrSky_IO == 3) {
    set.fr_io = fr_wifi_bt;
  } 
      
  if (WiFi_Mode == 1) {
    set.wfmode = ap;
  } else 
  if (WiFi_Mode == 2) {
    set.wfmode = sta;
  } else 
  if (WiFi_Mode == 3) {
    set.wfmode = sta_ap;
  } 
    
  if (Mav_WiFi_Protocol == 1) {
    set.wfproto = tcp;
  } else 
  if (Mav_WiFi_Protocol == 2) {
    set.wfproto = udp;
  } 
               
  set.channel = APchannel;
  strcpy(set.apSSID, APssid);  
  strcpy(set.apPw, APpw);                          
  strcpy(set.staSSID, STAssid);           
  strcpy(set.staPw, STApw);   
  strcpy(set.host, HostName);        
  set.tcp_localPort = TCP_localPort;
  set.tcp_remotePort = TCP_remotePort;  
  set.udp_localPort = UDP_remotePort;
  set.udp_remotePort = UDP_localPort;  

  if ( BT_Mode == 1 ) {
    set.btmode = master;
  } else if ( BT_Mode == 2 ) {
    set.btmode = slave; 
  }  
  strcpy(set.btConnectToSlave, BT_ConnectToSlave); 
  
  #if (defined webSupport)   
    set.web_support =  true;   // this flag is not saved in eeprom
  #else
    set.web_support =  false;
  #endif

  #if defined webSupport
     RefreshHTMLButtons();
  #endif
     
  #if defined Debug_Web_Settings
      Log.println();
      Log.println("Debug Raw WiFi Settings : ");
      Log.print("web_support = "); Log.println(set.web_support);      
      Log.print("validity_check = "); Log.println(set.validity_check, HEX);     
      Log.print("frport = "); Log.println(set.frport);           
      Log.print("set.fr_io = "); Log.println(set.fr_io);                  
      Log.print("wifi mode = "); Log.println(set.wfmode);
      Log.print("wifi protocol = "); Log.println(set.wfproto);     
      Log.print("wifi channel = "); Log.println(set.channel);  
      Log.print("apSSID = "); Log.println(set.apSSID);
      Log.print("apPw = "); Log.println(set.apPw);
      Log.print("staSSID = "); Log.println(set.staSSID);
      Log.print("staPw = "); Log.println(set.staPw); 
      Log.print("Host = "); Log.println(set.host);           
      Log.print("tcp_localPort = "); Log.println(set.tcp_localPort);
      Log.print("tcp_remotePort = "); Log.println(set.tcp_remotePort);      
      Log.print("udp_localPort = "); Log.println(set.udp_localPort);
      Log.print("udp_remotePort = "); Log.println(set.udp_remotePort); 
      Log.print("bt mode = "); Log.println(set.btmode); 
      Log.print("Master to Slave Name = "); Log.println(set.btConnectToSlave); 
   
      Log.println(); 
  #endif    
}

//=================================================================================
