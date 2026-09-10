//**********************************************************************************************************
//*    ESP32S3_inet_and_dsp_radio_srv -- Internet and DSP radio with control function by web server
//*                                      Also you can control ESP32S3 using BLE(Bluetooth LE) GATT without WiFi.
//*
//**********************************************************************************************************
//**********************************************************************************************************
//*    audioI2S -- I2S audiodecoder for ESP32,  refer to https://github.com/schreibfaul1/ESP32-audioI2S
//*                <<<<< You must use ver 3.3.2 >>>>>                                                             *
//**********************************************************************************************************
//
// Internet radio, first release on 11/2018 for ESP32
//   Version 2  , Aug.05/2019
//   Version 3  , Aug.26/2024 for XIAO ESP32S3
//
// Revise for Internet and DSP : 1/10/2025 - 2/14/2025
// Revise for BLE function     : 3/6/2025 - 4/14/2025
// Revise for Rotary Encoder   : 9/6/2025 - 9/14/2025
// Revise for PCF8574          : 4/25/2026 
// Revise for DSP radio Recording        : 5/25/2026   only for XIAO SENSE
// Revise for INET Recording   : 6/25/2026   only for XIAO SENSE (Ver. 0.78)
//
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

#include "Rotary.h"

#include "Arduino.h"
#include "WiFiMulti.h"
#include "WebServer.h"
#include "Audio.h"
#include <Adafruit_GFX.h>       // install using lib tool

// **** Select a Display **** 
//#include <Adafruit_SSD1306.h>   // install using lib tool
#include <Adafruit_SH110X.h>      // install using lib tool

#include "Wire.h"
#include <esp_sntp.h>           // esp lib
#include <TimeLib.h>            // https://github.com/PaulStoffregen/Time
#include <Preferences.h>        // For permanent data
#include <RDA5807.h>            // install using lib tool
#include <driver/i2s.h>
//#include "driver/i2s_std.h"
//#include "ESP_I2S.h"
#include "driver/gpio.h"
#include "esp_rom_gpio.h"
//#include "soc/i2s_sig_map.h"
#include <Adafruit_PCF8574.h>   // I/O expander
#include <SdFat.h>            // for rename (ver 0.65)
#define MAXVOL    50          // inet max volume  <= 50
#define MAXVOL_R  15          // radio max volume  <= 15
#define LED_BULTIN    21      // XIAO esp32s3, low level on     
#define I2S_DOUT      2       //## esp32:25  #### old value 2
#define I2S_BCLK      1       //## esp32:26
#define I2S_LRC       44      //## esp32:22
//#define SLEEP         43      // not used : sleep timer control button pin #### old value 1 (ver:0.55)
#define PIN_SDA 5             // i2c
#define PIN_SCL 6             // i2c
//#define VOL_PIN1     3        // volume up
//#define VOL_PIN2     4        // volume down #### old value 44
// Enconder PINs
#define ENCODER_PIN_A 3        // volume
#define ENCODER_PIN_B 4        // volume
#define INT_PIN       2        // station change --> mode inet/dsp change , rotary encoder support
#define PCF_INT_PIN   42       // D11 on camera bord
//#define AOUT_SW      44      // audio output switch #### old value 4
#define OLED_I2C_ADDRESS  0x3C // Check the I2C bus of your OLED device
#define SCREEN_WIDTH  128      // OLED display width, in pixels
#define SCREEN_HEIGHT  64      // OLED display height, in pixels
#define OLED_RESET  -1         // Reset pin # (or -1 if sharing Arduino reset pin)
#define MAXSTNIDX     7        // station index 0-7          
#define MAXSCEDIDX    8        // schedule table index 0-8
#define VERSION_NR  " ver:0.81"    // REC function for INET support
// uuidgen : d392ca55-7a45-47db-adb9-26164b3e7a7b          
static BLEUUID IDBserviceUUID("00001805-7a45-47db-adb9-26164b3e7a7b");
static BLEUUID IDBcharUUID("00002a2b-7a45-47db-adb9-26164b3e7a7b");
static BLEUUID IDBcharDOWUUID("00002a09-7a45-47db-adb9-26164b3e7a7b");
static BLEUUID IDBcharPPCPUUID("00002a04-7a45-47db-adb9-26164b3e7a7b");
#define descriptorUUID "00002901-7a45-47db-adb9-26164b3e7a7b"
#define LOCAL_NAME "DSP_INET_radio"
// rec
#define RECORD_TIME   1         // in seconds, to estimate buffer full time
#define K32  32*1024            // 32KB
#define SAMPLE_RATE 24000U      // most applicable value for DSP Radio interference, 32K -> 24K   (Ver. 0.80)
//#define SAMPLE_RATE 16000U
//#define SAMPLE_RATE 32000U
#define SAMPLE_BITS 16
#define WAV_HEADER_SIZE 44
#define CHAN_NUM    2           // channel number, stereo is 2 
#define SAMPLE_RATE_MIC 32000U  

// Wav File recording and reading
#define MAX_RECORD_TIME  30               // Max limited recording time default in minutes
#define MAX_RECORD_TIME_LIMIT  300        // Max limited recording time limit in minutes
#define REC_FREQUENCY  20000000           // 1MHz-24MHz, apply for SPI & SD both
#define I2S_DMA_BUFFER  32                // number of  I2S_DMA_BUFFER ok:52
// I2S to DAC ex. PCM5102
#define I2S_DOUT_A    2
#define I2S_BCLK_A    1
#define I2S_LRC_A     44
#define I2S_NUM_A     0        // DAC I2S port number -> not used currently
#define I2S_DOUT      2        // V 0.76 
#define I2S_BCLK      1        // v 0.76
#define I2S_LRC       44
// I2S from DSP Radio
#define I2S_DIN_S       4
#define I2S_BCLK_S      3                                  
#define I2S_LRC_S       43 
// SPI with SD card drive of esp32s3 sense
#define SD_CS         21
#define SPI_MOSI      9
#define SPI_MISO      8
#define SPI_SCK       7

extern void gpio_matrix_out(uint32_t gpio_num, uint32_t signal_idx, bool out_inv, bool oen_inv);

BLEServer  *pServer = NULL;
BLEClient  *pClient = NULL;
BLEAddress *pBLEAddress = NULL;
BLEDescriptor *pDescriptor = new BLEDescriptor("00002901-7a45-47db-adb9-26164b3e7a7b");  
static bool connected = false;  // ble connection is active 
static bool advertise = false;  // ble advertise is active 
static bool wrote = false;      // ble onwrite() of decriptorcallback
static char strbuff[24];
static const char *stnStr[7] = {"ST0","ST1","ST2","ST3","ST4","ST5","ST6"}; // 3 char, preferences:key of radio station name 
int max_radio = 7;  // max radio station
int bdow = 0; // current browser day of week
int bstn = 0; // current browser radio station
String wstr = "";
//
//
Audio audio;
//Audio audio(false, 3, I2S_NUM_1); // change default i2s port number
WiFiMulti wifiMulti;
#ifdef _Adafruit_SSD1306_H_
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#endif
#ifdef _Adafruit_SH110X_H_
Adafruit_SH1106G oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#endif
RDA5807 radio;
bool pcf_active = false;
bool pcf_int = false;
bool pcf_int_ok = true;
Adafruit_PCF8574 pcf;
SdFat sdf;    // for rename (ver 0.65)
File file;
// WiFi
bool ble = false; // BLE mode
bool WiFi_OK = false;
String ssid =     "SSID1";         // WiFi 1, specify your access point
String password = "PASSWORD1";
String ssid2 =     "SSID2";      // WiFi 2, uncomment wifiMulti.addAP() below
String password2 = "PASSWORD2";          // if you want to specify it
// time
struct tm *tm;
int d_mon ;
int d_mday ;
int d_hour ;
int d_min = 99;
int d_sec ;
int d_wday ;
int d_year ;
int last_d_sec = 99;
int last_d_min = 99;
static const char *weekStr[7] = {"Sun","Mon","Tue","Wed","Thu","Fri","Sat"}; // 3 char, preferences:key of week schedule
int currIdx = 99;
int pofftm_h = 0;
int pofftm_m = 0;
const char* ntpServer = "ntp.nict.jp";
const long  gmtOffset_sec = 32400;
const int   daylightOffset_sec = 0;
// mode: inet or dsp
bool mode_chg_ok = true;
bool mode_chg_req = false;
bool mode_chg_int = false;
//
int volume = 2; // inet volume
int station = 0; // inet station url
uint32_t ct2,pt2,event;  // interval time 
uint32_t pt = 0;  // interval time for switch
bool sleepmode = false;
int s_remin = 99;
bool conn_ok = false;
// mode
int inet_radio = 0;  //0: DSP radio, 1: Internet radio on
bool led_onoff = true;
int loop_cnt = 0;
// rec
bool stop_read = true;   // priority SD read active in loop()
bool WAV_read = false;    // wav file read
bool REC_on = false;     // DSP recording on
bool REC_on_no_poff = false;    // recording on but not power off
bool MIC_rec_on = false; // MIC recording on
bool SD_open = false;    // SD open
bool SD_write = false;   // SD write ok
bool WAVE_HDR_write = false; // wavwfile headder wrote
bool I2S_err = false;    // any I2S(DSP) error detected
bool DSP2DAC = false;    // dsp(i2s_1)  to dac(i2s_0)
bool in_play = false;    // now playing
bool INET_in_play = false; // now playing when INET in play
bool initiate_recording = false; // initiate_recording
bool poff_after_rec = false; // recording  stop and poff
int last_blk = 0;
int avail_cnt = 0;
uint32_t record_size = (SAMPLE_RATE * SAMPLE_BITS * CHAN_NUM / 8) * RECORD_TIME; // possible size at once in byte
uint8_t *rec_buffer1 = NULL;  // PSRAM
uint8_t *rec_buffer2 = NULL;  // PSRAM
uint8_t *rec_buffer32k = NULL;  // PSRAM
uint8_t *rec_buffer32k_2 = NULL;  // PSRAM
uint8_t *jpg_buffer = NULL;   // PSRAM
int curr_buf = 1;             // current buff
uint32_t rec_bufp_w = 0;
uint32_t rec_bufp_r = 0;
uint32_t recorded_size = 0;
uint32_t total_recorded_size = 0;
uint32_t estimated_recorded_size = 0;
int rectime = MAX_RECORD_TIME;
//SdFile file;
char wave_filename[32];
//bool wav_2nd_time = false;
int wav_fcount = 1;
bool dsp_active = false;
const int cbl = 50; // circular buffer length
String cb[cbl];     // circular buffer to store SD file name
uint32_t cb_sz[cbl];// circular buffer to store SD file size

struct WavHeader_Struct {
  //   RIFF Section
  char RIFFSectionID[4];  // Letters "RIFF"
  uint32_t Size;          // Size of entire file 
  char RiffFormat[4];     // Letters "WAVE"

  //   Format Section
  char FormatSectionID[4];  // letters "fmt"
  uint32_t FormatSize;      // Size of format section
  uint16_t FormatID;        // 1=uncompressed PCM
  uint16_t NumChannels;     // 1=mono,2=stereo
  uint32_t SampleRate;      // 44100, 32000, 24000, 16000, 8000 etc.
  uint32_t ByteRate;        // =SampleRate * Channels * (BitsPerSample/8)
  uint16_t BlockAlign;      // =Channels * (BitsPerSample/8)
  uint16_t BitsPerSample;   // 8,16,24 or 32

  // Data Section
  char DataSectionID[4];  // The letters "data"
  uint32_t DataSize;      // Size of the data that follows
} WavHeader;

File WavFile;                                     // SD card directory

// camera
int photo_Count = 1;             // File Counter
bool camera_ok = false;          // Check camera status
bool sd_ok = false;              // Check sd status
bool shoot_s = false;            // shoot on/off by browser


// Preset internet station url and name
const char* station_url[]={
  "http://quincy.torontocast.com:2070/",  "http://quincy.torontocast.com:2020/",
  "http://jenny.torontocast.com:8062/stream",  "http://51.195.203.179:8002/stream",
  "http://216.235.89.171:80/hitlist",  "string 6"
};
const char* station_name[]={
  "JPopSakura",  "JPop hits",
  "J1GOLD",  "HotHits UK",
  "POWERHIT40",  "string 6"
}; // max 10 char
char stnurl[128];  // current internet station url
char stnname[24];  // current internet station name
int max_station = 5; // valid entry
// web server
WebServer server(80);  // port 80(default)
// Operation by server
int stoken = 0;  // server token, count up 
int s_srv = 1; // server request 
int s = 1; // toggle sleep
int a_srv = 1;
int b_srv = 1;
char titlebuf[166];
char rstr[128] = {'/n'};
// DSP Radio RDA5807
int  vol_r;
//int  lastvol;
int  stnIdx;
int  laststnIdx;
int  stnFreq[] = {8040, 8250, 8520, 9040, 9150, 7720, 7810, 7860}; // frequency of radio station
String  stnName[] = {"AirG", "NW", "NHK", "STV", "HBC", "NW_2", "karos", "nosut"}; // name of radio station max 5 char
std::string stnList = "";

String msg = "none";
bool bassOnOff = false;
bool vol_ok = true;
bool stn_ok = true;
bool p_onoff_req = false;
bool p_on = false;
uint32_t currentFrequency;
float lastfreq;
struct elm {  // program
   int stime; // strat time(min)
   int fidx;  // frequency table index
   int duration; // length min
   int volstep; // volume
   int poweroff; // if 1, power off after duration
   int scheduled; // if 1, schedule done for logic
};
struct elm entity[7][MAXSCEDIDX + 1] = {
{{390,1,59,2,1,0},{540,6,59,1,0,0},{600,0,59,1,0,0},{660,3,119,1,0,0},{780,1,59,1,0,0},{840,0,59,1,0,0},{900,6,59,1,0,0},{1140,3,119,1,0,0},{1410,0,29,1,1,0}}, // sun
{{390,4,59,2,1,0},{480,3,119,1,0,0},{600,6,59,1,0,0},{720,2,119,1,0,0},{840,1,119,1,0,0},{0,0,0,0,0,0},{1020,1,119,1,0,0},{1200,6,89,1,0,0},{1410,0,29,1,1,0}}, // mon
{{390,4,59,2,1,0},{480,3,119,1,0,0},{720,2,89,1,0,0},{840,1,119,1,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0},{1020,1,119,1,0,0},{1200,0,89,1,0,0},{1410,0,29,1,1,0}}, // tue
{{390,4,59,2,1,0},{480,3,119,1,0,0},{720,2,89,1,0,0},{840,1,119,1,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0},{1020,1,119,1,0,0},{1200,0,89,1,0,0},{1410,0,29,1,1,0}}, // wed
{{390,4,59,2,1,0},{480,3,119,1,0,0},{600,6,59,1,0,0},{720,2,119,4,0,0},{840,1,119,1,0,0},{960,1,59,1,0,0},{1080,1,119,1,0,0},{1200,0,89,1,0,0},{1290,3,59,1,1,0}}, // thu
{{390,4,59,2,1,0},{480,3,119,1,0,0},{660,0,59,1,0,0},{720,2,119,1,0,0},{840,6,119,1,0,0},{0,0,0,0,0,0},{1080,1,119,1,0,0},{1200,1,89,1,0,0},{1290,3,59,1,1,1}}, // fri
{{390,0,29,2,0,0},{420,2,119,1,0,0},{540,2,110,1,0,0},{720,2,119,1,0,0},{840,2,119,1,0,0},{960,2,119,1,0,0},{1080,4,59,1,0,0},{1140,3,119,1,0,0},{1260,0,89,1,1,0}}  // sat
};
Preferences preferences; // Permanent data "WF0":WiFi SSID, "vol":inet volume, "volr":dsp vol_r , "stix":dsp stnIdx, "stn":inet station
// encoder
// Encoder control variables
volatile int encoderCount = 0;
Rotary encoder = Rotary(ENCODER_PIN_A, ENCODER_PIN_B);
int RE_Clicks = 0;  // 1:Sleep, 2:Staion(STN), 3:Reset(VOL)
int RE_Mode = 0;    // 0:VOL, 1:STN
int RE_ct = 0;
int RE_pt = 0;
//
int setWeeksced(String wstr);
int setStation(String wstr);

class MyDescriptorCallbacks: public BLEDescriptorCallbacks {
  void onWrite(BLEDescriptor *pDesciptor) {
    uint8_t *value = pDescriptor->getValue();
    int vl = pDescriptor->getLength();
    int i;
    wrote = true;
    if (vl >= sizeof(strbuff)) vl = 22;
    Serial.print("***** New value: ");
    memcpy(strbuff, value, vl);
    strbuff[vl] = '\n';
    strbuff[vl+1] = 0;
    if (strbuff[0]=='W') { // Day of Week selection
       bdow = strbuff[2] - '1';
       if (bdow < 0 || bdow >= 7) {
         Serial.println("Description err data");
         bdow=0;
        }
    }
    else if (strbuff[0]=='S') { // Radio Staion selection
       bstn = strbuff[2] - '0';
       if (bstn < 0 || bstn >= MAXSTNIDX) {
         Serial.println("Description err data");
         bstn=0;
        }
    }
    Serial.println(strbuff);
  }
};

class MyCallbacks: public BLECharacteristicCallbacks {
  int typ;
  public:
   MyCallbacks(int t){ typ = t;};
  void onWrite(BLECharacteristic *pCharacteristic) {
    Serial.println("MyCallbacks Write");
    wstr = "";
    msg = "";
    std::string value = pCharacteristic->getValue();
    if (value.length() > 0) {
      Serial.print("***** New value: ");
      for (int i = 0; i < value.length(); i++) {
        Serial.print(value[i]);
        wstr += value[i];
      }
      Serial.println();
      int rc = 0;
      if (value.length() < 40) {
        rc = setStation(wstr);   // register new station
        //if (rc != 0) Serial.println(msg);
      } else {
        rc = setWeeksced(wstr);  // register new schedule
        //if (rc != 0) Serial.println(msg);
      }
      Serial.println(msg);
    }
  }
  void onRead(BLECharacteristic *pCharacteristic) {
    std::string wsced ="";
    char htstr[180];
    Serial.println("MyCallbacks Read");
    if (typ == 0)  return; // nop if not necessary
    int i=bdow;
      wsced = weekStr[bdow];
      wsced += ";";
      for(int j = 0; j <= MAXSCEDIDX; j++) {
        sprintf(htstr,"%d:%02d,%d,%d,%d,%d",entity[i][j].stime / 60,entity[i][j].stime % 60,entity[i][j].fidx,entity[i][j].duration,entity[i][j].volstep,entity[i][j].poweroff);
        wsced += htstr;
        if (j != MAXSCEDIDX) wsced += ";";
      }
      wsced += ";";
    wsced += "\n\0";
    pCharacteristic->setValue(wsced);
  }
};
class MyCallbacks2: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    Serial.println("MyCallbacks2 Write");
    wstr = "";
    msg = "";
    std::string value = pCharacteristic->getValue();
    if (value.length() > 0) {
      Serial.print("***** New value: ");
      for (int i = 0; i < value.length(); i++) {
        Serial.print(value[i]);
        wstr += value[i];
      }
      Serial.println();
      if (value.length() < 40) {
        setStation(wstr);   // registor new station
      } else {
        setWeeksced(wstr);  // register new schedule
      }
      Serial.println(msg);
    }
  }
  void onRead(BLECharacteristic *pCharacteristic) {
    std::string stn ="";
    char htstr[80];
    float tf;
    Serial.println("MyCallbacks2 Read");
    int i=bstn;
      stn = stnStr[bstn];
      stn += ",";
      tf = stnFreq[bstn]/100.0;
      sprintf(htstr,"%3.1f",tf);
      stn += htstr;
      stn += ",";
      stn += stnName[bstn].c_str();
    stn += "\n\0";
    pCharacteristic->setValue(stn);
  }
};

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer, esp_ble_gatts_cb_param_t *param) {
    pBLEAddress = new BLEAddress(param->connect.remote_bda);
    Serial.printf("MyServerCallbacks Server Connected: %s\n", pBLEAddress->toString().c_str());
    connected = true;
    advertise = false;
    pServer->getAdvertising()->stop();  // stop Advertise
    Serial.println("MyServerCallbacks Stop Advertising");
  }

  void onDisconnect(BLEServer* pServer) {
    Serial.println("MyServerCallbacks Server Disconnected");
    connected = false;
  }
};
int setStation(String val1){
  String instr[12] = {"\n"};
  int ix = split(val1,',',instr);
  if (ix != 3) {
    msg = "different number of arguments.";
    return 4;
  } else {
    //msg = "arguments. ok.";
    instr[0].trim();
    String str = instr[0].substring(2,3);
    int stn = str.toInt();
    char tstr[80];
    sprintf(tstr,"%s:%s==%d", instr[0], str, stn);
    Serial.println(tstr);
    if (stn > MAXSTNIDX-1) { msg = "invalid number of stations."; return 4;}
    else {
      // normal process
      msg = "OK! Processing.";
      if (instr[0].substring(0,1).equals("W")) {// wifi setup
         preferences.putString("WF0", val1);
         Serial.println("Regiter WiFi info: " + val1);
      } else {
        instr[1].trim();
        float freq = instr[1].toFloat();
        if (freq < 50 || freq > 108) {
          msg = "invalid frequency.";
          return 4;
        }
        stnFreq[stn] = freq * 100;
        instr[2].trim();
        stnName[stn] = instr[2];
        preferences.putString(stnStr[stn], val1);  // save permanently
        Serial.println("Regiter new station: " + val1);
      }
      msg = "OK! Done.";
      return 0;
    }
  } 

}
void generate_wav_header(uint8_t *wav_header, uint32_t wav_size, uint32_t sample_rate)
{
  // See this for reference: http://soundfile.sapp.org/doc/WaveFormat/
  uint32_t file_size = wav_size + WAV_HEADER_SIZE - 8;
  //uint32_t byte_rate = SAMPLE_RATE * SAMPLE_BITS / 8;
  uint32_t byte_rate = sample_rate * SAMPLE_BITS / 8;
  const uint8_t set_wav_header[] = {
    'R', 'I', 'F', 'F', // ChunkID
    file_size, file_size >> 8, file_size >> 16, file_size >> 24, // ChunkSize
    'W', 'A', 'V', 'E', // Format
    'f', 'm', 't', ' ', // Subchunk1ID
    0x10, 0x00, 0x00, 0x00, // Subchunk1Size (16 for PCM)
    0x01, 0x00, // AudioFormat (1 for PCM)
    //0x01, 0x00, // NumChannels (1 channel)
    0x02, 0x00, // NumChannels (2 channel)
    sample_rate, sample_rate >> 8, sample_rate >> 16, sample_rate >> 24, // SampleRate
    byte_rate, byte_rate >> 8, byte_rate >> 16, byte_rate >> 24, // ByteRate
    //0x02, 0x00, // BlockAlign mono
    0x04, 0x00, // BlockAlign stereo
    0x10, 0x00, // BitsPerSample (16 bits)
    'd', 'a', 't', 'a', // Subchunk2ID
    wav_size, wav_size >> 8, wav_size >> 16, wav_size >> 24, // Subchunk2Size
  };
  const uint8_t set_wav_header_m[] = {
    'R', 'I', 'F', 'F', // ChunkID
    file_size, file_size >> 8, file_size >> 16, file_size >> 24, // ChunkSize
    'W', 'A', 'V', 'E', // Format
    'f', 'm', 't', ' ', // Subchunk1ID
    0x10, 0x00, 0x00, 0x00, // Subchunk1Size (16 for PCM)
    0x01, 0x00, // AudioFormat (1 for PCM)
    0x01, 0x00, // NumChannels (1 channel)
    sample_rate, sample_rate >> 8, sample_rate >> 16, sample_rate >> 24, // SampleRate
    byte_rate, byte_rate >> 8, byte_rate >> 16, byte_rate >> 24, // ByteRate
    0x02, 0x00, // BlockAlign
    0x10, 0x00, // BitsPerSample (16 bits)
    'd', 'a', 't', 'a', // Subchunk2ID
    wav_size, wav_size >> 8, wav_size >> 16, wav_size >> 24, // Subchunk2Size
  };
  if (REC_on) 
    memcpy(wav_header, set_wav_header, sizeof(set_wav_header));
  else
    memcpy(wav_header, set_wav_header_m, sizeof(set_wav_header_m));
}

int i2s_install(std::string type) {
  // Set up I2S Processor configuration
  const i2s_config_t i2s_config_dsp = { // for DSP radio
    .mode = i2s_mode_t(I2S_MODE_SLAVE | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = i2s_bits_per_sample_t(16),
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT, // R-chan, L-chan
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S), //I2S Philips standard
    .intr_alloc_flags = 0, //
    .dma_buf_count = 32,   // #### ex. 52
    .dma_buf_len = 512,
    .use_apll = false
  };
  const i2s_config_t i2s_config_net = { // for INET
    .mode = i2s_mode_t(I2S_MODE_SLAVE | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
//    .sample_rate = 16000U,
    .bits_per_sample = i2s_bits_per_sample_t(16),
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT, // R-chan, L-chan
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S), //I2S Philips standard
    .intr_alloc_flags = 0, //
    .dma_buf_count = 32,   // #### ex. 52
    .dma_buf_len = 512,
    .use_apll = false
  };
  const i2s_config_t i2s_config_mic = { // for MIC
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX| I2S_MODE_PDM),
    .sample_rate = SAMPLE_RATE_MIC,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S ,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 64,
    .dma_buf_len = 64,
    .use_apll = false,
    .tx_desc_auto_clear   = false,
    .fixed_mclk           = 0  
  };
  /*
  const i2s_config_t i2s_config_dac = {  // for DAC out
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = 32000U,  // Note, this will be changed later
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,  // high interrupt priority
    .dma_buf_count = 16,                       // 16 buffers
    .dma_buf_len = 512,                        // 512 bytes per buffer
    .use_apll = 0,
    .tx_desc_auto_clear = true,
    .fixed_mclk = -1                           
  };// --> NOT USED */
  const i2s_config_t i2s_config_dac = {  // for DAC out
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = SAMPLE_RATE,  // Note, this will be changed later
    .bits_per_sample = i2s_bits_per_sample_t(16),
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S | I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,  
    .dma_buf_count = 32,                        // 32 buffers
    .dma_buf_len = 512,                         // 512 bytes per buffer
    .use_apll = 0,
    .tx_desc_auto_clear = true,
    .fixed_mclk = -1                           
  };// --> NOT USED
  int erResult;
  if (type=="DSP") {
    erResult = i2s_driver_install(I2S_NUM_1, &i2s_config_dsp, 0, NULL);
  } else if (type=="MIC") {
    erResult = i2s_driver_install(I2S_NUM_0, &i2s_config_mic, 0, NULL);  // PDM is only num_0 OK
  } else if (type=="DAC") {
    erResult = i2s_driver_install(I2S_NUM_1, &i2s_config_dac, 0, NULL); //  not used
    Serial.println("i2s install(DAC)"); // 
  } else if (type=="NET") {
    erResult = i2s_driver_install(I2S_NUM_1, &i2s_config_net, 0, NULL); //  for INET
    Serial.println("i2s install(NET)"); //
  } else erResult = ESP_ERR_INVALID_ARG;
  if (erResult!=ESP_OK) Serial.printf("i2s intall %s err(%d)\n", type.c_str(), erResult);
  return(erResult);
}

int i2s_setpin(std::string type) {
  // Set I2S pin configuration
  const i2s_pin_config_t pin_config_dsp = { // from DSP radio
    .bck_io_num = I2S_BCLK_S,
    .ws_io_num = I2S_LRC_S,
    .data_out_num = -1,
    .data_in_num = I2S_DIN_S
  };
  const i2s_pin_config_t pin_config_net = { // from INET
    .bck_io_num = I2S_BCLK_S,
    .ws_io_num = I2S_LRC_S,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_DIN_S
  };
  const i2s_pin_config_t pin_config_mic = { // from MIC
    .bck_io_num = I2S_PIN_NO_CHANGE, //  clock 
    .ws_io_num = 42,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = 41 //  Data
  };
  const i2s_pin_config_t pin_config_dac = {  // to DAC out
    .bck_io_num = I2S_BCLK_A,           //  clock 
    //.bck_io_num = I2S_BCLK_S,           //  clock 
    .ws_io_num = I2S_LRC_A,             //  Word select
    //.ws_io_num = I2S_LRC_S,             //  Word select
    .data_out_num = I2S_DOUT_A,         //  Data out
    .data_in_num = I2S_DOUT_A    //    loop back   
  };
  int erResult;
  if (type=="DSP") {
    erResult = i2s_set_pin(I2S_NUM_1, &pin_config_dsp);
  } else if (type=="MIC") {
    erResult = i2s_set_pin(I2S_NUM_0, &pin_config_mic); // 
  }  else if (type=="DAC") {
    erResult = i2s_set_pin(I2S_NUM_1, &pin_config_dac); // not used
    Serial.println("i2s setpin(DAC)"); // 
  }  else if (type=="NET") {
    erResult = i2s_set_pin(I2S_NUM_1, &pin_config_net); // for INET
    Serial.println("i2s setpin(NET)"); // 
  } else erResult = ESP_ERR_INVALID_ARG;
  if (erResult!=ESP_OK) Serial.printf("i2s set pin %s err(%d)\n",type, erResult);
  return(erResult);
}
void initiate_rec(int debug) {
  char ts[80];
  // Init output pins
  gpio_config_t io_conf = {
      .pin_bit_mask = (1ULL << GPIO_NUM_3) | (1ULL << GPIO_NUM_4 ) | (1ULL << GPIO_NUM_43),
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE
  };
  total_recorded_size = 0;
  last_blk = 0;
  estimated_recorded_size = rectime * SAMPLE_RATE * SAMPLE_BITS * 60 * 2 / 8;  // recording time min
  pofftm_h = (d_hour * 60 + d_min + rectime) / 60;  // auto stop after recording time min
  pofftm_m = (d_hour * 60 + d_min + rectime) % 60;
  sprintf(ts,"%02d:%02d %s",pofftm_h,pofftm_m,"poff or recording stop scheduled");
  Serial.println(ts); 
  REC_on_no_poff = true;
  esp_err_t err = 0;
  if (inet_radio == 0) { // dsp
    err = i2s_install("DSP");  // I2S_NUM_1
    err += i2s_setpin("DSP");
    if (err != ESP_OK) {
      Serial.println("Failed to initialize I2S for DSP!");
      I2S_err = true;
    }
  } else { // inet
    if (!I2S_err && (inet_radio == 1)) { // inet
      radio.powerDown();
      dsp_active = false;
      gpio_config(&io_conf);
      err = i2s_install("NET");  // I2S_NUM_1 using GPIO 3, 4, 43 
      err += i2s_setpin("NET");
      // Set internal signal ID
      uint32_t sig_from_gpio1  = REG_GET_FIELD(GPIO_FUNC0_OUT_SEL_CFG_REG + (GPIO_NUM_1 * 4),  GPIO_FUNC0_OUT_SEL);
      uint32_t sig_from_gpio2  = REG_GET_FIELD(GPIO_FUNC0_OUT_SEL_CFG_REG + (GPIO_NUM_2 * 4),  GPIO_FUNC0_OUT_SEL);
      uint32_t sig_from_gpio44 = REG_GET_FIELD(GPIO_FUNC0_OUT_SEL_CFG_REG + (GPIO_NUM_44 * 4), GPIO_FUNC0_OUT_SEL);
      // Routing signals to target GPIOs
      gpio_matrix_out(GPIO_NUM_3,  sig_from_gpio1,  false, false);
      gpio_matrix_out(GPIO_NUM_4,  sig_from_gpio2,  false, false);
      gpio_matrix_out(GPIO_NUM_43, sig_from_gpio44, false, false);   
    }
  }
  if(!I2S_err && !SD.begin(SD_CS, SPI, REC_FREQUENCY, "/sd")){ // SD mount
    Serial.println("Failed to mount SD Card!");
    I2S_err = true;
    i2s_stop(I2S_NUM_1);  // nomore DSP I2S now
  }
  if (!I2S_err) {
    REC_on = true; // start REC ok
    REC_on_no_poff = true;
    msg = "control: record";
    Serial.println("REC initiate OK!");
  } else {
    msg = "control: record err";
    Serial.println("REC initiate fail!");
  }
}

int split(String data, char delimiter, String *dst){
  int index = 0;
  int arraySize = (sizeof(data))/sizeof((data[0]));
  int datalength = data.length();
  
  for(int i = 0; i < datalength; i++){
    char tmp = data.charAt(i);
    if( tmp == delimiter ){
      index++;
      if( index > (arraySize - 1)) return -1;
    }
    else dst[index] += tmp;
  }
  return (index + 1);
}
int dayofWeek(String dow) {
  dow.trim();
  //Serial.println(dow);
  if (dow.equals("Sun")) return 0; 
  else if (dow.equals("Mon")) return 1;
  else if (dow.equals("Tue")) return 2;
  else if (dow.equals("Wed")) return 3;
  else if (dow.equals("Thu")) return 4;
  else if (dow.equals("Fri")) return 5;
  else if (dow.equals("Sat")) return 6;
  else return 9;
}

int setWeeksced(String val1){
  String instr[12] = {"\n"};
  String instr2[8] = {"\n"};
  String instr3[4] = {"\n"};
  int ix = split(val1,';',instr);
  if (ix != 11) {
    msg = "different number of arguments.";
    return 4;
  } else {
    //msg = "arguments. ok.";
    int down = dayofWeek(instr[0]);
    if (down > 6) { msg = "invalid day of week."; return 4;}
    else {
      // normal process
      msg = "normal process.";
      instr[0].trim();
      Serial.println(instr[0]);
      for(int j = 0; j <= MAXSCEDIDX; j++) {
        instr[j+1].trim();
        msg = "normal process 2.";
        //Serial.println(instr[j+1]);
        String val2 = instr[j+1];
        ix = split(val2,',',instr2);
        if (ix != 5) { 
            msg = "different number of  2nd level arguments.";
            return 4;
        } else {
            //for(int i = 0; i < 5; i++) {
              msg = "OK! Processing.";
              //Serial.println(instr2[i]);
              val2 = instr2[0];
              ix = split(val2,':',instr3);
              if (ix != 2) {
                msg = "different number of  3rd level arguments.";
                return 4;
              }
              instr3[0].trim();
              instr3[1].trim();
              entity[down][j].stime = instr3[0].toInt() * 60 + instr3[1].toInt();
              instr3[0] = "";
              instr3[1] = "";
              instr2[0] = "";

              entity[down][j].fidx = instr2[1].toInt();
              // todo: check fidx
              instr2[1] = "";
              entity[down][j].duration = instr2[2].toInt();
              instr2[2] = "";
              entity[down][j].volstep = instr2[3].toInt();
              instr2[3] = "";
              entity[down][j].poweroff = instr2[4].toInt();
              instr2[4] = "";
              entity[down][j].scheduled = 0; // reset
              preferences.putString(weekStr[down], val1);  // save permanently
            //}
        }
        
      }
      if (d_wday==down) { //  Is schedule of today changed?  (ver:0.52)
        pofftm_h = 0;     // clear power off time
        pofftm_m = 0;
        Serial.println("reset power off time");
      }
      msg = "OK! Done.";
      return 0;
    }
  } 
}
void handleRoot(void)
{
  String html;
  String val1;
  String val2;
  String val3;
  String val4;
  String val5;
  String val6;
  String val7;
  String val8;
  String val9;
  String val10;
  String val11;
  String val12;
  String html_btn0 = "<p><div class=\"lay_i\"><input type=\"submit\" name=\"dsp\"  value=\"dsp_radio\" class=\"btn_g\"><input type=\"submit\" name=\"inet\" value=\"inet_radio\" class=\"btn\"></div></p>";
  String html_btn1 = "<p><div class=\"lay_i\"><input type=\"submit\" name=\"dsp\"  value=\"dsp_radio\" class=\"btn\"><input type=\"submit\" name=\"inet\" value=\"inet_radio\" class=\"btn_g\"></div></p>";
  String html_btn2 = "<p><div class=\"lay_i\"><input type=\"submit\" name=\"record\"  value=\"Recording_Function\" class=\"btn\"></div></p>";
  String html_p1; 
  char htstr[180];
  char stnno[4];    
  char tno[4];
  char stnmsg[12];
  bool responsed = false;
  stnmsg[0]=0;

  Serial.println("web received");
  if (inet_radio == 0) html_p1 = html_btn0; else html_p1 = html_btn1;
  html_p1 += html_btn2; // Recording function
  if (server.method() == HTTP_POST) { // submitted with string
    val1 = server.arg("daysced");
    val2 = server.arg("vup");
    val3 = server.arg("vdown");
    val4 = server.arg("stnup");
    val5 = server.arg("stndown");
    val6 = server.arg("sleep");
    val7 = server.arg("stoken");
    val8 = server.arg("stnset");
    val9 = server.arg("pwonoff");
    val10 = server.arg("dsp");
    val11 = server.arg("inet");
    val12 = server.arg("record");
    if (val7.length() != 0) { // server token
      Serial.print("stoken:");
      String s_stoken = server.arg("stoken");
      int t_stoken = s_stoken.toInt();
      Serial.println(s_stoken);
      msg = "stoken:" + s_stoken;
      if (stoken > t_stoken) {
        Serial.println("redirect");
        msg = "Post converted to Get";
        responsed = true;
        server.send(307, "text/html", "<!DOCTYPE html><html><head><meta charset=\"utf-8\"><meta http-equiv=\"refresh\" content=\"0;url=/\"></head></html>");
      }
    }
    if (!responsed){
      ////
      if (val10.length() != 0) {
        Serial.println("dsp");
        if (mode_chg_ok && (inet_radio == 1)) {
          // inet --> dsp
          html_p1 = html_btn0;
          html_p1 += html_btn2;  // rec func
          mode_chg_ok = false;
          mode_chg_req = true;
        }
        msg = "dsp";
      }
      else if (val11.length() != 0) {
        Serial.println("inet");
        if (mode_chg_ok && (inet_radio == 0)) {
          // dsp --> inet
          html_p1 = html_btn1;
          html_p1 += html_btn2; // rec func
          mode_chg_ok = false;
          mode_chg_req = true;
        }
        /*WiFiClient client = server.client(); 
        client.println("HTTP/1.1 307 Temporary Redirect");
        client.println("Location: /ir");
        client.println("Connection: Close");
        client.print("<HEAD>");
        client.print("<meta http-equiv=\"refresh\" content=\"0;url=/ir\">");
        client.print("</head>");
        client.println();
        client.stop();*/
        msg = "inet";
      } else if (val12.length() != 0) {
        Serial.println("redirect to /rec");
        msg = "Post converted to Get";
        responsed = true;
        server.send(307, "text/html", "<!DOCTYPE html><html><head><meta charset=\"utf-8\"><meta http-equiv=\"refresh\" content=\"0;url=/rec\"></head></html>");
      } else if (val8.length() != 0) {
        Serial.println("station set");
        if (val8 == "set") {
          Serial.print("Set_staion: ");
          // Get input station setting
          String s_stnurl = server.arg("stnurl");
          String s_stnname = server.arg("stnname");
          String s_stnno = server.arg("stnno");
          Serial.print(s_stnno); Serial.print(" "); Serial.print(s_stnname); Serial.print(" ");
          Serial.println(s_stnurl);
          // Set current
          strcpy(stnurl,s_stnurl.c_str());
          strcpy(stnname,s_stnname.c_str());
          station = s_stnno.toInt() - 1;
          if (station < max_station && station >= 0) {
            //bool conn_ok = false; // ver 0.62
            i2s_start(I2S_NUM_0); // start I2S of audio 
            conn_ok=audio.connecttohost(s_stnurl.c_str()); // change station
            if (conn_ok) {
                Serial.println("Set new staion OK");
                msg = "Set new staion OK";
            } else {
                Serial.println("Set new staion failed");
                i2s_stop(I2S_NUM_0); // stop I2S of audio because of noisy
                strcpy(stnmsg,"Err!");
                msg = "Set new staion error";
            }
          } else msg = "illegal station number";
        } else if (val8 == "save") {
          String s_stnurl = server.arg("stnurl");
          String s_stnname = server.arg("stnname");
          String s_stnno = server.arg("stnno");
          int stnno = s_stnno.toInt();
          Serial.print(s_stnno); Serial.print(" "); Serial.print(s_stnname); Serial.print(" ");
          Serial.println(s_stnurl);
          if (stnno - 1 <= max_station && stnno -1 >= 0) {
            char tstr[166];
            sprintf(tstr,"%s%d","st", stnno);
            preferences.putString(tstr,s_stnurl);
            sprintf(tstr,"%s%d","nm", stnno);
            preferences.putString(tstr,s_stnname);
            msg = "new station saved";
          } else msg = "illegal station number";
        }
        else msg = "ignore it";
      }
      else if (val2.length() != 0) {
        Serial.println("vup");
        b_srv=0; 
        msg = "control vup";
      }
      else if (val3.length() != 0) {
        Serial.println("vdown");
        a_srv=0; 
        msg = "control vdown";
      }
      else if (val4.length() != 0) {
        Serial.println("stnup");
        station_setting();
        msg = "control stnup";
      }
      else if (val5.length() != 0) {
        Serial.println("stndown");
        station_setting();
        msg = "control stndown";
      }
      else if (val6.length() != 0) {
        Serial.println("sleep");
        //sleep_setting();
        s_srv = 0; // ver o.60
        msg = "control sleep";
      }
      else if (val9.length() != 0) {
        Serial.println("pwonoff");
        if (inet_radio==0 || inet_radio==1) { // dsp radio, or inet (changed)
          power_onoff_setting(); 
          msg = "control pwonoff";
        } else msg = "ignore it";
      }
      else if (val1.length() != 0) {
        Serial.println("daysched");
        if (inet_radio>=0) { // dsp radio or inet radio
          int rc = setWeeksced(val1); 
          //msg = "control daysched";
        } else msg = "ignore it";
      }
      else {
        // nop        
      } 
    }
  } 
  if (!responsed) {
    html = "<!DOCTYPE html><html><head><meta charset=\"utf-8\"><title>DSP Inet Radio</title>";
    html +="</head><body><form action=\"\" method=\"post\">";
    html += "<p><h2>DSP Radio & Internet Radio</h2></p>";
    html += html_p1;
    html += "<style>.lay_i input:first-of-type{margin-right: 20px;}</style>";
    html += "<style>.btn {width: 300px; padding: 10px; box-sizing: border-box; border: 1px solid #68779a; background: #cbe8fa; cursor: pointer;}</style>";
    html += "<style>.btn_y {width: 300px; padding: 10px; box-sizing: border-box; border: 1px solid #68779a; background: #ffff8a; cursor: pointer;}</style>";
    html += "<style>.btn_g {width: 300px; padding: 10px; box-sizing: border-box; border: 1px solid #68779a; background: #99ff99; cursor: pointer;}</style>";
    //html += "<script>function stokenupd() {const hdtoken = document.getElementById('stoken'); hdtoken.value += 1;}</script>";
    html += "<p>Control Functions</p>";
    html += "<p><div class=\"lay_i\"><input type=\"submit\" name=\"vup\"  value=\"volume up\" class=\"btn\"><input type=\"submit\" name=\"vdown\" value=\"volume down\" class=\"btn\"></div></p>";
    html += "<p><div class=\"lay_i\"><input type=\"submit\" name=\"stnup\"  value=\"station change\" class=\"btn\"></div></p>";
    html += "<p><div class=\"lay_i\"><input type=\"submit\" name=\"pwonoff\"  value=\"dsp_radio pwr_on_off\" class=\"btn_y\">";
    html += "<input type=\"submit\" name=\"sleep\"  value=\"inet_radio sleep\" class=\"btn_y\"></div></p>";
    html += "<p>Volume: ";
    if (inet_radio==1) sprintf(tno,"%d",volume); else sprintf(tno,"%d",vol_r);
    html += tno;
    html += "</p>";
    html += "<p>Response: " + msg + "</p>";
    html += "<p><h3>Internet Radio Info</h3></p>";
    html_p1 = String(stnname);
    html += "<p>Now playing: " + html_p1 + " (" + stnurl +  ") "  + "</p>";
    html_p1 = String(titlebuf);
    html += "<p>" + html_p1 + "</p>";
    html += "<p>Station List: </p>";
    html += "<p><ol>";
    for (int j=0; j < max_station; j++) {
      html += "<li>";
      sprintf(htstr, "%.10s%s%s", station_name[j],  " - ",  station_url[j]);
      html +=  htstr;
      html += "</li>";
    }
    html += "</ol></p>";
    html += "<p></p><p>Preference Settings</p>";
    html += "<table style=\”border:none;\”><tr>";
    html += "<td>Station URL:</td>";
    if (strlen(stnmsg)==0) {
      html += "<td></td>";
    } else {
      html += "<td><font color=\"#ff4500\">Err!</font></td>";
    }
    html += "<td><input type=\"text\" name=\"stnno\" size=\"3\" maxlength=\"2\" value=\"";
    html += station + 1;
    html += "\"></td><td>";
    html += "<td><input type=\"text\" name=\"stnname\" size=\"18\" maxlength=\"23\" value=\"";
    html += stnname;
    html += "\"></td><td>";
    html += "<input type=\"text\" name=\"stnurl\" size=\"64\" maxlength=\"127\" value=\"";
    html += stnurl;
    html += "\">";
    html += "<button type=\"submit\" name=\"stnset\" value=\"set\">TEST SET</button>";
    html += "<button type=\"submit\" name=\"stnset\" value=\"save\">SAVE</button>";
    html += "</td></tr></table>";
    html += "<input type=\"hidden\" name=\"stoken\" value=\"";
    stoken += 1;
    html += stoken;
    html += "\">"; 
    //
    html += "<p><h3>Schedule Setting</h3></p>";
    html += "<p>Select a day of the week, change it, then submit.</p>";
    html += "<p>";
    html += "<input type=\"text\" id=\"daysced\" name=\"daysced\" size=\"120\" value=\"\">";
    html += "</p><p><input type=\"submit\" value=\"submit\" class=\"btn\"></p></form>";
    html += "<p>Response: " + msg + "</p>";
    html += "<p>Arguments of enrty: Start time(hour:min),Station(See below),Duration(min),Volume,Pweroff</p>";
    html += "<p>Station List: 0=" + stnName[0] + ",1=" + stnName[1] + ",2=" + stnName[2] + ",3=" + stnName[3] + ",4=" + stnName[4];
    html += ",5=" + stnName[5] + ",6=" + stnName[6] +  ",7=" + stnName[7] + ",above 50 = inet_radio</p>";
    html += "<script>";
//    html += "let entity = [[[390,1,59,4,1],[540,6,59,2,0],[600,0,59,2,0],[660,3,119,2,0],[780,1,59,2,0],[840,0,59,2,0],[900,1,59,2,0],[1140,3,119,2,0],[1410,0,29,2,1]],";
//    html += "[[390,1,59,4,1],[540,6,59,2,0],[600,0,59,2,0],[660,3,119,2,0],[780,1,59,2,0],[840,0,59,2,0],[900,1,59,2,0],[1140,3,119,2,0],[1410,0,29,2,1]]]";
    html += "let entity = [";
    for (int i = 0; i < 7; i++){
      html += "[";
      for(int j = 0; j <= MAXSCEDIDX; j++) {
        sprintf(htstr,"['%d:%02d',%d,%d,%d,%d]",entity[i][j].stime / 60,entity[i][j].stime % 60,entity[i][j].fidx,entity[i][j].duration,entity[i][j].volstep,entity[i][j].poweroff);
        html += htstr;
        if (j != MAXSCEDIDX) html += ",";
      }
      html += "]";
      if (i != 6) html += ",";
    }
    html += "];";
    html += "let week = [\"Sun\",\"Mon\",\"Tue\",\"Wed\",\"Thu\",\"Fri\",\"Sat\"];";
    html += "document.write('<table id=\"tbl\" border=\"1\" style=\"border-collapse: collapse\">');";
    html += "for (let i = 0; i < 7; i++){";
    html += "let wstr ='';";
    html += "wstr ='<tr>' + '<td>' + '<input type=\"radio\" name=\"week\" value=\"\" onclick=\"setinput(' + i + ')\">' + '</td>' + '<td>' + week[i] + '</td>';";
    html += "document.write(wstr);";
    html += "for (let j = 0; j < 9; j++){";
    html += "document.write('<td>');";
    html += "document.write(entity[i][j]);";
    html += "document.write('</td>');}";
    html += "document.write('</tr>');";
    html += "}";
    html += "document.write('</table>');";
    html += "function setinput(trnum) {";
    html += "var input = document.getElementById(\"daysced\");";
    html += "var table = document.getElementById(\"tbl\");";
    html += "var cells = table.rows[trnum].cells;";
    html += "let istr = '';";
    html += "for (let j = 1; j <= 10; j++){";
    html += "istr = istr + cells[j].innerText + ';';";
    html += "}";
    html += "input.value = istr;";
    html += "}";
    html += "</script>";
    //
    html += "</form></p></body>";
    html += "</html>";
    server.send(200, "text/html", html);
    Serial.println("web send response");
  }
//}
}
void handleRec(void)
{
  String html;
  String val1, val2, val3, val4, val5, val6, val7, val8, val9, val10, val11, val12, val13;
  char ts[40];
  //const int cbl = 30; // circular buffer length
  //String cb[cbl];     // circular buffer to store SD file name
  uint32_t total_file_size = 0;
  bool no_refresh = false;

  bool responsed = false;
  String html_btn0 = "<p><div class=\"lay_i\"><input type=\"submit\" name=\"record_start\"  value=\"Start_DSP_Recording\" class=\"btn\"><div class=\"triangle-right\"></div><input type=\"submit\" name=\"rec_stop\" value=\"Stop_DSP_Recording\" class=\"btn\"></div></p>";
  String html_btn1 = "<p><div class=\"lay_i\"><input type=\"submit\" name=\"record_start\"  value=\"Start_DSP_Recording\" class=\"btn_g\"><div class=\"triangle-right\"></div><input type=\"submit\" name=\"rec_stop\" value=\"Stop_DSP_Recording\" class=\"btn\"></div></p>";
  String html_btn3 = "<p><div class=\"lay_i\"><input type=\"submit\" name=\"record_start\"  value=\"DSP_Recording_in_Progress\" class=\"btn_r\"><div class=\"triangle-right\"></div><input type=\"submit\" name=\"rec_stop\" value=\"Stop_DSP_recording\" class=\"btn\"></div></p>";
  String html_btn4 = "<p><div class=\"lay_i\"><input type=\"submit\" name=\"play_stop\"  value=\"Stop_Play\" class=\"btn\"><input type=\"submit\" name=\"forward_5min\"  value=\"Forward_5_min\" class=\"btn\"></div></p>";;
  String html_btn5 = "<p><div class=\"lay_i\"><input type=\"submit\" name=\"play_stop\"  value=\"Stop_Play\" class=\"btn_y\"><input type=\"submit\" name=\"forward_5min\"  value=\"Forward_5_min\" class=\"btn\"></div></p>";;
  String html_btn6 = "<p><div class=\"lay_i\"><input type=\"submit\" name=\"init_camera\"  value=\"Init_Camera\" class=\"btn\"><div class=\"triangle-right\"></div><input type=\"submit\" name=\"shoot_camera\" value=\"Shoot_Camera\" class=\"btn\"><input type=\"submit\" name=\"stream_camera\" value=\"Stream_Camera\" class=\"btn\"></div></p>";
  String html_btn7 = "<p><div class=\"lay_i\"><input type=\"submit\" name=\"init_camera\"  value=\"Init_Camera\" class=\"btn_r\"><div class=\"triangle-right\"></div><input type=\"submit\" name=\"shoot_camera\" value=\"Shoot_Camera\" class=\"btn\"><input type=\"submit\" name=\"stream_camera\" value=\"Stream_Camera\" class=\"btn\"></div></p>";
  String html_btn8 = "<p><div class=\"lay_i\"><input type=\"submit\" name=\"mic_rec_start\"  value=\"Start_MIC_Recording\" class=\"btn\"><div class=\"triangle-right\"></div><input type=\"submit\" name=\"mic_rec_stop\" value=\"Stop_MIC_Recording\" class=\"btn\"></div></p>";
  String html_btn9 = "<p><div class=\"lay_i\"><input type=\"submit\" name=\"mic_rec_start\"  value=\"Start_MIC_Recording\" class=\"btn_g\"><div class=\"triangle-right\"></div><input type=\"submit\" name=\"mic_rec_stop\" value=\"Stop_MIC_Recording\" class=\"btn\"></div></p>";
  String html_btn10 = "<p><div class=\"lay_i\"><input type=\"submit\" name=\"mic_rec_start\"  value=\"MIC_Recording_in_Progress\" class=\"btn_r\"><div class=\"triangle-right\"></div><input type=\"submit\" name=\"mic_rec_stop\" value=\"Stop_MIC_recording\" class=\"btn\"></div></p>";
  String html_p1, html_p2, html_p3; 
  html_btn6 = "";  // to stop  display
  html_btn7 = "";  // to stop  display
  html_btn8 = "";  // to stop  display
  html_btn9 = "";  // to stop  display
  html_btn10 = "";  // to stop  display
  html_p1 = html_btn0;
  html_p2 = html_btn4;
  html_p3 = html_btn8;
  Serial.println("web received(Rec)");
  val2 = server.arg("rec_stop");
  msg = "";
  if ((server.method() == HTTP_POST) && (!REC_on || val2.length() != 0)) { // submitted with string
    val1 = server.arg("record_start");
    val2 = server.arg("rec_stop");
    val3 = server.arg("play_stop");
    val4 = server.arg("stoken");
    val5 = server.arg("format");
    val6 = server.arg("status");
    val7 = server.arg("forward_5min");
    val8 = server.arg("init_camera");
    val9 = server.arg("shoot_camera");
    val10 = server.arg("stream_camera");
    val11 = server.arg("mic_rec_start");
    val12 = server.arg("mic_rec_stop");
    val13 = server.arg("rectime");
    if (val4.length() != 0) { // server token
      Serial.print("stoken:");
      String s_stoken = server.arg("stoken");
      int t_stoken = s_stoken.toInt();
      Serial.println(s_stoken);
      msg = "stoken:" + s_stoken;
      if (stoken > t_stoken) {
        Serial.println("redirect-rec");
        msg = "Post converted to Get";
        responsed = true;
        server.send(303, "text/html", "<!DOCTYPE html><html><head><meta charset=\"utf-8\"><meta http-equiv=\"refresh\" content=\"0;url=/rec\"></head></html>");
        no_refresh = true;
      }
    } else{ ;
    }
  }
  if (!responsed) {
    if (val2.length() != 0) { // rec stop request   
      Serial.println("rec stop");
      if (REC_on) { // Is recoding active ?
        pofftm_h = (d_hour * 60 + d_min + 1) / 60;
        pofftm_m = (d_hour * 60 + d_min + 1) % 60;
        sprintf(ts,"%02d:%02d %s",pofftm_h,pofftm_m,"poff or recording stop scheduled");
        Serial.println(ts); 
        msg = "control: rec stop scheduled, wait a few minutes";
        REC_on_no_poff = true;
      } else {
        msg = "control: rec stop ignored";        
      }
      no_refresh = true;
    } else 
    if (val3.length() != 0) { // play stop req   
      Serial.println("play_stop");
      msg = "control: play stop";
      stop_read = true;    // Stop read
      no_refresh = true;
    } else
    if (val1.length() != 0) {
        Serial.println("record");
        //if (!REC_on && stop_read) {   #####
        if (!REC_on) {
          int rectm = val13.toInt(); // rectime
          if (rectm <= MAX_RECORD_TIME_LIMIT) {
            rectime = rectm;  // ok set it 
            initiate_rec(0);
          } else {
            // too long
            msg = "control: too long time";
            Serial.println("record ignored, too long time");
          }
        } else {
          msg = "control: record ignored";
          Serial.println("record ignored");
        }
    } else 
    if (val6.length() != 0) {
      msg = "now recording is active";
      Serial.println("record ignored(active)");
      no_refresh = true;
    } else
    if (val5.length() != 0){
      msg = "invalid format";
      no_refresh = true;
    }  else
    if (val7.length() != 0) { // forward
      Serial.println("web: forward.");
      msg = "control: forward";
      no_refresh = true;
      if (in_play) { // is playing ?  #### stop_read -> in_play
        uint32_t f_size = audio.getFileSize(); // in bytes
        uint32_t f_pos = audio.getFilePos();
        f_pos += record_size * 5 * 60 ; // 5 min
        if (f_pos < f_size)  {
           audio.setFilePos(f_pos);
           Serial.printf("forward to: %d\n", f_pos);
        } else Serial.println("web: forward over file size.");
      }
    } else
    if (val9.length() != 0) { // shoot
      Serial.println("web: shoot.");
      msg = "control: shoot";
      no_refresh = true;
      if (camera_ok && !REC_on && !MIC_rec_on) {
        msg = "control: shoot";
        shoot_s = true;
      } else {
        msg = "err: camera not initialized or SD busy";
      }
    } else
    if (val8.length() != 0) { // init camera
      Serial.println("web: init camera.");
      msg = "control: init camera";
      no_refresh = true;
      if (!camera_ok) {
        //bool init_ok =set_camera(FRAMESIZE_XGA);  ####
        bool init_ok = false;
        if (init_ok) {
          msg = "control: init camera ok";
          Serial.println("web: init camera ok.");
        }    
      }
    } else
    if (val10.length() != 0) { // stream
      //responsed = true;
      Serial.println("web: stream.");
      msg = "control: stream";
      no_refresh = true;
      if (camera_ok) {
        Serial.println("redirect-stream");
        msg = "Stream req redirect";
        server.send(303, "text/html", "<!DOCTYPE html><html><head><meta charset=\"utf-8\"><meta http-equiv=\"refresh\" content=\"0;url=/stream\"></head></html>");
        responsed = true;
      } else {
        msg = "err: camera not initialized";
      }
    } else
    if(val11 != 0) { // MIC rec start
      Serial.println("web: MIC rec start.");
      msg = "control: MIC rec start";
        if (!REC_on && !in_play && !MIC_rec_on) {  // #### stop_rread -> in_play
          total_recorded_size = 0;
          last_blk = 0;
          estimated_recorded_size = MAX_RECORD_TIME * SAMPLE_RATE * SAMPLE_BITS * 60 / 8;  // MAX_RECORD_TIME min
          pofftm_h = (d_hour * 60 + d_min + MAX_RECORD_TIME) / 60;  // auto stop after MAX_RECORD_TIME min
          pofftm_m = (d_hour * 60 + d_min + MAX_RECORD_TIME) % 60;
          sprintf(ts,"%02d:%02d %s",pofftm_h,pofftm_m,"poff or recording stop scheduled");
          Serial.println(ts); 
          REC_on_no_poff = true;

          esp_err_t err = i2s_install("MIC");
          //I2S.setAllPins(-1, 42, 41, -1, -1); // XIAO ESP32S3 SENSE PIN USAGE
          i2s_setpin("MIC");         
          if (err != ESP_OK) {
            Serial.println("Failed to initialize I2S!");
            I2S_err = true;
          } else 
            Serial.println("mic i2s ok"); 
          if(!I2S_err && !SD.begin(SD_CS, SPI, REC_FREQUENCY, "/sd")){ // SD mount
            Serial.println("Failed to mount SD Card!");
            I2S_err = true;
            i2s_driver_uninstall(I2S_NUM_1);  // no more DSP I2S now
          }
          if (!I2S_err) {
            MIC_rec_on = true; // start REC ok
            REC_on_no_poff = true;
            msg = "control: record";
            radio.powerDown();
            dsp_active = false;
            Serial.println("mic rec on"); 
          } else {
            msg = "control: record err";
          }
        } else {
          msg = "control: record ignored";
        }
            
    } else
    if (val12 != 0) { // MIC rec stop
      Serial.println("web: MIC rec stop.");
      msg = "control: MIC rec stop";
      if (MIC_rec_on) { // Is recoding active ?
        pofftm_h = (d_hour * 60 + d_min + 1) / 60;
        pofftm_m = (d_hour * 60 + d_min + 1) % 60;
        sprintf(ts,"%02d:%02d %s",pofftm_h,pofftm_m,"poff or recording stop scheduled");
        Serial.println(ts); 
        msg = "control: rec stop scheduled, wait a few minutes";
        REC_on_no_poff = true;
      } else {
        msg = "control: rec stop ignored";        
      }
      no_refresh = true;
    }
    else {
        //nop
    }     
  }
  if (REC_on_no_poff) {
    if (MIC_rec_on) {html_p1 = html_btn0 + html_btn10;} else {html_p1 = html_btn3 + html_btn8;}
  } else {
    if (MIC_rec_on) {html_p1 = html_btn0 + html_btn10;} else {html_p1 = html_btn0 + html_btn8;}
  }
  if (camera_ok) html_p1 += html_btn7; else html_p1 += html_btn6;
  if (!in_play) html_p2 = html_btn4; else html_p2 = html_btn5;  // #### stop_read -> in_play
  if (!responsed) {
    html = "<!DOCTYPE html><html><head><meta charset=\"utf-8\"><title>DSP Radio Recording</title>";
    html += "</head><body><p><h3>Recording and Playing&nbsp;&nbsp;(experimental)</h3>&nbsp;&nbsp;<a href=\"/\">Back</a></p><form action=\"\" method=\"post\">";
    //html += "<style>.lay_i input:first-of-type{margin-right: 20px;}</style>";
    html += "<style>.lay_i input {margin-right: 20px;}</style>";
    html += "<style>.btn {width: 300px; padding: 10px; box-sizing: border-box; border: 1px solid #68779a; background: #cbe8fa; cursor: pointer;}</style>";
    html += "<style>.btn_y {width: 300px; padding: 10px; box-sizing: border-box; border: 1px solid #68779a; background: #ffff8a; cursor: pointer;}</style>";
    html += "<style>.btn_g {width: 300px; padding: 10px; box-sizing: border-box; border: 1px solid #68779a; background: #99ff99; cursor: pointer;}</style>";
    html += "<style>.btn_r {width: 300px; padding: 10px; box-sizing: border-box; border: 1px solid #68779a; background: #FFA5A5; cursor: pointer;}</style>";
    html += "<style>.triangle-right {display: inline-block; border-style: solid; border-width: 8px 0 8px 18px; border-color: transparent transparent transparent #000; margin-right: 16px; position:relative; top: 4px;}</style>";
    html += "<script> async function deleteFile(fname) { ";
    html += "if (fname.length === 0) { alert('No file!'); return; }";
    html += "if (!confirm(fname + ', ' +'Delete this file?')) return;";
    html += "try { const response = await fetch('/wavf?fname=' + fname + '&delete=yes' );";
    html += " if (response.ok) { alert('Deleted ');  } else { alert('Error '); } ";
    html += " } catch (error) {  alert('Fail: ' + error.message); } ";
    html += "}";
    html += " async function renameFile(fname) { ";
    html += "var input = prompt('Rename: ' + fname + ' ?'); ";
    html += "if (input !== null) { ";
    html += "try { const response = await fetch('/wavf?fname=' + fname + '&rename=' + input);";
    html += " if (response.ok) { alert('Renamed ');  } else { alert('Error '); } ";
    html += " } catch (error) {  alert('Fail: ' + error.message); } ";
    html += " }";
    html += "}";
    html += "</script>";
    html += html_p1;
    html += "<table style=\”border:none;\”><tr>";
    html += "<td>Recording Time:</td>";
    html += "<td><input type=\"text\" name=\"rectime\" size=\"3\" maxlength=\"3\" value=\"";
    html += String(rectime);
    html += "\"></td><td>(min)</td></tr></table>";
    html += "<input type=\"hidden\" name=\"stoken\" value=\"";
    stoken += 1;
    html += stoken;
    html += "\">"; 
    html += "<p><form action=\"/wavf\" method=\"post\">";
    //html += "<p><div class=\"lay_i\"><input type=\"submit\" name=\"play_stop\"  value=\"STOP PLAY\" class=\"btn\"></div></p>";
    html += html_p2;
    html += "</form></p>";
    html += "<p>Response: " + msg + "</p>";
    html += "<p><h3>Files:</h3>&nbsp;&nbsp;Press 'file name link' to play</p>";
    if(!REC_on && !MIC_rec_on) {
      if (!SD.begin(SD_CS, SPI, REC_FREQUENCY, "/sd")){ // SD mount
         Serial.println("Failed to mount SD Card!");
         html += "<p>Failed to mount SD Card!</p>";
      } else {
        File root = SD.open("/");
        uint64_t tb = SD.totalBytes();
        uint32_t tbi = tb/(1024*1024); // MB
        uint64_t ub = SD.usedBytes();
        uint32_t ubi = ub/(1024*1024); // MB
        String sdinfo_tb(tbi);
        String sdinfo_ub(ubi);
        String sdinfo_ra( ( (tbi-ubi) * 1024) / ( ( ( (SAMPLE_RATE * SAMPLE_BITS * CHAN_NUM / 8 ) /1024  ) * 60) )  );
        html += "<p>&nbsp;&nbsp;total size(MB):&nbsp;&nbsp;" + sdinfo_tb; 
        html += "&nbsp;&nbsp;used size(MB):&nbsp;&nbsp;" + sdinfo_ub;
        html += "&nbsp;&nbsp; remaining amount(minutes):&nbsp;&nbsp;" + sdinfo_ra + "</p>";
        bool isDir = false;
        String fname;
        int cbix = 0; // circular buffer index
        int fcnt = 0;
        while (true) {
          String filename = root.getNextFileName(&isDir);
          if (filename == "") break; // nomore files
          if (!isDir) { // not directory
            if (filename.length() > 4) {
              fname = filename.substring(1,28);
              cb[cbix] = fname;
              cbix++;
              fcnt++;
              if (cbix >= cbl) cbix = 0; // reset index
            }
          }
        }
        String sdinfo_fi(fcnt);
        String sdinfo_lf(cbl);
        Serial.printf("filecount: %d\n", fcnt);
        if (fcnt > 0) { //Are there Files?
          html += "<p>&nbsp;&nbsp;total " + sdinfo_fi + " files&nbsp;&nbsp;"; 
          html += "&nbsp;&nbsp;(max listed " + sdinfo_lf + " files)<p>";
          int rdix = 0;
          for (int i = 0; i < cbl && i < fcnt; i++) {
            if (rdix >=  cbl) rdix = 0;
            //if (cb[rdix].length()==25 && cb[rdix].substring(22,25)=="jpg") 
            if (cb[rdix].endsWith(".jpg") || cb[rdix].endsWith(".JPG") )
              html += "<p><a href='/jpgf?fname=" + cb[rdix] + "'>" + cb[rdix] + "</a>";
            else 
              html += "<p><a href='/wavf?fname=" + cb[rdix] + "'>" + cb[rdix] + "</a>";
            String ts = "/" + cb[rdix];
            char tstr[32] = {'\n'};
            uint32_t fsize;
            File wfile;
            ts.toCharArray(tstr, 29);
            if (!no_refresh) { // aboid to read from SD
              wfile = SD.open((char *)tstr, FILE_READ);
              fsize = wfile.size();
              cb_sz[rdix] = fsize;  // save it
            } else {
              fsize = cb_sz[rdix];  // restore from memory
            }
            total_file_size = total_file_size + fsize/1024;
            int fminutes = fsize / (SAMPLE_RATE * SAMPLE_BITS * CHAN_NUM / 8) / 60 + 1;
            ts = String(fsize/1024);
            html += "&nbsp;&nbsp;&nbsp;&nbsp;file size(KB):&nbsp;&nbsp;" + ts + "&nbsp;&nbsp;";
            ts = String(fminutes);
            html += "&nbsp;&nbsp;length(minutes):&nbsp;&nbsp;" + ts + "&nbsp;&nbsp;";
            html += "<button type=\"button\" onclick=\"deleteFile('" + cb[rdix] + "')\">DELETE</button>" + "&nbsp;&nbsp;";
            html += "<button type=\"button\" onclick=\"renameFile('" + cb[rdix] + "')\">RENAME</button>" + "</p>";
            if (!no_refresh) wfile.close();
            rdix++;
          }
        } else {
          html += "<p>no files.</p>";
        }   
        root.close();
      }
    } else {
      html += "Recording in progress. If you want to stop recording, press Stop_recording button,<br>";
      html += "and wait a few minutes.<br>";
    }
    html += "</body>";
    html += "</html>";
    Serial.printf("Total file size(KB): %d\n", total_file_size);
    server.send(200, "text/html", html);
  }
  Serial.println("web send response(Rec)");
}
void handleWavf() {
  String html;
  String val1;
  String val2;
  String val3;
  String val4;
  bool file_del = false;
  bool file_rename = false;
  bool responsed = false;
  char tstr[101] = {'/n'};
  //SdFat sdf;    // for rename (ver 0.65)
  SdFile file;
  Serial.println("Play start");
  val1 = server.arg(0);
  val2 = server.arg(1); // delete or rename 
  val3 = server.arg("delete");
  val4 = server.arg("rename");
 if ((val3.length() != 0) && (val2=="yes")) {
    file_del = true;
    Serial.println("File delete");
  } else if ((val4.length() != 0) && (val2.length() != 0)) {
    val4 = "/" + val4;
    val4.toCharArray(tstr, val4.length() + 1);
    file_rename = true;
    Serial.printf("File rename: %s\n", tstr);
  } 
  WiFiClient client = server.client();
  if (!client.connected()) {
    Serial.println("Client disconnected");
    return;
  }
  //char tstr[101] = {'/n'};
  //rstr[0] = '/n';
  val1 = "/" + val1;
  val1.toCharArray(rstr, val1.length() + 1);
  Serial.println(rstr);
  if (!REC_on && !MIC_rec_on) {
    SD.begin(SD_CS, SPI, REC_FREQUENCY, "/sd");
    WavFile = SD.open((char *)rstr, FILE_READ);  // Open the wav file
    if (WavFile == false) {
      Serial.println("Could not open wavfile");
      if (file_del) {
        responsed = true;
        server.send(404, "text/html", "<!DOCTYPE html><html><head><meta charset=\"utf-8\"><meta http-equiv=\"refresh\" content=\"0;url=/rec?format=no_file\"></head></html>");
      }
    }
    else {
      if (memcmp(rstr, "/inet_", 6) == 0) { // inet url 
        char inet_url[100] = {'\n'};
        int p = 0;
        while ( WavFile.available() && (p <= 100) )
        {
          char bc[2] = {'\n'};
          WavFile.read((uint8_t*)bc, 1);
          if( (bc[0] == 0x0d) || (bc[0] == 0x0a) || (bc[0] == 0x20) ) break;
          inet_url[p] = bc[0];
          p++;
        }
        if ( (p > 10) && (p <= 99) ) {  // may be url
          Serial.printf("inet detected: %s\n", inet_url);
          i2s_start(I2S_NUM_0); // start I2S of audio 
          bool conn_ok = audio.connecttohost(inet_url);
          if (conn_ok) {
            pcf.digitalWrite(6, LOW);
            stop_read = false; // ok, start it
            WAV_read = true;
            in_play = true; // now playing
            inet_radio = 1;
            radio.powerDown();
            dsp_active = false;
          } else {
            i2s_stop(I2S_NUM_0); // stop I2S of audio because of noisy
            stop_read = true; // cannot connect
          }
        }
        WavFile.close();
      }   else   {
        if (file_del) {
          WavFile.close();
          SD.remove(val1); // delete file
          Serial.println("File deleted");
          responsed = true;
          server.send(200, "text/html", "<!DOCTYPE html><html><head><meta charset=\"utf-8\"></head><body>Deleted.</body></html>");
        } else if (file_rename) {
          WavFile.close();
          sdf.begin(SD_CS, REC_FREQUENCY);
          if (sdf.rename(rstr, tstr)) { // rename file
            Serial.printf("File renamed: %s to %s\n", rstr, tstr);
            server.send(200, "text/html", "<!DOCTYPE html><html><head><meta charset=\"utf-8\"></head><body>Renamed.</body></html>");
          } else {
            Serial.printf("File rename error: %s to %s\n", rstr, tstr);
            server.send(400, "text/html", "<!DOCTYPE html><html><head><meta charset=\"utf-8\"></head><body>Rename error.</body></html>");
          }
          responsed = true;
        } else {
          WavFile.read((byte*)&WavHeader, 44);                    // Read  WAV header, first 44 bytes of the file.
          int rc = DumpWAVHeader(&WavHeader);                     // confirm  header data
          WavFile.close();  //
          if (rc <= 1) {  // wav or mp3 ?
            i2s_start(I2S_NUM_0); // start port of I2S Audio 
            delay(100);
            bool cc = audio.connecttoFS(SD, rstr); // play this file in the SD
            //bool  cc = false;
            if (cc) {
              pcf.digitalWrite(6, LOW);
              stop_read = false; // ok, start it
              WAV_read = true;
              in_play = true;
              if (inet_radio == 1) INET_in_play = true;
              inet_radio = 1;
              radio.powerDown();
              dsp_active = false;  
            } else {
              Serial.printf("connectFS fail");
              i2s_stop(I2S_NUM_0); // stop I2S of audio because of noisy
              stop_read = true; // stop it  
            }
          }
        }
      }
    }
  } else {
    msg = "Now recording is active";
    Serial.println("redirect-invalid-status");
    responsed = true;
    server.send(303, "text/html", "<!DOCTYPE html><html><head><meta charset=\"utf-8\"><meta http-equiv=\"refresh\" content=\"0;url=/rec?status=invalid\"></head></html>");
  }
  if (!responsed) {
    if (!stop_read) {
      //server.send(200, "text/plain", "Ok Play start. To stop Play, press <a href=\"/rec\">backward</a>, then press Stop_Play button on the screen.");
      server.send(200, "text/html", "<!DOCTYPE html><html><head><meta charset=\"utf-8\"></head><body>Ok Play start. To stop Play, press &nbsp;<a href=\"/rec\">backward</a>, then press Stop_Play button on the screen.</body></html>");
      Serial.println("Play continue");
    } else {
      msg = "invalid format";
      Serial.println("redirect-invalid-format");
      // never use 301 redirect, it's parmanent. 
      server.send(303, "text/html", "<!DOCTYPE html><html><head><meta charset=\"utf-8\"><meta http-equiv=\"refresh\" content=\"0;url=/rec?format=invalid\"></head></html>");
    }
  }
}

void handleJpgf() {
  String val1;
  const char* JPG_HEADER = 
  "HTTP/1.1 200 OK\r\n"
  "Content-Type: image/jpeg\r\n"
  "Content-Length: %d\r\n"
  "\r\n";

  //byte readArray[40000]; // 40k  for SVGA
  Serial.println("Photo start");
  val1 = server.arg(0);
  val1 = "/" + val1;
  //Serial.println(val1);
  WiFiClient client = server.client();
  if (!client.connected()) {
    Serial.println("Client disconnected");
    return;
  }
  char tstr[32] = {'/n'};
  val1.toCharArray(tstr, 29);
  Serial.println(tstr);
  File jpgfile = SD.open((char *)tstr, FILE_READ);
  long fsize = 0;
  if (jpgfile) { // open ok
    fsize = jpgfile.size();
    Serial.printf("filesize: %d\n", fsize);
    long i = 0;
    if (fsize < 300000) { // SVGA:40000, XGA:300000
      while(jpgfile.available()) {
        byte rb = jpgfile.read();
        //readArray[i++] = rb;
        //(uint8_t *)(jpg_buffer + i) = rb;
        memset((uint8_t *)(jpg_buffer + i), rb ,1);
        i++;
      }
      Serial.printf("readsize: %d\n", i);
      if (i!=0) {
        client.printf(JPG_HEADER, i);
        int j = i / 1000;
        int k = i % 1000;
        int m = 0;
        for (int n = 0; n < j ; n++) {
          client.write((uint8_t *)jpg_buffer + m, 1000);
          m = m + 1000;
        }
        if (k > 0) client.write((uint8_t *)jpg_buffer + m, k);
        client.print("\r\n");
      } else {
        server.send(200, "text/plain", "file size = 0");
      }

    } else {
      Serial.println("File size is too big");
    }
    jpgfile.close();
  } else {
    Serial.println("file open err");
    server.send(200, "text/plain", "file open err");
  }
  Serial.println("Photo end");
}

void handleNotFound(void)
{
  server.send(404, "text/plain", "Not Found.");
}

void SDCardInit() {
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);  // SD card chips select, must use GPIO 21 (ESP323 sense)
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  SPI.setFrequency(REC_FREQUENCY);  // 10 - 24MHz
  if (!SD.begin(SD_CS, SPI, REC_FREQUENCY, "/sd")) {
    Serial.println("Error talking to SD card!");
  } else sd_ok = true;
}

void setup()
{
  //bool ble = false;
  struct tm tm_init;
  struct timeval tv = { 1710000000 + gmtOffset_sec , 0 };  // initial value is 2024/3/9 16:00 + 9:00
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  //pinMode(AOUT_SW, OUTPUT);
  //digitalWrite(AOUT_SW, HIGH);*
  Serial.begin(115200);
  SDCardInit();
  delay(50);
  
  // DSP I2S_1 RX
  gpio_set_direction((gpio_num_t)3, GPIO_MODE_INPUT); 
  gpio_set_direction((gpio_num_t)4, GPIO_MODE_INPUT);
  gpio_set_direction((gpio_num_t)43, GPIO_MODE_INPUT);
  // DAC I2S_0 TX
  gpio_set_direction((gpio_num_t)1, GPIO_MODE_OUTPUT);
  gpio_set_direction((gpio_num_t)2, GPIO_MODE_OUTPUT);
  gpio_set_direction((gpio_num_t)44, GPIO_MODE_OUTPUT);
  
    //pinMode(VOL_PIN1, INPUT_PULLUP);
    //pinMode(VOL_PIN2, INPUT_PULLUP);
    // Encoder pins
    //pinMode(ENCODER_PIN_A, INPUT_PULLUP);
    //pinMode(ENCODER_PIN_B, INPUT_PULLUP);

    //pinMode(INT_PIN, INPUT_PULLUP);
    pinMode(PCF_INT_PIN, INPUT_PULLUP);
    //pinMode(SLEEP, INPUT_PULLUP); // sleep after about 60 min when button was pressed
    //
    //attachInterrupt(INT_PIN,modechange_setting, FALLING); // to change mode inet/dsp
    //attachInterrupt(INT_PIN,station_setting, FALLING); // to change station
    attachInterrupt(PCF_INT_PIN,switch_setting, FALLING); // switches to operate
    //attachInterrupt(VOL_PIN1,volup_setting, FALLING); // to change volue up
    //attachInterrupt(VOL_PIN2,voldown_setting, FALLING); // to change volume down
    // Encoder interrupt
    //attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), rotaryEncoder, CHANGE);
    //attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), rotaryEncoder, CHANGE);
    //attachInterrupt(ENCODER_PIN_A, rotaryEncoder, CHANGE);
    //attachInterrupt(ENCODER_PIN_B, rotaryEncoder, CHANGE);

    //attachInterrupt(SLEEP,sleep_setting, FALLING); // to change sleep on/off
  Wire.setPins(PIN_SDA, PIN_SCL);  
  Wire.begin(); //
  // set dsp dac parameter  
  Wire.beginTransmission(0x11);
  Wire.write(0x04); // REG4
  Wire.write(0b10001000); // RDSIEN, De-emphasis 50μs
  Wire.write(0b01000000); // I2S Enabled
  Wire.endTransmission(); // stop transmitting
  delay(5);
  /*Wire.beginTransmission(0x011);
  Wire.write(0x05); // REG5 DAC volume
  Wire.write(0b10001000); // int mode,  seek th (default)
  Wire.write(0b10000001); // lna, volume = 1
  Wire.endTransmission(); // stop transmitting
  delay(5);*/
  Wire.beginTransmission(0x011);
  Wire.write(0x06); // REG6
  Wire.write(0b00000010); //  MASTER, DATA_SIGNED
  //Wire.write(0b00000000); //  MASTER, DATA_UNSIGNED #####2025/10/3
  //Wire.write(0b00010010); // SLAVE, DATA_SIGNED #####2025/10/3
  //Wire.write(0b10000000); // 48KBPS
  //Wire.write(0b01110000); // 44.1KBPS
  //Wire.write(0b01100000); // 32KBPS
  Wire.write(0b01010000); // 24KBPS
  //Wire.write(0b00000000); // 8KBPS
  //Wire.write(0b00110000); // 16KBPS
  Wire.endTransmission(); // stop transmitting
  delay(5);
  #ifdef _Adafruit_SSD1306_H_
    oled.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDRESS);
  #endif
  #ifdef _Adafruit_SH110X_H_
    oled.begin(OLED_I2C_ADDRESS, true);
  #endif
    oled.clearDisplay();
    //oled.setTextColor(SSD1306_WHITE);
    oled.setTextColor(1);
  // restore saved settings
    preferences.begin("inet_p", false);
    //preferences.clear(); // for debug
    String wfval = preferences.getString("WF0","");
    if (wfval != "") { // WiFi info
      String instr[4] = {"\n"};
      int ix = split(wfval,',',instr);
      instr[1].trim();
      instr[2].trim();
      ssid = instr[1];
      Serial.print("WiFi ssid : "); Serial.println(ssid);
      password = instr[2];
    }
    for (int i = 0; i < MAXSTNIDX; i++){
       String val1 = preferences.getString(stnStr[i],"");       
       stnList += val1.c_str();
       stnList += ";";   
       if (val1 != "") {
         Serial.println(val1);
         int rc = setStation(val1);
         if (rc != 0) Serial.println(msg);
       }
    }
    for (int i = 0; i < 7; i++){
       String val1 = preferences.getString(weekStr[i],"");       
       if (val1 != "") {
         //Serial.println(val1);
         int rc = setWeeksced(val1);
       }
    }
    for (int i = 0; i < max_station; i++){
       char tstr[166];
       String *tcp;
       sprintf(tstr,"%s%d","st",i+1);
       String val1 = preferences.getString(tstr,"");       
       if (val1 != "") {
         tcp = new String(val1.c_str());
         Serial.print(tcp->c_str()); Serial.print(",");        
         station_url[i] = tcp->c_str();
         sprintf(tstr,"%s%d","nm",i+1);
         val1 = preferences.getString(tstr,"");
         tcp = new String(val1.c_str());
         Serial.println(tcp->c_str());
         station_name[i] = tcp->c_str();
       }
    }
    volume = preferences.getInt("vol", -1);
    if (volume < 0) volume = 3;
    vol_r = preferences.getInt("volr", -1);
    if (vol_r < 0)  vol_r = 2;
    //lastvol = vol_r;
    //
    stnIdx = preferences.getInt("stix", -1);
    if (stnIdx < 0)  stnIdx = 3;
    lastfreq = stnFreq[stnIdx];
    laststnIdx = stnIdx;
    //
    station = preferences.getInt("stn", -1);
    if (station < 0) station = 0;
    //
  event = 0;
  pt2=millis();
    oled.setTextSize(2); // Draw 2X-scale text
    oled.setCursor(0, 0);
    oled.print("Inet Radio");
    oled.setCursor(0, 15);
    oled.print(VERSION_NR);
    oled.display();
    WiFi.mode(WIFI_STA);
    wifiMulti.addAP(ssid2.c_str(), password2.c_str());  // uncomment if you want second wifi access point
    //wifiMulti.addAP(ssid.c_str(), password.c_str());  
    wifiMulti.run();   // It may be connected to strong one
    delay(1000);  // Wait for Wifi ready
    while (!ble) {
      if(WiFi.status() == WL_CONNECTED){ WiFi_OK = true; break; }  // WiFi connect OK then next step
      Serial.println("WiFi Err");
      oled.setCursor(0, 30);
      oled.print("WiFi Err");
      oled.display();
      for (int l = 0; l < 200; l++) {
        if (s == 0) { // sleep sw push ?
          settimeofday(&tv, NULL); // Set temp time
          s = 1; // reset
          ble = bleservice();
          if (ble) { // service is initialized
            oled.setCursor(0, 45);
            oled.print("BLE mode");
            oled.display();
            delay(2000);
            break;
          } else {
            oled.setCursor(0, 45);
            oled.print("BLE ERR");
            oled.display();
            delay(2000);
          } 
        }
        else delay(1000); // 200 * 1000         
      }
      if (!ble) {
        WiFi.disconnect(true);
        delay(3000);
        wifiMulti.run();
        delay(1000);  // Wait for Wifi ready
      } else break;      
    }
  oled.setCursor(0, 30);
  oled.print("WiFi OK");
  oled.display();
  // time 
  if (!ble) {
    wifisyncjst(); // refer time and day
    //
    int ownrst_ind = preferences.getInt("ownrst",-1);
    if (ownrst_ind==99) { // own restart
      // current time
      time_t t = time(NULL);
      tm = localtime(&t);
      d_mon  = tm->tm_mon+1;
      d_mday = tm->tm_mday;
      d_hour = tm->tm_hour;
      d_min  = tm->tm_min;
      d_sec  = tm->tm_sec;
      d_wday = tm->tm_wday;
      // check schedule
      for(int i = 0; i <= MAXSCEDIDX; i++) {
         if ((entity[d_wday][i].stime <= d_hour * 60 + d_min) && 
              ((entity[d_wday][i].stime + entity[d_wday][i].duration) >= (d_hour * 60 + d_min ))
            ) {
                  entity[d_wday][i].scheduled = 1;
              }
      }
      preferences.putInt("ownrst", 0); // reset own restart
    }
  }
  radio.setup(); // Stats the receiver with default valuses. Normal operation
  delay(100);
  radio.setBand(2); //
  radio.setSpace(0); //
  radio.setVolume(vol_r);
  radio.setFrequency(stnFreq[stnIdx]);  // 
  delay(100);
  p_on = true;
  dsp_active = true;
  if (!ble) {
    audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
    audio.setVolume(volume); 
    i2s_stop(I2S_NUM_0); // stop I2S of audio because of noisy
    strcpy(stnurl,station_url[station]); // use preset
    strcpy(stnname,station_name[station]);
    bool conn_ok = false;
    WiFi.setTxPower(WIFI_POWER_17dBm); // TX power strong   #### xiao < 17dbm
    Serial.println("TX power 17dBm");
    server.on("/", handleRoot);
    server.on("/wavf", handleWavf);
    server.on("/rec", handleRec);
    server.on("/jpgf", handleJpgf);
    server.onNotFound(handleNotFound);
    server.begin();
    Serial.print("IP = ");
    Serial.println(WiFi.localIP());
    IPAddress ipadr = WiFi.localIP();
      oled.setCursor(0, 45);
      oled.printf("IP:%d.%d", ipadr[2],ipadr[3]);
      oled.display();
  }
    titlebuf[0] = 0;
    //WiFi.setTxPower(WIFI_POWER_17dBm); // TX power 
    digitalWrite(LED_BUILTIN, LOW); // led on
  wav_fcount = preferences.getInt("wavf_no", -1);
  if (wav_fcount < 0) wav_fcount = 1;

  //pcf8574 init
  if (!pcf.begin(0x20, &Wire)) {
    Serial.println("Couldn't find PCF8574");
  } else {
    Serial.println("Find PCF8574");
    pcf_active = true;
    for (int p=0; p<6; p++) {  // 5 contact point switch
      pcf.pinMode(p, INPUT_PULLUP);
      pcf.digitalWrite(p, HIGH);
    }
    pcf.pinMode(6, OUTPUT);  // ADG884 control
    pcf.digitalWrite(6, HIGH); 
    //pcf.digitalWrite(6, LOW);   /// #### sw to pcm5102
    pcf.pinMode(7, OUTPUT);  // inet control (ver 0.65)
    pcf.digitalWrite(7, HIGH); 
    delay(10);
  }
  // PSRAM malloc for recording
  rec_buffer1 = (uint8_t *)ps_malloc(record_size);
  rec_buffer2 = (uint8_t *)ps_malloc(record_size);
  rec_buffer32k = (uint8_t *)ps_malloc(1024*128+32768);
  //rec_buffer32k_2 = (uint8_t *)ps_malloc(1024*32+1024);
  jpg_buffer = (uint8_t *)ps_malloc(1024*300);  // SVGA:40k, XGA:300K
  if (rec_buffer1 != NULL && rec_buffer2 != NULL && rec_buffer32k != NULL && jpg_buffer != NULL) {
    memset(rec_buffer32k, 0, 1024*32); // 0 clear
  } else { 
    Serial.printf("malloc failed!\n");
    I2S_err = true;    
  }
  Serial.printf("Buffer: %d bytes\n", ESP.getPsramSize() - ESP.getFreePsram());
  //SDCardInit();
  
  delay(2000); // time to see ipaddress

}
void wifisyncjst() {
  // get jst from NTP server
  int lcnt = 0;
  configTzTime("JST-9", "ntp.nict.jp", "ntp.jst.mfeed.ad.jp");
  delay(500);
  // get sync time
  while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET) {
    delay(500);
    lcnt++;
    if (lcnt > 100) {
      Serial.println("time not sync within 50 sec");
      break;
    }
  }
}
// Use Rotary.h and  Rotary.cpp implementation to process encoder via interrupt
void rotaryEncoder()
{ // rotary encoder events
  uint8_t encoderStatus = encoder.process();
  if (encoderStatus)
  {
    if (encoderStatus == DIR_CW)
    {
      encoderCount = 1;
    }
    else
    {
      encoderCount = -1;
    }
  }
}

bool bleservice()
{
  Serial.println("\nBLEServer");

  //settimeofday(&tv, NULL); // Set temp time

  BLEDevice::init(LOCAL_NAME);
  BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT);  //

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  // Service and Characteristic
  BLEService *pService = pServer->createService(IDBserviceUUID);
  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
   IDBcharUUID,
   BLECharacteristic::PROPERTY_READ |
   BLECharacteristic::PROPERTY_WRITE
  );
  pCharacteristic->setCallbacks(new MyCallbacks(0));
  pCharacteristic->setValue(stnList);  // Radio Staions List
  // Descriptor
  pDescriptor->setValue("1710032400");
  pDescriptor->setCallbacks(new MyDescriptorCallbacks());
  pCharacteristic->addDescriptor(pDescriptor);
  // DOW Char
  BLECharacteristic *pCharacteristic2 = pService->createCharacteristic(
   IDBcharDOWUUID,
   BLECharacteristic::PROPERTY_READ |
   BLECharacteristic::PROPERTY_WRITE
  );
  pCharacteristic2->setCallbacks(new MyCallbacks(1));
  // PPCP Char
  BLECharacteristic *pCharacteristic3 = pService->createCharacteristic(
   IDBcharPPCPUUID,
   BLECharacteristic::PROPERTY_READ |
   BLECharacteristic::PROPERTY_WRITE
  );
  pCharacteristic3->setCallbacks(new MyCallbacks2());

  String wsced = "";
  char htstr[180];
  char sstr[480];
  sstr[0] = 'g';
  sstr[1] = 's';
  sstr[2] = 0;
  std::string cstr;
  Serial.println("Set Char for Read");
  int i=bdow;
  wsced = weekStr[bdow];
  wsced += ";";
  for(int j = 0; j <= MAXSCEDIDX; j++) {
    sprintf(htstr,"%d:%02d,%d,%d,%d,%d",entity[i][j].stime / 60,entity[i][j].stime % 60,entity[i][j].fidx,entity[i][j].duration,entity[i][j].volstep,entity[i][j].poweroff);
    wsced += htstr;
    if (j != MAXSCEDIDX) wsced += ";";
  }
  wsced += ";";
  wsced += "\n\0";
  Serial.println(wsced);
  //  pCharacteristic->setValue(wsced);
  cstr = wsced.c_str();
  pCharacteristic2->setValue(cstr);
  pCharacteristic3->setValue("ST1,80.4,AirG"); // Radio Staion Parameter
  // start service 
  pService->start();
  return true;
}

void loop()
{
  char ts[80];
  char slp[8];
  float tf;
  char wave_filename_t[32];
  //File file;
  if (connected) { // ble connected now?
    delay(300);
    digitalWrite(LED_BUILTIN, LOW);   // turn pilot LED on 
    delay(300);                        // wait for a while
    digitalWrite(LED_BUILTIN, HIGH);    // turn pilot LED off 
  }
  if (!connected && !advertise) {  // ble connection is not active
    advertise = true;
    pServer->getAdvertising()->start();
    Serial.println("Start Advertising");
  }
  if (wrote) { // If there is a ble signal, run it.
    wrote = false;
    if ((strbuff[0] - '0' >= 0) && (strbuff[0] - '9' <= 9)) { // Unix time(sec) ?
      long t_sec = 0;
      String *tstr = new String(strbuff);
      t_sec = tstr->toInt();
      struct timeval tv = { t_sec + gmtOffset_sec, 0 };  // set current time
      settimeofday(&tv, NULL);
    } else { // command input
      if (strbuff[0]=='v') {  // v+ or v- : volume
        if (strbuff[1]=='+') { // on
          volup_setting(); // v+
        } else { // other off
          voldown_setting();  // maybe v-
        }  
      } else if (strbuff[0]=='s') { // s+ or s- : station select
        if (strbuff[1]=='+') { // on
          station_setting(); // s+
        } else { // other off
          station_setting2(); // maybe s-
        }  
      } else if (strbuff[0]=='o') {
          power_onoff_setting(); // on or off req 
      }
    }
  }
  // if wifi is ok, check web server req
  if (WiFi.status() == WL_CONNECTED) server.handleClient();
  //
  if (mode_chg_req) { // mode change request
    if (mode_chg_int) {
      Serial.println("mode_chg_req by int");
      mode_chg_int = false;
    }
    if (inet_radio==0) { // dsp -> inet
      strcpy(stnurl,station_url[station]); // use preset
      strcpy(stnname,station_name[station]);
      //bool conn_ok = false;  ver 0.62
      WiFi.setTxPower(WIFI_POWER_17dBm); // TX power strong   #### xiao < 17dbm
      Serial.println("TX power 17dBm");
      i2s_start(I2S_NUM_0); // start I2S of audio 
      conn_ok = audio.connecttohost(stnurl); 
      if (!conn_ok) { // conect failure
        i2s_stop(I2S_NUM_0); // stop I2S of audio because of noisy
        Serial.println("Fail to connect");
      } //else { // connect ok ver 0.62
        
        inet_radio = 1;  //1: Internet radio
        //digitalWrite(AOUT_SW, LOW);
        pcf.digitalWrite(6, LOW); 
        radio.powerDown();
        dsp_active = false;
        audio.setVolume(volume); 
        p_on = true;
      //}    ver 0.62
      mode_chg_ok = true;
      mode_chg_req = false;
      stop_read = false;
    } else { // inet -> dsp
      audio.stopSong(); // inet stop, then set dsp_radio on
      i2s_stop(I2S_NUM_0); // stop I2S of audio because of noisy
      server.stop();
      // disconnect WiFi to restart 
      WiFi.disconnect(true);
      //Serial.println("WiFi Reconnect");
      oled.setTextSize(2); // Draw 2X-scale text
      oled.clearDisplay();
      oled.setCursor(0, 0);
      //oled.print("WiFi Recon");
      oled.print("Restart");
      oled.display();
      Serial.println("Restart");
      //Serial.println("WiFi Reconnect");
      delay(1500);
      // restart  system
      preferences.putInt("ownrst",99);
      ESP.restart();
      // 
      wifiMulti.run();
      delay(1000);
      while (true) {
        if(WiFi.status() == WL_CONNECTED){ break; }  // WiFi connect OK then next step
        Serial.println("WiFi Err");
        oled.setCursor(0, 30);
        oled.print("WiFi Err");
        oled.display();
        WiFi.disconnect(true);
        delay(1000*300);  // Wait for Wifi ready
      }
      server.on("/", handleRoot);
      server.onNotFound(handleNotFound);
      server.begin();
      //  
      digitalWrite(LED_BUILTIN, LOW);     // led on  
      p_onoff_req = true;
      inet_radio = 0;  //0: DSP radio
      p_on = false;
      mode_chg_ok = true;
      mode_chg_req = false;
    }
  }
  // Check switch request
  uint32_t ct = millis();
  //if (pcf_active && ((ct - pt) > 200)) { // check manual switch input
  if (pcf_active && pcf_int) { // check manual switch input
    pcf_int = false;
    for (int p=0; p <= 7; p++) {
      if (p==6) {;} 
      else if (! pcf.digitalRead(p)) { // pressed ?
        Serial.println("Switch pressed");
        switch(p) {  // 0 to 4 : 5_DirKey, 5 : additional button1, 7: additional button2
          case 0:  // F
                volup_setting();
                break;
          case 1:  // B
                voldown_setting(); 
                break;
          case 2: // L
          case 3: // R
                if (WAV_read) stop_read = true; 
                else 
                if (REC_on || MIC_rec_on) { // Is recoding active ?
                  Serial.println("sw: rec stop");
                  pofftm_h = (d_hour * 60 + d_min + 1) / 60;
                  pofftm_m = (d_hour * 60 + d_min + 1) % 60;
                  sprintf(ts,"%02d:%02d %s",pofftm_h,pofftm_m,"poff or recording stop scheduled");
                  Serial.println(ts); 
                  msg = "control: rec stop scheduled, wait a few minutes";
                  REC_on_no_poff = true;
                } else { // DSP radio mode
                  if (p==2) station_setting2(); else station_setting1();  
                }                //else station_setting2();
                //break;
                //if (WAV_read) stop_read = true;
                //else station_setting1();  
                break;
          case 4:  // M
                sleep_setting(); // push twice means recording  (ver 0.65)
                break;
          case 5:  // none
                break;
          case 7:  // mode(dsp/inet) change
                modechange_setting();
                break;
          default: ;
        
        }
      }
      pt = ct;
    }
    pcf_int_ok = true; 
      //delay(50);
  }
  // Check if the encoder has moved.
  if (encoderCount != 0)
  {
    if (encoderCount == 1) {
      if (RE_Mode==0) {
        volup_setting();
      } else {
        station_setting1(); 
      }
    }
    else {
      if (RE_Mode==0) {
        voldown_setting();
      } else {
        station_setting2(); 
      }
    }
    encoderCount = 0;
  }
  
  if (inet_radio==1 || inet_radio==0) { // process server request
    if (a_srv==0 || b_srv==0) { // by server operation
      Serial.println("Server vol change");
      updatevolume(a_srv, b_srv);
      a_srv=1;
      b_srv=1;
    }
    if (s==0 || s_srv==0 || RE_Clicks !=0) { // button or by server operation or Click cnt active timer
      RE_ct = millis();
      if (RE_Clicks==0 && s_srv!=0) {
        RE_pt = RE_ct;
        RE_Clicks++;
        Serial.println("RE_Clicks==0++");
      } else {
        if ( (RE_pt!=0 && (RE_ct - RE_pt)>=3000) || s_srv==0) {
          Serial.println("RE_ct>3000 or s_srv==0");
          if (RE_Clicks==1 || s_srv==0) { // sleep
            sleepmode = !sleepmode;
            if (sleepmode) {
              Serial.println("Sleep timer ON");
              s_remin = 60;
            } else {
              Serial.println("Sleep timer OFF");
              s_remin = 60;
            }
          } else if (RE_Clicks == 2){
            //RE_Mode = 1; // STN mode
            Serial.println("sw: record");
            initiate_rec(0);
          } else {
            RE_Mode = 0; // reset
            Serial.println("VOL mode");
          }
          RE_pt = 0;
          RE_Clicks = 0;
        } else if (s==0) { // click ?
          RE_Clicks++;
          Serial.println("RE_Clicks!=0++");
        }
      }
      s=1;
      s_srv=1;
    }
  }
  if (inet_radio==1) {
    loop_cnt ++;
    if (loop_cnt > 500) { // about 3 sec in normal process
      loop_cnt = 0;
      led_onoff = !led_onoff;
      if (led_onoff) {
        digitalWrite(LED_BUILTIN, LOW); // on
        oled_display(1); // inet info
      }
      else {
        digitalWrite(LED_BUILTIN, HIGH); // off
        oled_display(0); // time
      }
    }
    audio.loop(); // Inter net radio function call
    if (stop_read) { // SD read stop
      WAV_read = false;
      in_play = false;
      if (INET_in_play) {
        INET_in_play = false;
        event = 1;
        stop_read = false;
      } else {
        p_onoff_req = true;
        dsp_active = false;
        p_on =false;
        pcf.digitalWrite(6, HIGH);
        audio.stopSong();
        i2s_stop(I2S_NUM_0); // stop audio port
        inet_radio = 0;
      }
      Serial.println("Play end.");
      //stop_read = false;
    }

  }
  if ((event==1) && (inet_radio==1)) { // interrupt of change station request ?
      char tstr[166];
      event = 0; // clear 
      strcpy(stnurl,station_url[station]); 
      strcpy(stnname,station_name[station]);
      i2s_start(I2S_NUM_0); // start I2S of audio 
      conn_ok = audio.connecttohost(station_url[station]); // change station.
      // save change
      sprintf(tstr,"%s%d","st", station + 1);
      preferences.putString(tstr,stnurl);
      sprintf(tstr,"%s%d","nm", station + 1);
      preferences.putString(tstr,stnname);
      preferences.putInt("stn",station);
      stn_ok = true;
  }
  // power off process
  if (inet_radio==1) {
    if (p_onoff_req) {
      if (p_on) {
        audio.stopSong();
        i2s_stop(I2S_NUM_0); // stop I2S of audio because of noisy
        Serial.println("inet pw off");
        p_on = false;
      } else {
        Serial.println("inet pw on");
        i2s_start(I2S_NUM_0); // start I2S of audio 
        conn_ok = audio.connecttohost(station_url[station]); 
        delay(100);
        //digitalWrite(AOUT_SW, LOW);
        pcf.digitalWrite(6, LOW); 
        p_on = true;
      }
      p_onoff_req = false;
    }
  }
  if (inet_radio==0 /*|| inet_radio==1*/) {
    if (p_onoff_req) {
      if (p_on) {
        radio.powerDown();
        dsp_active = false;
        Serial.println("radio pw off");
        p_on = false;
      } else {
        radio.powerUp();
        dsp_active = true;
        delay(100);
        stnIdx = preferences.getInt("stix", -1);
        if (stnIdx==-1) stnIdx = 3;
        radio.setFrequency(stnFreq[stnIdx]);
        radio.setVolume(vol_r);
        delay(50);
        lastfreq = stnFreq[stnIdx];
        laststnIdx = stnIdx;
        Serial.println("radio pw on");
        //digitalWrite(AOUT_SW, HIGH);
        pcf.digitalWrite(6, HIGH); 
        p_on = true;
      }
      p_onoff_req = false;
    }
    if (laststnIdx != stnIdx) {
      preferences.putInt("stix", stnIdx); // moved, ver 0.61
      Serial.print("stn changed:");
      Serial.println(stnIdx);
      radio.setFrequency(stnFreq[stnIdx]);
      radio.setVolume(vol_r);     // ver 0.62
      lastfreq = stnFreq[stnIdx];
      laststnIdx = stnIdx;
      stn_ok = true;
    }
    oled.clearDisplay();
    oled.setTextSize(2); // Draw 2X-scale text
    // display current time
    time_t t = time(NULL);
    tm = localtime(&t);
    d_mon  = tm->tm_mon+1;
    d_mday = tm->tm_mday;
    d_hour = tm->tm_hour;
    d_min  = tm->tm_min;
    d_sec  = tm->tm_sec;
    d_wday = tm->tm_wday;
    d_year = tm->tm_year;
    //Serial.print("time ");
    sprintf(wave_filename_t, "/mug%04d%02d%02d%02d%02d%02d_", d_year + 1900, d_mon, d_mday, d_hour, d_min, d_sec);
    if ((last_d_sec != d_sec) && (inet_radio == 0)) {
      //sprintf(ts,"%02d:%02d:%02d",d_hour,d_min,d_sec);
      sprintf(ts, "%02d-%02d %s", d_mon, d_mday, weekStr[d_wday]);
      //Serial.println(ts);
      oled.setTextSize(2); // Draw 2X-scale text
      oled.clearDisplay();
      oled.setCursor(0, 0);
      oled.print(ts);
      //Serial.println(ts);
      if (sleepmode) sprintf(slp,"%s","SLP:");
      else {
        if (RE_Mode==0 || pcf_active) // RE VOL or PCF7534 active
          if (REC_on) sprintf(slp,"%s","W V:"); else sprintf(slp,"%s","V V:");  // (ver 0.63)
        else  // RE STN
          sprintf(slp,"%s","S S:");
      } 
      //sprintf(ts, "%02d-%02d %s", d_mon, d_mday, weekStr[d_wday]);
      sprintf(ts,"%02d:%02d:%02d",d_hour,d_min,d_sec);
      //sprintf(ts, "RSSI:%03d", radio.getRssi());
      oled.setCursor(0, 15);
      oled.print(ts);
      int pi = p_on ? 1 : 0; // power indicator
      if (sleepmode) 
        sprintf(ts, "%.4s%02d %s%01d", slp, s_remin, "P:", pi);
      else {
        int num_vs ;
        if (RE_Mode==0) num_vs = vol_r; else num_vs = stnIdx;
        sprintf(ts, "%.4s%02d %s%01d", slp, num_vs, "P:", pi);
      }
      oled.setCursor(0, 30);
      oled.print(ts);
      tf = lastfreq/100.0;
      sprintf(ts, "%3.1f S:%03d", tf, radio.getRssi()); // frequency and signal strength
      oled.setCursor(0, 45);
      oled.print(ts);
      oled.display();
      if (sleepmode) check_sleep();
      last_d_sec = d_sec; // 
      //delay(300); // if inet radio off , delay ok here
    }
  }
  if (inet_radio==0 || inet_radio==1) {
    if (DSP2DAC) { // experimental code,  DSP2DAC is false
      // read dsp and write dac
      //static int16_t tx_buffer[1024];
      //int16_t rx_buffer[4096];
      static size_t bytes_written = 0;
      static size_t bytes_read = 0;
      static size_t bytes_read_0_cnt = 0;
      //int32_t tval;
      /*i2s_read(I2S_NUM_1, rec_buffer32k, 32767, &bytes_read, 10);
      if (bytes_read > 0 ) {
          for (uint32_t i = 0; i < bytes_read; i += SAMPLE_BITS/8) {
            (*(uint16_t *)(rec_buffer32k + i)) >>= 2;
          }

        i2s_write(I2S_NUM_0, rec_buffer32k, bytes_read, &bytes_written, 100);
      }*/
      i2s_read(I2S_NUM_1, rec_buffer32k + rec_bufp_r, 32767 * 4, &bytes_read, 1);
      if (bytes_read > 0 ) {
          for (uint32_t i = 0; i < bytes_read; i += SAMPLE_BITS/8) {
            //(*(int16_t *)(rec_buffer32k + rec_bufp_r + i)) >>= 2;  // volume down
            //((int16_t) (int32_t)(*(int16_t *)(rec_buffer32k + rec_bufp_r + i)) * 205 + 512) >>= 10;
            (*(int16_t *)(rec_buffer32k + rec_bufp_r + i)) /= 5;  // volume down
            /*tval = (int32_t)(*(int16_t *)(rec_buffer32k + rec_bufp_r + i));
           *(int16_t *)(rec_buffer32k + rec_bufp_r + i) = (int16_t)(tval * 205 + 512) >> 10;*/
          }
          i2s_write(I2S_NUM_0, rec_buffer32k + rec_bufp_r, bytes_read, &bytes_written, 200);
          rec_bufp_r = rec_bufp_r + bytes_read;
          if (rec_bufp_r > 32767 * 4) {
            rec_bufp_r  = 0;
            Serial.printf("Read I2S 128K, last write: %d, no data: %d.\n", bytes_written, bytes_read_0_cnt );
            bytes_read_0_cnt = 0;
          }
          bytes_read = 0;   
      } else bytes_read_0_cnt++;
      /*if (rec_bufp_r - rec_bufp_w > 8192 || ((rec_bufp_r > 8192) && ( rec_bufp_w > rec_bufp_r))){
         i2s_write(I2S_NUM_0, rec_buffer32k + rec_bufp_w, 32767, &bytes_written, 50);
         rec_bufp_w = rec_bufp_w + bytes_written;
         if (rec_bufp_w > 32767*4) {
           rec_bufp_w = 0;
           Serial.printf("Write I2S 12K.\n");
         }
      }*/
    }
    if ((REC_on || MIC_rec_on ) && !I2S_err) {
      // Start recording
      uint32_t sample_size = 0;
      uint8_t *rec_buffer = NULL;
      uint32_t avail_size = 0;
      uint32_t BytesWritten = 0;
      avail_size = 1024 * 32;  // 
      if (curr_buf==1) rec_buffer = rec_buffer1; else rec_buffer = rec_buffer2;
      i2s_read(I2S_NUM_1, rec_buffer + recorded_size, avail_size, &sample_size, 1); // read from DSP or MIC
      if (sample_size == 0) {
        //Serial.printf("Record Failed!\n");
        //I2S_err = true;
        ;
      } else { // read ok
        if (MIC_rec_on)
        {// Increase volume
          for (uint32_t i = recorded_size; i < recorded_size + sample_size; i += SAMPLE_BITS/8) {
            (*(uint16_t *)(rec_buffer+i)) <<= 4;
          }
        }  else if (inet_radio == 1) { // Increase volume, value is plus or minus
          for (uint32_t i = recorded_size; i < recorded_size + sample_size; i += SAMPLE_BITS/8) {
            (*(int16_t *)(rec_buffer+i)) *= 5;
          }
        }
        recorded_size =  recorded_size + sample_size;
        avail_cnt ++;
        if (recorded_size >= K32*2) {
          //Serial.println("Start recording 32k");
          if (curr_buf==1) {
            // switch buffer area
            memcpy(rec_buffer2, rec_buffer1 + (K32*2), recorded_size - (K32*2));
            rec_buffer = rec_buffer2 + recorded_size - (K32*2);
            curr_buf = 2;
          } else { // curr_buff 2
            memcpy(rec_buffer1, rec_buffer2 + (K32*2), recorded_size - (K32*2));
            rec_buffer = rec_buffer1 + recorded_size - (K32*2);
            curr_buf = 1;
          }
          recorded_size = recorded_size - (K32*2);
          SD_write = true; 
        }
      }
      if (SD_write) {
        // write SD
        if (!WAVE_HDR_write) {
          // write wave file header
          sprintf(wave_filename, "%s%03d.wav", wave_filename_t, wav_fcount);  // 3 digit (v 0.77, 2026.4.19)
          wav_fcount++;
          preferences.putInt("wavf_no", wav_fcount);
          file = SD.open(wave_filename, FILE_WRITE);

          // Write the header to the WAV file
          uint8_t wav_header[WAV_HEADER_SIZE];
          if (REC_on) {
            if (inet_radio == 0) generate_wav_header(wav_header, estimated_recorded_size, SAMPLE_RATE);
            else generate_wav_header(wav_header, estimated_recorded_size, audio.getSampleRate());
          }
          else 
            generate_wav_header(wav_header, estimated_recorded_size, SAMPLE_RATE_MIC);
          memset(rec_buffer32k, 0, 1024*32);
          //file.write(wav_header, WAV_HEADER_SIZE);
          memcpy(rec_buffer32k, wav_header, WAV_HEADER_SIZE);
          file.write(rec_buffer32k, 1024 * 32); // filler
          total_recorded_size = K32;
          Serial.printf("WAVE file header wrote.\n");
          WAVE_HDR_write = true;
        }
        // write SD data
        if (total_recorded_size/(K32*10) != last_blk) {
           Serial.printf("Available %d times,Left over %d bytes, use buff %d.\n", avail_cnt, recorded_size, curr_buf);
           last_blk = total_recorded_size / (K32*10);
        }
        //Serial.printf("Writing to the file ...\n");
        if (curr_buf==1) rec_buffer = rec_buffer2; else rec_buffer = rec_buffer1;
        int w_sz = file.write(rec_buffer, K32/*recorded_size*/); 
        w_sz = file.write(rec_buffer + K32, K32/*recorded_size*/); 

        //if (file.write(rec_buffer, recorded_size) != recorded_size) {
        if (w_sz != K32/*recorded_size*/) {
          // Retry it, once
          delay(10);
          int w_sz_r = file.write(rec_buffer + w_sz, K32/*recorded_size*/ - w_sz); 
          if (w_sz_r != K32/*recorded_size*/ - w_sz) {
            Serial.printf("Write file and retry Failed! wz:%d, rd:%d\n", w_sz + w_sz_r, recorded_size);
            I2S_err = true;
          } else {
            Serial.printf("Write file failed, and retry success! wz:%d, rd:%d\n", w_sz + w_sz_r, recorded_size);
          }
          total_recorded_size = total_recorded_size + w_sz + w_sz_r;
        } else  total_recorded_size = total_recorded_size + K32*2/*recorded_size*/;
        avail_cnt = 0;       
        SD_write = false;
      }
    }
  }
  // Recording
  if (initiate_recording) {
    initiate_recording = false;
    initiate_rec(1);
  }
  // Check Schedule 
  if (last_d_min != d_min) {
    last_d_min = d_min;
    if (pofftm_h == d_hour && pofftm_m == d_min && p_on) { // power off time ?
      p_onoff_req = true;
      pofftm_h = 0;
      pofftm_m = 0;
        if (REC_on || MIC_rec_on) {
          // note : abandon remainning record in the buffer, which is not so important.
          uint8_t wav_header[WAV_HEADER_SIZE];
          file.seek(0);
          if (REC_on) {
            if (inet_radio == 0) generate_wav_header(wav_header, estimated_recorded_size, SAMPLE_RATE);
            else generate_wav_header(wav_header, estimated_recorded_size, audio.getSampleRate());
          }
          else
            generate_wav_header(wav_header, total_recorded_size, SAMPLE_RATE_MIC);
          file.write(wav_header, WAV_HEADER_SIZE);
          Serial.printf("WAVE file header updated.\n");
          file.close();
          i2s_driver_uninstall(I2S_NUM_1);  // no more need
          Serial.printf("Last recorded %d, Total %d bytes.\n", recorded_size, total_recorded_size); 
          Serial.printf("The recording is over.\n");
          recorded_size = 0;
          REC_on = false;
          stop_read = true; // no playing before
          WAVE_HDR_write = false;
          I2S_err = false;
          if (REC_on_no_poff) {
            if (!poff_after_rec) p_onoff_req = false;
            poff_after_rec = false;
            REC_on_no_poff = false;
            if (MIC_rec_on) {      
              radio.powerUp();
              dsp_active = true;
            }
          }
          MIC_rec_on = false;
        }

    } else if (!REC_on && !MIC_rec_on) {
      for(int i = 0; i <= MAXSCEDIDX; i++) {
        if ((entity[d_wday][i].stime == 0) && !((i == 0) && (entity[d_wday][i].duration > 0))) {     
          //nop
          //Serial.println(d_min);
        } else {
          //Serial.println(entity[d_wday][i].stime);
          if ((entity[d_wday][i].stime <= d_hour * 60 + d_min) && 
              ((entity[d_wday][i].stime + entity[d_wday][i].duration) >= (d_hour * 60 + d_min ))
              && (entity[d_wday][i].scheduled != 1)) {
            if (lastfreq == stnFreq[entity[d_wday][i].fidx] && entity[d_wday][i].fidx < 50 && inet_radio==0) {
              //entity[d_wday][i].scheduled = 1; // mark it scheduled
            } else {          
              //radio.setFrequency(stnFreq[entity[d_wday][i].fidx]);
              stnIdx =  entity[d_wday][i].fidx;
              if (stnIdx >= 50) {
                int  lstation = station;
                station = stnIdx - 51; // 51->0, 52->1, and so on (ver 0.54)
                if (station > max_station || station < 0) station = 0; // (ver 0.54)
                strcpy(stnurl,station_url[station]); 
                strcpy(stnname,station_name[station]);
                if (inet_radio==0) mode_chg_req = true;
                if (inet_radio==1 && station != lstation) event = 1;       // change station
                volume = entity[d_wday][i].volstep; // inet
                preferences.putInt("stn", station);
                preferences.putInt("vol", volume);
              } else {
                if (inet_radio==1) mode_chg_req = true;
                vol_r = entity[d_wday][i].volstep;    // dsp
                preferences.putInt("stix", stnIdx);
                preferences.putInt("volr", vol_r);
              }
            }                     
            
            currIdx = i;
            entity[d_wday][i].scheduled = 1; // mark it scheduled
            Serial.println("scheduled");
            if (entity[d_wday][i].poweroff==1 || entity[d_wday][i].poweroff==4 || entity[d_wday][i].poweroff==5) { // power off or recording?
              pofftm_h = (entity[d_wday][i].stime + entity[d_wday][i].duration) / 60; // set power off time
              pofftm_m = (entity[d_wday][i].stime + entity[d_wday][i].duration) % 60;
              sprintf(ts,"%02d:%02d %s",pofftm_h,pofftm_m,"poff or recoding stop will be scheduled");
              Serial.println(ts);
              if (entity[d_wday][i].poweroff==4 || entity[d_wday][i].poweroff==5) { // initiate recording
                rectime = entity[d_wday][i].duration;
                if (entity[d_wday][i].poweroff==5) poff_after_rec = true;
                REC_on_no_poff = true;
                initiate_recording = true;
                Serial.println("initiate recording req");
              }
            }
            if (p_on==false && (inet_radio == 0 && !mode_chg_req)) {  // dsp_radio
              p_onoff_req = true;  //  if power off currently then power on req
              pofftm_h = 0;        // reset
              pofftm_m = 0;
              Serial.println("pw on req");
            } 
          } 
        }
      }
    }
  }
}

// display
void oled_display(int dmode){
  char ts[80];
  char buf[64];
  char buf1[64];
  char slp[8];
     // display current time
    time_t t = time(NULL);
    tm = localtime(&t);
    d_mon  = tm->tm_mon+1;
    d_mday = tm->tm_mday;
    d_hour = tm->tm_hour;
    d_min  = tm->tm_min;
    d_sec  = tm->tm_sec;
    d_wday = tm->tm_wday;
    //Serial.print("time ");
  if (!REC_on) {
  oled.clearDisplay();
  oled.setTextSize(2); // Draw 2X-scale text
  if (dmode==0) { // time
    if (last_d_sec != d_sec) { // inet info
      //sprintf(ts,"%02d:%02d:%02d",d_hour,d_min,d_sec);
      sprintf(ts, "%02d-%02d %s", d_mon, d_mday, weekStr[d_wday]);
      oled.setTextSize(2); // Draw 2X-scale text
      oled.setCursor(0, 0);
      oled.print(ts);
      //Serial.println(ts);
      //sprintf(ts, "%02d-%02d %s", d_mon, d_mday, weekStr[d_wday]);
      sprintf(ts,"%02d:%02d:%02d",d_hour,d_min,d_sec);
      oled.setCursor(0, 15);
      oled.print(ts);
      //Serial.println(ts);
      last_d_sec = d_sec; // 
    }
  } else {
    if (sleepmode) {
      sprintf(buf1,"SLP%02d V:%02d", s_remin, volume);
    } else if (!p_on) {
      sprintf(buf1,"OFF Vol:%02d", volume);
    } else {
      char conn[4]; // ver 0.62
      if (conn_ok) {
        if (WAV_read) sprintf(conn, "%s","SD ");  else sprintf(conn, "%s","NET"); 
      }
      else {
        if (WAV_read) sprintf(conn, "%s","SD ");  else sprintf(conn, "%s", "NC ");  // ver 0.62
      }
      if (RE_Mode==0 || pcf_active) {
        sprintf(buf1,"%s Vol:%02d", conn, volume); // ver 0.62
      } else {
        sprintf(buf1,"%s Stn:%02d", conn, station + 1); // ver 0.62
      }
    }
    if (WAV_read) sprintf(buf,"%.10s","WAV FILE"); else sprintf(buf,"%.10s",stnname);
    oled.setCursor(0, 0);
    oled.print(buf);
    oled.setCursor(0, 15);
    oled.print(buf1);
  }
  oled.setCursor(0, 35);
  oled.setTextSize(1); // Draw 1X-scale text
  if (WAV_read) oled.print(rstr); else oled.print(titlebuf); // file name or title info
  oled.display();
  } // not REC_on
  if (sleepmode) {
    check_sleep();
  }
}
// Check if sleep
void check_sleep()
{
  if (last_d_min != d_min) {
    last_d_min = d_min;
    s_remin --;
    if (s_remin==0) {  // fall asleep
      server.stop();
      // disconnect WiFi to sleep 
      WiFi.disconnect(true);
      oled.setTextSize(2); // Draw 2X-scale text
      oled.clearDisplay();
      oled.setCursor(0, 0);
      oled.print("Sleep!!");
      oled.display();
      Serial.println("Sleep");
      s = 1;
      delay(30000);
      //
      if (s==0) ESP.restart(); //if cancelled within 30 sec then restart
      oled.clearDisplay();  // put off
      oled.display(); 
      digitalWrite(LED_BUILTIN, HIGH); // put off
      esp_deep_sleep_start();  // sleep forever
    }
  }
}
// change volume
void updatevolume(uint8_t a, uint8_t b)
{
  if (b==0) { // push vol up
    if (inet_radio==1) {
      volume ++;
      if (volume > MAXVOL) volume = MAXVOL;// inet
    }    
    else if (inet_radio==0) {
      vol_r ++;
      if (vol_r > MAXVOL_R) vol_r = MAXVOL_R; // dsp
    }
  } else if (a==0) { // push vol down
    if (inet_radio==1) {
      volume --;     
      if (volume < 0) volume = 0;
    }
    else if (inet_radio==0) {
      vol_r --;
      if (vol_r < 0) vol_r = 0;
    } 
  }
  Serial.print("vl:"); 
  if (inet_radio==1) {
    Serial.println(volume);
    audio.setVolume(volume);
    preferences.putInt("vol",volume);
  } else if (inet_radio==0) {
    Serial.println(vol_r);
    radio.setVolume(vol_r);
    preferences.putInt("volr",vol_r);
  }
}
// volume int routine
void volup_setting(){
  ct2=millis();
  if ((ct2-pt2)>250) {
    //updatevolume(1, 0);
    b_srv = 0;  // (ver:0.55)
  }
  pt2=ct2;
}
void voldown_setting(){
  ct2=millis();
  if ((ct2-pt2)>250) {
    //updatevolume(0, 1);
    a_srv = 0;  // (ver:0.55)
  }
  pt2=ct2; 
}

int DumpWAVHeader(WavHeader_Struct* Wav) {
  if (memcmp(Wav->RIFFSectionID, "RIFF", 4) != 0) {
    Serial.print("Not a RIFF format file - ");
    PrintData(Wav->RIFFSectionID, 4);
    if (memcmp(Wav->RIFFSectionID, "ID3", 3) == 0) {
      Serial.println(" May be a MP3 format file.");
      return (1);
    }
    return(5);
  } 
  if (memcmp(Wav->RiffFormat, "WAVE", 4) != 0) {
    Serial.print("Not a WAVE file - ");
    PrintData(Wav->RiffFormat, 4);
    return(4);
  }
  if (memcmp(Wav->FormatSectionID, "fmt", 3) != 0) {
    Serial.print("fmt ID not present - ");
    PrintData(Wav->FormatSectionID, 3);
    return(3);
  }
  if (memcmp(Wav->DataSectionID, "data", 4) != 0) {
    Serial.print("data ID not present - ");
    PrintData(Wav->DataSectionID, 4);
    return(2);
  }
  // All looks good, dump the data
  Serial.print("Total size :");
  Serial.println(Wav->Size);
  Serial.print("Format section size :");
  Serial.println(Wav->FormatSize);
  Serial.print("Wave format :");
  Serial.println(Wav->FormatID);
  Serial.print("Channels :");
  Serial.println(Wav->NumChannels);
  Serial.print("Sample Rate :");
  Serial.println(Wav->SampleRate);
  Serial.print("Byte Rate :");
  Serial.println(Wav->ByteRate);
  Serial.print("Block Align :");
  Serial.println(Wav->BlockAlign);
  Serial.print("Bits Per Sample :");
  Serial.println(Wav->BitsPerSample);
  Serial.print("Data Size :");
  Serial.println(Wav->DataSize);
  return(0);
}

void PrintData(const char* Data, uint8_t NumBytes) {
  for (uint8_t i = 0; i < NumBytes; i++)
    Serial.print(Data[i]);
  Serial.println();
}

// optional
void audio_info(const char *info){
    Serial.print("info        "); Serial.println(info);
}
void audio_id3data(const char *info){  //id3 metadata
    Serial.print("id3data     ");Serial.println(info);
}
void audio_eof_mp3(const char *info){  //end of file
    stop_read = true; // END OF play file
    Serial.print("eof_mp3     ");Serial.println(info);
}
void audio_showstation(const char *info){
    Serial.print("station     ");Serial.println(info);
}
void audio_showstreaminfo(const char *info){
    Serial.print("streaminfo  ");Serial.println(info);
}
void audio_showstreamtitle(const char *info){
    sprintf(titlebuf,"%.128s",info);  // server data
    oled_display(1); // display inet info
}
void audio_bitrate(const char *info){
    Serial.print("bitrate     ");Serial.println(info);
}
void audio_commercial(const char *info){  //duration in sec
    Serial.print("commercial  ");Serial.println(info);
}
void audio_icyurl(const char *info){  //homepage
    if (strlen(info) == 0) stop_read = true; // maybe connect error 
    Serial.print("icyurl      ");Serial.println(info);
}
void audio_lasthost(const char *info){  //stream URL played
    Serial.print("lasthost    ");Serial.println(info);
}
void audio_eof_speech(const char *info){
    Serial.print("eof_speech  ");Serial.println(info);
}
void power_onoff_setting() {
  if (p_onoff_req==false) {  // wait last req
     p_onoff_req = true;  // req
  }
}
// sleep int routine
void sleep_setting(){
  ct2=millis();
  if ((ct2-pt2)>250) {
    s = 0;  // toggle sleep on/off
  }
  pt2=ct2;
}
// switches
void switch_setting() {
  if (pcf_int_ok) {
     pcf_int_ok = false;
     pcf_int = true;
  }
  if (!WiFi_OK && !ble) s = 0;  // Initiate BLE mode (Ver 0.78)
}
// modechange int routine
void modechange_setting(){
  ct2=millis();
  if ((ct2-pt2)>250) {
          mode_chg_ok = false;
          mode_chg_req = true;
          mode_chg_int = true; // to identify by int
  }
  pt2=ct2;
}
// station change int routine
void station_setting(){
  ct2=millis();
  //delay(10);  // no effect here
  if ((ct2-pt2)>250) {
    //Serial.print("st:");
    if (inet_radio==1) { // 
      station = station + 1;  // inet
      if (station>=max_station) station = 0;
      //Serial.println(station);
      event=1; 
    }
    else if (inet_radio==0) station_setting1(); // dsp radio
  }
  pt2=ct2;
 } 
// RE support station change
void station_setting1() { 
  if (stn_ok) {  // wait last req
    stn_ok = false;
    if (inet_radio==0) { // dsp radio
      stnIdx++;
      if (stnIdx > MAXSTNIDX) stnIdx = 0;  // turn around to support single button
      //Serial.print("stnIdx+:");
      //Serial.println(stnIdx);
      //preferences.putInt("stix", stnIdx); ver 0.61
    } else {
      station = station + 1;  // inet
      if (station>=max_station) station=0;
      //Serial.println(station);
      event=1; 
    }
  }
}
void station_setting2() {
  if (stn_ok) {  // wait last req
    stn_ok = false;
    if (inet_radio==0) { // dsp radio
      stnIdx--;
      if (stnIdx < 0) stnIdx = MAXSTNIDX; // turn around to support single button
      //Serial.print("stnIdx-:");
      //Serial.println(stnIdx);
      //preferences.putInt("stix", stnIdx); ver 0.61
    } else {
      station = station - 1;  // inet
      if (station<0) station = max_station - 1;
      //Serial.println(station);
      event=1; 
    }
  }
}
