/*   
 *  Last Update 2021/04/13
 *  Author : M. Fachrial Yunizar
 *  github : github.com/fachrialyunizar
 *  Versi : WeatherStation 2.0
 *  ENV_APP : Production
 *  Debug : False
 */

#include <SPI.h> 
#include <Wire.h>
#include "SX1272.h"
#include "Sensor.h"
#include "Humi.h"
#include "Temp.h"
#include <DHT.h>
#include <Adafruit_BMP280.h>

#pragma GCC diagnostic ignored "-Wwrite-strings"

#define ETSI_EUROPE_REGULATION
#define BAND900

#ifdef ETSI_EUROPE_REGULATION
#define MAX_DBM 14
#endif

#ifdef  BAND900
const uint32_t DEFAULT_CHANNEL=CH_12_900;
#endif

#define PABOOST

#define WITH_EEPROM
#define WITH_APPKEY
#define FLOAT_TEMP

#define LORAMODE  1
uint8_t node_addr= 5;

unsigned int idlePeriodInSec = 61;

#define FORCE_DEFAULT_VALUE

#ifdef WITH_APPKEY
uint8_t my_appKey[4]={5, 6, 7, 8};
#endif

#if defined __SAMD21G18A__ && not defined ARDUINO_SAMD_FEATHER_M0
#define PRINTLN                   SerialUSB.println("")              
#define PRINT_CSTSTR(fmt,param)   SerialUSB.print(F(param))
#define PRINT_STR(fmt,param)      SerialUSB.print(param)
#define PRINT_VALUE(fmt,param)    SerialUSB.print(param)
#define FLUSHOUTPUT               SerialUSB.flush();
#else
#define PRINTLN                   Serial.println("")              
#define PRINT_CSTSTR(fmt,param)   Serial.print(F(param))
#define PRINT_STR(fmt,param)      Serial.print(param)
#define PRINT_VALUE(fmt,param)    Serial.print(param)
#define FLUSHOUTPUT               Serial.flush();
#endif

#ifdef WITH_EEPROM
#include <EEPROM.h>
#endif

#define DEFAULT_DEST_ADDR 1

#ifdef WITH_ACK
#define NB_RETRIES 2
#endif

#ifdef LOW_POWER
#if defined __MK20DX256__ || defined __MKL26Z64__ || defined __MK64FX512__ || defined __MK66FX1M0__
#define LOW_POWER_PERIOD 1
#include <Snooze.h>
SnoozeTimer timer;
SnoozeBlock sleep_config(timer);
#else
#define LOW_POWER_PERIOD 1
#include "LowPower.h"

#ifdef __SAMD21G18A__
#include "RTCZero.h"
RTCZero rtc;
#endif
#endif
unsigned int nCycle = idlePeriodInSec*1000/LOW_POWER_PERIOD;
#endif

unsigned long nextTransmissionTime=0L;
uint8_t message[100];
int loraMode=LORAMODE;

#ifdef WITH_EEPROM
struct sx1272config {

  uint8_t flag1;
  uint8_t flag2;
  uint8_t seq;
  uint8_t addr;  
  unsigned int idle_period;  
  uint8_t overwrite;
};

sx1272config my_sx1272config;
#endif

#ifdef WITH_RCVW

#define DELAY_BEFORE_RCVW 5000

long getCmdValue(int &i, char* strBuff=NULL) {
  
    char seqStr[5]="******";
    
    int j=0;
    while ((char)message[i]!='#' && (i < strlen((char*)message)) && j<strlen(seqStr)) {
            seqStr[j]=(char)message[i];
            i++;
            j++;
    }
    
    seqStr[j]='\0';
    
    if (strBuff) {
            strcpy(strBuff, seqStr);        
    }
    else
            return (atol(seqStr));
}   
#endif


const int number_of_sensors = 2;

Sensor* sensor_ptrs[number_of_sensors];

char *ftoa(char *a, double f, int precision)
{
 long p[] = {0,10,100,1000,10000,100000,1000000,10000000,100000000};
 
 char *ret = a;
 long heiltal = (long)f;
 itoa(heiltal, a, 10);
 while (*a != '\0') a++;
 *a++ = '.';
 long desimal = abs((long)((f - heiltal) * p[precision]));
 itoa(desimal, a, 10);
 return ret;
}

//////////////////DEKLARASI VARIABLE////////////////////////

#define DHTPIN 23
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

const int analogPin = A0;
int sensorLDR = 0;
int outputLDR = 0;

Adafruit_BMP280 bmp; 

const byte interruptPin3 = 3;
int count1 = 0;
const float rain_gauge = 1.346; // in mm or 0,053 inci
float curah_hujan = 0;
const long interval1 = 60000;
unsigned long previousMillis1 = 0;

const byte interruptPin18 = 18;
int count2 = 0;
int rpm;
const float pi = 3.14159265;
const int r =80;
float speedWind = 0;

const int  A = 41; 
const int  B = 45;
const int  C = 49;
const int  D = 22;
const int  E = 25;
const int  F = 29;
const int  G = 33;
const int  H = 37;
    int u = 0;
    int tl = 0;
    int t = 0;
    int tgr = 0;
    int s = 0;
    int bd = 0;
    int b = 0;
    int bl = 0;
const long interval2 = 100;
unsigned long previousMillis2 = 0;
const char *windD[] = {"Utara", "Timur_Laut", "Timur", "Tenggara", "Selatan", "Barat_Daya", "Barat", "Barat_Laut"};
String windDir;


////////////////////////////////////////////////////////////

void setup()
{  
  pinMode(interruptPin3, INPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPin3), counter1, RISING);

  pinMode(interruptPin18, INPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPin18), counter2, RISING);

  Serial.begin (38400);
  
  dht.begin();
  int e;

    Serial.println(F("Republic of IoT"));

  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }

         bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
         Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
         Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
         Adafruit_BMP280::FILTER_X16,      /* Filtering. */
         Adafruit_BMP280::STANDBY_MS_500);

  pinMode(A, INPUT);
  pinMode(B, INPUT);
  pinMode(C, INPUT);
  pinMode(D, INPUT);
  pinMode(E, INPUT);
  pinMode(F, INPUT);
  pinMode(G, INPUT);
  pinMode(H, INPUT);

#ifdef LOW_POWER
  bool low_power_status = IS_LOWPOWER;
#ifdef __SAMD21G18A__
  rtc.begin();
#endif  
#else
  bool low_power_status = IS_NOT_LOWPOWER;
#endif

// Sensor(nomenclature, is_analog, is_connected, is_low_power, pin_read, pin_power, pin_trigger=-1)
   sensor_ptrs[0] = new Temp("TR", IS_ANALOG, IS_CONNECTED, low_power_status, (uint8_t) A0, (uint8_t)5);
   sensor_ptrs[1] = new Humi("HU", IS_ANALOG, IS_CONNECTED, low_power_status,  (uint8_t) A1, (uint8_t) 9);

  delay(3000);
 
#if defined __SAMD21G18A__ && not defined ARDUINO_SAMD_FEATHER_M0 
  SerialUSB.begin(38400);
#else
  Serial.begin(38400);  
#endif 
  // Print a start message
  PRINT_CSTSTR("%s","Generic LoRa sensor\n");

#ifdef ARDUINO_AVR_PRO
  PRINT_CSTSTR("%s","Arduino Pro Mini detected\n");  
#endif
#ifdef ARDUINO_AVR_NANO
  PRINT_CSTSTR("%s","Arduino Nano detected\n");   
#endif
#ifdef ARDUINO_AVR_MINI
  PRINT_CSTSTR("%s","Arduino MINI/Nexus detected\n");  
#endif
#ifdef ARDUINO_AVR_MEGA2560
  PRINT_CSTSTR("%s","Arduino Mega2560 detected\n");  
#endif

#ifdef __AVR_ATmega328P__
  PRINT_CSTSTR("%s","ATmega328P detected\n");
#endif 
#ifdef __AVR_ATmega32U4__
  PRINT_CSTSTR("%s","ATmega32U4 detected\n");
#endif 
#ifdef __AVR_ATmega2560__
  PRINT_CSTSTR("%s","ATmega2560 detected\n");
#endif 

  sx1272.ON();

#ifdef WITH_EEPROM
  EEPROM.get(0, my_sx1272config);

  if (my_sx1272config.flag1==0x12 && my_sx1272config.flag2==0x35) {
    PRINT_CSTSTR("%s","Get back previous sx1272 config\n");

    sx1272._packetNumber=my_sx1272config.seq;
    PRINT_CSTSTR("%s","Using packet sequence number of ");
    PRINT_VALUE("%d", sx1272._packetNumber);
    PRINTLN;

#ifdef FORCE_DEFAULT_VALUE
    PRINT_CSTSTR("%s","Forced to use default parameters\n");
    my_sx1272config.flag1=0x12;
    my_sx1272config.flag2=0x35;
    my_sx1272config.seq=sx1272._packetNumber;
    my_sx1272config.addr=node_addr;
    my_sx1272config.idle_period=idlePeriodInSec;    
    my_sx1272config.overwrite=0;
    EEPROM.put(0, my_sx1272config);
#else
    if (my_sx1272config.addr!=0 && my_sx1272config.overwrite==1) {
      
        PRINT_CSTSTR("%s","Used stored address\n");
        node_addr=my_sx1272config.addr;        
    }
    else
        PRINT_CSTSTR("%s","Stored node addr is null\n"); 

    if (my_sx1272config.idle_period!=0 && my_sx1272config.overwrite==1) {
      
        PRINT_CSTSTR("%s","Used stored idle period\n");
        idlePeriodInSec=my_sx1272config.idle_period;        
    }
    else
        PRINT_CSTSTR("%s","Stored idle period is null\n");                 
#endif  

#ifdef WITH_AES
    DevAddr[3] = (unsigned char)node_addr;
#endif            
    PRINT_CSTSTR("%s","Using node addr of ");
    PRINT_VALUE("%d", node_addr);
    PRINTLN;   

    PRINT_CSTSTR("%s","Using idle period of ");
    PRINT_VALUE("%d", idlePeriodInSec);
    PRINTLN;     
  }
  else {
    my_sx1272config.flag1=0x12;
    my_sx1272config.flag2=0x35;
    my_sx1272config.seq=sx1272._packetNumber;
    my_sx1272config.addr=node_addr;
    my_sx1272config.idle_period=idlePeriodInSec;
    my_sx1272config.overwrite=0;
  }
#endif
  
  e = sx1272.setMode(loraMode);
  PRINT_CSTSTR("%s","Setting Mode: state ");
  PRINT_VALUE("%d", e);
  PRINTLN;

  sx1272._enableCarrierSense=true;
#ifdef LOW_POWER
  sx1272._RSSIonSend=false;
#endif 

  e = sx1272.setChannel(DEFAULT_CHANNEL);
  PRINT_CSTSTR("%s","Setting Channel: state ");
  PRINT_VALUE("%d", e);
  PRINTLN;
  
#ifdef PABOOST
  sx1272._needPABOOST=true;
#else
#endif

  e = sx1272.setPowerDBM((uint8_t)MAX_DBM);
  PRINT_CSTSTR("%s","Setting Power: state ");
  PRINT_VALUE("%d", e);
  PRINTLN;
  
  e = sx1272.setNodeAddress(node_addr);
  PRINT_CSTSTR("%s","Setting node addr: state ");
  PRINT_VALUE("%d", e);
  PRINTLN;
  
  // Print a success message
  PRINT_CSTSTR("%s","SX1272 successfully configured\n");

  delay(500);
}

void loop(void)
{
////////////////// PROGRAM WEATHER STATION /////////////////////

  sensorLDR = analogRead(analogPin);
  outputLDR = map(sensorLDR, 0, 1023, 255, 0);

  unsigned long currentMillis1 = millis();

  if (currentMillis1 - previousMillis1 >= interval1) {
    curah_hujan = (rain_gauge*count1); 
    rpm = (count2/18);
    speedWind = (rpm*(2*pi*r)/60/1000);
    count1 = 0;
    count2 = 0;
    previousMillis1 = currentMillis1;
  }

  unsigned long currentMillis2 = millis();
  if (currentMillis2 - previousMillis2 >= interval2) {
      u = digitalRead(A);
      tl = digitalRead(B);
      t = digitalRead(C);
      tgr = digitalRead(D);
      s = digitalRead(E);
      bd = digitalRead(F);
      b = digitalRead(G);
      bl = digitalRead(H);

        if (u == HIGH) {
        } else{
          windDir = String(windD[0]);
        }
    
        if (tl == HIGH) {
        } else{
          windDir = String(windD[1]);
        }

        if (t == HIGH) {
        } else{
          windDir = String(windD[2]);
        }

        if (tgr == HIGH) {
        } else{
          windDir = String(windD[3]);
        }

        if (s == HIGH) {
        } else{
          windDir = String(windD[4]);
        }

        if (bd == HIGH) {
        } else{
          windDir = String(windD[5]);
        }

        if (b == HIGH) {
        } else{
          windDir = String(windD[6]);
        }

        if (bl == HIGH) {
        } else{
          windDir = String(windD[7]);
        }
    
      previousMillis2 = currentMillis2;
      }

  
  long startSend;
  long endSend;
  uint8_t app_key_offset=0;
  int e;

#ifndef LOW_POWER
  if (millis() > nextTransmissionTime) {
#endif



#ifdef WITH_APPKEY
      app_key_offset = sizeof(my_appKey);
      memcpy(message,my_appKey,app_key_offset);
#endif

      int temp;
      uint8_t r_size;

      int hum1 = dht.readHumidity();
      int temp1 = dht.readTemperature();

      float bmp1 = bmp.readPressure();
      float bmp2 = bmp.readAltitude(1013.25);
    
      char final_str[80]= "\\!";
      char aux[6] = "";
      char dataLDR[5];
      char dataTemp[5];
      char dataHumi[5];
      char dataPress[10];
      char dataAlti[10];
      char dataHujan[5];
      char dataWind[5];
      char dataWindD[20];
      String str1;
      String str2;
      String str3;
      String str4;
      String str5;
      String str6;
      String str7;
      str1 = String(outputLDR);
      str1.toCharArray(dataLDR,5);
      str2 = String(hum1);
      str2.toCharArray(dataHumi,5);
      str3 = String(temp1);
      str3.toCharArray(dataTemp,5);
      str4 = String(bmp1);
      str4.toCharArray(dataPress,10);
      str5 = String(bmp2);
      str5.toCharArray(dataAlti,10);
      str6 = String(curah_hujan);
      str6.toCharArray(dataHujan,5);
      str7 = String(speedWind);
      str7.toCharArray(dataWind,5);
      windDir.toCharArray(dataWindD,20);

      sprintf(final_str, "/e.php?s=%s&t=%s&u=%s&v=%s&w=%s&x=%s&y=%s&z=%s", dataLDR, dataTemp, dataHumi, dataPress, dataAlti, dataHujan, dataWind, dataWindD);

      r_size=sprintf((char*)message+app_key_offset, final_str);

      PRINT_CSTSTR("%s","Reading ");
      //PRINT_VALUE("%d", value);
      PRINTLN;   
      delay(100);

      PRINT_CSTSTR("%s","Sending ");
      PRINT_STR("%s",(char*)(message+app_key_offset));
      PRINTLN;
      
      PRINT_CSTSTR("%s","Real payload size is ");
      PRINT_VALUE("%d", r_size);
      PRINTLN;
      
      int pl=r_size+app_key_offset;
      
      sx1272.CarrierSense();
      
      startSend=millis();

      uint8_t p_type=PKT_TYPE_DATA;
      
#ifdef WITH_APPKEY
      p_type = p_type | PKT_FLAG_DATA_WAPPKEY;
#endif

      sx1272.setPacketType(p_type);
      
#ifdef WITH_ACK
      int n_retry=NB_RETRIES;
      
      do {
        e = sx1272.sendPacketTimeoutACK(DEFAULT_DEST_ADDR, message, pl);

        if (e==3)
          PRINT_CSTSTR("%s","No ACK");
        
        n_retry--;
        
        if (n_retry)
          PRINT_CSTSTR("%s","Retry");
        else
          PRINT_CSTSTR("%s","Abort");  
          
      } while (e && n_retry);          
#else      
      e = sx1272.sendPacketTimeout(DEFAULT_DEST_ADDR, message, pl); //program kirim paket
#endif
  
      endSend=millis();
    
#ifdef WITH_EEPROM
      // save packet number for next packet in case of reboot
      my_sx1272config.seq=sx1272._packetNumber;
      EEPROM.put(0, my_sx1272config);
#endif
      
      PRINT_CSTSTR("%s","LoRa pkt seq ");
      PRINT_VALUE("%d", sx1272.packet_sent.packnum);
      PRINTLN;
    
      PRINT_CSTSTR("%s","LoRa Sent in ");
      PRINT_VALUE("%ld", endSend-startSend);
      PRINTLN;
          
      PRINT_CSTSTR("%s","LoRa Sent w/CAD in ");
      PRINT_VALUE("%ld", endSend-sx1272._startDoCad);
      PRINTLN;

      PRINT_CSTSTR("%s","Packet sent, state ");
      PRINT_VALUE("%d", e);
      PRINTLN;

#ifdef WITH_RCVW
      PRINT_CSTSTR("%s","Wait for ");
      PRINT_VALUE("%d", DELAY_BEFORE_RCVW-1000);
      PRINTLN;
      //wait a bit
      delay(DELAY_BEFORE_RCVW-1000);

      PRINT_CSTSTR("%s","Wait for incoming packet\n");
      // wait for incoming packets
      e = sx1272.receivePacketTimeout(10000);
    
      if (!e) {
         int i=0;
         int cmdValue;
         uint8_t tmp_length;

         sx1272.getSNR();
         sx1272.getRSSIpacket();
         
         tmp_length=sx1272._payloadlength;

         sprintf((char*)message, "^p%d,%d,%d,%d,%d,%d,%d\n",
                   sx1272.packet_received.dst,
                   sx1272.packet_received.type,                   
                   sx1272.packet_received.src,
                   sx1272.packet_received.packnum, 
                   tmp_length,
                   sx1272._SNR,
                   sx1272._RSSIpacket);
                                   
         PRINT_STR("%s",(char*)message);         
         
         for ( ; i<tmp_length; i++) {
           PRINT_STR("%c",(char)sx1272.packet_received.data[i]);
           
           message[i]=(char)sx1272.packet_received.data[i];
         }
         
         message[i]=(char)'\0';    
         PRINTLN;
         FLUSHOUTPUT;   

        i=0;

        if (message[i]=='/' && message[i+1]=='@') {
    
            PRINT_CSTSTR("%s","Parsing command\n");      
            i=i+2;   

            switch ((char)message[i]) {

                  case 'A': 

                      i++;
                      cmdValue=getCmdValue(i);
                      
                      if (cmdValue > 255)
                              cmdValue = 255;
                      if (cmdValue < 2)
                              cmdValue = node_addr;
                      node_addr=cmdValue; 
#ifdef WITH_AES
                      DevAddr[3] = (unsigned char)node_addr;
#endif
                      
                      PRINT_CSTSTR("%s","Set LoRa node addr to ");
                      PRINT_VALUE("%d", node_addr);  
                      PRINTLN;
                      e = sx1272.setNodeAddress(node_addr);
                      PRINT_CSTSTR("%s","Setting LoRa node addr: state ");
                      PRINT_VALUE("%d",e);     
                      PRINTLN;           

#ifdef WITH_EEPROM
                      my_sx1272config.addr=node_addr;
                      my_sx1272config.overwrite=1;
                      EEPROM.put(0, my_sx1272config);
#endif

                      break;        

                  case 'I': 

                      i++;
                      cmdValue=getCmdValue(i);

                      if (cmdValue < 1)
                              cmdValue = idlePeriodInSec;
                      idlePeriodInSec=cmdValue; 
                      
                      PRINT_CSTSTR("%s","Set duty-cycle to ");
                      PRINT_VALUE("%d", idlePeriodInSec);  
                      PRINTLN;         

#ifdef WITH_EEPROM
                      // save new node_addr in case of reboot
                      my_sx1272config.idle_period=idlePeriodInSec;
                      my_sx1272config.overwrite=1;
                      EEPROM.put(0, my_sx1272config);
#endif

                      break;  
 
                  default:
      
                    PRINT_CSTSTR("%s","Unrecognized cmd\n");       
                    break;
            }
        }          
      }
      else
        PRINT_CSTSTR("%s","No packet\n");
#endif

#ifdef LOW_POWER
      PRINT_CSTSTR("%s","Switch to power saving mode\n");

      e = sx1272.setSleepMode();

      if (!e)
        PRINT_CSTSTR("%s","Successfully switch LoRa module in sleep mode\n");
      else  
        PRINT_CSTSTR("%s","Could not switch LoRa module in sleep mode\n");
        
      FLUSHOUTPUT

#ifdef __SAMD21G18A__
      // For Arduino M0 or Zero we use the built-in RTC
#else      
      nCycle = idlePeriodInSec*1000/LOW_POWER_PERIOD + random(2,4);
      for (int i=0; i<nCycle; i++) {  

#if defined ARDUINO_AVR_PRO || defined ARDUINO_AVR_NANO || defined ARDUINO_AVR_UNO || defined ARDUINO_AVR_MINI || defined __AVR_ATmega32U4__         
          LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_OFF);

#elif defined ARDUINO_AVR_MEGA2560
          LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_OFF);
          
#elif defined __MK20DX256__ || defined __MKL26Z64__ || defined __MK64FX512__ || defined __MK66FX1M0__  
#ifdef LOW_POWER_HIBERNATE
          Snooze.hibernate(sleep_config);
#else            
          Snooze.deepSleep(sleep_config);
#endif  
#else
          delay(LOW_POWER_PERIOD*1000);
#endif                        
          PRINT_CSTSTR("%s",".");
          FLUSHOUTPUT
          delay(10);                        
      }
      
      delay(50);
#endif 

#else
      PRINT_VALUE("%ld", nextTransmissionTime);
      PRINTLN;
      PRINT_CSTSTR("%s","Will send next value at\n");
      nextTransmissionTime=millis()+(unsigned long)idlePeriodInSec*1000+(unsigned long)random(15,60);
      PRINT_VALUE("%ld", nextTransmissionTime);
      PRINTLN;
  }
#endif
} 

void counter1() {
  count1++;
  }

void counter2() {
  count2++;
  }
