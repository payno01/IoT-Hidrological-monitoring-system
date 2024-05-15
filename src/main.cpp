//Device type: 
#define ULTRASONIC

#define THINGER_SERVER "" //HERE THINGER SERVER
//#define THINGER_SERVER ""

#define THINGER_SERIAL_DEBUG
//#define _DISABLE_TLS_
#include <Arduino.h>
#include <EEPROM.h>

// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial
#define SerialAT Serial1

// Select your modem:
#define TINY_GSM_MODEM_BC660
#define TINY_GSM_DEBUG SerialMon
//#define DUMP_AT_COMMANDS
#ifndef TINY_GSM_RX_BUFFER
#define TINY_GSM_RX_BUFFER 1024
#endif

//set hibernation parameters
#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  10        /* Time ESP32 will go to sleep (in seconds) */

// Can be installed from Library Manager or https://github.com/vshymanskyy/TinyGSM
#include <TinyGsmClient.h>
#include <ThingerTinyGSM.h>
#include <ThingerESP32OTA.h>
#include "arduino_secrets.h"


#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(Serial1, Serial);
ThingerTinyGSM thing(USERNAME, DEVICE_ID, DEVICE_CREDENTIAL, debugger);
#else
ThingerTinyGSM thing(USERNAME, DEVICE_ID, DEVICE_CREDENTIAL, SerialAT);
#endif

ThingerESP32OTA ota(thing);
String iccid;

// module config version
#define MODULE_CONFIG_VERSION 3

// hexacore pins
#define HEXACORE_DEVELOPMENT
#ifdef HEXACORE_DEVELOPMENT
   #define PIN_MODEM_RESET 13
   #define PIN_MODEM_WAKEUP 12
   #define PIN_MODEM_PWR_KEY 19
   #define PIN_MODULE_LED 21
   #define PIN_MODEM_RX 32
   #define PIN_MODEM_TX 33

   #define RELAY_PIN 27
   #define DOOR_STATE_PIN 14

   //Pins para ultrasonidos
   #define TRIG_PIN 21
   #define ECHO_PIN 22

// M.2 pins
#else
   #define PIN_MODEM_RESET 5
   #define PIN_MODEM_WAKEUP 18
   #define PIN_MODEM_PWR_KEY 23
   #define PIN_MODULE_LED 21
   #define PIN_MODEM_RX 4
   #define PIN_MODEM_TX 13

   #define RELAY_PIN 27
   #define DOOR_STATE_PIN 12
#endif



// other variables

int data_is_sent=0;
float measurements[4]={0,0,0,0};

void read_sensor_ultrasonic(){
  float av=0, avq=0;

   for(int i=1; i<5; i++){

      digitalWrite(TRIG_PIN, LOW);
      delayMicroseconds(2);
      digitalWrite(TRIG_PIN, HIGH);
      delayMicroseconds(10);
      digitalWrite(TRIG_PIN, LOW);
      unsigned long aux = pulseIn(ECHO_PIN, HIGH);
      Serial.print("distance: ");
      Serial.print(aux);
      
      delay(100);
      av+=(aux-av)/i;
      avq+=(av*av-avq)/i;
      Serial.print(" - ");
      Serial.println(av);

     }
      measurements[0]=av;
      measurements[1]=avq;

}

void read_sensor_soil(){

 float av=0, av2=0, avq=0, avq2=0;

 for(int i=1; i<10; i++){
   
       av+=(analogRead(14)-av)/i;
       av2+=(analogRead(25)-av2)/i;

       avq+=(av*av-avq)/i;
       avq2*=(av2*av2-avq2)/i;

       delay(100);
     }
     
   measurements[0]=av;
   measurements[1]=avq-av*av;
   measurements[2]=av2;
   measurements[3]=avq2-av2*av2;

}


void setup() {
   pinMode(PIN_MODEM_RESET,   OUTPUT);
   pinMode(PIN_MODEM_WAKEUP,  OUTPUT);
   pinMode(PIN_MODEM_PWR_KEY, OUTPUT);
   pinMode(DOOR_STATE_PIN, INPUT);
   pinMode(RELAY_PIN, OUTPUT);

   pinMode(TRIG_PIN, OUTPUT);
   pinMode(ECHO_PIN, INPUT);

   pinMode(14, INPUT);
   pinMode(25, INPUT);


   Serial.begin(115200);
   Serial1.begin(115200, SERIAL_8N1, PIN_MODEM_RX, PIN_MODEM_TX);

   delay(400);

   thing["data"] >> [](pson & out){
      out["voltaje"]= thing.getTinyGsm().getBattVoltage();

      #ifdef ULTRASONIC
         out["sensor1"] = measurements[0];
         out["var_s1"] = measurements[1];
      #else
         out["sensor1"] = measurements[0];
         out["var_s1"] = measurements[1];
         out["sensor2"] = measurements[2];
         out["var_s2"] = measurements[3];

      #endif
      data_is_sent=1;
   };

   thing["modem"] >> [](pson & out){
      out["modem"] =    thing.getTinyGsm().getModemInfo().c_str();
      out["IMEI"] =     thing.getTinyGsm().getIMEI().c_str();
      out["CCID"] =     thing.getTinyGsm().getSimCCID().c_str();
      out["operator"] = thing.getTinyGsm().getOperator().c_str();
   };

   //  set APN
   thing.setAPN(APN_NAME, APN_USER, APN_PSWD);

   // configure hardware module reset
   thing.setModuleReset([]{
      // modem reset
      digitalWrite(PIN_MODEM_RESET, 1);
      delay(100);
      digitalWrite(PIN_MODEM_RESET, 0);
   });

   thing.initModem([](TinyGsm& modem){
      // read SIM ICCID
      iccid = modem.getSimCCID();
      THINGER_DEBUG_VALUE("NB-IOT", "SIM ICCID: ", iccid.c_str());

      // disable power save mode
      modem.sendAT("+CPSMS=0");
      modem.waitResponse();

      // disable eDRX
      modem.sendAT("+CEDRXS=0");
      modem.waitResponse();

      // edRX and PTW -> disabled
      modem.sendAT("+QEDRXCFG=0");
      modem.waitResponse();

      // initialize module configuration for the first time
      EEPROM.begin(1);
      auto state = EEPROM.readByte(0);
      if(state!=MODULE_CONFIG_VERSION){
         THINGER_DEBUG_VALUE("NB-IOT", "Configuring module with version: ", MODULE_CONFIG_VERSION);

         // stop modem functionality
         modem.sendAT("+CFUN=0");
         modem.waitResponse();

         // configure APN
         modem.sendAT("+QCGDEFCONT=\"IP\",\"" APN_NAME "\"");
         modem.waitResponse();

         // preferred search bands (for Spain)
         modem.sendAT("+QBAND=3,20,8,3");
         modem.waitResponse();

         // set preferred operators
         modem.sendAT("+COPS=4,2,\"21407\"");  // movistar
         //modem.sendAT("+COPS=4,2,\"21401\"");  // vodafone
         modem.waitResponse();

         // enable net led
         modem.sendAT("+QLEDMODE=1");
         modem.waitResponse();

         // full functionality
         modem.sendAT("+CFUN=1");
         modem.waitResponse();

         EEPROM.writeByte(0, MODULE_CONFIG_VERSION);
         EEPROM.commit();
      }else{
         THINGER_DEBUG_VALUE("NB-IOT", "Module already configured with version: ", MODULE_CONFIG_VERSION);
      }
      EEPROM.end();
   });

   // configure ota block size to 
   ota.set_block_size(512);

   thing.set_credentials(USERNAME, iccid.c_str(), iccid.c_str());

 #ifdef ULTRASONIC
      read_sensor_ultrasonic();
   #else
      int waste = analogRead(14);
      waste = analogRead(25);
      delay(500);
   #endif

}

void loop() {

   //retrieve sensors data
   #ifdef ULTRASONIC
      read_sensor_ultrasonic();
   #else
      read_sensor_soil();
   #endif

   // iotmp handle, modem pwr on
  thing.handle();

  //  stream data to the platform bucket
  thing.stream("data");

   if(data_is_sent){
      pson data;                                 
      thing.get_property("sleeping_time", data); //retrieving data from the platform
      int sleeping_time=data["time"];
      if(sleeping_time!=0){
        Serial.print("going to sleep ");
        Serial.print(sleeping_time);
        Serial.println(" microseconds");
        thing.getTinyGsm().setPhoneFunctionality(false);
        thing.getTinyGsm().sleepEnable(true);
        esp_sleep_enable_timer_wakeup(sleeping_time * uS_TO_S_FACTOR);
        esp_deep_sleep_start();
      }
   }else if(millis()>150000){
      Serial.print("force to sleep 15 mins");
      thing.getTinyGsm().setPhoneFunctionality(false);
      thing.getTinyGsm().sleepEnable(true);
      esp_sleep_enable_timer_wakeup(900 * uS_TO_S_FACTOR);
      esp_deep_sleep_start();
   }
 
}
