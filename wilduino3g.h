#pragma once

/************************************************************************/
// 
//
//
/************************************************************************/
#if (ARDUINO >= 100)
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <limits.h>
#include <Wire.h>
#include <SdFat.h>
#include <ds3231.h>

#define RESP_SZ 200
#define DEV_ID_EEPROM_LOC 0x00
#define DEFAULT_TIMEOUT 5000
#define MAX_HTTP_RETRIES 5
#define MAX_POWER_RETRIES 5
#define VCC 3.3
#define AREF 2.5
#define CTRLZ 0x1A

// for SIM5320 modem type
enum
{
    SIM5320A,
    SIM5320E,
    SIM5320J
};


// for alarm
enum
{
    EVERY_SECOND,
    EVERY_MINUTE,
    EVERY_HOUR     ,
    EVERY_DAY    ,
    EVERY_WEEK    ,
    EVERY_MONTH    
};

enum
{
    NO_ALARM = 0,
    MINUTE_ALARM = 1,
    HOUR_ALARM = 2
};

enum
{
    STATE_MODULE_POWER_OFF = 0,
    STATE_MODULE_POWER_ON,
    STATE_MODULE_POWER_ON_WAIT,
    STATE_MODULE_READY,
    STATE_MODULE_SHUTTING_DOWN
};

class Wilduino3G 
{
public:
    // inputs
    const uint8_t pinPwrStatus      = 15;
    const uint8_t pinRI             = 23;
    const uint8_t pinVsolSense      = 28;
    const uint8_t pinNwkStatus      = 29;
    const uint8_t pinVbatSense      = 31;

    // outputs
    const uint8_t pinWake           = 14;
    const uint8_t pinPwrButton      = 21;
    const uint8_t pinDtr            = 22;
    const uint8_t pinFlightMode     = 30;
    const uint8_t pinRtcIntp        = 6;
    const uint8_t pin3GRstN         = 8;
    const uint8_t pinSdSeln         = 19;
    const uint8_t pinLevelSensor    = 24;
    const uint8_t pinArefEnb        = 20;

    const uint8_t intpNumRtc        = 2;

    char respBuf[RESP_SZ];
    uint8_t rssi;
    boolean sleeping = false;
    uint8_t retries = 0;
    SdFat sd;
    SdFile file;

    Wilduino3G();
    boolean begin(HardwareSerial *port, HardwareSerial *debug);
    uint8_t getIntp();
    static void rtcIntp();
    boolean rtcIntpRcvd();
    void poll();

    void drvrSend(const char* command);
    boolean drvrCheckResp(const char *expected, uint32_t timeout);
    boolean drvrCheckOK(uint32_t timeout);
    void drvrDumpResp();
    void drvrFlush();
    void drvrSleepMcu();
    uint32_t drvrElapsedTime(uint32_t);
    void drvrCmdEcho(boolean enable);
    void drvrTogglePower();
    boolean drvrPowerOff();
    boolean drvrInitDone();
    boolean drvrCheckReady();
    boolean drvrReset();
    void drvrHardReset();
    void drvrFlightMode(boolean enable);
    float drvrGetVbat();
    float drvrGetVsol();
    void drvrWriteDevId(uint16_t id);
    uint16_t drvrReadDevId();

    void mgmtGetInfo();
    int8_t mgmtGetRSSI();
    uint8_t mgmtGetNwkReg();

    int8_t httpOpen(const char *url, uint16_t port);
    boolean httpSend(const char *dir, const char *url, const char *data, uint16_t len);
    boolean httpGet(const char *dir, const char *url);
    boolean httpResp(const char *resp);

//    void httpStart();
//    void httpOpenSession(const char *url, uint16_t port, boolean ssl);
//    void httpSend(const char *data, uint32_t timeout);
//    void httpRecv(uint16_t size);

    boolean sdBegin(uint8_t csPin);
    boolean sdOpen(const char *filename, uint8_t mode = O_RDWR | O_CREAT | O_APPEND);
    void sdLs();
    boolean sdMkDir(const char *filepath);
    boolean sdExists(const char *filepath);
    boolean sdChDir(const char *filepath);    
    int16_t sdRead();
    void sdWrite(const char *data);
    void sdClose();
    void sdRemove(const char *filename);
    uint32_t sdAvailable();
    void sdLogMsg(const char *filename, const char *msg);
    void sdLogTimestampedMsg(const char *filename, const char *msg);
    static void sdDateTime(uint16_t *date, uint16_t *time);

    void rtcSetTime(int yr, int month, int day, int hour, int min, int sec);
    static struct ts rtcGetTime();
    void rtcPrintTime(char *datetime);
    void rtcPrintDate(char *datetime);
    void rtcPrintTimeAndDate(char *datetime);
    void rtcPrintFullTime(char *datetime);
    void rtcSetAlarm(uint8_t alarm, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec, uint8_t repeat);
    void rtcGetAlarm(uint8_t alarm, uint8_t *data);
    uint8_t rtcGetStatus();
    uint8_t rtcGetControl();
    void rtcSetStatus(uint8_t reg);
    void rtcClearAlarm(uint8_t alarm);
    void rtcEnableAlarm(uint8_t alarm);
    void rtcDisableAlarm(uint8_t alarm);
    float rtcGetTemp();

    uint8_t getModuleState();
    void stateModulePwrOn();
    void stateModulePwrOff();
    uint8_t drvrGetPwrStatus();
    void pollModuleFSM();


private:


    HardwareSerial *ser;
    HardwareSerial *dbg;
};