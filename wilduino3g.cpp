/************************************************************************/
// 
//
//
/************************************************************************/
#include "wilduino3g.h"

#define DEBUG 1

// allows printing or not printing based on the DEBUG VAR
#if (DEBUG == 1)
  #define DBG_PRINT(...)   dbg->print(__VA_ARGS__)
  #define DBG_PRINTLN(...) dbg->println(__VA_ARGS__)
  #define DBG_PRINTF(...)   printf(__VA_ARGS__)
#else
  #define DBG_PRINT(...)
  #define DBG_PRINTLN(...)
  #define DBG_PRINTF(...) 
#endif

#define MAX_POWER_WAIT 10000

static char tmp[500];
volatile boolean rtcFlag = false;
uint8_t pwrRetries;

uint8_t state = STATE_MODULE_POWER_OFF;
uint8_t nextState = STATE_MODULE_POWER_OFF;
uint32_t now = 0;

/************************************************************************/
// 
//
//
/************************************************************************/
Wilduino3G::Wilduino3G()
{
    rtcFlag = false;
}

/************************************************************************/
// 
//
//
/************************************************************************/
boolean Wilduino3G::begin(HardwareSerial *port, HardwareSerial *debug)
{
    uint8_t reg;

    ser = port;
    dbg = debug;

    ser->begin(115200);
    dbg->begin(57600);

    pinMode(pinPwrButton, OUTPUT);
    digitalWrite(pinPwrButton, HIGH);

    pinMode(pinDtr, OUTPUT);
    digitalWrite(pinDtr, HIGH);

    pinMode(pinFlightMode, OUTPUT);
    digitalWrite(pinFlightMode, HIGH);

    pinMode(pinWake, OUTPUT);
    digitalWrite(pinWake, HIGH);
    
    pinMode(pin3GRstN, OUTPUT);
    digitalWrite(pin3GRstN, HIGH);

    pinMode(pinArefEnb, OUTPUT);
    digitalWrite(pinArefEnb, LOW);

    DS3231_init(DS3231_INTCN);

    for (int i=0; i<2; i++)
    {
        rtcClearAlarm(i+1);
    }
    reg = rtcGetStatus();
    reg &= ~(DS3231_OSF | DS3231_EN32K);
    rtcSetStatus(reg);

    attachInterrupt(intpNumRtc, Wilduino3G::rtcIntp, FALLING); 

    sdBegin(pinSdSeln);
    file.dateTimeCallback(sdDateTime);

    // use external 2.5V reference
    analogReference(EXTERNAL);

    DBG_PRINTLN(F("Wilduino 3G Driver v0.5"));
    DBG_PRINTLN(F("Initialization complete"));

    return true;
}

/************************************************************************/
// 
//
//
/************************************************************************/
void Wilduino3G::poll()
{
    /*
    if (ser->available())
    {
        dbg->write(ser->read());
    }
    */
}

/************************************************************************/
// 
//
//
/************************************************************************/
uint8_t Wilduino3G::getIntp()
{
    if (rtcFlag)
    {
        uint8_t alm;
        uint8_t status = rtcGetStatus();

        status &= (DS3231_A2F | DS3231_A1F);

        if (status & DS3231_A1F)
        {
            alm = MINUTE_ALARM;
            status &= ~DS3231_A1F;
        }
        else if (status & DS3231_A2F)
        {
            alm = HOUR_ALARM;
            status &= ~DS3231_A2F;
        }

        // if we get two interrupts, we don't clear the rtcFlag until all interrupts
        // have been serviced.
        if (status == 0)
        {
            rtcFlag = false;
        }

        rtcClearAlarm(alm);
        rtcEnableAlarm(alm);
        return alm;
    }
    else
    {
        return 0;
    }
}

/********************************************************************/
//
//
//
/********************************************************************/
void Wilduino3G::rtcIntp()
{
  rtcFlag = true;
}

/********************************************************************/
//
//
//
/********************************************************************/
boolean Wilduino3G::rtcIntpRcvd()
{
    return rtcFlag;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 3G Driver related functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/********************************************************************/
//
//  pollModuleFSM()
//  Module state machine handler
//
/********************************************************************/
void Wilduino3G::pollModuleFSM()
{
    wdt_reset();
    switch (state)
    {
        case STATE_MODULE_POWER_OFF:
            pwrRetries = 0;
            if (drvrGetPwrStatus() == HIGH)
            {
                // power is on
                DBG_PRINTLN(F("STATE_MODULE_POWER_OFF: Module shutting down."));
                if (drvrPowerOff())
                {
                    nextState = STATE_MODULE_SHUTTING_DOWN;
                    now = millis();
                }
            }
        break;

        case STATE_MODULE_POWER_ON:
            if (pwrRetries < MAX_POWER_RETRIES)
            {
                DBG_PRINTLN(F("Turning ON"));
                dbg->flush();
                drvrTogglePower();
                now = millis();
                nextState = STATE_MODULE_POWER_ON_WAIT;
            }
            else
            {
                DBG_PRINTLN(F("ERROR: Module Not Powered On"));
                DBG_PRINTLN(F("STATE_MODULE_POWER_ON: Transitioning to STATE_MODULE_SHUTTING_DOWN"));
                drvrPowerOff();
                pwrRetries = 0;
                nextState = STATE_MODULE_SHUTTING_DOWN;
                now = millis();
            }
        break;

        case STATE_MODULE_POWER_ON_WAIT:
            if (drvrElapsedTime(now) < MAX_POWER_WAIT)
            {
                wdt_reset();

                // check if module is powered up
                if (drvrGetPwrStatus() == HIGH)
                {          
                    // check if init sequence done
                    if (drvrInitDone())
                    {
                        // check if command line is responsive
                        if (drvrCheckReady())
                        {
                            nextState = STATE_MODULE_READY;
                            DBG_PRINTLN(F("STATE_MODULE_POWER_ON: Transitioning to STATE_MODULE_READY"));
                        }  
                    }      
                } 
            }
            else
            {
                DBG_PRINTF("Failed. Retrying on retry #%d.\n", pwrRetries);
                DBG_PRINTLN(F("STATE_MODULE_POWER_ON: Transitioning to STATE_MODULE_POWER_ON"));
                nextState = STATE_MODULE_POWER_ON;
                pwrRetries++;
            } 
        break;

        case STATE_MODULE_READY:
        break;

        case STATE_MODULE_SHUTTING_DOWN:
            if (drvrElapsedTime(now) < MAX_POWER_WAIT)
            {
                if (drvrGetPwrStatus() == LOW)
                {                
                    nextState = STATE_MODULE_POWER_OFF;
                    DBG_PRINTLN(F("STATE_MODULE_SHUTTING_DOWN: Transitioning to STATE_MODULE_POWER_OFF"));
                } 
            }
            else
            {
                if (drvrGetPwrStatus() == HIGH)
                {
                    drvrPowerOff();
                    now = millis();
                }
            }
        break;

        default:
            nextState = STATE_MODULE_POWER_OFF;
            DBG_PRINTLN(F("ERROR: We are in DEFAULT case"));
            DBG_PRINTLN(F("DEFAULT CASE: Transitioning to STATE_MODULE_POWER_OFF"));
        break;
    }
    state = nextState;
}

/************************************************************************/
//    
//    
/************************************************************************/
uint8_t Wilduino3G::getModuleState()
{
     return state;
}

/************************************************************************/
//    
//    
/************************************************************************/
void Wilduino3G::stateModulePwrOn()
{
    nextState = STATE_MODULE_POWER_ON;
}

/************************************************************************/
//    
//    
/************************************************************************/
void Wilduino3G::stateModulePwrOff()
{
    nextState = STATE_MODULE_SHUTTING_DOWN;
    drvrPowerOff();
    now = millis();
}

/************************************************************************/
//    getPwrStatus
//    Get power status of module
/************************************************************************/
uint8_t Wilduino3G::drvrGetPwrStatus()
{
     return digitalRead(pinPwrStatus);
}

/************************************************************************/
//    flushSerInput
//    Removes any data from receive input
/************************************************************************/
void Wilduino3G::drvrFlush()
{
    while (ser->available() > 0)
    {
        ser->read();
    }
}

/************************************************************************/
//    reset 3G Module
//    Reset 3G module
/************************************************************************/
void Wilduino3G::drvrHardReset()
{
    digitalWrite(pin3GRstN, LOW);
    delay(500);
    digitalWrite(pin3GRstN, HIGH);
    delay(500);
    while (!drvrInitDone()) 
    {
        ;
    }
}


/************************************************************************/
// 
//
//
/************************************************************************/
void Wilduino3G::drvrCmdEcho(boolean enable) 
{
    if (enable)
    {
        drvrSend("ATE1\r\n");
    }
    else
    {
        drvrSend("ATE0\r\n");
    }
    drvrCheckOK(2000);
}

/************************************************************************/
// 
//
//
/************************************************************************/
void Wilduino3G::drvrSend(const char* command) 
{
    drvrFlush();

    sprintf(tmp, "%s\r\n", command);
    ser->print(tmp);
}

/************************************************************************/
// 
//
//
/************************************************************************/
boolean Wilduino3G::drvrCheckResp(const char *expected, uint32_t timeout)
{
    uint32_t now;
    boolean respRcvd = false;
    uint16_t idx = 0;

    memset(respBuf, 0, sizeof(respBuf)); // clear buffer to allow for string functions
      
    // check for response
    now = millis();
    while (drvrElapsedTime(now) < timeout)
    {
        if (ser->available() > 0)
        {
            char c = ser->read();
            respBuf[idx++] = c;
            if (idx == RESP_SZ-1)
            {
                // too much data in response buffer
                break;
            }

            if (c == '\n')
            {
                idx = 0 ; // reset the index
                if (strcmp(respBuf, expected) == 0)
                {
                    // we have a match
                    sprintf(tmp, "Correct response received: %s\r\n", respBuf);
                    dbg->print(tmp);
                    respRcvd = true;
                    break;
                } 
                memset(respBuf, 0, sizeof(respBuf));
            }
        }
    }
    drvrFlush();
    return respRcvd;  
}

/************************************************************************/
// 
//  
//
/************************************************************************/
boolean Wilduino3G::drvrCheckOK(uint32_t timeout)
{
   
    if (drvrCheckResp("OK\r\n", timeout))
    {
      //DBG_PRINTLN("Command Success");
      return true;
    }
    else
    {
      DBG_PRINTLN("Command Failed");
      return false;
    }
}

/************************************************************************/
// 
//
//
/************************************************************************/
void Wilduino3G::drvrDumpResp()
{
  dbg->print(respBuf);
}

/**************************************************************************/
// 
/**************************************************************************/
void Wilduino3G::drvrSleepMcu()
{
    uint8_t portCReg, portDReg;
    uint8_t ddrCReg, ddrDReg;

    // LOG


    // disable UARTs
    UCSR0B = 0x00;
    UCSR1B = 0x00;

    // set all inputs
    portCReg = PORTC;
    ddrCReg = DDRC;
    PORTC = 0x00;
    DDRC = 0x00; 

    portDReg = PORTD;
    ddrDReg = DDRD;
    PORTD = 0x00;
    DDRD = 0x00;

    delay(100);

    ADCSRA &= ~(1 << ADEN);    // Disable ADC

    // write sleep mode
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    wdt_disable();      // disable watchdog while sleeping
    sleep_enable();     // setting up for sleep ...
    sleep_mode();
    // sleeping here

    // waking up here
    UCSR0B = 0x98;
    UCSR1B = 0x98;
    ADCSRA |= (1 << ADEN); 

    PORTC = portCReg;
    DDRC = ddrCReg;

    PORTD = portDReg;
    DDRD = ddrDReg;
}

/**************************************************************************/
// 
/**************************************************************************/
float Wilduino3G::drvrGetVbat()
{
    uint16_t bat;
    float volts;

    digitalWrite(pinArefEnb, HIGH);
    delay(50);
    bat = analogRead(pinVbatSense);
    volts = (float)(bat * AREF/1023.0);
    digitalWrite(pinArefEnb, LOW);
    return (volts * 2.0);
}   

/**************************************************************************/
// 
/**************************************************************************/
float Wilduino3G::drvrGetVsol()
{
    uint16_t sol;
    float volts;

    digitalWrite(pinArefEnb, HIGH);
    delay(50);
    sol = analogRead(pinVsolSense);
    volts = (float)(sol*AREF/1023.0);
    digitalWrite(pinArefEnb, LOW);
    return (volts * 2.0);
}   

/**************************************************************************/
// 
/**************************************************************************/
void Wilduino3G::drvrWriteDevId(uint16_t id)
{
    while(!eeprom_is_ready());
    eeprom_write_block(&id, DEV_ID_EEPROM_LOC, 2);
}

/**************************************************************************/
// 
/**************************************************************************/
uint16_t Wilduino3G::drvrReadDevId()
{
    uint16_t id;
    while(!eeprom_is_ready());
    eeprom_read_block(&id, DEV_ID_EEPROM_LOC, 2);
    return id;
}

/**************************************************************************/
// 
/**************************************************************************/
void Wilduino3G::drvrTogglePower()
{
    // turn on power switch for modem
    digitalWrite(pinPwrButton, LOW);
    delay(1000);
    digitalWrite(pinPwrButton, HIGH);
}

/**************************************************************************/
// 
/**************************************************************************/
boolean Wilduino3G::drvrPowerOff()
{
    drvrSend("AT+CPOF\r\n");
    if (drvrCheckOK(2000))
    {
        return true;
    }
    return false;
}

/**************************************************************************/
// 
/**************************************************************************/
boolean Wilduino3G::drvrInitDone()
{
    // check to see if it powered up properly
    if (drvrCheckResp("PB DONE\r\n", 7000))
    {
        return true;
    }
    return false;
}

/**************************************************************************/
// 
/**************************************************************************/
boolean Wilduino3G::drvrCheckReady()
{
    drvrSend("AT\r\n");
    if (drvrCheckOK(2000))
    {
        return true;
    }
    return false;
}

/**************************************************************************/
// 
/**************************************************************************/
boolean Wilduino3G::drvrReset()
{
    drvrSend("AT+CRESET");
    return (drvrCheckOK(DEFAULT_TIMEOUT));
}

/************************************************************************/
// elapsedTime - calculates time elapsed from startTime
// startTime : time to start calculating
/************************************************************************/
uint32_t Wilduino3G::drvrElapsedTime(uint32_t startTime)
{
  uint32_t stopTime = millis();
  
  if (stopTime >= startTime)
  {
    return stopTime - startTime;
  }
  else
  {
    return (ULONG_MAX - (startTime - stopTime));
  }
}

/**************************************************************************/
// 
/**************************************************************************/
void Wilduino3G::drvrFlightMode(boolean enable)
{
    uint8_t val = (enable == false);
    digitalWrite(pinFlightMode, val);
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 3G Management related functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/********************************************************************/
//    mgmtGetInfo:
//    get info from modules
//
/********************************************************************/
void Wilduino3G::mgmtGetInfo()
{
    DBG_PRINTLN("Sending ATI command");
    drvrSend("ATI\r\n");
    drvrCheckOK(2000);
}

/********************************************************************/
//    

//
/********************************************************************/
int8_t Wilduino3G::mgmtGetRSSI()
{
    uint32_t now;
    drvrSend("AT+CSQ\r\n");
    now = millis();

    while (drvrElapsedTime(now) < DEFAULT_TIMEOUT)
    {
        if (ser->available() > 0)
        {
            String str = ser->readStringUntil('\n');
            //dbg->print("Signal Quality Received: ");
            //dbg->println(str);
            
            if (str.indexOf("+CSQ:") != -1)
            {
                String substr = str.substring(str.indexOf(':')+1, str.indexOf(','));
                substr.trim();
                //printf("Correct response received: %s\r\n", substr.c_str());
                return substr.toInt();
            }
        }
    }    
    return -1;
}

/********************************************************************/
//    

//
/********************************************************************/
uint8_t Wilduino3G::mgmtGetNwkReg()
{
    uint32_t now;
    drvrSend("AT+CREG?\r\n");
    now = millis();

    while (drvrElapsedTime(now) < DEFAULT_TIMEOUT)
    {
        if (ser->available() > 0)
        {
            String str = ser->readStringUntil('\n');
            
            if (str.indexOf("+CREG:") != -1)
            {
                String substr = str.substring(str.indexOf(',')+1, str.indexOf('\n'));
                substr.trim();
                return substr.toInt();
            }
        }
    }    
    return 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// HTTP related functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/********************************************************************/
//
//
//
/********************************************************************/
int8_t Wilduino3G::httpOpen(const char *url, uint16_t port)
{
    for (retries=0; retries<MAX_HTTP_RETRIES; retries++)
    {
        wdt_reset();
        sprintf(tmp, "AT+CHTTPACT=\"%s\",%d\r\n", url, port);
        dbg->print(tmp);
        ser->print(tmp);
        if (drvrCheckResp("+CHTTPACT: REQUEST\r\n", 7000))
        {
            return retries;
        }
    }
    dbg->print("Opening site failed\n");
    return -1;
}

/********************************************************************/
//
//
//
/********************************************************************/
boolean Wilduino3G::httpSend(const char *dir, const char *url, const char *data, uint16_t len)
{
    char httpReq[1000];

    sprintf(httpReq, "POST %s HTTP/1.1\r\nHost: %s\r\nContent-Type: text/csv\r\nContent-Length: %d\r\n\r\n%s\r\n\r\n", dir, url, len, data);
    dbg->print(httpReq);
    ser->print(httpReq);
    ser->write(CTRLZ); // terminate request

    if (drvrCheckResp("http/1.1 201 created\r\n", DEFAULT_TIMEOUT))
    {
        delay(1000);
        ser->flush();
        return true;
    }
    else
    {
        delay(1000);
        ser->flush();
        return false;
    }           
}

/********************************************************************/
//
//
//
/********************************************************************/
boolean Wilduino3G::httpGet(const char *dir, const char *url)
{
    char httpReq[500];
    sprintf(httpReq, "GET %s HTTP/1.1\r\nHost: %s\r\nContent-Length: 0\r\n\r\n", dir, url);
    ser->print(httpReq);
    ser->write(CTRLZ); // terminate request
    return drvrCheckOK(7000);
}

/********************************************************************/
//
//
//
/********************************************************************/
boolean Wilduino3G::httpResp(const char *resp)
{
    if (drvrCheckResp(resp, DEFAULT_TIMEOUT))
    {
        return true;
    }
    return false;
}

/********************************************************************/
//
//
//
/********************************************************************/
/*
void Wilduino3G::httpOpenSession(const char *url, uint16_t port, boolean ssl)
{    
    sprintf(tmp, "AT+CHTTPSOPSE=\"%s\", %d, %d\r\n", port, ssl+1);
    ser->print(tmp);
}
*/
/********************************************************************/
//
//    TODO: remove the delay and check for proper response
//
/********************************************************************/
/*
void Wilduino3G::httpSend(const char *data, uint32_t timeout)
{
    uint16_t len = strlen(data);
    sprintf(tmp, "AT+CHTTPSSEND=%s\r\n", len);
    ser->print(tmp);

    // get the proper response, ie: prompt here
    delay(1000); // change this to properly look for the prompt
    
    ser->print(data);
    ser->print("\r\n\r\n");
}
*/

/********************************************************************/
//
//
//
/********************************************************************/
/*
void Wilduino3G::httpRecv(uint16_t size)
{
  char buf[100];
  sprintf(buf, "AT+CHTTPSRECV=%d\r\n", size);
  Serial1.print(buf);
}
*/


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// RTC related functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**************************************************************************/
// 
/**************************************************************************/
void Wilduino3G::rtcSetTime(int yr, int mon, int day, int hour, int min, int sec)
{
    struct ts time;
    memset(&time, 0, sizeof(time));

    time.year = yr;
    time.mon = mon;
    time.mday = day;
    time.hour = hour;
    time.min = min;
    time.sec = sec;
    DS3231_set(time);
}

/**************************************************************************/
// 
/**************************************************************************/
struct ts Wilduino3G::rtcGetTime()
{
    struct ts time;
    DS3231_get(&time);
    return time;
}

/**************************************************************************/
// 
/**************************************************************************/
void Wilduino3G::rtcPrintTime(char *datetime)
{
  struct ts time = rtcGetTime();
  memset(tmp, 0, sizeof(tmp));
  sprintf(tmp, "%02d:%02d:%02d", time.hour, time.min, time.sec);
  memcpy(datetime, tmp, strlen(tmp)+1);
}

/**************************************************************************/
// 
/**************************************************************************/
void Wilduino3G::rtcPrintDate(char *datetime)
{
  struct ts time = rtcGetTime();
  memset(tmp, 0, sizeof(tmp));
  sprintf(tmp, "%04d/%02d/%02d", time.year, time.mon, time.mday);
  memcpy(datetime, tmp, strlen(tmp)+1);
}


/**************************************************************************/
// 
/**************************************************************************/
void Wilduino3G::rtcPrintTimeAndDate(char *datetime)
{
  struct ts time = rtcGetTime();
  memset(tmp, 0, sizeof(tmp));
  sprintf(tmp, "%04d/%02d/%02d,%02d:%02d:%02d", time.year, time.mon, time.mday, time.hour, time.min, time.sec);
  memcpy(datetime, tmp, strlen(tmp)+1);
}

/**************************************************************************/
// 
/**************************************************************************/
void Wilduino3G::rtcPrintFullTime(char *datetime)
{
  struct ts time = rtcGetTime();
  memset(tmp, 0, sizeof(tmp));
  sprintf(tmp, "%02d/%02d/%02d %02d:%02d:%02d", time.year-2000, time.mon, time.mday, time.hour, time.min, time.sec);
  memcpy(datetime, tmp, strlen(tmp)+1);
}

/**************************************************************************/
// 
/**************************************************************************/
void Wilduino3G::rtcSetAlarm(uint8_t alarm, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec, uint8_t repeat)
{
    uint8_t i;
    uint8_t flags[5] = {0};

    // flags define what calendar component to be checked against the current time in order
    // to trigger the alarm - see datasheet
    // A1M1 (seconds) (0 to enable, 1 to disable)
    // A1M2 (minutes) (0 to enable, 1 to disable)
    // A1M3 (hour)    (0 to enable, 1 to disable) 
    // A1M4 (day)     (0 to enable, 1 to disable)
    // DY/DT          (dayofweek == 1/dayofmonth == 0)
    switch (repeat)
    {
      case EVERY_SECOND:
      for (i=0; i<4; i++) flags[i] = 1;
      break;

      case EVERY_MINUTE:
      for (i=1; i<4; i++) flags[i] = 1;
      break;

      case EVERY_HOUR:
      for (i=2; i<4; i++) flags[i] = 1;
      break;

      case EVERY_DAY:
      for (i=3; i<4; i++) flags[i] = 1;
      break;

      default:
      memset(flags, 0, sizeof(flags)/sizeof(uint8_t));
      break;
    }

    switch (alarm)
    {
        case MINUTE_ALARM:
            DS3231_clear_a1f();
            DS3231_set_a1(sec, min, hour, day, flags); 
        break;

        case HOUR_ALARM:
            DS3231_clear_a2f();
            DS3231_set_a2(min, hour, day, flags); 
        break;

        default:
        return;
    }

    rtcEnableAlarm(alarm);
}

/**************************************************************************/
// 
/**************************************************************************/
void Wilduino3G::rtcGetAlarm(uint8_t alarm, uint8_t *data)
{
    uint8_t addr;
    switch (alarm)
    {
        case MINUTE_ALARM:
            addr = DS3231_ALARM1_ADDR;
        break;

        case HOUR_ALARM:
            addr = DS3231_ALARM2_ADDR;
        break;

        default:
        return;
    }

    Wire.beginTransmission(DS3231_I2C_ADDR);
    Wire.write(addr);
    Wire.endTransmission();
    Wire.requestFrom(DS3231_I2C_ADDR, 5);

    for (int i = 0; i < 5; i++) 
    {
        data[i] = Wire.read();
    }
}

/**************************************************************************/
// 
/**************************************************************************/
uint8_t Wilduino3G::rtcGetControl()
{
    return DS3231_get_creg();
}

/**************************************************************************/
// 
/**************************************************************************/
uint8_t Wilduino3G::rtcGetStatus()
{
    return DS3231_get_sreg();
}

/**************************************************************************/
// 
/**************************************************************************/
void Wilduino3G::rtcSetStatus(uint8_t reg)
{
    DS3231_set_sreg(reg);
    return;
}

/**************************************************************************/
// 
/**************************************************************************/
void Wilduino3G::rtcClearAlarm(uint8_t alarm)
{

    switch (alarm)
    {
        case MINUTE_ALARM:
            DS3231_clear_a1f();
        break;

        case HOUR_ALARM:
            DS3231_clear_a2f();
        break;

        default:
        return;
    }
}

/**************************************************************************/
// 
/**************************************************************************/
void Wilduino3G::rtcEnableAlarm(uint8_t alarm)
{
    uint8_t reg;
    switch (alarm)
    {
        case MINUTE_ALARM:
            reg = DS3231_get_creg();
            DS3231_set_creg(reg | DS3231_INTCN | DS3231_A1IE);
        break;

        case HOUR_ALARM:
            reg = DS3231_get_creg();
            DS3231_set_creg(reg | DS3231_INTCN | DS3231_A2IE);
        break;
    
        default:
        return;
    }
}


/**************************************************************************/
// 
/**************************************************************************/
void Wilduino3G::rtcDisableAlarm(uint8_t alarm)
{
    uint8_t reg = DS3231_get_addr(DS3231_INTCN);

    switch (alarm)
    {
        case MINUTE_ALARM:
            reg &= ~DS3231_A1IE;
        break;

        case HOUR_ALARM:
            reg &= ~DS3231_A2IE;
        break;

        default:
        return;
    }
    DS3231_set_addr(DS3231_INTCN, reg);
}

/**************************************************************************/
// 
/**************************************************************************/
float Wilduino3G::rtcGetTemp()
{
    float temp = DS3231_get_treg();
    return temp;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SD related functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**************************************************************************/
// 
/**************************************************************************/
boolean Wilduino3G::sdBegin(uint8_t csPin)
{
    boolean st = sd.begin(csPin, SPI_HALF_SPEED);
    if (!st)
    {
        DBG_PRINTLN(F("[ERROR] Card failed, or not present"));
        sd.initErrorHalt();
    }
    return st;
}

/**************************************************************************/
// 
/**************************************************************************/
boolean Wilduino3G::sdOpen(const char *filename, uint8_t mode)
{
    return file.open(filename, mode);
}

/**************************************************************************/
// 
/**************************************************************************/
void Wilduino3G::sdLs()
{
    sd.ls(LS_R);
}

/**************************************************************************/
// 
/**************************************************************************/
int16_t Wilduino3G::sdRead()
{
    return file.read();
}

/**************************************************************************/
// 
/**************************************************************************/
void Wilduino3G::sdWrite(const char *data)
{
    file.print(data);
}

/**************************************************************************/
// 
/**************************************************************************/
void Wilduino3G::sdClose()
{
    file.close();
}

/**************************************************************************/
// 
/**************************************************************************/
void Wilduino3G::sdRemove(const char *filename)
{
    sd.remove(filename);
}

/**************************************************************************/
// 
/**************************************************************************/
uint32_t Wilduino3G::sdAvailable()
{
    return file.available();
}

/**************************************************************************/
// 
/**************************************************************************/
boolean Wilduino3G::sdMkDir(const char *filepath)
{
    return sd.mkdir(filepath);
}

/**************************************************************************/
// 
/**************************************************************************/
boolean Wilduino3G::sdChDir(const char *filepath)
{
    return sd.chdir(filepath);
}

/**************************************************************************/
// 
/**************************************************************************/
boolean Wilduino3G::sdExists(const char *filename)
{
    return sd.exists(filename);
}

/**************************************************************************/
// 
/**************************************************************************/
void Wilduino3G::sdLogMsg(const char *filename, const char *msg)
{
    if (sdOpen(filename))
    {
        sdWrite(msg);
        dbg->print(msg);
        sdClose();
    }
}

/**************************************************************************/
// 
/**************************************************************************/
void Wilduino3G::sdLogTimestampedMsg(const char *filename, const char *msg)
{
    char errBuf[500];
    struct ts time = rtcGetTime();

    if (sdOpen(filename))
    {
        sprintf(errBuf, "%04d/%02d/%02d,%02d:%02d:%02d - %s.\n", time.year, time.mon, time.mday, time.hour, time.min, time.sec, msg);
        sdWrite(errBuf);
        dbg->print(errBuf);
        sdClose();
    }
}

/**************************************************************************/
// 
/**************************************************************************/
void Wilduino3G::sdDateTime(uint16_t *date, uint16_t *time)
{
    struct ts now = rtcGetTime();

    // return date using FAT_DATE macro to format fields
    *date = FAT_DATE(now.year, now.mon, now.mday);

    // return time using FAT_TIME macro to format fields
    *time = FAT_TIME(now.hour, now.min, now.sec);
}
