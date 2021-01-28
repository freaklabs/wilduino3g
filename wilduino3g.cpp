/************************************************************************/
// 
//
//
/************************************************************************/
#include "saboten3g.h"

#define DEBUG 1

// allows printing or not printing based on the DEBUG VAR
#if (DEBUG == 1)
  #define DBG_PRINT(...)   dbg->print(__VA_ARGS__)
  #define DBG_PRINTLN(...) dbg->println(__VA_ARGS__)
#else
  #define DBG_PRINT(...)
  #define DBG_PRINTLN(...)
#endif


static char tmp[500];
volatile boolean rtcFlag = false;

/************************************************************************/
// 
//
//
/************************************************************************/
Saboten3G::Saboten3G()
{
    pinMode(pinPwrButton, OUTPUT);
    digitalWrite(pinPwrButton, HIGH);

    pinMode(pinDtr, OUTPUT);
    digitalWrite(pinDtr, HIGH);

    pinMode(pinFlightMode, OUTPUT);
    digitalWrite(pinFlightMode, HIGH);

    pinMode(pinWake, OUTPUT);
    digitalWrite(pinWake, HIGH);

    pinMode(pinCdet, INPUT);
    digitalWrite(pinCdet, HIGH);

    pinMode(pinLevelEnb, OUTPUT);
    digitalWrite(pinLevelEnb, LOW);

    pinMode(pinGpsEnb, OUTPUT);
    digitalWrite(pinGpsEnb, LOW);
    
    pinMode(pin3GRstN, OUTPUT);
    digitalWrite(pin3GRstN, HIGH);

    pinMode(pinArefEnb, OUTPUT);
    digitalWrite(pinArefEnb, LOW);

    pinMode(10, OUTPUT);
    digitalWrite(10, HIGH);

    rtcFlag = false;

    Wire.begin();
}

/************************************************************************/
// 
//
//
/************************************************************************/
boolean Saboten3G::begin(HardwareSerial *port, SoftwareSerial *gpsSerial, HardwareSerial *debug)
{
    uint8_t reg;

    ser = port;
    dbg = debug;
    gps = gpsSerial;

    ser->begin(115200);
    dbg->begin(57600);
    gps->begin(9600);


    DS3231_init(DS3231_INTCN);

    for (int i=0; i<2; i++)
    {
        rtcClearAlarm(i+1);
    }
    reg = rtcGetStatus();
    reg &= ~(DS3231_OSF | DS3231_EN32K);
    rtcSetStatus(reg);

    attachInterrupt(RTC_INTP, Saboten3G::rtcIntp, FALLING); 

    sdBegin(pinSdSeln);
    file.dateTimeCallback(sdDateTime);

    // use external 2.5V reference
    analogReference(EXTERNAL);

    DBG_PRINTLN(F("Initialization complete"));
    return true;
}

/************************************************************************/
// 
//
//
/************************************************************************/
void Saboten3G::poll()
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
uint8_t Saboten3G::getIntp()
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
void Saboten3G::rtcIntp()
{
  rtcFlag = true;
}

/********************************************************************/
//
//
//
/********************************************************************/
boolean Saboten3G::rtcIntpRcvd()
{
    return rtcFlag;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 3G Driver related functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/************************************************************************/
//    flushSerInput
//    Removes any data from receive input
/************************************************************************/
void Saboten3G::drvrFlush()
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
void Saboten3G::drvrHardReset()
{
    digitalWrite(pin3GRstN, LOW);
    delay(500);
    digitalWrite(pin3GRstN, HIGH);
    delay(500);
}


/************************************************************************/
// 
//
//
/************************************************************************/
void Saboten3G::drvrCmdEcho(boolean enable) 
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
void Saboten3G::drvrSend(const char* command) 
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
boolean Saboten3G::drvrCheckResp(const char *expected, uint32_t timeout)
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
boolean Saboten3G::drvrCheckOK(uint32_t timeout)
{
   
    if (drvrCheckResp("OK\r\n", timeout))
    {
      DBG_PRINTLN("Command Success");
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
void Saboten3G::drvrDumpResp()
{
  dbg->print(respBuf);
}

/**************************************************************************/
// 
/**************************************************************************/
void Saboten3G::drvrSleepMcu()
{
    uint8_t portBReg, portCReg, portDReg;
    uint8_t ddrBReg, ddrCReg, ddrDReg;

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
float Saboten3G::drvrGetVbat()
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
float Saboten3G::drvrGetVsol()
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
void Saboten3G::drvrWriteDevId(uint16_t id)
{
    while(!eeprom_is_ready());
    eeprom_write_block(&id, DEV_ID_EEPROM_LOC, 2);
}

/**************************************************************************/
// 
/**************************************************************************/
uint16_t Saboten3G::drvrReadDevId()
{
    uint16_t id;
    while(!eeprom_is_ready());
    eeprom_read_block(&id, DEV_ID_EEPROM_LOC, 2);
    return id;
}

/**************************************************************************/
// 
/**************************************************************************/
boolean Saboten3G::drvrPowerOn()
{
    uint8_t resp = false;
    
    for (int i=0; i<POWER_RETRIES; i++)
    {
        DBG_PRINTLN("POWERON");
        // turn on power switch for modem
        digitalWrite(pinPwrButton, LOW);
        delay(2000);
        digitalWrite(pinPwrButton, HIGH);
        delay(2000);

        wdt_reset(); // kick the dog

        // check to see if it powered up properly
        if ((resp = drvrCheckResp("PB DONE\r\n", 7000)) == true)
        {
            break;
        }

        wdt_reset();
        
        // check to see if power is on. If it is, then break also
        drvrSend("ATI\r\n");
        if (drvrCheckOK(2000))
        {
            resp = true;
            break;
        }
    }
    
    return resp;
}

/**************************************************************************/
// 
/**************************************************************************/
boolean Saboten3G::drvrPowerOff()
{
    // check to see if power is on. If it is, then break also
    for (int i=0; i<POWER_RETRIES; i++)
    {
        drvrSend("ATI\r\n");
        if (drvrCheckOK(2000))
        {
            // turn off power switch for modem
            digitalWrite(pinPwrButton, LOW);
            delay(2000);
            digitalWrite(pinPwrButton, HIGH);
            delay(2000);
            return true;
        }
    }
    return false;
}

/**************************************************************************/
// 
/**************************************************************************/
boolean Saboten3G::drvrReset()
{
    drvrSend("AT+CRESET");
    return (drvrCheckOK(DEFAULT_TIMEOUT));
}

/************************************************************************/
// elapsedTime - calculates time elapsed from startTime
// startTime : time to start calculating
/************************************************************************/
uint32_t Saboten3G::drvrElapsedTime(uint32_t startTime)
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
void Saboten3G::drvrFlightMode(boolean enable)
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
void Saboten3G::mgmtGetInfo()
{
    DBG_PRINTLN("Sending ATI command");
    drvrSend("ATI\r\n");
    drvrCheckOK(2000);
}

/********************************************************************/
//    
//
/********************************************************************/
int8_t Saboten3G::mgmtGetRSSI()
{
    char key[] = " +:,\n";
    char *p;

    DBG_PRINTLN("Sending AT+CSQ command");
    drvrSend("AT+CSQ\r\n");
    if (drvrCheckOK(DEFAULT_TIMEOUT))
    {
        drvrDumpResp();
    }
    else
    {
        DBG_PRINTLN("Could not get RSSI");
        return -1;
    }

    p = strtok(respBuf, key);
    while (p)
    {
      p = strtok(NULL, key);
      if (memcmp(p, "CSQ", sizeof("CSQ")) == 0)
      {
          p = strtok(NULL, key);
          rssi = atoi(p);
            return rssi;
      }

    }
    return -1;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// HTTP related functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/********************************************************************/
//
//
//
/********************************************************************/
boolean Saboten3G::httpOpen(const char *url, uint16_t port)
{
    for (retries=0; retries<MAX_HTTP_RETRIES; retries++)
    {
        wdt_reset();
        sprintf(tmp, "AT+CHTTPACT=\"%s\",%d\r\n", url, port);
        dbg->print(tmp);
        ser->print(tmp);
        if (drvrCheckResp("+CHTTPACT: REQUEST\r\n", 7000))
        {
            return true;
        }
    }
    dbg->print("Opening site failed\n");
    return false;
}

/********************************************************************/
//
//
//
/********************************************************************/
boolean Saboten3G::httpSend(const char *dir, const char *url, const char *data, uint16_t len)
{
    char httpReq[1000];

    sprintf(httpReq, "POST %s HTTP/1.1\r\nHost: %s\r\nContent-Type: text/csv\r\nCache-Control: no-cache\r\nContent-Length: %d\r\n\r\n%s\r\n\r\n", dir, url, len, data);
    ser->print(httpReq);
    ser->write(CTRLZ); // terminate request

    if (drvrCheckResp("http/1.0 200 ok\r\n", DEFAULT_TIMEOUT))
    {
        return true;
    }
    else
    {
        return false;
    }
}

/********************************************************************/
//
//
//
/********************************************************************/
boolean Saboten3G::httpGet(const char *dir, const char *url)
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
boolean Saboten3G::httpResp(const char *resp)
{
    if (drvrCheckResp(resp, DEFAULT_TIMEOUT))
    {
        return true;
    }
}

/********************************************************************/
//
//
//
/********************************************************************/
/*
void Saboten3G::httpOpenSession(const char *url, uint16_t port, boolean ssl)
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
void Saboten3G::httpSend(const char *data, uint32_t timeout)
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
void Saboten3G::httpRecv(uint16_t size)
{
  char buf[100];
  sprintf(buf, "AT+CHTTPSRECV=%d\r\n", size);
  Serial1.print(buf);
}
*/

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// GPS related functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**************************************************************************/
// Full power down of GPS. This shuts down power to GPS.
/**************************************************************************/
void Saboten3G::gpsEnable()
{
    digitalWrite(pinGpsEnb, HIGH);
}

/**************************************************************************/
// Full power up of GPS. This turns on power to the module
/**************************************************************************/
void Saboten3G::gpsDisable()
{
    digitalWrite(pinGpsEnb, LOW);
}

/**************************************************************************/
// Turn off RF section of GPS. This is not full power down. It will still
// maintain memory
/**************************************************************************/
void Saboten3G::gpsRadioOff()
{
    uint8_t GPSoff[] = {0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0x00, 0x00,0x08, 0x00, 0x16, 0x74};
    gpsSendUBX(GPSoff, sizeof(GPSoff)/sizeof(uint8_t));
}

/**************************************************************************/
// Turn n RF section of GPS. It will still retain memory.
/**************************************************************************/
void Saboten3G::gpsRadioOn()
{
    uint8_t GPSon[] = {0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0x00, 0x00,0x09, 0x00, 0x17, 0x76};
    gpsSendUBX(GPSon, sizeof(GPSon)/sizeof(uint8_t));
}

/**************************************************************************/
//drain received data from gps port
/**************************************************************************/
void Saboten3G::gpsFlush()
{
    while (gps->available())
    {
        gps->read();        
    }
}

/**************************************************************************/
// 
/**************************************************************************/
void Saboten3G::gpsQuiet()
{
  gps->println("$PUBX,40,GGA,0,0,0,0*5A");
  gps->println("$PUBX,40,GSA,0,0,0,0*4E");
  gps->println("$PUBX,40,RMC,0,0,0,0*47");
  gps->println("$PUBX,40,GSV,0,0,0,0*59");
  gps->println("$PUBX,40,VTG,0,0,0,0*5E");
  gps->println("$PUBX,40,GLL,0,0,0,0*5C");
}

/********************************************************************/
//
/********************************************************************/ 
void Saboten3G::gpsPowerSaveMode() 
{
  //Set GPS to Power Save Mode
  uint8_t setPSM[] = { 0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x01, 0x1A, 0x9D };
  gpsSendUBX(setPSM, sizeof(setPSM)/sizeof(uint8_t));
}


/**************************************************************************/
// 
/**************************************************************************/
void Saboten3G::gpsPoll()
{
    gps->println("$PUBX,00*33");
}

/**************************************************************************/
// 
/**************************************************************************/
void Saboten3G::gpsPollTime()
{
    gps->println("$PUBX,04*37");
}

/**************************************************************************/
// 
/**************************************************************************/
void Saboten3G::gpsRead(char *data)
{
    DBG_PRINTLN("TODO: Getting GPS Data");
    const char *gps = "This is the GPS Data";
    memcpy(data, gps, strlen(gps)+1);
}

/********************************************************************/
// Send a byte array of UBX protocol to the GPS
/********************************************************************/
void Saboten3G::gpsSendUBX(uint8_t *msg, uint8_t len) 
{
  for(int i=0; i<len; i++) 
  {
    gps->write(msg[i]);
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// RTC related functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**************************************************************************/
// 
/**************************************************************************/
void Saboten3G::rtcSetTime(int yr, int mon, int day, int hour, int min, int sec)
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
struct ts Saboten3G::rtcGetTime()
{
    struct ts time;
    DS3231_get(&time);
    return time;
}

/**************************************************************************/
// 
/**************************************************************************/
void Saboten3G::rtcPrintTime(char *datetime)
{
  struct ts time = rtcGetTime();
  memset(tmp, 0, sizeof(tmp));
  sprintf(tmp, "%02d:%02d:%02d", time.hour, time.min, time.sec);
  memcpy(datetime, tmp, strlen(tmp)+1);
}

/**************************************************************************/
// 
/**************************************************************************/
void Saboten3G::rtcPrintDate(char *datetime)
{
  struct ts time = rtcGetTime();
  memset(tmp, 0, sizeof(tmp));
  sprintf(tmp, "%04d/%02d/%02d", time.year, time.mon, time.mday);
  memcpy(datetime, tmp, strlen(tmp)+1);
}


/**************************************************************************/
// 
/**************************************************************************/
void Saboten3G::rtcPrintTimeAndDate(char *datetime)
{
  struct ts time = rtcGetTime();
  memset(tmp, 0, sizeof(tmp));
  sprintf(tmp, "%04d/%02d/%02d,%02d:%02d:%02d", time.year, time.mon, time.mday, time.hour, time.min, time.sec);
  memcpy(datetime, tmp, strlen(tmp)+1);
}

/**************************************************************************/
// 
/**************************************************************************/
void Saboten3G::rtcPrintFullTime(char *datetime)
{
  struct ts time = rtcGetTime();
  memset(tmp, 0, sizeof(tmp));
  sprintf(tmp, "%02d/%02d/%02d %02d:%02d:%02d", time.year-2000, time.mon, time.mday, time.hour, time.min, time.sec);
  memcpy(datetime, tmp, strlen(tmp)+1);
}

/**************************************************************************/
// 
/**************************************************************************/
void Saboten3G::rtcSetAlarm(uint8_t alarm, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec, uint8_t repeat)
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
void Saboten3G::rtcGetAlarm(uint8_t alarm, uint8_t *data)
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
uint8_t Saboten3G::rtcGetControl()
{
    return DS3231_get_creg();
}

/**************************************************************************/
// 
/**************************************************************************/
uint8_t Saboten3G::rtcGetStatus()
{
    return DS3231_get_sreg();
}

/**************************************************************************/
// 
/**************************************************************************/
void Saboten3G::rtcSetStatus(uint8_t reg)
{
    DS3231_set_sreg(reg);
    return;
}

/**************************************************************************/
// 
/**************************************************************************/
void Saboten3G::rtcClearAlarm(uint8_t alarm)
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
void Saboten3G::rtcEnableAlarm(uint8_t alarm)
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
void Saboten3G::rtcDisableAlarm(uint8_t alarm)
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
uint8_t Saboten3G::rtcGetTemp()
{
    float temp = DS3231_get_treg();
    return round(temp);
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SD related functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**************************************************************************/
// 
/**************************************************************************/
boolean Saboten3G::sdBegin(uint8_t csPin)
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
boolean Saboten3G::sdOpen(const char *filename, uint8_t mode)
{
    return file.open(filename, mode);
}

/**************************************************************************/
// 
/**************************************************************************/
void Saboten3G::sdLs()
{
    sd.ls(LS_R);
}

/**************************************************************************/
// 
/**************************************************************************/
int16_t Saboten3G::sdRead()
{
    return file.read();
}

/**************************************************************************/
// 
/**************************************************************************/
boolean Saboten3G::sdWrite(const char *data)
{
    file.print(data);
}

/**************************************************************************/
// 
/**************************************************************************/
boolean Saboten3G::sdClose()
{
    file.close();
}

/**************************************************************************/
// 
/**************************************************************************/
boolean Saboten3G::sdRemove(const char *filename)
{
    sd.remove(filename);
}


/**************************************************************************/
// 
/**************************************************************************/
uint32_t Saboten3G::sdAvailable()
{
    return file.available();
}

/**************************************************************************/
// 
/**************************************************************************/
boolean Saboten3G::sdMkDir(const char *filepath)
{
    return sd.mkdir(filepath);
}

/**************************************************************************/
// 
/**************************************************************************/
boolean Saboten3G::sdChDir(const char *filepath)
{
    return sd.chdir(filepath);
}

/**************************************************************************/
// 
/**************************************************************************/
boolean Saboten3G::sdExists(const char *filename)
{
    return sd.exists(filename);
}

/**************************************************************************/
// 
/**************************************************************************/
void Saboten3G::sdDateTime(uint16_t *date, uint16_t *time)
{
    struct ts now = rtcGetTime();

    // return date using FAT_DATE macro to format fields
    *date = FAT_DATE(now.year, now.mon, now.mday);

    // return time using FAT_TIME macro to format fields
    *time = FAT_TIME(now.hour, now.min, now.sec);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Level sensor related functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**************************************************************************/
// 
/**************************************************************************/
void Saboten3G::levelOn()
{
    digitalWrite(pinLevelEnb, HIGH);
}

/**************************************************************************/
// 
/**************************************************************************/
void Saboten3G::levelOff()
{
    digitalWrite(pinLevelEnb, LOW);
}

/**************************************************************************/
// 
/**************************************************************************/
uint32_t Saboten3G::levelRead()
{   
    uint32_t val = 0;

    levelOn();
    digitalWrite(pinArefEnb, HIGH);
    delay(500);

    float raw = analogRead(pinLevelSensor);
    raw *= 5.004 * (AREF/VCC);  // datasheet spec: 5120 counts max / 1023 counts max for Arduino ADC = 5.004
    val = raw;
    // average level readings over 10 readings
      
/*
    for (int i=0; i<16; i++)
    {
        float raw = analogRead(pinLevelSensor);
        raw *= 5.004 * (AREF/VCC);  // datasheet spec: 5120 counts max / 1023 counts max for Arduino ADC = 5.004
        val += raw;  
    }
    val >>= 4; // divide by 16
*/
    levelOff();
    digitalWrite(pinArefEnb, LOW);
    return val;
}



