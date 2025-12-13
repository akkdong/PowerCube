// NmeaParser.cpp
//

#include <stdlib.h>
#include <memory.h>
#include "NmeaParser.h"
#include "logger.h"


//////////////////////////////////////////////////////////////////////////////
// class NmeaParser

NmeaParser::NmeaParser()
{
}


int NmeaParser::begin()
{
    reset();

    return 0;
}

void NmeaParser::reset()
{
    //
    valid = 0;
    fixQuality = 0;
    latitude = 0.0f;
    longitude = 0.0f;
    altitude = 0.0f;
    speed = 0.0f;
    track = 0.0f;

    time = 0;
    date = 0;

    temparture = 0.0f;
    pressure = 0.0f;
    altitudeBaro = 0.0f;
    vspeed = 0.0f;
    mute = 0;

    //
    last_key = 0;
    last_keystat = 0;

    //
    memset(&parserContext, 0, sizeof(parserContext));
    //
#if USE_DATEQUEUE
    dataQueue.reset();
#endif
}

int NmeaParser::update(int ch)
{
    int type = 0; // 1: GPS, 2: VARIO, 3: KEY

    //
#if USE_DATEQUEUE
    dataQueue.push(ch);
#endif

    if (ch == '$')
    {
        memset(&parserContext, 0, sizeof(parserContext));
    }
    else if (ch == ',')
    {
        parseField();

        parserContext.fieldPos = 0;
        parserContext.field[0] = '\0';
        parserContext.fieldNum = parserContext.fieldNum + 1;

        parserContext.checksum = parserContext.checksum ^ (uint8_t)ch;
    }
    else if (ch == '*')
    {
        parseField();

        parserContext.fieldPos = 0;
        parserContext.field[0] = '\0';
        parserContext.fieldNum = parserContext.fieldNum + 1;

        parserContext.validateChecksum = 1;
    }
    else if (ch == '\r')
    {
        // ignore
    }
    else if (ch == '\n')
    {
#if USE_DATEQUEUE
        #if ACCEPT_GPS_ONLY
        if (parserContext.statement == STAT_GGA || parserContext.statement == STAT_RMC)
        #else
        if (parserContext.statement != STAT_UNKNOWN)
        #endif
            dataQueue.acceptReserve();
        else
            dataQueue.rejectReserve();
#endif
            
        if (checkCRC())
        {
            switch ((uint8_t)parserContext.statement)
            {
            case STAT_GGA:
                LOGd("GGA: %d, %f, %f, %f", 
                    parserContext.gga.time, 
                    parserContext.gga.latitude,
                    parserContext.gga.longitude,
                    parserContext.gga.altitude);

                fixQuality = parserContext.gga.fixQuality;
                latitude = parserContext.gga.latitude;
                longitude = parserContext.gga.longitude;
                altitude = parserContext.gga.altitude;
                if (parserContext.gga.time == 0 || time != parserContext.gga.time)
                    time = parserContext.gga.time;
                else
                    type = 1;
                break;
            case STAT_RMC:
                LOGd("RMC: %d, %.*f, %.*f, %d",
                    parserContext.rmc.time,
                    2,
                    parserContext.rmc.speed,
                    1,
                    parserContext.rmc.track,
                    parserContext.rmc.date);

                valid = parserContext.rmc.status == 'A' ? 1 : 0;
                latitude = parserContext.rmc.latitude;
                longitude = parserContext.rmc.longitude;
                speed = parserContext.rmc.speed;
                track = parserContext.rmc.track;
                date = parserContext.rmc.date;
                if (/*parserContext.rmc.time == 0 ||*/ time != parserContext.rmc.time)
                    time = parserContext.rmc.time;
                else
                    type = 1;
                break;
            case STAT_VAR:
                LOGd("VAR: %f, %f, %f, %f, %d",
                    parserContext.var.temperature,
                    parserContext.var.pressure,
                    parserContext.var.altitude,
                    parserContext.var.vspeed,
                    parserContext.var.mute);

                temparture = parserContext.var.temperature;
                pressure = parserContext.var.pressure;
                altitudeBaro = parserContext.var.altitude;
                vspeed = parserContext.var.vspeed;
                mute = parserContext.var.mute;
                type = 2;
                break;
            case STAT_KBD:
                LOGd("KBD: %d, %d",
                    parserContext.kbd.key,
                    parserContext.kbd.state);

                last_key = parserContext.kbd.key;
                last_keystat = parserContext.kbd.state;
                type = 3;
                break;
            }
        }
        else
        {
            // invalid checksum
        }
    }
    else
    {
        parserContext.field[parserContext.fieldPos] = ch;
        parserContext.fieldPos = parserContext.fieldPos + 1;
        parserContext.field[parserContext.fieldPos] = '\0';

        if (!parserContext.validateChecksum)
            parserContext.checksum = parserContext.checksum ^ (uint8_t)ch;
    }

    return type;
}

void NmeaParser::parseField()
{
    if (parserContext.fieldNum == 0)
    {
        if (parserContext.fieldPos == 5)
        {
            if (parserContext.field[0] == 'G' && (parserContext.field[1] == 'P' || parserContext.field[1] == 'N'))
            {
                if (strcmp(&parserContext.field[2], "GGA") == 0)
                    parserContext.statement = STAT_GGA;
                else if (strcmp(&parserContext.field[2], "RMC") == 0)
                    parserContext.statement = STAT_RMC;
                /*
                else if (strcmp(&parserContext.field[2], "GSA") == 0)
                    parserContext.statement = STAT_GSA;
                else if (strcmp(&parserContext.field[2], "GSV") == 0)
                    parserContext.statement = STAT_GSV;
                */
            }
            else if (parserContext.field[0] == 'M' && parserContext.field[1] == '3')
            {
                if (strcmp(&parserContext.field[2], "VAR") == 0)
                    parserContext.statement = STAT_VAR;
                else if (strcmp(&parserContext.field[2], "KBD") == 0)
                    parserContext.statement = STAT_KBD;
            }
        }
    }
    else
    {
        if (parserContext.statement == STAT_GGA)
        {
            //  1: UTC of position fix, HHMMSS
            //  2: Latitude, DDNN.NN
            //  3: N or S
            //  4: Longitude, DDDNN.NN
            //  5: E or W
            //  6: Fix quality: 0=Valid, 1=GNSS fix(SPS), 2=DGPS fix, 3=PPS fix, 4=Real Time Kinematic, 6=Estimated, 7=Manual input mode, 8=Simulation mode
            //  7: Number of satellites being tracked, 00 ~ 12
            //  8: Horizontal dilution of precision
            //  9: Altitude, above mean sea level (geoid): DDD.DDD
            // 10: Altitude unit, M=meter(?)
            switch (parserContext.fieldNum)
            {
            case 1:
                parserContext.gga.time = strToTime(parserContext.field);
                break;
            case 2:
                parserContext.gga.latitude = nmeaToDecimal(atof(parserContext.field));
                break;
            case 3:
                if (parserContext.field[0] != 'N')
                    parserContext.gga.latitude = 0 - parserContext.gga.latitude;
                break;
            case 4:
                parserContext.gga.longitude = nmeaToDecimal(atof(parserContext.field));
                break;
            case 5:
                if (parserContext.field[0] != 'E')
                    parserContext.gga.longitude = 0 - parserContext.gga.longitude;
                break;
            case 6:
                parserContext.gga.fixQuality = atoi(parserContext.field);
                break;
            case 7:
            case 8:
                break;
            case 9:
                parserContext.gga.altitude = atof(parserContext.field);
                break;
            case 10:
                break;
            }
        }
        else if (parserContext.statement == STAT_RMC)
        {
            //  1: UTC of position fix, HHMMSS.mmm
            //  2: Status A=Active, V=Void
            //  3: Latitude, DDNN.NN
            //  4: N or S
            //  5: Longitude, DDDNN.NNN
            //  6: E or W
            //  7: Speed over the ground in knots
            //  8: Track angle in degrees, True
            //  9: Date, ddmmyy
            // 10: (empty)
            // 11: (empty)
            // 12: Mode Indicator: A=Autonomous, D=Differenctial, E=Estimated, F=Float RTK, M=Manual input, N=No fix, P=Precise, R=Real time kinematic, S=Simulator
            // 13: Navigational Status: S=Safe, C=Caution, U=Unsafe, V=Void
            switch (parserContext.fieldNum)
            {
            case 1:
                parserContext.rmc.time = strToTime(parserContext.field);
                break;
            case 2:
                parserContext.rmc.status = parserContext.field[0];
                break;
            case 3:
                parserContext.rmc.latitude = nmeaToDecimal(atof(parserContext.field));
                break;
            case 4:
                if (parserContext.field[0] != 'N')
                    parserContext.rmc.latitude = 0 - parserContext.rmc.latitude;
                break;
            case 5:
                parserContext.rmc.longitude = nmeaToDecimal(atof(parserContext.field));
                break;
            case 6:
                if (parserContext.field[0] != 'E')
                    parserContext.rmc.longitude = 0 - parserContext.rmc.longitude;
                break;
            case 7:
                parserContext.rmc.speed = atof(parserContext.field) * 1.853; // convert knot to km/h
                break;
            case 8:
                parserContext.rmc.track = atof(parserContext.field);
                break;
            case 9:
                parserContext.rmc.date = strToDate(parserContext.field, parserContext.rmc.time);
                break;
            }
        
        }
        else if (parserContext.statement == STAT_VAR)
        {
            // 1: Tempearture, DD.D
            // 2: Pressure, DDDDDD
            // 3: Altitude, DD.D
            // 4: Vertical speed, DD.D
            // 5: Mute, D
            switch (parserContext.fieldNum)
            {
            case 1:
                parserContext.var.temperature = atof(parserContext.field);
                break;
            case 2:
                parserContext.var.pressure = atof(parserContext.field);
                break;
            case 3:
                parserContext.var.altitude = atof(parserContext.field);
                break;
            case 4:
                parserContext.var.vspeed = atof(parserContext.field);
                break;
            case 5:
                parserContext.var.mute = atoi(parserContext.field);
                break;
            }
        }
        else if (parserContext.statement == STAT_KBD)
        {
            // 1: Key: 0xB0=Enter, 0xB1=Escape, 0xD8=Left, 0xD7=Right
            // 2: State: 1=Pressed, 2=Released, 4=Long pressed
            switch (parserContext.fieldNum)
            {
            case 1:
                parserContext.kbd.key = atoi(parserContext.field);
                break;
            case 2:
                parserContext.kbd.state = atoi(parserContext.field);
                break;
            }
        }
        else
        {
            // unknown or unsupported statement
        }
    }
}

bool NmeaParser::checkCRC()
{
    uint8_t crc = hexaToNumber(parserContext.field);
    return crc == parserContext.checksum;
}

time_t NmeaParser::strToTime(const char* str)
{
    // hhmmss.ss
    for (int i = 0; i < 6; i++)
    {
        if (str[i] < '0' || '9' < str[i])
            return 0;
    }

    int h = ((str[0] - '0') * 10) + (str[1] - '0');
    int m = ((str[2] - '0') * 10) + (str[3] - '0');
    int s = ((str[4] - '0') * 10) + (str[5] - '0');

    return h * 3600 + m * 60 + s;
}

time_t NmeaParser::strToDate(const char* str, time_t timeOfDay)
{
    // ddmmyy
    for (int i = 0; i < 6; i++)
    {
        if (str[i] < '0' || '9' < str[i])
            return 0;
    }

    struct tm _tm;
    memset(&_tm, 0, sizeof(_tm));    
    _tm.tm_mday = ((str[0] - '0') * 10) + (str[1] - '0');         // tm_mday : 1 ~ 31
    _tm.tm_mon = ((str[2] - '0') * 10) + (str[3] - '0') - 1;      // tm_mon: 0 ~ 11
    _tm.tm_year = ((str[4] - '0') * 10) + (str[5] - '0') + 100;   // tm_year : 0 -> 1900, nm_year : 0 -> 2000, nmea + 2000 - 1900 -> nmea + 100
    _tm.tm_hour = 0;
    _tm.tm_min = 0;
    _tm.tm_sec = 0;

    return mktime(&_tm) + timeOfDay + 619315200; // add 619315200 to compensate Old GPS error
}

float NmeaParser::nmeaToDecimal(float nmea)
{
	int dd = (int)(nmea / 100);
	float ss = nmea - (float)(dd * 100.0);

	return (float)dd + ss / 60.0;
}

long NmeaParser::floatToCoordi(float value)
{
    // DDDMM.mmmm -> DDDMMmmm (with round up)
    float temp = value * 1000.0f;
	return (long)temp;
}

uint8_t NmeaParser::hexaToNumber(char ch)
{
    if ('0' <= ch && ch <= '9')
        return ch - '0';
    if ('a' <= ch && ch <= 'f')
        return ch - 'a' + 10;
    if ('A' <= ch && ch <= 'F')
        return ch - 'A' + 10;

    return 0;
}

uint8_t NmeaParser::hexaToNumber(const char* str)
{
    uint8_t value = 0;
    const char* ptr = str;
    while (*ptr)
        value = value * 16 + hexaToNumber(*ptr++);
    return value;
}
