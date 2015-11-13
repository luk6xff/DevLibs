
#include "gts4E60.h"
#include <string.h>



GTS4E60::GTS4E60(PinName tx, PinName rx, PinName shutdownPin) : mGpsSerial(tx,rx),mShutdownPin(shutdownPin)
{
    mShutdownPin =0;
    mGpsSerial.baud(GTS4E60_SERIAL_DEFAULT_BAUD );
    init();
}

int GTS4E60::write(const char* data)
{
    return mGpsSerial.printf(data);
}

//private methods

float GTS4E60::nmeaToDecimal(float deg_coord, char nsew)
{
    int degree = (int)(deg_coord/100.0f);
    float minutes = deg_coord - degree*100;
    float dec_deg = minutes / 60.0f;
    float decimal = degree + dec_deg;
    if (nsew == 'S' || nsew == 'W') {
        decimal *= -1; // return negative value
    }
    return decimal;

    //using trunc
    /*
    if(ns =='S') {
    latitude   *= -1.0;
    }
    if(ew =='W') {
    longitude  *= -1.0;
    }
    float degrees = trunc(latitude / 100.0f);
    float minutes = latitude - (degrees * 100.0f);
    latitude = degrees + minutes / 60.0f;
    degrees = trunc(longitude / 100.0f);
    minutes = longitude - (degrees * 100.0f);
    longitude = degrees + minutes / 60.0f;
    */

}

float GTS4E60::trunc(float v)
{
    if(v < 0.0) {
        v*= -1.0;
        v = floor(v);
        v*=-1.0;
    } else {
        v = floor(v);
    }
    return v;
}


void GTS4E60::readData()
{
    while(mGpsSerial.getc() != '$');  //wait for the correct NMEA protocol message
    for(int i=0; i<GTS4E60_NMEA_BUF_SIZE; i++) {
        mNmeaData[i] = mGpsSerial.getc();
        if(mNmeaData[i] == '\r') {
            mNmeaData[i] = 0;
            return;
        }
    }
    error("overflowed message limit");
}

void GTS4E60::readData(uint8_t nmeaSentence)
{
    if(nmeaSentence>=NR_OF_SUPPORTED_NMEA_SENTENCES)return;
    int counter =0;
    while(counter<GTS4E60_NMEA_BUF_SIZE) {
        while(mGpsSerial.getc() != '$');  //wait for the correct NMEA protocol message
        counter++;
        char buf[6];
        for(int i =0; i<5; i++) {
            buf[i]=mGpsSerial.getc();
        }
        buf[5]='\0';
        if(strcmp(buf,nmeaSentencesString[nmeaSentence])==0) {
            strcpy(mNmeaData,buf);
        } else continue;
        for(int i=5; i<GTS4E60_NMEA_BUF_SIZE; i++) {
            mNmeaData[i] = mGpsSerial.getc();
            if(mNmeaData[i] == '\r') {
                mNmeaData[i] = 0;
                return;
            }
        }
    }
    error("overflowed message limit");

}

//public methods

void GTS4E60::init()
{
    memset(mNmeaData,0,GTS4E60_NMEA_BUF_SIZE);
    //GPGAA
    mFixType= 0;
    mSatellites = 0;
    mHdop= 0;
    mAltitude= 0.0;
    mUnits= ' ';

    // RMC
    mLongitude= 0.0;
    mLatitude = 0.0;
    NS=' ';
    EW=' ';
    mDataStatus= 'V';

    //GSV
    mNumberOfMsgs=0;
    mMsgNumber=0;
    mSatellitesInView=0;
    wait(1);
    setStatusType(IDLE_STATE);
}


int GTS4E60::isDataAvailable()
{

    return mGpsSerial.readable();
}

uint8_t GTS4E60::parseData(uint8_t param)
{
    uint8_t retVal=INCORRECT_DATA;
    if(param==NULL)
        readData();
    else
        readData(param);
    // Check if there is a GPGGA snetence
    if(sscanf(mNmeaData, "GPGGA, %2d%2d%f, %*f, %*c, %*f, %*c, %d, %d, %*f, %f", &mUtcTime.hours, &mUtcTime.minutes, &mUtcTime.seconds, &mFixType, &mSatellites, &mAltitude) >=1) {
        retVal = GGA;
        if(mFixType == 0) {
            mFix = "Invalid or not available";
            return NO_FIX_FOUND;
        }
        if(mSatellites==0) {
            return NO_SATELLITES;
        }
    }
    //if there is a GPGSA sentence - not used here :)
    /*
    else if(sscanf(mNmeaData, "GPGSA, %c, %c, %d ....", &mMode1, &mMode2, &mSatelitesUsed....) >=1){
    retVal= GSA;
    }
    */

    //if there is a GPRMC sentence
    else if(sscanf(mNmeaData, "GPRMC, %2d%2d%f, %c, %f, %c, %f, %c, %f, %f, %2d%2d%2d", &mUtcTime.hours, &mUtcTime.minutes, &mUtcTime.seconds, &mDataStatus, &mLatitude, &NS, &mLongitude, &EW, &mSpeedKn, &mHeading, &mDate.day, &mDate.month, &mDate.year) >=1) {
        retVal = RMC;
        if(mDataStatus=='V')
            return INVALID_STATUS;

        mDate.year += 2000;
        mLatitude= nmeaToDecimal(mLatitude,NS);
        mLongitude= nmeaToDecimal(mLongitude,EW);

        if(mFixType == 1) {
            mFix = "Positive";
        }
        if(mFixType == 2) {
            mFix = "Differential";
        }
        if(mHeading > 0.00 && mHeading < 45.00) {
            mCardinal = "NNE";
        } else if(mHeading == 45.00) {
            mCardinal = "NE";
        } else if(mHeading > 45.00 && mHeading < 90.00) {
            mCardinal = "ENE";
        } else if(mHeading == 90.00) {
            mCardinal = "E";
        } else if(mHeading > 90.00 && mHeading < 135.00) {
            mCardinal = "ESE";
        } else if(mHeading == 135.00) {
            mCardinal = "SE";
        } else if(mHeading > 135.00 && mHeading < 180.00) {
            mCardinal = "SSE";
        } else if(mHeading == 180.00) {
            mCardinal = "S";
        } else if(mHeading > 180.00 && mHeading < 225.00) {
            mCardinal = "SSW";
        } else if(mHeading == 225.00) {
            mCardinal = "SW";
        } else if(mHeading > 225.00 && mHeading < 270.00) {
            mCardinal = "WSW";
        } else if(mHeading == 270.00) {
            mCardinal = "W";
        } else if(mHeading > 270.00 && mHeading < 315.00) {
            mCardinal = "WNW";
        } else if(mHeading == 315.00) {
            mCardinal = "NW";
        } else if(mHeading > 315.00 && mHeading < 360.00) {
            mCardinal = "NNW";
        } else if(mHeading == 360.00 || mHeading == 0.00) {
            mCardinal = "N";
        }
        mSpeedKm = mSpeedKn*1.852;

    }

    //if there is a GPGSV sentence
    else if(sscanf(mNmeaData, "GPGSV, %d, %d, %d", &mNumberOfMsgs, &mMsgNumber , &mSatellitesInView) >=1) {
        retVal=GSV;
    }
    return retVal;
}





//getter methods
UTC_Time GTS4E60:: getTime()
{
    return mUtcTime;
}

Date GTS4E60:: getDate()
{
    return mDate;
}

float GTS4E60:: getLongitude()
{
    return mLongitude;
}

float GTS4E60::getLatitude()
{
    return mLatitude;
}

float GTS4E60::getAltitude()
{
    return mAltitude;
}

float GTS4E60::getSpeedKn()
{
    return mSpeedKn;
}

float GTS4E60::getSpeedKm()
{
    return mSpeedKm;
}

int   GTS4E60::getSatelites()
{
    return mSatellites;
}

//float GTS4E60::getCourseT() {}
//float GTS4E60::getCourseM() {}

int GTS4E60::getFixType()
{
    return mFixType;
}

int GTS4E60::getSatellites()
{
    return mSatellites;
}

int GTS4E60:: getStatus()
{
    return mDataStatus;
}

char GTS4E60:: getNS()
{
    return NS;
}
char GTS4E60:: getEW()
{
    return EW;
}

float GTS4E60:: getHeading()
{
    return mHeading;
}


