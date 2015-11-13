/*
  @file gts4E60.h

  @brief GTS-4E-60 GPS Module (FIBOCOM) module Library
         Easy to change for other module

  @Author lukasz uszko(luszko@op.pl)

  Tested on FRDM-KL46Z and FRDM-KL25Z

  Copyright (c) 2014 lukasz uszko
  Released under the MIT License (see http://mbed.org/license/mit)

  Nice tutorial about degree formats and ways of computing them:
  http://home.online.no/~sigurdhu/Deg_formats.htm

  NMEA protocol reference manual:
  https://www.sparkfun.com/datasheets/GPS/NMEA%20Reference%20Manual1.pdf

  Documentation regarding GTS-4E-60 GPS Module might be found here:
  http://www.fibocom.com/product/2-1-3-2.html
*/

//DEMO - HOW TO USE:
/*
---------------------------------------- DEMO: 1 version ----------------------------------------
#include "mbed.h"
#include "gts4E60.h"
#define GPS_PIN_RX  PTE17    //UART2 on frdmkl46z
#define GPS_PIN_TX  PTE16
int main()
{
    GTS4E60 gps(GPS_PIN_TX,GPS_PIN_RX);
    Serial debug(USBTX, USBRX);
    debug.baud(115200);
    while(1) {
        if(gps.isDataAvailable()) {
            if(gps.parseData()) {
                struct UTC_Time utcTime= gps.getTime();
                struct Date date= gps.getDate();
                debug.printf("GPS_UTC_TIME: %02d:%02d:%02.3f\r\n",utcTime.hours, utcTime.minutes, utcTime.seconds);
                debug.printf("GPS_DATE: %02d.%02d.%02d\r\n", date.day, date.month, date.year);
                debug.printf("GPS_DATA: fixtype: %d, satelites: %d, altitude: %f, speed: %f, heading: %f\r\n",gps.getFixType(), gps.getSatellites(), gps.getAltitude(), gps.getSpeedKm(), gps.getHeading());
                debug.printf("GPS_DATA: status: %c, latitude: %f, ns :%c, longitude: %f, ew: %c\r\n",gps.getStatus(), gps.getLatitude(), gps.getNS(), gps.getLongitude(), gps.getEW());
            } else {
                debug.printf("NO GPS FIX FOUND\r\n");
            }
        }
    }
    return 0;
}

---------------------------------------- DEMO: 2 version  error handling----------------------------------------
#include "mbed.h"
#include "gts4E60.h"
#define GPS_PIN_RX  PTE17    //UART2 on frdmkl46z
#define GPS_PIN_TX  PTE16
int main()
{
    GTS4E60 gps(GPS_PIN_TX,GPS_PIN_RX);
    Serial usbDebug(USBTX, USBRX);
    usbDebug.baud(115200);
    while(1) {
        if(gps.isDataAvailable()) {
            uint8_t ret= gps.parseData();
            if(ret==ERROR) {
                usbDebug.printf("ERROR INCORRECT DATA\r\n");
            } else if(ret==NO_FIX_FOUND) {
                usbDebug.printf("NO GPS FIX FOUND\r\n");
            } else if(ret==NO_SATELLITES) {
                usbDebug.printf("NO SATELLITES FOUND\r\n");
            } else if(ret==INVALID_STATUS) {
                usbDebug.printf("STATUS INVALID\r\n");
            } else {
                struct UTC_Time utcTime= gps.getTime();
                struct Date date= gps.getDate();
                usbDebug.printf("GPS_UTC_TIME: %02d:%02d:%02.3f\r\n",utcTime.hours, utcTime.minutes, utcTime.seconds);
                usbDebug.printf("GPS_DATE: %02d.%02d.%02d\r\n", date.day, date.month, date.year);
                usbDebug.printf("GPS_DATA: fixtype: %d, satelites: %d, altitude: %f, speed: %f, heading: %f\r\n",gps.getFixType(), gps.getSatellites(), gps.getAltitude(), gps.getSpeedKm(), gps.getHeading());
                usbDebug.printf("GPS_DATA: status: %c, latitude: %f, ns :%c, longitude: %f, ew: %c\r\n",gps.getStatus(), gps.getLatitude(), gps.getNS(), gps.getLongitude(), gps.getEW());
            }
        }
    }
    return 0;
}
*/
#ifndef __GTS4E60_H__
#define __GTS4E60_H__

#include "mbed.h"
#include "BufferedSerial.h"
#include <string>

#define GTS4E60_SERIAL_DEFAULT_BAUD       9600
#define GTS4E60_SERIAL_TIMEOUT            10000
#define GTS4E60_SERIAL_EOL                "\r\n"
#define GTS4E60_NMEA_BUF_SIZE             512


typedef enum {
    //NMEA SENTENCES handled by the module: $GPGGA, $GPGSA, $GPRMC, $GPGSV
    GGA = 0,
    GSA = 1,
    RMC = 2,
    GSV = 3,
    NR_OF_SUPPORTED_NMEA_SENTENCES,
    //parseData() return paramteters
    INCORRECT_DATA =5,
    NO_FIX_FOUND= 6,
    NO_SATELLITES= 7,
    INVALID_STATUS= 8,
    IDLE_STATE
} GTS4E60_Utility;
static const char* nmeaSentencesString[NR_OF_SUPPORTED_NMEA_SENTENCES]= {"GPGGA","GPGSA","GPRMC","GPGSV"};


//deafault serial port on FRDM KL46Z:
// UART2:
// RX-PTE17
// TX-PTE16

//useful data structs
struct UTC_Time {
    UTC_Time() {
        hours =0;
        minutes =0;
        seconds=0;
    }
    int hours;
    int minutes;
    float seconds;
};


struct Date {
    Date() {
        day =0;
        month =0;
        year =0;
    }
    int day;
    int month;
    int year;
};



class GTS4E60
{
public:
    GTS4E60 (PinName tx, PinName rx,PinName shutdownPin);
    int write(const char* data); //?
    void init();
    uint8_t parseData(uint8_t param =NULL);
    int isDataAvailable();

//getters
    UTC_Time getTime();
    Date getDate();
    float getLongitude();
    float getLatitude();
    float getAltitude();
    float getSpeedKn();
    float getSpeedKm();
    int   getSatelites();
    float getCourseT();
    float getCourseM();
    int   getFixType();
    int   getSatellites();
    int   getStatus();
    char  getNS();
    char  getEW();
    float getHeading();
    
    inline int getDataFromRx() {
        return mGpsSerial.getc();
    }
    
    inline GTS4E60_Utility getStatusType(void){
        return mStatus;
    }
    
    inline void setStatusType(GTS4E60_Utility error){
             mStatus=error;
    }

// navigational functions - maybe in future
    float calcCourseTo(float, float);
    double calcDistToKm(float, float);
    double calcDistToM(float, float);

  

private:
    // shutdown pin
    DigitalOut mShutdownPin;
    float trunc ( float v);
    float nmeaToDecimal(float deg_coord, char nsew);
    void readData();
    void readData(uint8_t nmeaSentence);
    Serial mGpsSerial;
    char mNmeaData[GTS4E60_NMEA_BUF_SIZE];

    // GGA - Global Positioning System Fixed Data
    struct UTC_Time mUtcTime;         // UTC time
    int mFixType;        // 0 = no fix;  1 = fix;  2=differential fix
    int mSatellites;     // number of satellites used
    float mHdop;
    float mAltitude;
    char mUnits;

    //GSA
    //not used here

    // RMC - Recommended Minimmum Specific GNS Data
    char mDataStatus;// RMC data status A = Data Valid; V = Data Not valid;
    float mLatitude;
    float mLongitude;
    char NS, EW;
    float mSpeedKn;      // speed in knots/hour
    float mSpeedKm;      // speed in kilometres/hour
    float mHeading;      // heading in degrees derived from previous & current location
    struct Date mDate;

    //GSV - GNSS Satellites in View
    int mNumberOfMsgs;
    int mMsgNumber;
    int mSatellitesInView;

    //useful variables
    string mFix;
    string mCardinal;
    
    GTS4E60_Utility mStatus;

};



#endif // __GTS4E60_H__
