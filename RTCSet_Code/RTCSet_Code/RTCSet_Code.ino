#include "mbed.h"
#include <mbed_mktime.h>

/*
This code is used to set the RTC, use it whenever using a new Giga to set the correct time
Have to adjust the time in the RTCset function
Needs a battery connected to the VRTC pin to work and keep time
*/

constexpr unsigned long printInterval { 1000 };
unsigned long printNow {};

void setup() {
  Serial.begin(9600);
  RTCset();
}

void loop() {
      if (millis() > printNow) {
        Serial.print("System Clock:          ");
        Serial.println(getLocaltime());
        printNow = millis() + printInterval;
    }
}

void RTCset()  // Set cpu RTC
{    
  tm t;
            t.tm_sec = (0);       // 0-59
            t.tm_min = (6);        // 0-59
            t.tm_hour = (10);         // 0-23
            t.tm_mday = (10);   // 1-31
            t.tm_mon = (11);       // 0-11  "0" = Jan, -1 
            t.tm_year = ((24)+100);   // year since 1900,  current year + 100 + 1900 = correct year
            set_time(mktime(&t));       // set RTC clock                                 
}

String getLocaltime()
{
    char buffer[32];
    tm t;
    _rtc_localtime(time(NULL), &t, RTC_4_YEAR_LEAP_YEAR_SUPPORT);
    strftime(buffer, 32, "%Y-%m-%d %k:%M:%S", &t);
    return String(buffer);
}