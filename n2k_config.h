#pragma once

//#define TESTING               // Enable canned data and other testing features. Not for production.
//#define SCREEN_CAPTURE        // Enable serial server to grab screenshots.

//#define USE_METRIC            // Define to use m (depth), celcius (temp)
//#define USE_METRIC_PRESSURE   // Define to use mbar (pressure) vs inHg

#ifdef SCREEN_CAPTURE
#define BAUDRATE    921600
#else
#define BAUDRATE    115200
#endif



