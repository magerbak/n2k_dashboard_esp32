#pragma once

#include <stdint.h>

double rad2Deg(double rad);
double deg2Rad(double rad);
double getSignedBearing(double bearing);

double fixedFrac2Double(uint16_t x, double precision = 1.0);
double meters2Nm(double m);
double meters2Ft(double m);
double metersPerSec2Kts(double spd);

bool areFloatsEqual(double a, double b, double epsilon);


