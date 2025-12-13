// utils_geo.h
//

#ifndef __UTILS_GEO_H__
#define __UTILS_GEO_H__

#include <stdint.h>
#include <math.h>

#define PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105


////////////////////////////////////////////////////////////////////////////////////////////////
//

#define _MIN(x, y)              ((x) < (y) ? (x) : (y))
#define _MAX(x, y)              ((x) > (y) ? (x) : (y))
#define _CLAMP(x, min, max)     (((x) < (min)) ? (min) : (((x) > (max) ? (max) : (x))))
#define _ABS(x)                 ((x) > 0 ? (x) : (-(x)))


////////////////////////////////////////////////////////////////////////////////////////////////
//

inline double TO_RADIAN(double deg)
{
    return deg / 180.0 * PI;
}

inline double TO_DEGREE(double rad)
{
    return rad * (180.0 / PI);
}



////////////////////////////////////////////////////////////////////////////////////////////////
//

int16_t     GET_BEARING(double lat1, double lon1, double lat2, double lon2);
double      GET_DISTANCE(double lat1, double lon1, double lat2, double lon2);


#endif // __UTILS_GEO_H__
