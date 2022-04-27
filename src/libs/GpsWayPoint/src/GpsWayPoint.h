

#ifndef GPS_WAY_POINT_h
#define GPS_WAY_POINT_h

#include "Arduino.h"


struct WayPoint_t {
    float longitude;
    float latitude;
};

struct WayPointDouble_t {
    double longitude;
    _LONG_DOUBLE latitude;
};


class GpsWayPoint {
public:
    GpsWayPoint(float longitude, float latitude);
    WayPoint_t getPoint(void);
    float getLongitude(void);
    float getLatitude(void);
    float getDistanceTo(float lon1, float lat1, float lon2, float lat2);
    float getDistanceTo(GpsWayPoint targetPoint);
    float getDistanceFrom(float lon1, float lat1);
    float getAngleTo(GpsWayPoint targetPoint);
    float getAngleTo(float lon, float lat);
    static double courseTo(long lat1, long lon1, long lat2, long lon2, double* distance);
    
private:
    WayPoint_t _point;
};


#endif