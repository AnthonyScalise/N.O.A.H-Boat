

#include "GpsWayPoint.h"
#include "Arduino.h"
#include "Math.h"

#define DEG_TO_RAD (double)(3.141592654/180.0)
#define RAD_TO_DEG (double)(180.0/3.141592654)
// #define RAD_TO_DEG 57.295779513082320876798154814105F
// #define EARTH_RADIUS 6371000.0F
#define EARTH_RADIUS (double)(6371000.0)
#define E_PLUS_TEN 1000000000.0F
#define PI 3.1415926535897932384626433832795F 



GpsWayPoint::GpsWayPoint(float longitude, float latitude) {
    this->_point = {
        longitude,
        latitude
    };
}

WayPoint_t GpsWayPoint::getPoint(void) {
    return(this->_point);
}

float GpsWayPoint::getLongitude(void) {
    return(this->_point.longitude);
}

float GpsWayPoint::getLatitude(void) {
    return(this->_point.latitude);
}

float GpsWayPoint::getDistanceTo(float lon1, float lat1, float lon2, float lat2) {
    float latDistance = ((lat2 - lat1) * DEG_TO_RAD);
    float lonDistance = ((lon2 - lon1) * DEG_TO_RAD);
    float calc1 = (powf(sinf(latDistance / 2.0F), 2.0F) + (cosf(lat1 * DEG_TO_RAD) *
                   cosf(lat2 * DEG_TO_RAD) * powf(sinf(lonDistance / 2.0F), 2.0F)));
    float calc2 = (2.0F * atan2f(sqrtf(calc1), sqrtf(1.0F - calc1)));
    float distance = (EARTH_RADIUS * calc2);
    return(distance);
}

float GpsWayPoint::getDistanceTo(GpsWayPoint targetPoint) {
    float lon1 = this->_point.longitude;
    float lat1 = this->_point.latitude;
    float lon2 = targetPoint._point.longitude;
    float lat2 = targetPoint._point.latitude;
    return(getDistanceTo(lon1, lat1, lon2, lat2));
}

float GpsWayPoint::getDistanceFrom(float lon1, float lat1) {
    float lon2 = this->_point.longitude;
    float lat2 = this->_point.latitude;
    return(getDistanceTo(lon1, lat1, lon2, lat2));
}

// float GpsWayPoint::getAngleTo(GpsWayPoint targetPoint) {
//     float y = (sinf(targetPoint._point.longitude - this->_point.longitude) * cosf(this->_point.latitude));
//     float x = ((cosf(this->_point.latitude) * sinf(targetPoint._point.latitude)) -
//                (sinf(this->_point.latitude) * cosf(targetPoint._point.latitude) *
//                 cosf(targetPoint._point.longitude - this->_point.longitude)));
//     float thetaRad = atan2f(y, x);
//     float thetaDeg = (360.0F - (thetaRad * RAD_TO_DEG));
//     return(thetaDeg);
// }

float GpsWayPoint::getAngleTo(GpsWayPoint targetPoint) {
    float lonDiff = (((targetPoint._point.longitude - this->_point.longitude)*22)/1260);
    float angDist = acos(cos(((90-targetPoint._point.latitude)*22)/1260)*
                    cos(((90-this->_point.latitude)*22)/1260)+sin(((90-targetPoint._point.latitude)*22)/1260)*
                    sin(((90-this->_point.latitude)*22)/1260)*cos(lonDiff));
    float azimuthRad = asin((sin(((90-targetPoint._point.latitude)*22)/1260)*sin(lonDiff))/sin(angDist));
    float azimuthDeg = degrees(azimuthRad);
    return(azimuthDeg);
}

float GpsWayPoint::getAngleTo(float lon, float lat) {
    float lonDiff = (((this->_point.longitude - lon)*22)/1260);
    float angDist = acos(cos(((90-this->_point.latitude)*22)/1260)*
                    cos(((90-lat)*22)/1260)+sin(((90-this->_point.latitude)*22)/1260)*
                    sin(((90-lat)*22)/1260)*cos(lonDiff));
    float azimuthRad = asin((sin(((90-this->_point.latitude)*22)/1260)*sin(lonDiff))/sin(angDist));
    float azimuthDeg = degrees(azimuthRad);
    return(azimuthDeg);
}

double GpsWayPoint::courseTo(long lat1, long lon1, long lat2, long lon2, double* distance) {
    double dlam, dphi;
    double radius = EARTH_RADIUS;
    dphi = (DEG_TO_RAD * (lat1+lat2)*0.5e-6);
    double cphi = cos(dphi);
    dphi = (DEG_TO_RAD * (lat2 - lat1)*1.0e-6);
    dlam = (DEG_TO_RAD * (lon2 - lon1)*1.0e-6);
    dlam *= cphi;
    double bearing = (RAD_TO_DEG * atan2(dlam, dphi));
    if(bearing < 0) {
        bearing = bearing+360.0;
    }
    *distance = (radius * sqrt(dphi*dphi + dlam*dlam));
    return(bearing);
}