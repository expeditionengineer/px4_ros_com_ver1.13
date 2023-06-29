
#include <cmath>

// #include <catch2/catch_test_macros.hpp>

#include "nedDistToGeodetic.h"

const float MAJOR_AXIS = 6378137.0;
const float MINOR_AXIS = 6356752.31424518;

float radians(float degree)
{
    return 3.1415926535897 / 180.0 * degree;
}

ecefCoords geodetic2ecef(geodeticCoords *geoCoord)
{

    //     float lat = geoCoord->lat;
    //     float lon = geoCoord->lon;
    //     float alt = geoCoord->alt;

    // convert degrees to rediands:
    float latRad = radians(geoCoord->lat);
    float lonRad = radians(geoCoord->lon);
    float alt = geoCoord->alt;

    float N = pow(MAJOR_AXIS, 2) / sqrt(pow(MAJOR_AXIS, 2) * pow(cos(latRad), 2) + pow(MINOR_AXIS, 2) * pow(sin(latRad), 2));

    ecefCoords ecefCoord = ecefCoords();
    ecefCoord.x = (N + alt) * cos(latRad) * cos(lonRad);
    ecefCoord.y = (N + alt) * cos(latRad) * sin(lonRad);
    ecefCoord.z = (N * pow(MINOR_AXIS / MAJOR_AXIS, 2) + alt) * sin(latRad);

    return ecefCoord;
}

uvwCoords enu2uvw(float e1, float n1, float u1, float lat0, float lon0)
{

    float lat0Rad = radians(lat0);
    float lon0Rad = radians(lon0);

    float t = cos(lat0Rad) * u1 - sin(lat0Rad) * n1;
    float w = sin(lat0Rad) * u1 + cos(lat0Rad) * n1;

    float u = cos(lon0Rad) * t - sin(lon0Rad) * e1;
    float v = sin(lon0Rad) * t + cos(lon0Rad) * e1;

    return {u, v, w};
}

ecefCoords enu2ecef(geodeticCoords *geoCoord, enuCoords *enuCoord)
{
    ecefCoords ecefCoord = geodetic2ecef(geoCoord);
    uvwCoords uvwCoord = enu2uvw(enuCoord->e1, enuCoord->n1, enuCoord->u1, geoCoord->lat, geoCoord->lon);

    return {ecefCoord.x + uvwCoord.u, ecefCoord.y + uvwCoord.v, ecefCoord.z + uvwCoord.w};
}

float hypot(float x, float y)
{
    return sqrt(pow(x, 2) + pow(y, 2));
}

float degrees(float radians)
{
    return 180.0 / 3.1415926535897 * radians;
}

geodeticCoords ecef2geodetic(ecefCoords *ecef)
{

    float x = ecef->x;
    float y = ecef->y;
    float z = ecef->z;
    float r = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
    float E = sqrt(pow(MAJOR_AXIS, 2) - pow(MINOR_AXIS, 2));

    float u = sqrt(0.5 * (pow(r, 2) - pow(E, 2)) + 0.5 * sqrt(pow(pow(r, 2) - pow(E, 2), 2) + 4 * pow(E, 2) * pow(z, 2)));

    float Q = sqrt(pow(x, 2) + pow(y, 2));
    float huE = sqrt(pow(u, 2) + pow(E, 2));

    float Beta = atan(huE / u * z / Q);
    float dBeta = ((MINOR_AXIS * u - MAJOR_AXIS * huE + pow(E, 2)) * sin(Beta)) / (MAJOR_AXIS * huE / cos(Beta) - pow(E, 2) * cos(Beta));

    Beta = Beta + dBeta;

    float lat = atan(MAJOR_AXIS / MINOR_AXIS * tan(Beta));
    float lon = atan2(y, x);

    float alt = hypot(z - MINOR_AXIS * sin(Beta), Q - MAJOR_AXIS * cos(Beta));

    return {degrees(lat), degrees(lon), alt};
}

geodeticCoords enu2geodetic(enuCoords *enu, geodeticCoords *geo)
{
    ecefCoords ecef = enu2ecef(geo, enu);
    return ecef2geodetic(&ecef);
}

// TEST_CASE("Test geodetic to enu") {
//     geodeticCoords exampleCoord = {48.8562, 2.3508, 0.0674 * 1000.0};
//     ecefCoords computedCoords = geodetic2ecef(&exampleCoord);
//     REQUIRE(abs(computedCoords.x - 4200996.789553433) < 0.1);
//     REQUIRE(abs(computedCoords.y - 172460.3215373718) < 0.1);
//     REQUIRE(abs(computedCoords.z - 4780102.830875212) < 0.1);

//     enuCoords fiveMetersToNorth = {0.0, 5.0, 0.0};

//     ecefCoords resultOfTest = enu2ecef(&exampleCoord, &fiveMetersToNorth);

//     REQUIRE(abs(resultOfTest.x - 4200993.027419041) < 0.1);
//     REQUIRE(abs(resultOfTest.y - 172460.16709333402) < 0.1);
//     REQUIRE(abs(resultOfTest.z - 4780106.120630804) < 0.1);
//     // https://github.com/geospace-code/pymap3d

//     geodeticCoords geoResult = enu2geodetic(&fiveMetersToNorth, &exampleCoord);

//     REQUIRE(abs(geoResult.lat - 48.856244960742515) < 0.1);
//     REQUIRE(abs(geoResult.lon - 2.3507999999999996) < 0.1);

// }
