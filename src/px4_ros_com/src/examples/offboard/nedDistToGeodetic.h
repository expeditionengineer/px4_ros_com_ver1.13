struct ecefCoords {
    float x;
    float y;
    float z;
};

struct geodeticCoords {
    float lat;
    float lon;
    float alt;
};

struct uvwCoords {
    float u;
    float v;
    float w;
};

struct enuCoords {
    float e1;
    float n1;
    float u1;
};

ecefCoords geodetic2ecef(geodeticCoords* geoCoord);
uvwCoords enu2uvw(float e1, float n1, float u1, float lat0, float lon0);
float hypot(float x, float y) ;
float degrees(float radians);
geodeticCoords ecef2geodetic(ecefCoords* ecef);
geodeticCoords enu2geodetic(enuCoords* enu, geodeticCoords* geo);
