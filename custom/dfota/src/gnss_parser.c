#include "ql_type.h"
#include "ql_stdlib.h"
#include "ql_uart.h"
#include "gnss_parser.h"


static double nmea2decimal(const char* val, const char* hemi)
{
    // Convert DDMM.MMMM → decimal degrees
    double degMin = Ql_atof(val);
    int deg = (int)(degMin / 100);
    double minutes = degMin - (deg * 100);
    double dec = deg + minutes / 60.0;

    if (hemi[0] == 'S' || hemi[0] == 'W')
        dec = -dec;

    return dec;
}

int parseGPRMC(const char* nmea, GNSS_Info* info)
{
    // Example: $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,,*6A
    char type[8], utc[16], status[2], lat[16], ns[2], lon[16], ew[2],
         speed[16], course[16], date[16];

    int n = Ql_sscanf(nmea,
                      "$%6[^,],%15[^,],%1[^,],%15[^,],%1[^,],%15[^,],%1[^,],%15[^,],%15[^,],%15[^,]",
                      type, utc, status, lat, ns, lon, ew, speed, course, date);

    if (n < 10) {
        return -1; // parse failed
    }

    Ql_strcpy(info->utc, utc);
    Ql_strcpy(info->date, date);
    info->valid = (status[0] == 'A') ? 1 : 0;
    info->latitude = nmea2decimal(lat, ns);
    info->longitude = nmea2decimal(lon, ew);
    info->speed = Ql_atof(speed);
    info->course = Ql_atof(course);

    return 0;
}
