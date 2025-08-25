
typedef struct {
    double latitude;
    double longitude;
    double speed;     // knots
    double course;    // degrees
    char utc[16];
    char date[16];
    int valid;        // 1 = valid fix
} GNSS_Info;


static double nmea2decimal(const char* val, const char* hemi);
int parseGPRMC(const char* nmea, GNSS_Info* info);