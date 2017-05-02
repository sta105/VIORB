#ifndef IMU_3DM_GX3_25_H
#define IMU_3DM_GX3_25_H

#include <stdint.h>

// imu's data rate 200Hz
#define IMU_3DMDATA_RATE 200
// imu's primary buffer length is 43 bytes, with the 44'th data - the trigger flag for camera
#define IMU_3DMDATA_LEN 43
// 1/62500 second/count, 62500 count per second, used in timer
#define TIMER_CNTPERSEC 62500

// image 200Hz/ imu 20Hz, 10 IMUs between images
#define IMUCNTPERIMG 10

class IMU_3DM_GX3_25
{
public:
    IMU_3DM_GX3_25();

//    bool dataget;   // data received
//    bool dataok;    // checksum right
    unsigned char data_[44];

    float acc[3];
    float gyr[3];
    float mag[3];
    uint32_t timerI;     // 1/62500 second/count, 62500 count per second
    double timerD;
    unsigned char trigFlag;

    void copy_data(unsigned char* buf);
    bool extract_data();

    static bool validate_checksum(const unsigned char* data, uint16_t length);
    static float extract_float(unsigned char* addr);
    static int extract_int(unsigned char* addr);
};

#endif // IMU_3DM_GX3_25_H
