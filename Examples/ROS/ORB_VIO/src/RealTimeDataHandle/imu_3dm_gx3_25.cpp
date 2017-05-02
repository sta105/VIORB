#include "imu_3dm_gx3_25.h"
#include <ros/ros.h>

IMU_3DM_GX3_25::IMU_3DM_GX3_25()
{
}

void IMU_3DM_GX3_25::copy_data(unsigned char* buf)
{
    for(int i=0; i <= IMU_3DMDATA_LEN; i++)
    {
        data_[i] = buf[i];
    }
}

bool IMU_3DM_GX3_25::extract_data()
{
    // data[0]: 0xCB
    unsigned int k=1;
    //    for (unsigned int i = 0; i<3;i++, k+=4)
    for (unsigned int i = 0; i < 3; i++, k += 4)
        acc[i] = extract_float(&(data_[k]));
    for (unsigned int i = 0; i < 3; i++, k += 4)
        gyr[i] = extract_float(&(data_[k]));
    for (unsigned int i = 0; i < 3; i++, k += 4)
        mag[i] = extract_float(&(data_[k]));
//    for (unsigned int i = 0; i < 9; i++, k += 4)
//        M[i] = extract_float(&(data_[k]));
    timerI = extract_int(&(data_[k]));
    timerD = timerI / 62500.0;

    trigFlag = data_[43];

    ROS_ASSERT(k==37);
//    if(k!=43)
//        ROS_ERROR("k!=43, sholdn't");
}

float IMU_3DM_GX3_25::extract_float(unsigned char* addr)
{
  float tmp;

  *((unsigned char*)(&tmp) + 3) = *(addr);
  *((unsigned char*)(&tmp) + 2) = *(addr+1);
  *((unsigned char*)(&tmp) + 1) = *(addr+2);
  *((unsigned char*)(&tmp)) = *(addr+3);

  return tmp;
}

int IMU_3DM_GX3_25::extract_int(unsigned char* addr)
{
  int tmp;

  *((unsigned char*)(&tmp) + 3) = *(addr);
  *((unsigned char*)(&tmp) + 2) = *(addr+1);
  *((unsigned char*)(&tmp) + 1) = *(addr+2);
  *((unsigned char*)(&tmp)) = *(addr+3);

  return tmp;
}

bool IMU_3DM_GX3_25::validate_checksum(const unsigned char *data, unsigned short length)
{
  unsigned short chksum = 0;
  unsigned short rchksum = 0;

  for (unsigned short i = 0; i < length - 2; i++)
    chksum += data[i];

  rchksum = data[length - 2] << 8;
  rchksum += data[length - 1];

  return (chksum == rchksum);
}
