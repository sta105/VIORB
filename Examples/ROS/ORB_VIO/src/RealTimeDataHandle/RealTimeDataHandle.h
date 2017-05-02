#ifndef ORBVIO_REALTIMEDATAHANDLE_H
#define ORBVIO_REALTIMEDATAHANDLE_H

#include <queue>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
//#include "myserial.h"
#include "imu_3dm_gx3_25.h"
#include <ros/ros.h>
#include <serial/serial.h>
#include <thread>
#include "../../../include/System.h"
#include "../../../src/IMU/imudata.h"

using namespace std;

#define IGNORE_FIRST10_IMUPKG

class DataSync
{
public:
    DataSync(ros::NodeHandle& nh_, ORB_SLAM2::System* pslam, double imageDelaySec);
    ORB_SLAM2::System* mpSLAM;

    ros::NodeHandle& nh;

    std::string imageTopic;
    void SetImageTopic(const std::string imgtopic_) {imageTopic = imgtopic_;}
    double imageMsgDelaySec;

    //
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    unsigned int imageCBCnt;

    //
    bool repub_flag;
    std::string image_repub_topic;
    ros::Publisher imageRePub;

    // start time
    bool firstImageFlag;
    ros::Time startTime;

    std::mutex mutexIMUData;
    std::list<ORB_SLAM2::IMUData> listIMUData;
    void AddIMUData(const double& gx, const double& gy, const double& gz,
                    const double& ax, const double& ay, const double& az,
                    const double& t)
    {
        unique_lock<mutex> mutexIMUData;
        listIMUData.push_back(ORB_SLAM2::IMUData(gx,gy,gz,ax,ay,az,t));
    }
    const ORB_SLAM2::IMUData& getFirstIMUData(void) const
    {
        unique_lock<mutex> mutexIMUData;
        return listIMUData.front();
    }

};

class IMUDataHandle
{
public:
    IMUDataHandle(DataSync* pdc):serial_thread(NULL), quitflag(false), pdatasync(pdc)
    {};

    ~IMUDataHandle()
    {
        quitflag=true;
        if(serial_thread->joinable())
        {
            ROS_INFO("wait for serial thread quit");
            serial_thread->join();
        }
        serial_thread = NULL;

        myserial.close();
    }

    void InitSerial(const string& port, const int& baud, const int& toms=1000);
    void StartEB90(void);

    thread* serial_thread;
    serial::Serial myserial;
    void imuSerialLoop(void);
    bool quitflag;
    DataSync* pdatasync;
    ros::Publisher imuRePub;

    unsigned char imubuf[(1+IMU_3DMDATA_LEN)*(IMUCNTPERIMG+1)];
};

#endif
