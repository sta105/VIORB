#include "RealTimeDataHandle.h"

void IMUDataHandle::StartEB90()
{
    unsigned char data[2]={0xEB,0x90};
    //myserial.writeData(data,2);
    myserial.write(data,2);
    unsigned char rd[10];
    //int readlen=myserial.readData(rd,2);
    int tmpcnt=0,readlen;

    while(myserial.available()<2)
    {
        usleep(1000);
        if(tmpcnt++>1000)
        {
            ROS_ERROR("try 1000 no EB 90 echo, exit");
            exit(-1);
        }
    }
    readlen = myserial.read(rd,2);
    if(readlen!=2)
    {
        ROS_ERROR("readlen!=2 at the beginning, readlen=%d, data: %2x %2x",readlen,rd[0],rd[1]);
    }
    printf("EB90 echo %d bytes: %x %x\n",readlen,rd[0],rd[1]);
    if(rd[0]!=0xEB || rd[1]!=0x90 || readlen!=2)
        ROS_ERROR("EB 90 start error");
}


void IMUDataHandle::InitSerial(const string& portname, const int& baudrate, const int& timeoutms)
{
    myserial.setPort(portname);
    myserial.setBaudrate(baudrate);
    auto t= serial::Timeout::simpleTimeout(timeoutms);
    myserial.setTimeout(t);
    myserial.open();
    if(!myserial.isOpen())
    {
        ROS_ERROR("serial open failed");
        exit(-1);
    }

}

void IMUDataHandle::imuSerialLoop(void)
{
    bool firstflag = true;
    //static int64_t imuStartTimeI=0;
    static double imuStartTimerD=0.0;
    //static size_t imupkgcnt = 0;
    static double curTimerD, preTimerD;

    static double rosStartTimer=0.0/*,rosCurTimer=0.0*/;

    static int imufirst10pkgcnt=0;

    ros::Rate r(400);
    int loopcnt=0;
    while(ros::ok() && !quitflag)
    {
        size_t availablesize = myserial.available();
        //cerr<<"IMU availablesize: "<<availablesize<<endl;
        if(availablesize > IMU_3DMDATA_LEN)
        {
            // first data should be 0x
            //myserial.readData(imubuf,1);
            size_t cnt = myserial.read(imubuf,1);
            while(imubuf[0] != 0xCB)
            {
                if(cnt++>1000)
                {
                    ROS_ERROR("1000 data no 0xCB, exit");
                    exit(-1);
                }
                ROS_ERROR("0xCB error: %2x, read another 1 byte for head",imubuf[0]);
                myserial.read(imubuf,1);
            }
            // other 43 bytes (first 0xCB has been read and stored in imubuf[0])
            int readlen = myserial.read(imubuf+1, IMU_3DMDATA_LEN);
            // validate checksum for 0~42, trig flag not need
            if(IMU_3DM_GX3_25::validate_checksum(imubuf, IMU_3DMDATA_LEN) && readlen==IMU_3DMDATA_LEN)
            {
                // 0~43, 43 is trig flag
                if(imubuf[IMU_3DMDATA_LEN]==0 || imubuf[IMU_3DMDATA_LEN]==1 )
                {
                    // extract data
                    IMU_3DM_GX3_25 imudata;
                    imudata.copy_data(imubuf);
                    imudata.extract_data();
                    ROS_ASSERT(imudata.trigFlag==0 || imudata.trigFlag==1);

                    if(imufirst10pkgcnt<10)
                    {
                        imufirst10pkgcnt++;
                        ROS_INFO("package: %d, trig flag: %d",imufirst10pkgcnt,imubuf[IMU_3DMDATA_LEN]);
                        if(imufirst10pkgcnt==10)
                        {
                            if(imubuf[IMU_3DMDATA_LEN]!=1)
                            {
                                ROS_ERROR("the 10'th package is not trigger");
                                exit(-1);
                            }

                            rosStartTimer = ros::Time::now().toSec();
                            //imuStartTimeI = imudata.timerI;
                            imuStartTimerD = imudata.timerD;
                            ROS_WARN("imu start timer D: %.4f , I: %d",imuStartTimerD,imudata.timerI);
                        }
                    }

                    // record the first timer of IMU
                    if(firstflag)
                    {
                        preTimerD = imudata.timerD;
                        firstflag = false;
                        ROS_INFO("get first package");
                        for(int i=0;i<=IMU_3DMDATA_LEN;i++)
                        {
                            printf("%2x ",imubuf[i]);
                        }
                        printf("\n");

                    }

                    // sync with first image message
                    if(!pdatasync->firstImageFlag && imufirst10pkgcnt>=10)
                    {
                        // compute timestamp for IMU message
                        curTimerD = imudata.timerD;
                        double deltaimutime = curTimerD - preTimerD;
                        if(fabs(deltaimutime-0.005)>0.001)
                        {
                            ROS_ERROR("fabs(deltaimutime-0.005)>0.001, imu message delta time: %.6f",deltaimutime);
                        }
                        // consider the timer rollover
                        if(deltaimutime < 0)
                        {
                            imuStartTimerD -= (0x100000000)/62500.0;
                            ROS_WARN("A timer jump from %.6f to %.6f, delta time: %.6f",preTimerD,curTimerD,deltaimutime);
                        }
                        preTimerD = curTimerD;

                        // time since first timer
                        double deltaIMUTimeFromStart = curTimerD - imuStartTimerD ;

                        // Add imu data
                        double imut = pdatasync->startTime.toSec() + deltaIMUTimeFromStart + pdatasync->imageMsgDelaySec;
                        pdatasync->AddIMUData(imudata.gyr[0], imudata.gyr[1], imudata.gyr[2],
                                              imudata.acc[0]*9.80665, imudata.acc[2]*9.80665, imudata.acc[2]*9.80665,
                                              imut );

                        static int testtimecomparecnt=2000;
                        if(fabs(deltaimutime-0.005)>0.0005)
                        {
                            ROS_WARN("deltaimutime = %.6f",(deltaimutime));
                            double rosTimerFromStart = ros::Time::now().toSec() - rosStartTimer;
                            ROS_WARN("dt: %.5f, time from start: %.5f, ros time: %.5f, diff: %.5f",
                                     deltaimutime,deltaIMUTimeFromStart,rosTimerFromStart,rosTimerFromStart-deltaIMUTimeFromStart);
                            ROS_WARN("ros::Time::now vs imu msg.stamp: %.6f vs %.6f, diff: %.6f",
                                     ros::Time::now().toSec(), imut, ros::Time::now().toSec()-imut);
                        }
                        if(++testtimecomparecnt>=2000)
                        {
                            testtimecomparecnt=0;
                            double rosTimerFromStart = ros::Time::now().toSec() - rosStartTimer;
                            ROS_INFO("dt: %.5f, time from start: %.5f, ros time: %.5f, diff: %.5f",
                                     deltaimutime,deltaIMUTimeFromStart,rosTimerFromStart,rosTimerFromStart-deltaIMUTimeFromStart);
                            ROS_INFO("ros::Time::now vs imu msg.stamp: %.6f vs %.6f, diff: %.6f",
                                     ros::Time::now().toSec(), imut, ros::Time::now().toSec()-imut);
                        }
                    }
                    // if image not ready, use ros::Time::now
                    else
                    {
                        //if(pdatasync->firstImageFlag)
                            //ROS_WARN("image not ready, no imu data published");
                        if(imufirst10pkgcnt<10)
                            ROS_WARN("the %d's package, ignore",imufirst10pkgcnt);
                    }

                }
                else
                {
                    ROS_ERROR("last byte not 0/1 trig flag");
                    if(firstflag)
                    {
                        ROS_ERROR("\n\n first package error. need reset");
                        exit(-1);
                    }
                }

            }
            else
            {
                ROS_ERROR("read %d data, checksum error. package: ",readlen);
                printf("package %d: ",loopcnt);
                for(int i=0;i<=IMU_3DMDATA_LEN;i++)
                {
                    printf("%2x ",imubuf[i]);
                }
                printf("\n");
                if(firstflag)
                {
                    ROS_ERROR("\n\n first package error. need reset");
                    exit(-1);
                }
            }

            //            printf("package %d: ",loopcnt);
            //            for(int i=0;i<=IMU_3DMDATA_LEN;i++)
            //            {
            //                printf("%2x ",imubuf[i]);
            //            }
            //            printf("\n");
        }

        else
        {
            //ROS_INFO("%lu available",availablesize);
        }

        if(loopcnt++>2500)
        {
            loopcnt=0;
            //ROS_INFO("test");
        }
        //usleep(4000);
        r.sleep();
    }
}


DataSync::DataSync(ros::NodeHandle& nh_, ORB_SLAM2::System* pslam, double imageDelaySec):
    nh(nh_), mpSLAM(pslam), firstImageFlag(true)
{
    imageCBCnt = 0;
    imageMsgDelaySec = imageDelaySec;
}

/*
 * Time line of IMU and Image data
 * IMU data: 200Hz , trigger camera every 10 package, i.e. Image 20Hz
 * I: IMU data, T: IMU data and trigger camera
 * IMU (5 imus/image for example):
 * I I I I T I I I I T
 *         |         |
 * camera: |         |
 *         ---|      ---|       --- means image delay after triiger
 *            image     image
 *
 *
 * */
void DataSync::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    static double premsgtime = 0;
    static long int imgcnt = 0;
    if(firstImageFlag)
    {
        startTime = msg->header.stamp;
        premsgtime = startTime.toSec();
        ROS_WARN_STREAM("Record start time: "<<startTime);

        // reset the callback counter
        imageCBCnt = 0;

        firstImageFlag = false;
    }

    double dtime = msg->header.stamp.toSec() - premsgtime;
    if(fabs(dtime-0.05)>0.001)
        ROS_ERROR("image message delta time = %.6f",dtime);

    imgcnt += static_cast<int>(round(dtime*20));

    // image timestamp
    double imagetime = startTime.toSec() + 0.05*imgcnt;


    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return ;
    }

    // image timestamp delay has been considered in IMUData._t

    // Wait imu data
    double tlastimu=0;
    {
        unique_lock<mutex> mutexIMUData;
        const ORB_SLAM2::IMUData& lastimu = listIMUData.back();
        tlastimu = lastimu._t;
    }
    while(tlastimu < imagetime)
    {
        unique_lock<mutex> mutexIMUData;
        const ORB_SLAM2::IMUData& lastimu = listIMUData.back();
        tlastimu = lastimu._t;
        ROS_WARN("sleep 10ms to wait imu data, last imutime - imagetime = %.1f ms",1000*(tlastimu-imagetime));
        usleep(10000);
    }

    // Get image data
    cv::Mat im = cv_ptr->image.clone();

    // Get imu data
    std::vector<ORB_SLAM2::IMUData> vimu;
    {
        unique_lock<mutex> mutexIMUData;
        double tmpt=0;
        do
        {
            const ORB_SLAM2::IMUData& frontimu = listIMUData.front();
            vimu.push_back(frontimu);
            listIMUData.pop_front();

            const ORB_SLAM2::IMUData& frontimu_new = listIMUData.front();
            tmpt = frontimu_new._t;
        }while(tmpt < imagetime);
    }

    // Track
    mpSLAM->TrackMonoVI(im, vimu, imagetime);



    // Some other preparation code below.

    static int testimgcnt=2000;
    if(++testimgcnt>=200)
    {
        testimgcnt=0;

        //if(fabs(imageTime2.toSec()-msg->header.stamp.toSec())>1e-3)

        ROS_INFO("ros::Time::now vs msg.stamp: %.6f vs %.6f, difference: %.6f",ros::Time::now().toSec(),msg->header.stamp.toSec(),
                 ros::Time::now().toSec()-msg->header.stamp.toSec());
        ROS_INFO("image msg stamp from start: %.6f",msg->header.stamp.toSec()-startTime.toSec());

    }


    // increment callback counter
    imageCBCnt++;
    premsgtime = msg->header.stamp.toSec();

}
