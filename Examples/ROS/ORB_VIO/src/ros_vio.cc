/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

#include "MsgSync/MsgSynchronizer.h"

#include "../../../src/IMU/imudata.h"
#include "../../../src/IMU/configparam.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>

#include "ORB_VIO/viorb_msg.h"
#include "../../../include/Converter.h"

//#undef RUN_REALTIME

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    ORB_SLAM2::System* mpSLAM;
};

void PublishSLAMData(ros::Publisher& slamdatapub, const ORB_SLAM2::SLAMData& slamdata)
{
    ORB_VIO::viorb_msg slamdatamsg;
    slamdatamsg.header.stamp = ros::Time(slamdata.Timestamp);
    // VINSInitFlag
    slamdatamsg.VINSInitFlag.data = slamdata.VINSInitFlag;
    // TrackStatus
    slamdatamsg.TrackStatus.data = slamdata.TrackingStatus;
    // -1-notready,0-noimage,1-initing,2-OK
    if(slamdata.TrackingStatus > 1 && slamdata.VINSInitFlag==true)
    {
        cv::Mat Rwi = slamdata.Rwi;

        // Qwi
        Eigen::Matrix<double,3,3> eigRwi = ORB_SLAM2::Converter::toMatrix3d(Rwi);
        Eigen::Quaterniond qwi(eigRwi);
        geometry_msgs::Quaternion qwimsg;
        qwimsg.x = qwi.x();
        qwimsg.y = qwi.y();
        qwimsg.z = qwi.z();
        qwimsg.w = qwi.w();
        slamdatamsg.Qwi = qwimsg;

        // gw
        geometry_msgs::Point gw;
        gw.x = slamdata.gw.at<float>(0);
        gw.y = slamdata.gw.at<float>(1);
        gw.z = slamdata.gw.at<float>(2);
        slamdatamsg.gw = gw;

        // Tic(Ric&tic)
        cv::Mat Riw = Rwi.t();

        cv::Mat Rcw = slamdata.Tcw.rowRange(0,3).colRange(0,3).clone();
        cv::Mat tcw = slamdata.Tcw.rowRange(0,3).col(3).clone();
        cv::Mat Rwc = Rcw.t();
        cv::Mat twc = -Rwc*tcw;

        cv::Mat tic = Riw*twc;
        cv::Mat Ric = Riw*Rwc;

        Eigen::Matrix<double,3,3> eigRic = ORB_SLAM2::Converter::toMatrix3d(Ric);
        Eigen::Quaterniond qic(eigRic);

        geometry_msgs::Quaternion qicmsg;
        qicmsg.x = qic.x();
        qicmsg.y = qic.y();
        qicmsg.z = qic.z();
        qicmsg.w = qic.w();

        geometry_msgs::Point ticmsg;
        ticmsg.x = tic.at<float>(0);
        ticmsg.y = tic.at<float>(1);
        ticmsg.z = tic.at<float>(2);

        geometry_msgs::Pose Ticmsg;
        Ticmsg.orientation = qicmsg;
        Ticmsg.position = ticmsg;

        slamdatamsg.Tic = Ticmsg;
    }

    slamdatapub.publish(slamdatamsg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    ORB_SLAM2::ConfigParam config(argv[2]);

    /**
     * @brief added data sync
     */
    double imageMsgDelaySec = config.GetImageDelayToIMU();
    ORBVIO::MsgSynchronizer msgsync(imageMsgDelaySec);
    ros::NodeHandle nh;
#ifdef RUN_REALTIME
    ros::Subscriber imagesub = nh.subscribe(config._imageTopic, /*200*/ 2, &ORBVIO::MsgSynchronizer::imageCallback, &msgsync);
    ros::Subscriber imusub = nh.subscribe(config._imuTopic, 200, &ORBVIO::MsgSynchronizer::imuCallback, &msgsync);
#endif
    sensor_msgs::ImageConstPtr imageMsg;
    std::vector<sensor_msgs::ImuConstPtr> vimuMsg;

    // 3dm imu output per g. 1g=9.80665 according to datasheet
    const double g3dm = 9.80665;
    const bool bAccMultiply98 = config.GetAccMultiply9p8();



#ifndef RUN_REALTIME
    std::string bagfile = config._bagfile;
    rosbag::Bag bag;
    bag.open(bagfile,rosbag::bagmode::Read);

    std::vector<std::string> topics;
    std::string imutopic = config._imuTopic;
    std::string imagetopic = config._imageTopic;
    topics.push_back(imagetopic);
    topics.push_back(imutopic);

    rosbag::View view(bag, rosbag::TopicQuery(topics));
#endif

    ORB_SLAM2::SLAMData slamdata;
    ros::Publisher slamdatapub = nh.advertise<ORB_VIO::viorb_msg>("VIORB/SLAMData", 10);
//    ros::Publisher Rwipub = nh.advertise<geometry_msgs::Quaternion>("VIORB/Rwi",10);
//    ros::Publisher gwpub = nh.advertise<geometry_msgs::Point>("VIORB/gw",10);
//    ros::Publisher VINSInitpub = nh.advertise<std_msgs::Bool>("VIORB/VINSInit",10);
//    ros::Publisher TrackStatuspub = nh.advertise<std_msgs::Int8>("VIORB/TrackStatus",10);

    ros::Rate r(1000);
#ifdef RUN_REALTIME
    ROS_WARN("Run realtime");
    while(ros::ok())
    {
#else
    ROS_WARN("Run non-realtime");
    BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {
        sensor_msgs::ImuConstPtr simu = m.instantiate<sensor_msgs::Imu>();
        if(simu!=NULL)
            msgsync.imuCallback(simu);
        sensor_msgs::ImageConstPtr simage = m.instantiate<sensor_msgs::Image>();
        if(simage!=NULL)
            msgsync.imageCallback(simage);
#endif

        bool bdata = msgsync.getRecentMsgs(imageMsg,vimuMsg);
        if(bdata)
        {
            //if(msgsync.getImageMsgSize()>=2) ROS_ERROR("image queue size: %d",msgsync.getImageMsgSize());
            std::vector<ORB_SLAM2::IMUData> vimuData;
            //ROS_INFO("image time: %.3f",imageMsg->header.stamp.toSec());
            for(unsigned int i=0;i<vimuMsg.size();i++)
            {
                sensor_msgs::ImuConstPtr imuMsg = vimuMsg[i];
                double ax = imuMsg->linear_acceleration.x;
                double ay = imuMsg->linear_acceleration.y;
                double az = imuMsg->linear_acceleration.z;
                if(bAccMultiply98)
                {
                    ax *= g3dm;
                    ay *= g3dm;
                    az *= g3dm;
                }
                ORB_SLAM2::IMUData imudata(imuMsg->angular_velocity.x,imuMsg->angular_velocity.y,imuMsg->angular_velocity.z,
                                ax,ay,az,imuMsg->header.stamp.toSec());
                vimuData.push_back(imudata);
                //ROS_INFO("imu time: %.3f",vimuMsg[i]->header.stamp.toSec());
            }

            // Copy the ros image message to cv::Mat.
            cv_bridge::CvImageConstPtr cv_ptr;
            try
            {
                cv_ptr = cv_bridge::toCvShare(imageMsg);
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return -1;
            }

            // Consider delay of image message
            //SLAM.TrackMonocular(cv_ptr->image, imageMsg->header.stamp.toSec() - imageMsgDelaySec);
            // Below to test relocalizaiton
            cv::Mat im = cv_ptr->image.clone();
            {
                // To test relocalization
                static double startT=-1;
                if(startT<0)
                    startT = imageMsg->header.stamp.toSec();
                //if(imageMsg->header.stamp.toSec() > startT+25 && imageMsg->header.stamp.toSec() < startT+25.3)
                if(imageMsg->header.stamp.toSec() < startT+config._testDiscardTime)
                    im = cv::Mat::zeros(im.rows,im.cols,im.type());
            }
            //***C++11 Style:***
            //std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
            SLAM.TrackMonoVI(im, vimuData, imageMsg->header.stamp.toSec() - imageMsgDelaySec);
            //std::chrono::steady_clock::time_point end= std::chrono::steady_clock::now();
            //ROS_INFO_STREAM( "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() <<std::endl );

            SLAM.GetSLAMData(slamdata);
            PublishSLAMData(slamdatapub,slamdata);

            //SLAM.TrackMonoVI(cv_ptr->image, vimuData, imageMsg->header.stamp.toSec() - imageMsgDelaySec);
            //cv::imshow("image",cv_ptr->image);//
#ifndef RUN_REALTIME
            // Wait local mapping end.
            bool bstop = false;
            
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
            while(!SLAM.bLocalMapAcceptKF())
            {
                if(!ros::ok())
                {
                    bstop=true;
                }
            };
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
            double vlocalbatime2 = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
            ROS_INFO("Waited: %f", vlocalbatime2);
            if(bstop)
                break;
#endif
        }

        //cv::waitKey(1);

        ros::spinOnce();
        r.sleep();
        if(!ros::ok())
            break;
    }



//    ImageGrabber igb(&SLAM);

//    ros::NodeHandle nodeHandler;
//    ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,&igb);

//    ros::spin();

    cout<<endl<<endl<<"press any key to shutdown"<<endl;
    getchar();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("/home/sicong/VIORB_new/ORB_SLAM2/tmp/KeyFrameTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryNavState("/home/sicong/VIORB_new/ORB_SLAM2/tmp/KeyFrameNavStateTrajectory.txt");

    ros::shutdown();

    return 0;
}

//void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
//{
//    // Copy the ros image message to cv::Mat.
//    cv_bridge::CvImageConstPtr cv_ptr;
//    try
//    {
//        cv_ptr = cv_bridge::toCvShare(msg);
//    }
//    catch (cv_bridge::Exception& e)
//    {
//        ROS_ERROR("cv_bridge exception: %s", e.what());
//        return;
//    }

//    mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
//}


