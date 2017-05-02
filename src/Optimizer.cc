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

#include "Optimizer.h"

#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

#include<Eigen/StdVector>

#include "Converter.h"

#include<mutex>

#include "IMU/configparam.h"
#include "IMU/g2otypes.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_gauss_newton.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_with_hessian.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_cholmod.h"

namespace ORB_SLAM2
{

//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------

void Optimizer::GlobalBundleAdjustmentNavState(Map* pMap, const cv::Mat& gw, int nIterations, bool* pbStopFlag, const unsigned long nLoopKF, const bool bRobust)
{
    vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
    vector<MapPoint*> vpMP = pMap->GetAllMapPoints();

    // Extrinsics
    Matrix4d Tbc = ConfigParam::GetEigTbc();
    Matrix3d Rbc = Tbc.topLeftCorner(3,3);
    Vector3d Pbc = Tbc.topRightCorner(3,1);
    // Gravity vector in world frame
    Vector3d GravityVec = Converter::toVector3d(gw);

    vector<bool> vbNotIncludedMP;
    vbNotIncludedMP.resize(vpMP.size());

    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    if(pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    long unsigned int maxKFid = 0;

    // Set KeyFrame vertices
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];
        if(pKF->isBad())
            continue;

        // PVR
        g2o::VertexNavStatePVR * vNSPVR = new g2o::VertexNavStatePVR();
        vNSPVR->setEstimate(pKF->GetNavState());
        vNSPVR->setId(pKF->mnId*2);
        vNSPVR->setFixed(pKF->mnId==0);
        optimizer.addVertex(vNSPVR);
        // Bias
        g2o::VertexNavStateBias * vNSBias = new g2o::VertexNavStateBias();
        vNSBias->setEstimate(pKF->GetNavState());
        vNSBias->setId(pKF->mnId*2+1);
        vNSBias->setFixed(pKF->mnId==0);
        optimizer.addVertex(vNSBias);

        if(pKF->mnId*2+1>maxKFid)
            maxKFid=pKF->mnId*2+1;
    }

    // Add NavState PVR/Bias edges
    const float thHuberNavStatePVR = sqrt(21.666);
    const float thHuberNavStateBias = sqrt(16.812);
    // Inverse covariance of bias random walk
#ifndef NOT_UPDATE_GYRO_BIAS
    Matrix<double,6,6> InvCovBgaRW = Matrix<double,6,6>::Identity();
    InvCovBgaRW.topLeftCorner(3,3) = Matrix3d::Identity()/IMUData::getGyrBiasRW2();       // Gyroscope bias random walk, covariance INVERSE
    InvCovBgaRW.bottomRightCorner(3,3) = Matrix3d::Identity()/IMUData::getAccBiasRW2();   // Accelerometer bias random walk, covariance INVERSE
#else
    Matrix<double,3,3> InvCovBgaRW = Matrix3d::Identity()/IMUData::getAccBiasRW2();   // Accelerometer bias random walk, covariance INVERSE
#endif

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF1 = vpKFs[i];
        if(pKF1->isBad())
        {
            cout<<"pKF is bad in gBA, id "<<pKF1->mnId<<endl;   //Debug log
            continue;
        }

        KeyFrame* pKF0 = pKF1->GetPrevKeyFrame();
        if(!pKF0)
        {
            if(pKF1->mnId!=0) cerr<<"Previous KeyFrame is NULL?"<<endl;  //Test log
            continue;
        }

        // PVR edge
        {
            g2o::EdgeNavStatePVR * epvr = new g2o::EdgeNavStatePVR();
            epvr->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(2*pKF0->mnId)));
            epvr->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(2*pKF1->mnId)));
            epvr->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(2*pKF0->mnId+1)));
            epvr->setMeasurement(pKF1->GetIMUPreInt());

            Matrix9d InvCovPVR = pKF1->GetIMUPreInt().getCovPVPhi().inverse();
            epvr->setInformation(InvCovPVR);
            epvr->SetParams(GravityVec);

            if(bRobust)
            {
                g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                epvr->setRobustKernel(rk);
                rk->setDelta(thHuberNavStatePVR);
            }

            optimizer.addEdge(epvr);
        }
        // Bias edge
        {
            g2o::EdgeNavStateBias * ebias = new g2o::EdgeNavStateBias();
            ebias->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(2*pKF0->mnId+1)));
            ebias->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(2*pKF1->mnId+1)));
            ebias->setMeasurement(pKF1->GetIMUPreInt());

            ebias->setInformation(InvCovBgaRW/pKF1->GetIMUPreInt().getDeltaTime());

            if(bRobust)
            {
                g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                ebias->setRobustKernel(rk);
                rk->setDelta(thHuberNavStateBias);
            }

            optimizer.addEdge(ebias);
        }

    }

    const float thHuber2D = sqrt(5.99);

    // Set MapPoint vertices
    for(size_t i=0; i<vpMP.size(); i++)
    {
        MapPoint* pMP = vpMP[i];
        if(pMP->isBad())
            continue;
        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
        const int id = pMP->mnId+maxKFid+1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);

       const map<KeyFrame*,size_t> observations = pMP->GetObservations();

        int nEdges = 0;
        //SET EDGES
        for(map<KeyFrame*,size_t>::const_iterator mit=observations.begin(); mit!=observations.end(); mit++)
        {

            KeyFrame* pKF = mit->first;
            if(pKF->isBad() || 2*pKF->mnId>maxKFid)
                continue;

            nEdges++;

            const cv::KeyPoint &kpUn = pKF->mvKeysUn[mit->second];

            if(pKF->mvuRight[mit->second]<0)
            {
                Eigen::Matrix<double,2,1> obs;
                obs << kpUn.pt.x, kpUn.pt.y;

                g2o::EdgeNavStatePVRPointXYZ* e = new g2o::EdgeNavStatePVRPointXYZ();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(2*pKF->mnId)));
                e->setMeasurement(obs);
                const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
                e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                if(bRobust)
                {
                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuber2D);
                }

                e->SetParams(pKF->fx,pKF->fy,pKF->cx,pKF->cy,Rbc,Pbc);

                optimizer.addEdge(e);
            }
            else
            {
                cerr<<"Stereo not supported"<<endl;
            }
        }

        if(nEdges==0)
        {
            optimizer.removeVertex(vPoint);
            vbNotIncludedMP[i]=true;
        }
        else
        {
            vbNotIncludedMP[i]=false;
        }
    }

    // Optimize!
    optimizer.initializeOptimization();
    optimizer.optimize(nIterations);

    // Recover optimized data

    //Keyframes
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];
        if(pKF->isBad())
            continue;
        //g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKF->mnId));
        //g2o::SE3Quat SE3quat = vSE3->estimate();
        g2o::VertexNavStatePVR* vNSPVR = static_cast<g2o::VertexNavStatePVR*>(optimizer.vertex(2*pKF->mnId));
        g2o::VertexNavStateBias* vNSBias = static_cast<g2o::VertexNavStateBias*>(optimizer.vertex(2*pKF->mnId+1));
        const NavState& nspvr = vNSPVR->estimate();
        const NavState& nsbias = vNSBias->estimate();
        NavState ns_recov = nspvr;
        ns_recov.Set_DeltaBiasGyr(nsbias.Get_dBias_Gyr());
        ns_recov.Set_DeltaBiasAcc(nsbias.Get_dBias_Acc());

        if(nLoopKF==0)
        {
            //pKF->SetPose(Converter::toCvMat(SE3quat));
            pKF->SetNavState(ns_recov);
            pKF->UpdatePoseFromNS(ConfigParam::GetMatTbc());
        }
        else
        {
            pKF->mNavStateGBA = ns_recov;

            pKF->mTcwGBA.create(4,4,CV_32F);
            //Converter::toCvMat(SE3quat).copyTo(pKF->mTcwGBA);
            cv::Mat Twb_ = cv::Mat::eye(4,4,CV_32F);
            Converter::toCvMat(pKF->mNavStateGBA.Get_RotMatrix()).copyTo(Twb_.rowRange(0,3).colRange(0,3));
            Converter::toCvMat(pKF->mNavStateGBA.Get_P()).copyTo(Twb_.rowRange(0,3).col(3));
            cv::Mat Twc_ = Twb_*ConfigParam::GetMatTbc();
            pKF->mTcwGBA = Converter::toCvMatInverse(Twc_);

            pKF->mnBAGlobalForKF = nLoopKF;
        }

//        //Debug log
//        cv::Mat tTwb1 = pKF->GetPoseInverse()*ConfigParam::GetMatT_cb();
//        if((Converter::toVector3d(tTwb1.rowRange(0,3).col(3))-pKF->GetNavState().Get_P()).norm()>1e-6)
//            cout<<"in gBA, Twc*Tcb != NavState for GBA KFs, id "<<pKF->mnId<<": "<<tTwb1.rowRange(0,3).col(3).t()<<"/"<<pKF->GetNavState().Get_P().transpose()<<endl;

    }

    //Points
    for(size_t i=0; i<vpMP.size(); i++)
    {
        if(vbNotIncludedMP[i])
            continue;

        MapPoint* pMP = vpMP[i];

        if(pMP->isBad())
            continue;
        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mnId+maxKFid+1));

        if(nLoopKF==0)
        {
            pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
            pMP->UpdateNormalAndDepth();
        }
        else
        {
            pMP->mPosGBA.create(3,1,CV_32F);
            Converter::toCvMat(vPoint->estimate()).copyTo(pMP->mPosGBA);
            pMP->mnBAGlobalForKF = nLoopKF;
        }
    }

}


int Optimizer::PoseOptimization(Frame *pFrame, Frame* pLastFrame, const IMUPreintegrator& imupreint, const cv::Mat& gw, const bool& bComputeMarg)
{
#ifndef NOT_UPDATE_GYRO_BIAS
    static bool dbg1fopen=false;
    static ofstream dbg1fPoseOptPVRErr,dbg1fPoseOptBiasErr,dbg1fPoseOptPriorErr;
    //if(dbgfop)
    if(!dbg1fopen)
    {cerr<<"try open 1"<<endl;
        dbg1fPoseOptPVRErr.open("/home/jp/opensourcecode/ORB_SLAM2/tmp/dbg1fPoseOptPVRErr.txt");
        dbg1fPoseOptBiasErr.open("/home/jp/opensourcecode/ORB_SLAM2/tmp/dbg1fPoseOptBiasErr.txt");
        dbg1fPoseOptPriorErr.open("/home/jp/opensourcecode/ORB_SLAM2/tmp/dbg1fPoseOptPriorErr.txt");
        if(dbg1fPoseOptPVRErr.is_open() && dbg1fPoseOptBiasErr.is_open() && dbg1fPoseOptPriorErr.is_open() )
        {
            cerr<<"file opened1"<<endl;
            dbg1fopen = true;
        }
        else
        {
            cerr<<"file open error in dbg1fPoseOpt"<<endl;
            dbg1fopen = false;
        }
        dbg1fPoseOptPVRErr<<std::fixed<<std::setprecision(6);
        dbg1fPoseOptBiasErr<<std::fixed<<std::setprecision(10);
        dbg1fPoseOptPriorErr<<std::fixed<<std::setprecision(6);
    }
#endif
    // Extrinsics
    Matrix4d Tbc = ConfigParam::GetEigTbc();
    Matrix3d Rbc = Tbc.topLeftCorner(3,3);
    Vector3d Pbc = Tbc.topRightCorner(3,1);
    // Gravity vector in world frame
    Vector3d GravityVec = Converter::toVector3d(gw);

    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType * linearSolver;

    //linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();
    linearSolver = new g2o::LinearSolverCholmod<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    //g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
    optimizer.setAlgorithm(solver);

    int nInitialCorrespondences=0;

    const int FramePVRId = 0;
    const int FrameBiasId = 1;
    const int LastFramePVRId = 2;
    const int LastFrameBiasId = 3;

    // Set Frame vertex PVR/Bias
    g2o::VertexNavStatePVR * vNSFPVR = new g2o::VertexNavStatePVR();
    {
        vNSFPVR->setEstimate(pFrame->GetNavState());
        vNSFPVR->setId(FramePVRId);
        vNSFPVR->setFixed(false);
        optimizer.addVertex(vNSFPVR);
    }
    g2o::VertexNavStateBias * vNSFBias = new g2o::VertexNavStateBias();
    {
        vNSFBias->setEstimate(pFrame->GetNavState());
        vNSFBias->setId(FrameBiasId);
        vNSFBias->setFixed(false);
        optimizer.addVertex(vNSFBias);
    }

    // Set LastFrame vertex
    g2o::VertexNavStatePVR * vNSFPVRlast = new g2o::VertexNavStatePVR();
    {
        vNSFPVRlast->setEstimate(pLastFrame->GetNavState());
        vNSFPVRlast->setId(LastFramePVRId);
        vNSFPVRlast->setFixed(false);
        optimizer.addVertex(vNSFPVRlast);
    }
    g2o::VertexNavStateBias * vNSFBiaslast = new g2o::VertexNavStateBias();
    {
        vNSFBiaslast->setEstimate(pLastFrame->GetNavState());
        vNSFBiaslast->setId(LastFrameBiasId);
        vNSFBiaslast->setFixed(false);
        optimizer.addVertex(vNSFBiaslast);
    }

    // Set prior edge for Last Frame, from mMargCovInv
    g2o::EdgeNavStatePriorPVRBias* eNSPrior = new g2o::EdgeNavStatePriorPVRBias();
    {
        eNSPrior->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(LastFramePVRId)));
        eNSPrior->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(LastFrameBiasId)));
        eNSPrior->setMeasurement(pLastFrame->mNavStatePrior);

        //eNSPrior->setInformation(pLastFrame->mMargCovInv);
#ifdef NOT_UPDATE_GYRO_BIAS
        Matrix<double,12,12> addcov = Matrix<double,12,12>::Identity();
        addcov.block<3,3>(0,0) *= 1e2;
        addcov.block<3,3>(3,3) *= 1;
        addcov.block<3,3>(6,6) *= 1e2;
        addcov.block<3,3>(9,9) *= 0;
        eNSPrior->setInformation(pLastFrame->mMargCovInv + addcov);
#else
        eNSPrior->setInformation(pLastFrame->mMargCovInv + Matrix<double,15,15>::Identity()*1e2);
#endif

        const float thHuberNavState = sqrt(30.5779);
        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
        eNSPrior->setRobustKernel(rk);
        rk->setDelta(thHuberNavState);

        optimizer.addEdge(eNSPrior);

//        cout<<"information for prior edge:"<<endl<<eNSPrior->information()<<endl;
//        cout<<"prior ns PV: "<<eNSPrior->measurement().Get_P().transpose()<<" , "<<eNSPrior->measurement().Get_V().transpose()<<endl;
//        cout<<"prior ns bg/dbg: "<<eNSPrior->measurement().Get_BiasGyr().transpose()<<" , "<<eNSPrior->measurement().Get_dBias_Gyr().transpose()<<endl;
//        cout<<"prior ns ba/dba: "<<eNSPrior->measurement().Get_BiasAcc().transpose()<<" , "<<eNSPrior->measurement().Get_dBias_Acc().transpose()<<endl;
    }

    // Set PVR edge between LastFrame-Frame
    g2o::EdgeNavStatePVR* eNSPVR = new g2o::EdgeNavStatePVR();
    {
        eNSPVR->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(LastFramePVRId)));
        eNSPVR->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(FramePVRId)));
        eNSPVR->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(LastFrameBiasId)));
        eNSPVR->setMeasurement(imupreint);

        Matrix9d InvCovPVR = imupreint.getCovPVPhi().inverse() ;
        //eNSPVR->setInformation(InvCovPVR);
        Matrix9d addcov = Matrix9d::Identity();
        addcov.block<3,3>(0,0) *= 1e2;
        addcov.block<3,3>(3,3) *= 1;
        addcov.block<3,3>(6,6) *= 1e2;
        eNSPVR->setInformation(InvCovPVR + addcov);

        eNSPVR->SetParams(GravityVec);

        const float thHuberNavStatePVR = sqrt(21.666);
        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
        eNSPVR->setRobustKernel(rk);
        rk->setDelta(thHuberNavStatePVR);

        optimizer.addEdge(eNSPVR);
    }
    // Set Bias edge between LastFrame-Frame
    g2o::EdgeNavStateBias* eNSBias = new g2o::EdgeNavStateBias();
    {
        eNSBias->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(LastFrameBiasId)));
        eNSBias->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(FrameBiasId)));
        eNSBias->setMeasurement(imupreint);
#ifndef NOT_UPDATE_GYRO_BIAS
        Matrix<double,6,6> InvCovBgaRW = Matrix<double,6,6>::Identity();
        InvCovBgaRW.topLeftCorner(3,3) = Matrix3d::Identity()/IMUData::getGyrBiasRW2();       // Gyroscope bias random walk, covariance INVERSE
        InvCovBgaRW.bottomRightCorner(3,3) = Matrix3d::Identity()/IMUData::getAccBiasRW2();   // Accelerometer bias random walk, covariance INVERSE
#else
        Matrix<double,3,3> InvCovBgaRW = Matrix3d::Identity()/IMUData::getAccBiasRW2();// Accelerometer bias random walk, covariance INVERSE
#endif
        eNSBias->setInformation(InvCovBgaRW/imupreint.getDeltaTime());

        const float thHuberNavStateBias = sqrt(16.812);
        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
        eNSBias->setRobustKernel(rk);
        rk->setDelta(thHuberNavStateBias);

        optimizer.addEdge(eNSBias);
    }

    // Set MapPoint vertices
    const int Ncur = pFrame->N;
    const int Nlast = pLastFrame->N;

    vector<g2o::EdgeNavStatePVRPointXYZOnlyPose*> vpEdgesMono;
    vector<size_t> vnIndexEdgeMono;
    vpEdgesMono.reserve(Ncur);
    vnIndexEdgeMono.reserve(Ncur);

    vector<g2o::EdgeNavStatePVRPointXYZOnlyPose*> vpEdgesMonoLast;
    vector<size_t> vnIndexEdgeMonoLast;
    vpEdgesMonoLast.reserve(Nlast);
    vnIndexEdgeMonoLast.reserve(Nlast);


    const float deltaMono = sqrt(5.991);

    {
    unique_lock<mutex> lock(MapPoint::mGlobalMutex);

    for(int i=0; i<Ncur; i++)
    {
        MapPoint* pMP = pFrame->mvpMapPoints[i];
        if(pMP)
        {
            // Monocular observation
            if(pFrame->mvuRight[i]<0)
            {
                nInitialCorrespondences++;
                pFrame->mvbOutlier[i] = false;

                Eigen::Matrix<double,2,1> obs;
                const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
                obs << kpUn.pt.x, kpUn.pt.y;

                //g2o::EdgeSE3ProjectXYZOnlyPose* e = new g2o::EdgeSE3ProjectXYZOnlyPose();
                g2o::EdgeNavStatePVRPointXYZOnlyPose* e = new g2o::EdgeNavStatePVRPointXYZOnlyPose();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(FramePVRId)));
                e->setMeasurement(obs);
                const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
                e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                e->setRobustKernel(rk);
                rk->setDelta(deltaMono);

                e->SetParams(pFrame->fx,pFrame->fy,pFrame->cx,pFrame->cy,Rbc,Pbc,Converter::toVector3d(pMP->GetWorldPos()));

                optimizer.addEdge(e);

                vpEdgesMono.push_back(e);
                vnIndexEdgeMono.push_back(i);
            }
            else  // Stereo observation
            {
                cerr<<"stereo shouldn't in poseoptimization"<<endl;
            }
        }

    }

    // Add Point-Pose edges for last frame
    for(int i=0; i<Nlast; i++)
    {
        MapPoint* pMP = pLastFrame->mvpMapPoints[i];
        if(pMP)
        {
            // Monocular observation
            if(pLastFrame->mvuRight[i]<0)
            {
                //nInitialCorrespondences++;
                pLastFrame->mvbOutlier[i] = false;

                Eigen::Matrix<double,2,1> obs;
                const cv::KeyPoint &kpUn = pLastFrame->mvKeysUn[i];
                obs << kpUn.pt.x, kpUn.pt.y;

                //g2o::EdgeSE3ProjectXYZOnlyPose* e = new g2o::EdgeSE3ProjectXYZOnlyPose();
                g2o::EdgeNavStatePVRPointXYZOnlyPose* e = new g2o::EdgeNavStatePVRPointXYZOnlyPose();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(LastFramePVRId)));
                e->setMeasurement(obs);
                const float invSigma2 = pLastFrame->mvInvLevelSigma2[kpUn.octave];
                e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                e->setRobustKernel(rk);
                rk->setDelta(deltaMono);

                e->SetParams(pLastFrame->fx,pLastFrame->fy,pLastFrame->cx,pLastFrame->cy,Rbc,Pbc,Converter::toVector3d(pMP->GetWorldPos()));

                optimizer.addEdge(e);

                vpEdgesMonoLast.push_back(e);
                vnIndexEdgeMonoLast.push_back(i);
            }
            else  // Stereo observation
            {
                cerr<<"stereo shouldn't in poseoptimization"<<endl;
            }
        }
    }
    }


    if(nInitialCorrespondences<3)
        return 0;

    // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
    // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
    const float chi2Mono[4]={5.991,5.991,5.991,5.991};
    //const float chi2Stereo[4]={7.815,7.815,7.815, 7.815};
    const int its[4]={10,10,10,10};

//    //Debug log
//    cout<<"total Points: "<<vpEdgesMono.size()<<endl;

    int nBad=0;
    int nBadLast=0;
    for(size_t it=0; it<4; it++)
    {
//if(!pFrame || !pLastFrame) cerr<<"!pFrame || !pLastFrame"<<endl;
//if(!vNSFPVR || !vNSFBias || !vNSFPVRlast || !vNSFBiaslast)  cerr<<"vertex pointer null?"<<endl;
//if(!eNSPrior) cerr<<"!eNSPrior"<<endl;

        // Reset estimates
        vNSFPVR->setEstimate(pFrame->GetNavState());
        vNSFBias->setEstimate(pFrame->GetNavState());
        vNSFPVRlast->setEstimate(pLastFrame->GetNavState());
        vNSFBiaslast->setEstimate(pLastFrame->GetNavState());
//cerr<<"before opt, ";
        //optimizer.setVerbose(true);
        optimizer.initializeOptimization(0);
        optimizer.optimize(its[it]);

//cerr<<"after opt, iter: "<<it;
        nBad=0;
        for(size_t i=0, iend=vpEdgesMono.size(); i<iend; i++)
        {
            g2o::EdgeNavStatePVRPointXYZOnlyPose* e = vpEdgesMono[i];

            const size_t idx = vnIndexEdgeMono[i];

            if(pFrame->mvbOutlier[idx])
            {
                e->computeError();
            }

            const float chi2 = e->chi2();

            if(chi2>chi2Mono[it])
            {
                pFrame->mvbOutlier[idx]=true;
                e->setLevel(1);
                nBad++;
            }
            else
            {
                pFrame->mvbOutlier[idx]=false;
                e->setLevel(0);
            }

            if(it==2)
                e->setRobustKernel(0);
        }

        nBadLast=0;
        for(size_t i=0, iend=vpEdgesMonoLast.size(); i<iend; i++)
        {
            g2o::EdgeNavStatePVRPointXYZOnlyPose* e = vpEdgesMonoLast[i];

            const size_t idx = vnIndexEdgeMonoLast[i];

            if(pLastFrame->mvbOutlier[idx])
            {
                e->computeError();
            }

            const float chi2 = e->chi2();

            if(chi2>chi2Mono[it])
            {
                pLastFrame->mvbOutlier[idx]=true;
                e->setLevel(1);
                nBadLast++;
            }
            else
            {
                pLastFrame->mvbOutlier[idx]=false;
                e->setLevel(0);
            }

            if(it==2)
                e->setRobustKernel(0);
        }

        //        //Debug log
        //        cout<<nBad<<" bad Points in iter "<<it<<", rest points: "<<optimizer.edges().size()<<endl;
        //        cout<<nBadLast<<" bad Points of last Frame in iter "<<it<<endl;
        //        cout<<"NavState edge chi2: "<<eNS->chi2()<<endl;

        //if(vpEdgesMono.size() - nBad < 10)
        if(optimizer.edges().size()<10)
            break;
    }

    // Debug log
    if(eNSPVR->chi2()>21.666) cout<<"F-F PVR edge chi2:"<<eNSPVR->chi2()<<endl;
    if(eNSBias->chi2()>16.812) cout<<"F-F Bias edge chi2:"<<eNSBias->chi2()<<endl;
    if(eNSPrior->chi2()>30.5779) cout<<"F-F Prior edge chi2:"<<eNSPrior->chi2()<<endl;
#ifndef NOT_UPDATE_GYRO_BIAS
    {
        eNSPVR->computeError();
        Vector9d errPVR=eNSPVR->error();
        for(int n=0;n<9;n++)
            dbg1fPoseOptPVRErr<<errPVR[n]<<" ";
        dbg1fPoseOptPVRErr<<std::endl;

        eNSBias->computeError();
        Vector6d errBias=eNSBias->error();
        for(int n=0;n<6;n++)
            dbg1fPoseOptBiasErr<<errBias[n]<<" ";
        dbg1fPoseOptBiasErr<<std::endl;

        eNSPrior->computeError();
        Vector15d errPrior=eNSPrior->error();
        for(int n=0;n<15;n++)
            dbg1fPoseOptPriorErr<<errPrior[n]<<" ";
        dbg1fPoseOptPriorErr<<std::endl;
    }
#endif
    // Recover optimized pose and return number of inliers
    g2o::VertexNavStatePVR* vNSPVR_recov = static_cast<g2o::VertexNavStatePVR*>(optimizer.vertex(FramePVRId));
    const NavState& nsPVR_recov = vNSPVR_recov->estimate();
    g2o::VertexNavStateBias* vNSBias_recov = static_cast<g2o::VertexNavStateBias*>(optimizer.vertex(FrameBiasId));
    const NavState& nsBias_recov = vNSBias_recov->estimate();
    NavState ns_recov = nsPVR_recov;
    ns_recov.Set_DeltaBiasGyr(nsBias_recov.Get_dBias_Gyr());
    ns_recov.Set_DeltaBiasAcc(nsBias_recov.Get_dBias_Acc());
    pFrame->SetNavState(ns_recov);
    pFrame->UpdatePoseFromNS(ConfigParam::GetMatTbc());

    //    g2o::VertexNavStatePVR* vNSPVRlast_recov = static_cast<g2o::VertexNavStatePVR*>(optimizer.vertex(LastFramePVRId));
    //    NavState nsPVRlast_recov = vNSPVRlast_recov->estimate();
    //    g2o::VertexNavStateBias* vNSBiaslast_recov = static_cast<g2o::VertexNavStateBias*>(optimizer.vertex(LastFrameBiasId));
    //    NavState nsBiaslast_recov = vNSBiaslast_recov->estimate();
    //    NavState nslast_recov = nsPVRlast_recov;
    //    nslast_recov.Set_DeltaBiasGyr(nsBiaslast_recov.Get_BiasGyr());
    //    nslast_recov.Set_DeltaBiasAcc(nsBiaslast_recov.Get_BiasAcc());
    //    pLastFrame->SetNavState(nslast_recov);
    //    pLastFrame->UpdatePoseFromNS(ConfigParam::GetMatTbc());

    // Compute marginalized Hessian H and B, H*x=B, H/B can be used as prior for next optimization in PoseOptimization
    if(bComputeMarg)
    {
        std::vector<g2o::OptimizableGraph::Vertex*> margVerteces;
        margVerteces.push_back(optimizer.vertex(FramePVRId));
        margVerteces.push_back(optimizer.vertex(FrameBiasId));
//
        //TODO: how to get the joint marginalized covariance of PVR&Bias
        g2o::SparseBlockMatrixXd spinv;
        optimizer.computeMarginals(spinv, margVerteces);
        //cout<<"marg result 2: "<<endl<<spinv<<endl;
        // spinv include 2 blocks, 9x9-(0,0) for PVR, 6x6-(1,1) for Bias
#ifndef NOT_UPDATE_GYRO_BIAS
        Matrix<double,15,15> margCov = Matrix<double,15,15>::Zero();
        margCov.topLeftCorner(9,9) = spinv.block(0,0)->eval();
        margCov.topRightCorner(9,6) = spinv.block(0,1)->eval();
        margCov.bottomLeftCorner(6,9) = spinv.block(1,0)->eval();
        margCov.bottomRightCorner(6,6) = spinv.block(1,1)->eval();
#else
        Matrix<double,12,12> margCov = Matrix<double,12,12>::Zero();
        margCov.topLeftCorner(9,9) = spinv.block(0,0)->eval();
        margCov.topRightCorner(9,3) = spinv.block(0,1)->eval();
        margCov.bottomLeftCorner(3,9) = spinv.block(1,0)->eval();
        margCov.bottomRightCorner(3,3) = spinv.block(1,1)->eval();
#endif
        //        margCovInv.topLeftCorner(9,9) = spinv.block(0,0)->inverse();
        //        margCovInv.bottomRightCorner(6,6) = spinv.block(1,1)->inverse();
        pFrame->mMargCovInv = margCov.inverse();
        pFrame->mNavStatePrior = ns_recov;

        //Debug log
        //cout<<"inv MargCov 2: "<<endl<<pFrame->mMargCovInv<<endl;
    }

    //Test log
    if( (nsPVR_recov.Get_BiasGyr()-nsBias_recov.Get_BiasGyr()).norm()>1e-6 || (nsPVR_recov.Get_BiasAcc()-nsBias_recov.Get_BiasAcc()).norm()>1e-6 )
    {
        std::cerr<<"1 recovered bias gyr not equal for PVR/Bias vertex"<<std::endl<<nsPVR_recov.Get_BiasGyr().transpose()<<" / "<<nsBias_recov.Get_BiasGyr().transpose()<<std::endl;
        std::cerr<<"1 recovered bias acc not equal for PVR/Bias vertex"<<std::endl<<nsPVR_recov.Get_BiasAcc().transpose()<<" / "<<nsBias_recov.Get_BiasAcc().transpose()<<std::endl;
    }
    if( (ns_recov.Get_dBias_Gyr()-nsBias_recov.Get_dBias_Gyr()).norm()>1e-6 || (ns_recov.Get_dBias_Acc()-nsBias_recov.Get_dBias_Acc()).norm()>1e-6 )
    {
        std::cerr<<"1 recovered delta bias gyr not equal to Bias vertex"<<std::endl<<ns_recov.Get_dBias_Gyr().transpose()<<" / "<<nsBias_recov.Get_dBias_Gyr().transpose()<<std::endl;
        std::cerr<<"1 recovered delta bias acc not equal to Bias vertex"<<std::endl<<ns_recov.Get_dBias_Acc().transpose()<<" / "<<nsBias_recov.Get_dBias_Acc().transpose()<<std::endl;
    }

    return nInitialCorrespondences-nBad;
}

int Optimizer::PoseOptimization(Frame *pFrame, KeyFrame* pLastKF, const IMUPreintegrator& imupreint, const cv::Mat& gw, const bool& bComputeMarg)
{
#ifndef NOT_UPDATE_GYRO_BIAS
    static bool dbg2fopen=false;
    static ofstream dbg2fPoseOptPVRErr,dbg2fPoseOptBiasErr;
    if(!dbg2fopen)
    {
        dbg2fPoseOptPVRErr.open("/home/jp/opensourcecode/ORB_SLAM2/tmp/dbg2fPoseOptPVRErr.txt");
        dbg2fPoseOptBiasErr.open("/home/jp/opensourcecode/ORB_SLAM2/tmp/dbg2fPoseOptBiasErr.txt");
        if(dbg2fPoseOptPVRErr.is_open() && dbg2fPoseOptBiasErr.is_open())
        {
            cerr<<"file opened2"<<endl;
            dbg2fopen = true;
        }
        else
        {
            cerr<<"file open error in dbg2fPoseOpt"<<endl;
            dbg2fopen = false;
        }
        dbg2fPoseOptPVRErr<<std::fixed<<std::setprecision(6);
        dbg2fPoseOptBiasErr<<std::fixed<<std::setprecision(10);
    }
#endif

    // Extrinsics
    Matrix4d Tbc = ConfigParam::GetEigTbc();
    Matrix3d Rbc = Tbc.topLeftCorner(3,3);
    Vector3d Pbc = Tbc.topRightCorner(3,1);
    // Gravity vector in world frame
    Vector3d GravityVec = Converter::toVector3d(gw);

    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType * linearSolver;

    //linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();
    linearSolver = new g2o::LinearSolverCholmod<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    //g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
    optimizer.setAlgorithm(solver);

    int nInitialCorrespondences=0;

    const int FramePVRId = 0;
    const int FrameBiasId = 1;
    const int LastKFPVRId = 2;
    const int LastKFBiasId = 3;

    // Set Frame vertex PVR/Bias
    g2o::VertexNavStatePVR * vNSFPVR = new g2o::VertexNavStatePVR();
    {
        vNSFPVR->setEstimate(pFrame->GetNavState());
        vNSFPVR->setId(FramePVRId);
        vNSFPVR->setFixed(false);
        optimizer.addVertex(vNSFPVR);
    }
    g2o::VertexNavStateBias * vNSFBias = new g2o::VertexNavStateBias();
    {
        vNSFBias->setEstimate(pFrame->GetNavState());
        vNSFBias->setId(FrameBiasId);
        vNSFBias->setFixed(false);
        optimizer.addVertex(vNSFBias);
    }

    // Set KeyFrame vertex PVR/Bias
    g2o::VertexNavStatePVR * vNSKFPVR = new g2o::VertexNavStatePVR();
    {
        vNSKFPVR->setEstimate(pLastKF->GetNavState());
        vNSKFPVR->setId(LastKFPVRId);
        vNSKFPVR->setFixed(true);
        optimizer.addVertex(vNSKFPVR);
    }
    g2o::VertexNavStateBias * vNSKFBias = new g2o::VertexNavStateBias();
    {
        vNSKFBias->setEstimate(pLastKF->GetNavState());
        vNSKFBias->setId(LastKFBiasId);
        vNSKFBias->setFixed(true);
        optimizer.addVertex(vNSKFBias);
    }

    // Set PVR edge between LastKF-Frame
    g2o::EdgeNavStatePVR* eNSPVR = new g2o::EdgeNavStatePVR();
    {
        eNSPVR->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(LastKFPVRId)));
        eNSPVR->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(FramePVRId)));
        eNSPVR->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(LastKFBiasId)));
        eNSPVR->setMeasurement(imupreint);

        Matrix9d InvCovPVR = imupreint.getCovPVPhi().inverse() ;
        //eNSPVR->setInformation(InvCovPVR);
        Matrix9d addcov = Matrix9d::Identity();
        addcov.block<3,3>(0,0) *= 1e2;
        addcov.block<3,3>(3,3) *= 1;
        addcov.block<3,3>(6,6) *= 1e2;
        eNSPVR->setInformation(InvCovPVR + addcov);

        eNSPVR->SetParams(GravityVec);

        const float thHuberNavStatePVR = sqrt(21.666);
        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
        eNSPVR->setRobustKernel(rk);
        rk->setDelta(thHuberNavStatePVR);

        optimizer.addEdge(eNSPVR);
    }
    // Set Bias edge between LastKF-Frame
    g2o::EdgeNavStateBias* eNSBias = new g2o::EdgeNavStateBias();
    {
        eNSBias->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(LastKFBiasId)));
        eNSBias->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(FrameBiasId)));
        eNSBias->setMeasurement(imupreint);

#ifndef NOT_UPDATE_GYRO_BIAS
        Matrix<double,6,6> InvCovBgaRW = Matrix<double,6,6>::Identity();
        InvCovBgaRW.topLeftCorner(3,3) = Matrix3d::Identity()/IMUData::getGyrBiasRW2();       // Gyroscope bias random walk, covariance INVERSE
        InvCovBgaRW.bottomRightCorner(3,3) = Matrix3d::Identity()/IMUData::getAccBiasRW2();   // Accelerometer bias random walk, covariance INVERSE
#else
        Matrix<double,3,3> InvCovBgaRW = Matrix3d::Identity()/IMUData::getAccBiasRW2();   // Accelerometer bias random walk, covariance INVERSE
#endif
        eNSBias->setInformation(InvCovBgaRW/imupreint.getDeltaTime());

        const float thHuberNavStateBias = sqrt(16.812);
        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
        eNSBias->setRobustKernel(rk);
        rk->setDelta(thHuberNavStateBias);

        optimizer.addEdge(eNSBias);
    }

    // Set MapPoint vertices
    const int N = pFrame->N;

    vector<g2o::EdgeNavStatePVRPointXYZOnlyPose*> vpEdgesMono;
    vector<size_t> vnIndexEdgeMono;
    vpEdgesMono.reserve(N);
    vnIndexEdgeMono.reserve(N);

    const float deltaMono = sqrt(5.991);

    {
    unique_lock<mutex> lock(MapPoint::mGlobalMutex);

    for(int i=0; i<N; i++)
    {
        MapPoint* pMP = pFrame->mvpMapPoints[i];
        if(pMP)
        {
            // Monocular observation
            if(pFrame->mvuRight[i]<0)
            {
                nInitialCorrespondences++;
                pFrame->mvbOutlier[i] = false;

                Eigen::Matrix<double,2,1> obs;
                const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
                obs << kpUn.pt.x, kpUn.pt.y;

                //g2o::EdgeSE3ProjectXYZOnlyPose* e = new g2o::EdgeSE3ProjectXYZOnlyPose();
                g2o::EdgeNavStatePVRPointXYZOnlyPose* e = new g2o::EdgeNavStatePVRPointXYZOnlyPose();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(FramePVRId)));
                e->setMeasurement(obs);
                const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
                e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                e->setRobustKernel(rk);
                rk->setDelta(deltaMono);

                e->SetParams(pFrame->fx,pFrame->fy,pFrame->cx,pFrame->cy,Rbc,Pbc,Converter::toVector3d(pMP->GetWorldPos()));

                optimizer.addEdge(e);

                vpEdgesMono.push_back(e);
                vnIndexEdgeMono.push_back(i);
            }
            else  // Stereo observation
            {
                cerr<<"stereo shouldn't in poseoptimization"<<endl;
            }
        }

    }
    }
    //cout<<endl;


    if(nInitialCorrespondences<3)
        return 0;

    // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
    // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
    const float chi2Mono[4]={5.991,5.991,5.991,5.991};
    //const float chi2Stereo[4]={7.815,7.815,7.815, 7.815};
    const int its[4]={10,10,10,10};

//    //Debug log
//    cout<<"total Points: "<<vpEdgesMono.size()<<endl;

    int nBad=0;
    for(size_t it=0; it<4; it++)
    {
        // Reset estimate for vertex
        vNSFPVR->setEstimate(pFrame->GetNavState());
        vNSFBias->setEstimate(pFrame->GetNavState());

        optimizer.initializeOptimization(0);
        optimizer.optimize(its[it]);

        nBad=0;
        for(size_t i=0, iend=vpEdgesMono.size(); i<iend; i++)
        {
            g2o::EdgeNavStatePVRPointXYZOnlyPose* e = vpEdgesMono[i];

            const size_t idx = vnIndexEdgeMono[i];

            if(pFrame->mvbOutlier[idx])
            {
                e->computeError();
            }

            const float chi2 = e->chi2();

            if(chi2>chi2Mono[it])
            {
                pFrame->mvbOutlier[idx]=true;
                e->setLevel(1);
                nBad++;
            }
            else
            {
                pFrame->mvbOutlier[idx]=false;
                e->setLevel(0);
            }

            if(it==2)
                e->setRobustKernel(0);
        }

        //        //Debug log
        //        cout<<nBad<<" bad Points in iter "<<it<<", rest points: "<<optimizer.edges().size()<<endl;

        if(optimizer.edges().size()<10)
            break;
    }

    // Debug log
    if(eNSPVR->chi2()>21.666) cout<<"KF-F PVR edge chi2:"<<eNSPVR->chi2()<<endl;
    if(eNSBias->chi2()>16.812) cout<<"KF-F Bias edge chi2:"<<eNSBias->chi2()<<endl;

#ifndef NOT_UPDATE_GYRO_BIAS
    {
        eNSPVR->computeError();
        Vector9d errPVR=eNSPVR->error();
        for(int n=0;n<9;n++)
            dbg2fPoseOptPVRErr<<errPVR[n]<<" ";
        dbg2fPoseOptPVRErr<<std::endl;

        eNSBias->computeError();
        Vector6d errBias=eNSBias->error();
        for(int n=0;n<6;n++)
            dbg2fPoseOptBiasErr<<errBias[n]<<" ";
        dbg2fPoseOptBiasErr<<std::endl;
    }
#endif

    // Recover optimized pose and return number of inliers
    g2o::VertexNavStatePVR* vNSPVR_recov = static_cast<g2o::VertexNavStatePVR*>(optimizer.vertex(FramePVRId));
    const NavState& nsPVR_recov = vNSPVR_recov->estimate();
    g2o::VertexNavStateBias* vNSBias_recov = static_cast<g2o::VertexNavStateBias*>(optimizer.vertex(FrameBiasId));
    const NavState& nsBias_recov = vNSBias_recov->estimate();
    NavState ns_recov = nsPVR_recov;
    ns_recov.Set_DeltaBiasGyr(nsBias_recov.Get_dBias_Gyr());
    ns_recov.Set_DeltaBiasAcc(nsBias_recov.Get_dBias_Acc());
    pFrame->SetNavState(ns_recov);
    pFrame->UpdatePoseFromNS(ConfigParam::GetMatTbc());

    // Compute marginalized Hessian H and B, H*x=B, H/B can be used as prior for next optimization in PoseOptimization
    if(bComputeMarg)
    {
        std::vector<g2o::OptimizableGraph::Vertex*> margVerteces;
        margVerteces.push_back(optimizer.vertex(FramePVRId));
        margVerteces.push_back(optimizer.vertex(FrameBiasId));

        //TODO: how to get the joint marginalized covariance of PVR&Bias
        g2o::SparseBlockMatrixXd spinv;
        optimizer.computeMarginals(spinv, margVerteces);
        // spinv include 2 blocks, 9x9-(0,0) for PVR, 6x6-(1,1) for Bias
#ifndef NOT_UPDATE_GYRO_BIAS
        Matrix<double,15,15> margCovInv = Matrix<double,15,15>::Zero();
        margCovInv.topLeftCorner(9,9) = spinv.block(0,0)->inverse();
        margCovInv.bottomRightCorner(6,6) = spinv.block(1,1)->inverse();
#else
        Matrix<double,12,12> margCovInv = Matrix<double,12,12>::Zero();
        margCovInv.topLeftCorner(9,9) = spinv.block(0,0)->inverse();
        margCovInv.bottomRightCorner(3,3) = spinv.block(1,1)->inverse();
#endif
        pFrame->mMargCovInv = margCovInv;
        pFrame->mNavStatePrior = ns_recov;

//        //Debug log
//        cout<<"marg result 1: "<<endl<<spinv<<endl;
//        cout<<"margCovInv: "<<endl<<pFrame->mMargCovInv<<endl;
//        cout<<"marg ns PV: "<<pFrame->mNavStatePrior.Get_P().transpose()<<" , "<<pFrame->mNavStatePrior.Get_V().transpose()<<endl;
//        cout<<"marg ns bg/dbg: "<<pFrame->mNavStatePrior.Get_BiasGyr().transpose()<<" , "<<pFrame->mNavStatePrior.Get_dBias_Gyr().transpose()<<endl;
//        cout<<"marg ns ba/dba: "<<pFrame->mNavStatePrior.Get_BiasAcc().transpose()<<" , "<<pFrame->mNavStatePrior.Get_dBias_Acc().transpose()<<endl;
    }

    //Test log
    if( (nsPVR_recov.Get_BiasGyr()-nsBias_recov.Get_BiasGyr()).norm()>1e-6 || (nsPVR_recov.Get_BiasAcc()-nsBias_recov.Get_BiasAcc()).norm()>1e-6 )
    {
        std::cerr<<"recovered bias gyr not equal for PVR/Bias vertex"<<std::endl<<nsPVR_recov.Get_BiasGyr().transpose()<<" / "<<nsBias_recov.Get_BiasGyr().transpose()<<std::endl;
        std::cerr<<"recovered bias acc not equal for PVR/Bias vertex"<<std::endl<<nsPVR_recov.Get_BiasAcc().transpose()<<" / "<<nsBias_recov.Get_BiasAcc().transpose()<<std::endl;
    }
    if( (ns_recov.Get_dBias_Gyr()-nsBias_recov.Get_dBias_Gyr()).norm()>1e-6 || (ns_recov.Get_dBias_Acc()-nsBias_recov.Get_dBias_Acc()).norm()>1e-6 )
    {
        std::cerr<<"recovered delta bias gyr not equal to Bias vertex"<<std::endl<<ns_recov.Get_dBias_Gyr().transpose()<<" / "<<nsBias_recov.Get_dBias_Gyr().transpose()<<std::endl;
        std::cerr<<"recovered delta bias acc not equal to Bias vertex"<<std::endl<<ns_recov.Get_dBias_Acc().transpose()<<" / "<<nsBias_recov.Get_dBias_Acc().transpose()<<std::endl;
    }

    return nInitialCorrespondences-nBad;
}

#ifndef NOT_UPDATE_GYRO_BIAS
int Optimizer::PoseOptimization15DoF(Frame *pFrame, Frame* pLastFrame, const IMUPreintegrator& imupreint, const cv::Mat& gw, const bool& bComputeMarg)
{
    // Extrinsics
    Matrix4d Tbc = ConfigParam::GetEigTbc();
    Matrix3d Rbc = Tbc.topLeftCorner(3,3);
    Vector3d Pbc = Tbc.topRightCorner(3,1);
    // Gravity vector in world frame
    Vector3d GravityVec = Converter::toVector3d(gw);

    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType * linearSolver;

    //linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();
    linearSolver = new g2o::LinearSolverCholmod<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    //g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
    optimizer.setAlgorithm(solver);

    int nInitialCorrespondences=0;

    // Set Frame vertex
    const int FrameId = 0;
    const int LastFrameId = 1;
    g2o::VertexNavState * vNSF = new g2o::VertexNavState();
    vNSF->setEstimate(pFrame->GetNavState());
    vNSF->setId(FrameId);
    vNSF->setFixed(false);
    optimizer.addVertex(vNSF);

    // Set LastFrame vertex
    g2o::VertexNavState * vNSFlast = new g2o::VertexNavState();
    vNSFlast->setEstimate(pLastFrame->GetNavState());
    vNSFlast->setId(LastFrameId);
    vNSFlast->setFixed(false);
    optimizer.addVertex(vNSFlast);


    // Set prior edge for Last Frame, from mMargCovInv
    g2o::EdgeNavStatePrior* eNSPrior = new g2o::EdgeNavStatePrior();
    {
        eNSPrior->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(LastFrameId)));
        eNSPrior->setInformation(pLastFrame->mMargCovInv);
        eNSPrior->setMeasurement(pLastFrame->mNavStatePrior);

        const float thHuberNavState = sqrt(30.5779);
        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
        eNSPrior->setRobustKernel(rk);
        rk->setDelta(thHuberNavState);

        optimizer.addEdge(eNSPrior);
    }

    // Set edge between Frame-LastFrame
    g2o::EdgeNavState* eNS = new g2o::EdgeNavState();
    {
        eNS->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(LastFrameId)));
        eNS->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(FrameId)));
        eNS->setMeasurement(imupreint);

        // Test log
        if(imupreint.getDeltaTime() < 1e-3)
        {
            cerr<<"IMU pre-integrator delta time between 2 KFs too small: "<<imupreint.getDeltaTime()<<endl;
            cerr<<"No EdgeNavState added"<<endl;
        }

        // Use chi2inv() in MATLAB to compute the value corresponding to 0.95/0.99 prob. w.r.t 15DOF: 24.9958/30.5779
        const float thHuberNavState = sqrt(30.5779);
        // Inverse covariance of bias random walk
        Matrix<double,6,6> InvCovBgaRW = Matrix<double,6,6>::Identity();
        InvCovBgaRW.topLeftCorner(3,3) = Matrix3d::Identity()/IMUData::getGyrBiasRW2();       // Gyroscope bias random walk, covariance INVERSE
        InvCovBgaRW.bottomRightCorner(3,3) = Matrix3d::Identity()/IMUData::getAccBiasRW2();   // Accelerometer bias random walk, covariance INVERSE

        Matrix9d CovPVR = imupreint.getCovPVPhi() /*+ 1e-20*Matrix9d::Identity()*/;  // Add 1e-20 in case computed cov is zero..
        Matrix<double,15,15> InfoMat = Matrix<double,15,15>::Identity();
        InfoMat.topLeftCorner(9,9) = CovPVR.inverse();
        InfoMat.bottomRightCorner(6,6) = InvCovBgaRW/imupreint.getDeltaTime();

        eNS->setInformation(InfoMat);

        eNS->SetParams(GravityVec);

        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
        eNS->setRobustKernel(rk);
        rk->setDelta(thHuberNavState);

        optimizer.addEdge(eNS);

    }

    // Set MapPoint vertices
    const int Ncur = pFrame->N;
    const int Nlast = pLastFrame->N;

    vector<g2o::EdgeNavStatePointXYZOnlyPose*> vpEdgesMono;
    vector<size_t> vnIndexEdgeMono;
    vpEdgesMono.reserve(Ncur);
    vnIndexEdgeMono.reserve(Ncur);

    vector<g2o::EdgeNavStatePointXYZOnlyPose*> vpEdgesMonoLast;
    vector<size_t> vnIndexEdgeMonoLast;
    vpEdgesMonoLast.reserve(Nlast);
    vnIndexEdgeMonoLast.reserve(Nlast);


    const float deltaMono = sqrt(5.991);

    {
    unique_lock<mutex> lock(MapPoint::mGlobalMutex);

    for(int i=0; i<Ncur; i++)
    {
        MapPoint* pMP = pFrame->mvpMapPoints[i];
        if(pMP)
        {
            //            g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
            //            vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
            //            int id = pMP->mnId+10;
            //            vPoint->setId(id);
            //            vPoint->setFixed(true);
            //            optimizer.addVertex(vPoint);

            // Monocular observation
            if(pFrame->mvuRight[i]<0)
            {
                nInitialCorrespondences++;
                pFrame->mvbOutlier[i] = false;

                Eigen::Matrix<double,2,1> obs;
                const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
                obs << kpUn.pt.x, kpUn.pt.y;

                //g2o::EdgeSE3ProjectXYZOnlyPose* e = new g2o::EdgeSE3ProjectXYZOnlyPose();
                g2o::EdgeNavStatePointXYZOnlyPose* e = new g2o::EdgeNavStatePointXYZOnlyPose();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(FrameId)));
                e->setMeasurement(obs);
                const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
                e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                e->setRobustKernel(rk);
                rk->setDelta(deltaMono);

                e->SetParams(pFrame->fx,pFrame->fy,pFrame->cx,pFrame->cy,Rbc,Pbc,Converter::toVector3d(pMP->GetWorldPos()));
//                e->fx = pFrame->fx;
//                e->fy = pFrame->fy;
//                e->cx = pFrame->cx;
//                e->cy = pFrame->cy;
//                cv::Mat Xw = pMP->GetWorldPos();
//                e->Xw[0] = Xw.at<float>(0);
//                e->Xw[1] = Xw.at<float>(1);
//                e->Xw[2] = Xw.at<float>(2);

                optimizer.addEdge(e);

                vpEdgesMono.push_back(e);
                vnIndexEdgeMono.push_back(i);
            }
            else  // Stereo observation
            {
                cerr<<"stereo shouldn't in poseoptimization"<<endl;
            }
        }

    }

    // Add Point-Pose edges for last frame
    for(int i=0; i<Nlast; i++)
    {
        MapPoint* pMP = pLastFrame->mvpMapPoints[i];
        if(pMP)
        {
            // Monocular observation
            if(pLastFrame->mvuRight[i]<0)
            {
                //nInitialCorrespondences++;
                pLastFrame->mvbOutlier[i] = false;

                Eigen::Matrix<double,2,1> obs;
                const cv::KeyPoint &kpUn = pLastFrame->mvKeysUn[i];
                obs << kpUn.pt.x, kpUn.pt.y;

                //g2o::EdgeSE3ProjectXYZOnlyPose* e = new g2o::EdgeSE3ProjectXYZOnlyPose();
                g2o::EdgeNavStatePointXYZOnlyPose* e = new g2o::EdgeNavStatePointXYZOnlyPose();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(LastFrameId)));
                e->setMeasurement(obs);
                const float invSigma2 = pLastFrame->mvInvLevelSigma2[kpUn.octave];
                e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                e->setRobustKernel(rk);
                rk->setDelta(deltaMono);

                e->SetParams(pLastFrame->fx,pLastFrame->fy,pLastFrame->cx,pLastFrame->cy,Rbc,Pbc,Converter::toVector3d(pMP->GetWorldPos()));

                optimizer.addEdge(e);

                vpEdgesMonoLast.push_back(e);
                vnIndexEdgeMonoLast.push_back(i);
            }
            else  // Stereo observation
            {
                cerr<<"stereo shouldn't in poseoptimization"<<endl;
            }
        }
    }
    }


    if(nInitialCorrespondences<3)
        return 0;

    // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
    // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
    const float chi2Mono[4]={5.991,5.991,5.991,5.991};
    //const float chi2Stereo[4]={7.815,7.815,7.815, 7.815};
    const int its[4]={10,10,10,10};

//    //Debug log
//    cout<<"total Points: "<<vpEdgesMono.size()<<endl;

    int nBad=0;
    int nBadLast=0;
    for(size_t it=0; it<4; it++)
    {

        vNSF->setEstimate(pFrame->GetNavState());
        vNSFlast->setEstimate(pLastFrame->GetNavState());
        optimizer.initializeOptimization(0);
        optimizer.optimize(its[it]);

        nBad=0;
        for(size_t i=0, iend=vpEdgesMono.size(); i<iend; i++)
        {
            g2o::EdgeNavStatePointXYZOnlyPose* e = vpEdgesMono[i];

            const size_t idx = vnIndexEdgeMono[i];

            if(pFrame->mvbOutlier[idx])
            {
                e->computeError();
            }

            const float chi2 = e->chi2();

            if(chi2>chi2Mono[it])
            {
                pFrame->mvbOutlier[idx]=true;
                e->setLevel(1);
                nBad++;
            }
            else
            {
                pFrame->mvbOutlier[idx]=false;
                e->setLevel(0);
            }

            if(it==2)
                e->setRobustKernel(0);
        }

        nBadLast=0;
        for(size_t i=0, iend=vpEdgesMonoLast.size(); i<iend; i++)
        {
            g2o::EdgeNavStatePointXYZOnlyPose* e = vpEdgesMonoLast[i];

            const size_t idx = vnIndexEdgeMonoLast[i];

            if(pLastFrame->mvbOutlier[idx])
            {
                e->computeError();
            }

            const float chi2 = e->chi2();

            if(chi2>chi2Mono[it])
            {
                pLastFrame->mvbOutlier[idx]=true;
                e->setLevel(1);
                nBadLast++;
            }
            else
            {
                pLastFrame->mvbOutlier[idx]=false;
                e->setLevel(0);
            }

            if(it==2)
                e->setRobustKernel(0);
        }

//        //Debug log
//        cout<<nBad<<" bad Points in iter "<<it<<", rest points: "<<optimizer.edges().size()<<endl;
//        cout<<nBadLast<<" bad Points of last Frame in iter "<<it<<endl;
//        cout<<"NavState edge chi2: "<<eNS->chi2()<<endl;

        if(optimizer.edges().size()<10)
            break;
    }

//    // Recover optimized pose and return number of inliers
//    g2o::VertexSE3Expmap* vSE3_recov = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
//    g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
//    cv::Mat pose = Converter::toCvMat(SE3quat_recov);
//    pFrame->SetPose(pose);

    g2o::VertexNavState* vNS_recov = static_cast<g2o::VertexNavState*>(optimizer.vertex(FrameId));
    NavState ns_recov = vNS_recov->estimate();
    pFrame->SetNavState(ns_recov);
    pFrame->UpdatePoseFromNS(ConfigParam::GetMatTbc());

//    g2o::VertexNavState* vNSlast_recov = static_cast<g2o::VertexNavState*>(optimizer.vertex(LastFrameId));
//    NavState nslast_recov = vNSlast_recov->estimate();
//    pLastFrame->SetNavState(nslast_recov);
//    pLastFrame->UpdatePoseFromNS(ConfigParam::GetMatTbc());

//    //Debug log
//    cout<<"bias gyr/acc last:   "<<pLastFrame->GetNavState().Get_BiasGyr().transpose()<<" / "<<pLastFrame->GetNavState().Get_BiasAcc().transpose()<<endl;
//    cout<<"delta bias g/a last: "<<pLastFrame->GetNavState().Get_dBias_Gyr().transpose()<<" / "<<pLastFrame->GetNavState().Get_dBias_Acc().transpose()<<endl;
//    cout<<"bias gyr/acc:   "<<pFrame->GetNavState().Get_BiasGyr().transpose()<<" / "<<pFrame->GetNavState().Get_BiasAcc().transpose()<<endl;
//    cout<<"delta bias g/a: "<<pFrame->GetNavState().Get_dBias_Gyr().transpose()<<" / "<<pFrame->GetNavState().Get_dBias_Acc().transpose()<<endl;

    // Compute marginalized Hessian H and B, H*x=B, H/B can be used as prior for next optimization in PoseOptimization
    if(bComputeMarg)
    {
        g2o::SparseBlockMatrixXd spinv;
        optimizer.computeMarginals(spinv, optimizer.vertex(FrameId));
        pFrame->mMargCovInv = spinv.block(0,0)->inverse();
        pFrame->mNavStatePrior = ns_recov;
    }

    return nInitialCorrespondences-nBad;
}
//#endif

int Optimizer::PoseOptimization15DoF(Frame *pFrame, KeyFrame* pLastKF, const IMUPreintegrator& imupreint, const cv::Mat& gw, const bool& bComputeMarg)
{
    // Extrinsics
    Matrix4d Tbc = ConfigParam::GetEigTbc();
    Matrix3d Rbc = Tbc.topLeftCorner(3,3);
    Vector3d Pbc = Tbc.topRightCorner(3,1);
    // Gravity vector in world frame
    Vector3d GravityVec = Converter::toVector3d(gw);

    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType * linearSolver;

    //linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();
    linearSolver = new g2o::LinearSolverCholmod<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    //g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
    optimizer.setAlgorithm(solver);

    int nInitialCorrespondences=0;

    // Set Frame vertex
    const int FrameId = 0;
    const int LastKFId = 1;
    g2o::VertexNavState * vNSF = new g2o::VertexNavState();
    vNSF->setEstimate(pFrame->GetNavState());
    vNSF->setId(FrameId);
    vNSF->setFixed(false);
    optimizer.addVertex(vNSF);

    // Set KeyFrame vertex
    g2o::VertexNavState * vNSKF = new g2o::VertexNavState();
    vNSKF->setEstimate(pLastKF->GetNavState());
    vNSKF->setId(LastKFId);
    vNSKF->setFixed(true);
    optimizer.addVertex(vNSKF);

    // Set edge between Frame-LastKF
    g2o::EdgeNavState* eNS = new g2o::EdgeNavState();
    {
        eNS->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(LastKFId)));
        eNS->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(FrameId)));
        eNS->setMeasurement(imupreint);

        // Test log
        if(imupreint.getDeltaTime() < 1e-3)
        {
            cerr<<"IMU pre-integrator delta time between 2 KFs too small: "<<imupreint.getDeltaTime()<<endl;
            cerr<<"No EdgeNavState added"<<endl;
        }

        // Use chi2inv() in MATLAB to compute the value corresponding to 0.95/0.99 prob. w.r.t 15DOF: 24.9958/30.5779
        const float thHuberNavState = sqrt(30.5779);
        // Inverse covariance of bias random walk
        Matrix<double,6,6> InvCovBgaRW = Matrix<double,6,6>::Identity();
        InvCovBgaRW.topLeftCorner(3,3) = Matrix3d::Identity()/IMUData::getGyrBiasRW2();       // Gyroscope bias random walk, covariance INVERSE
        InvCovBgaRW.bottomRightCorner(3,3) = Matrix3d::Identity()/IMUData::getAccBiasRW2();   // Accelerometer bias random walk, covariance INVERSE

        Matrix9d CovPVR = imupreint.getCovPVPhi() /*+ 1e-20*Matrix9d::Identity()*/;  // Add 1e-20 in case computed cov is zero..
        Matrix<double,15,15> InfoMat = Matrix<double,15,15>::Identity();
        InfoMat.topLeftCorner(9,9) = CovPVR.inverse();
        InfoMat.bottomRightCorner(6,6) = InvCovBgaRW/imupreint.getDeltaTime();

        eNS->setInformation(InfoMat);

        eNS->SetParams(GravityVec);

        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
        eNS->setRobustKernel(rk);
        rk->setDelta(thHuberNavState);

        optimizer.addEdge(eNS);

    }

    // Set MapPoint vertices
    const int N = pFrame->N;

    vector<g2o::EdgeNavStatePointXYZOnlyPose*> vpEdgesMono;
    vector<size_t> vnIndexEdgeMono;
    vpEdgesMono.reserve(N);
    vnIndexEdgeMono.reserve(N);

    const float deltaMono = sqrt(5.991);

    {
    unique_lock<mutex> lock(MapPoint::mGlobalMutex);
    for(int i=0; i<N; i++)
    {
        MapPoint* pMP = pFrame->mvpMapPoints[i];
        if(pMP)
        {
            //            g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
            //            vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
            //            int id = pMP->mnId+10;
            //            vPoint->setId(id);
            //            vPoint->setFixed(true);
            //            optimizer.addVertex(vPoint);
            //              cout<<pMP->mnId<<", ";

            // Monocular observation
            if(pFrame->mvuRight[i]<0)
            {
                nInitialCorrespondences++;
                pFrame->mvbOutlier[i] = false;

                Eigen::Matrix<double,2,1> obs;
                const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
                obs << kpUn.pt.x, kpUn.pt.y;

                //g2o::EdgeSE3ProjectXYZOnlyPose* e = new g2o::EdgeSE3ProjectXYZOnlyPose();
                g2o::EdgeNavStatePointXYZOnlyPose* e = new g2o::EdgeNavStatePointXYZOnlyPose();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(FrameId)));
                e->setMeasurement(obs);
                const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
                e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                e->setRobustKernel(rk);
                rk->setDelta(deltaMono);

                e->SetParams(pFrame->fx,pFrame->fy,pFrame->cx,pFrame->cy,Rbc,Pbc,Converter::toVector3d(pMP->GetWorldPos()));
//                e->fx = pFrame->fx;
//                e->fy = pFrame->fy;
//                e->cx = pFrame->cx;
//                e->cy = pFrame->cy;
//                cv::Mat Xw = pMP->GetWorldPos();
//                e->Xw[0] = Xw.at<float>(0);
//                e->Xw[1] = Xw.at<float>(1);
//                e->Xw[2] = Xw.at<float>(2);

                optimizer.addEdge(e);

                vpEdgesMono.push_back(e);
                vnIndexEdgeMono.push_back(i);
            }
            else  // Stereo observation
            {
                cerr<<"stereo shouldn't in poseoptimization"<<endl;
            }
        }

    }
    }
    //cout<<endl;


    if(nInitialCorrespondences<3)
        return 0;

    // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
    // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
    const float chi2Mono[4]={5.991,5.991,5.991,5.991};
    //const float chi2Stereo[4]={7.815,7.815,7.815, 7.815};
    const int its[4]={10,10,10,10};

//    //Debug log
//    cout<<"total Points: "<<vpEdgesMono.size()<<endl;

    int nBad=0;
    for(size_t it=0; it<4; it++)
    {

        vNSF->setEstimate(pFrame->GetNavState());
        optimizer.initializeOptimization(0);
        optimizer.optimize(its[it]);

        nBad=0;
        for(size_t i=0, iend=vpEdgesMono.size(); i<iend; i++)
        {
            g2o::EdgeNavStatePointXYZOnlyPose* e = vpEdgesMono[i];

            const size_t idx = vnIndexEdgeMono[i];

            if(pFrame->mvbOutlier[idx])
            {
                e->computeError();
            }

            const float chi2 = e->chi2();

            if(chi2>chi2Mono[it])
            {
                pFrame->mvbOutlier[idx]=true;
                e->setLevel(1);
                nBad++;
            }
            else
            {
                pFrame->mvbOutlier[idx]=false;
                e->setLevel(0);
            }

            if(it==2)
                e->setRobustKernel(0);
        }

//        //Debug log
//        cout<<nBad<<" bad Points in iter "<<it<<", rest points: "<<optimizer.edges().size()<<endl;

        if(optimizer.edges().size()<10)
            break;
    }

//    // Recover optimized pose and return number of inliers
//    g2o::VertexSE3Expmap* vSE3_recov = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
//    g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
//    cv::Mat pose = Converter::toCvMat(SE3quat_recov);
//    pFrame->SetPose(pose);

    g2o::VertexNavState* vNS_recov = static_cast<g2o::VertexNavState*>(optimizer.vertex(FrameId));
    NavState ns_recov = vNS_recov->estimate();
    pFrame->SetNavState(ns_recov);
    pFrame->UpdatePoseFromNS(ConfigParam::GetMatTbc());

//    //Debug log
//    cout<<"bias gyr/acc:   "<<pFrame->GetNavState().Get_BiasGyr().transpose()<<" / "<<pFrame->GetNavState().Get_BiasAcc().transpose()<<endl;
//    cout<<"delta bias g/a: "<<pFrame->GetNavState().Get_dBias_Gyr().transpose()<<" / "<<pFrame->GetNavState().Get_dBias_Acc().transpose()<<endl;

    // Compute marginalized Hessian H and B, H*x=B, H/B can be used as prior for next optimization in PoseOptimization
    if(bComputeMarg)
    {
        g2o::SparseBlockMatrixXd spinv;
        optimizer.computeMarginals(spinv, optimizer.vertex(FrameId));
        pFrame->mMargCovInv = spinv.block(0,0)->inverse();
        pFrame->mNavStatePrior = ns_recov;
    }

    return nInitialCorrespondences-nBad;
}
#endif

void Optimizer::LocalBundleAdjustmentNavState(KeyFrame *pCurKF, const std::list<KeyFrame*> &lLocalKeyFrames, bool* pbStopFlag, Map* pMap, cv::Mat& gw, LocalMapping* pLM)
{
#ifndef NOT_UPDATE_GYRO_BIAS
    static bool dbgLBAfopen=false;
    static ofstream dbgLBAPVRErr,dbgLBABiasErr,dbgLBAPVRErr2,dbgLBABiasErr2;
    if(!dbgLBAfopen)
    {
        dbgLBAPVRErr.open("/home/jp/opensourcecode/ORB_SLAM2/tmp/dbgLBAPVRErr.txt");
        dbgLBABiasErr.open("/home/jp/opensourcecode/ORB_SLAM2/tmp/dbgLBABiasErr.txt");
        dbgLBAPVRErr2.open("/home/jp/opensourcecode/ORB_SLAM2/tmp/dbgLBAPVRErr2.txt");
        dbgLBABiasErr2.open("/home/jp/opensourcecode/ORB_SLAM2/tmp/dbgLBABiasErr2.txt");
        if(dbgLBAPVRErr.is_open() && dbgLBABiasErr.is_open() && dbgLBAPVRErr2.is_open() && dbgLBABiasErr2.is_open())
        {
            cerr<<"file opened."<<endl;
            dbgLBAfopen = true;
        }
        else
        {
            cerr<<"file open error in dbgLBAfopen"<<endl;
            dbgLBAfopen = false;
        }
        dbgLBAPVRErr<<std::fixed<<std::setprecision(6);
        dbgLBABiasErr<<std::fixed<<std::setprecision(10);
        dbgLBAPVRErr2<<std::fixed<<std::setprecision(6);
        dbgLBABiasErr2<<std::fixed<<std::setprecision(10);
    }
#endif

    // Check current KeyFrame in local window
    if(pCurKF != lLocalKeyFrames.back())
        cerr<<"pCurKF != lLocalKeyFrames.back. check"<<endl;

    // Extrinsics
    Matrix4d Tbc = ConfigParam::GetEigTbc();
    Matrix3d Rbc = Tbc.topLeftCorner(3,3);
    Vector3d Pbc = Tbc.topRightCorner(3,1);
    // Gravity vector in world frame
    Vector3d GravityVec = Converter::toVector3d(gw);

    // All KeyFrames in Local window are optimized
    for(list<KeyFrame*>::const_iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;
        pKFi->mnBALocalForKF = pCurKF->mnId;
    }

    // Local MapPoints seen in Local KeyFrames
    list<MapPoint*> lLocalMapPoints;
    for(list<KeyFrame*>::const_iterator lit=lLocalKeyFrames.begin() , lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        vector<MapPoint*> vpMPs = (*lit)->GetMapPointMatches();
        for(vector<MapPoint*>::iterator vit=vpMPs.begin(), vend=vpMPs.end(); vit!=vend; vit++)
        {
            MapPoint* pMP = *vit;
            if(pMP)
                if(!pMP->isBad())
                    if(pMP->mnBALocalForKF!=pCurKF->mnId)
                    {
                        lLocalMapPoints.push_back(pMP);
                        pMP->mnBALocalForKF=pCurKF->mnId;
                    }
        }
    }

    // Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local Keyframes
    list<KeyFrame*> lFixedCameras;
    // Add the KeyFrame before local window.
    KeyFrame* pKFPrevLocal = lLocalKeyFrames.front()->GetPrevKeyFrame();
    if(pKFPrevLocal)
    {
        // Test log
        if(pKFPrevLocal->isBad()) cerr<<"KeyFrame before local window is bad?"<<endl;
        if(pKFPrevLocal->mnBAFixedForKF==pCurKF->mnId) cerr<<"KeyFrame before local, has been added to lFixedKF?"<<endl;
        if(pKFPrevLocal->mnBALocalForKF==pCurKF->mnId) cerr<<"KeyFrame before local, has been added to lLocalKF?"<<endl;

        pKFPrevLocal->mnBAFixedForKF = pCurKF->mnId;
        if(!pKFPrevLocal->isBad())
            lFixedCameras.push_back(pKFPrevLocal);
        else
            cerr<<"pKFPrevLocal is Bad?"<<endl;
    }
    // Test log
    else {cerr<<"pKFPrevLocal is NULL?"<<endl;}
    // Covisible KeyFrames
    for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        map<KeyFrame*,size_t> observations = (*lit)->GetObservations();
        for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFi = mit->first;

            if(pKFi->mnBALocalForKF!=pCurKF->mnId && pKFi->mnBAFixedForKF!=pCurKF->mnId)
            {
                pKFi->mnBAFixedForKF=pCurKF->mnId;
                if(!pKFi->isBad())
                    lFixedCameras.push_back(pKFi);
            }
        }
    }

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);

    //g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    if(pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    int maxKFid = 0;

    // Set Local KeyFrame vertices
    for(list<KeyFrame*>::const_iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;
        int idKF = pKFi->mnId*2;
        // Vertex of PVR
        {
            g2o::VertexNavStatePVR * vNSPVR = new g2o::VertexNavStatePVR();
            vNSPVR->setEstimate(pKFi->GetNavState());
            vNSPVR->setId(idKF);
            vNSPVR->setFixed(false);
            optimizer.addVertex(vNSPVR);
        }
        // Vertex of Bias
        {
            g2o::VertexNavStateBias * vNSBias = new g2o::VertexNavStateBias();
            vNSBias->setEstimate(pKFi->GetNavState());
            vNSBias->setId(idKF+1);
            vNSBias->setFixed(false);
            optimizer.addVertex(vNSBias);
        }

        //        //Debug log
        //        cout<<"pvr/bias id: "<<vNSPVR->id()<<"/"<<vNSBias->id()<<endl;
        //        cout<<"pvr/bias gyr bias: "<<vNSPVR->estimate().Get_BiasGyr().transpose()<<" / "<<vNSBias->estimate().Get_BiasGyr().transpose()<<endl;
        //        cout<<"pvr/bias acc bias: "<<vNSPVR->estimate().Get_BiasAcc().transpose()<<" / "<<vNSBias->estimate().Get_BiasAcc().transpose()<<endl;
        //        cout<<"pvr/bias P: "<<vNSPVR->estimate().Get_P().transpose()<<" / "<<vNSBias->estimate().Get_P().transpose()<<endl;

        if(idKF+1>maxKFid)
            maxKFid=idKF+1;
        // Test log
        if(pKFi->mnId == 0) cerr<<"pKFi->mnId == 0, shouldn't in LocalBA of NavState"<<endl;
    }

    // Set Fixed KeyFrame vertices. Including the pKFPrevLocal.
    for(list<KeyFrame*>::iterator lit=lFixedCameras.begin(), lend=lFixedCameras.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;
        int idKF = pKFi->mnId*2;
        // For common fixed KeyFrames, only add PVR vertex
        {
            g2o::VertexNavStatePVR * vNSPVR = new g2o::VertexNavStatePVR();
            vNSPVR->setEstimate(pKFi->GetNavState());
            vNSPVR->setId(idKF);
            vNSPVR->setFixed(true);
            optimizer.addVertex(vNSPVR);
        }
        // For Local-Window-Previous KeyFrame, add Bias vertex
        if(pKFi == pKFPrevLocal)
        {
            g2o::VertexNavStateBias * vNSBias = new g2o::VertexNavStateBias();
            vNSBias->setEstimate(pKFi->GetNavState());
            vNSBias->setId(idKF+1);
            vNSBias->setFixed(true);
            optimizer.addVertex(vNSBias);
        }

        if(idKF+1>maxKFid)
            maxKFid=idKF+1;
    }

    // Edges between KeyFrames in Local Window
    // and
    // Edges between 1st KeyFrame of Local Window and its previous (fixed)KeyFrame - pKFPrevLocal
    vector<g2o::EdgeNavStatePVR*> vpEdgesNavStatePVR;
    vector<g2o::EdgeNavStateBias*> vpEdgesNavStateBias;
    // Use chi2inv() in MATLAB to compute the value corresponding to 0.95/0.99 prob. w.r.t 15DOF: 24.9958/30.5779
    // 12.592/16.812 for 0.95/0.99 6DoF
    // 16.919/21.666 for 0.95/0.99 9DoF
    //const float thHuberNavState = sqrt(30.5779);
    const float thHuberNavStatePVR = sqrt(21.666);
    const float thHuberNavStateBias = sqrt(16.812);
    // Inverse covariance of bias random walk
#ifndef NOT_UPDATE_GYRO_BIAS
    Matrix<double,6,6> InvCovBgaRW = Matrix<double,6,6>::Identity();
    InvCovBgaRW.topLeftCorner(3,3) = Matrix3d::Identity()/IMUData::getGyrBiasRW2();       // Gyroscope bias random walk, covariance INVERSE
    InvCovBgaRW.bottomRightCorner(3,3) = Matrix3d::Identity()/IMUData::getAccBiasRW2();   // Accelerometer bias random walk, covariance INVERSE
#else
    Matrix<double,3,3> InvCovBgaRW = Matrix3d::Identity()/IMUData::getAccBiasRW2();   // Accelerometer bias random walk, covariance INVERSE
#endif

    for(list<KeyFrame*>::const_iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        KeyFrame* pKF1 = *lit;                      // Current KF, store the IMU pre-integration between previous-current
        KeyFrame* pKF0 = pKF1->GetPrevKeyFrame();   // Previous KF

        // PVR edge
        {
            g2o::EdgeNavStatePVR * epvr = new g2o::EdgeNavStatePVR();
            epvr->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(2*pKF0->mnId)));
            epvr->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(2*pKF1->mnId)));
            epvr->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(2*pKF0->mnId+1)));
            epvr->setMeasurement(pKF1->GetIMUPreInt());

            Matrix9d InvCovPVR = pKF1->GetIMUPreInt().getCovPVPhi().inverse();
            epvr->setInformation(InvCovPVR);
            epvr->SetParams(GravityVec);

            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            epvr->setRobustKernel(rk);
            rk->setDelta(thHuberNavStatePVR);

            optimizer.addEdge(epvr);
            vpEdgesNavStatePVR.push_back(epvr);
        }
        // Bias edge
        {
            g2o::EdgeNavStateBias * ebias = new g2o::EdgeNavStateBias();
            ebias->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(2*pKF0->mnId+1)));
            ebias->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(2*pKF1->mnId+1)));
            ebias->setMeasurement(pKF1->GetIMUPreInt());

            ebias->setInformation(InvCovBgaRW/pKF1->GetIMUPreInt().getDeltaTime());

            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            ebias->setRobustKernel(rk);
            rk->setDelta(thHuberNavStateBias);

            optimizer.addEdge(ebias);
            vpEdgesNavStateBias.push_back(ebias);
        }

        // Test log
        if(pKF1->GetIMUPreInt().getDeltaTime() < 1e-3)
        {
            cerr<<"IMU pre-integrator delta time between 2 KFs too small: "<<pKF1->GetIMUPreInt().getDeltaTime()<<endl;
            cerr<<"No EdgeNavState added"<<endl;
            continue;
        }
        if(lit == lLocalKeyFrames.begin())
        {
            // First KF in Local Window, link (fixed) pKFPrevLocal
            if(pKF0 != pKFPrevLocal) cerr<<"pKF0 != pKFPrevLocal for 1st KF in Local Window, id: "<<pKF0->mnId<<","<<pKFPrevLocal->mnId<<endl;
        }
        else
        {
            // KFs in Local Window, link another local KF
        }
    }

    // Set MapPoint vertices
    const int nExpectedSize = (lLocalKeyFrames.size()+lFixedCameras.size())*lLocalMapPoints.size();

    vector<g2o::EdgeNavStatePVRPointXYZ*> vpEdgesMono;
    vpEdgesMono.reserve(nExpectedSize);

    vector<KeyFrame*> vpEdgeKFMono;
    vpEdgeKFMono.reserve(nExpectedSize);

    vector<MapPoint*> vpMapPointEdgeMono;
    vpMapPointEdgeMono.reserve(nExpectedSize);

    const float thHuberMono = sqrt(5.991);

    for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        MapPoint* pMP = *lit;
        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
        int id = pMP->mnId+maxKFid+1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);

        cv::Mat Pw = pMP->GetWorldPos();

        const map<KeyFrame*,size_t> observations = pMP->GetObservations();

        // Set edges between KeyFrame and MapPoint
        for(map<KeyFrame*,size_t>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFi = mit->first;

            if(!pKFi->isBad())
            {
                const cv::KeyPoint &kpUn = pKFi->mvKeysUn[mit->second];

                // Monocular observation
                if(pKFi->mvuRight[mit->second]<0)
                {
                    Eigen::Matrix<double,2,1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    g2o::EdgeNavStatePVRPointXYZ* e = new g2o::EdgeNavStatePVRPointXYZ();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(2*pKFi->mnId)));
                    e->setMeasurement(obs);
                    const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];

                    e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberMono);

                    e->SetParams(pKFi->fx,pKFi->fy,pKFi->cx,pKFi->cy,Rbc,Pbc);

                    optimizer.addEdge(e);
                    vpEdgesMono.push_back(e);
                    vpEdgeKFMono.push_back(pKFi);
                    vpMapPointEdgeMono.push_back(pMP);
                }
                else
                {
                    // Test log
                    cerr<<"Stereo not supported yet, why here?? check."<<endl;
                }
            }
        }
    }

    if(pbStopFlag)
        if(*pbStopFlag)
            return;

    // First try
    //optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(5);

    bool bDoMore= true;

    if(pbStopFlag)
        if(*pbStopFlag)
            bDoMore = false;

    if(bDoMore)
    {
    // Check inlier observations
    for(size_t i=0, iend=vpEdgesMono.size(); i<iend;i++)
    {
        g2o::EdgeNavStatePVRPointXYZ* e = vpEdgesMono[i];
        MapPoint* pMP = vpMapPointEdgeMono[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>5.991 || !e->isDepthPositive())
        {
            e->setLevel(1);
        }

        e->setRobustKernel(0);
    }

    // Check inlier observations
    int tmpcnt=0;
    for(size_t i=0, iend=vpEdgesNavStatePVR.size(); i<iend; i++)
    {
        g2o::EdgeNavStatePVR* e = vpEdgesNavStatePVR[i];
        if(e->chi2()>21.666)
        {
            //e->setLevel(1);
                        cout<<"1 PVRedge "<<i<<", chi2 "<<e->chi2()<<". ";
                        tmpcnt++;
        }
        //e->setRobustKernel(0);

#ifndef NOT_UPDATE_GYRO_BIAS
        e->computeError();
        Vector9d err=e->error();
        for(int n=0;n<9;n++)
            dbgLBAPVRErr<<err[n]<<" ";
        dbgLBAPVRErr<<endl;
#endif
    }
        //cout<<endl<<"edge bias ns bad: ";
    for(size_t i=0, iend=vpEdgesNavStateBias.size(); i<iend; i++)
    {
        g2o::EdgeNavStateBias* e = vpEdgesNavStateBias[i];
        if(e->chi2()>16.812)
        {
            //e->setLevel(1);
                        cout<<"1 Bias edge "<<i<<", chi2 "<<e->chi2()<<". ";
                        tmpcnt++;
        }
        //e->setRobustKernel(0);

#ifndef NOT_UPDATE_GYRO_BIAS
        e->computeError();
        Vector6d err=e->error();
        for(int n=0;n<6;n++)
            dbgLBABiasErr<<err[n]<<" ";
        dbgLBABiasErr<<endl;
#endif
    }
    if(tmpcnt>0)
        cout<<endl;

    // Optimize again without the outliers
    optimizer.initializeOptimization(0);
    optimizer.optimize(10);

    }

    //
    vector<pair<KeyFrame*,MapPoint*> > vToErase;
    vToErase.reserve(vpEdgesMono.size());

    double PosePointchi2=0, PosePosechi2=0, BiasBiaschi2=0;
    // Check inlier observations
    for(size_t i=0, iend=vpEdgesMono.size(); i<iend;i++)
    {
        g2o::EdgeNavStatePVRPointXYZ* e = vpEdgesMono[i];
        MapPoint* pMP = vpMapPointEdgeMono[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>5.991 || !e->isDepthPositive())
        {
            KeyFrame* pKFi = vpEdgeKFMono[i];
            vToErase.push_back(make_pair(pKFi,pMP));
        }

        PosePointchi2 += e->chi2();
    }

    // Debug log
    // Check inlier observations
    int tmpcnt2=0;
    for(size_t i=0, iend=vpEdgesNavStatePVR.size(); i<iend; i++)
    {
        g2o::EdgeNavStatePVR* e = vpEdgesNavStatePVR[i];
        if(e->chi2()>21.666)
        {
                        cout<<"2 PVRedge "<<i<<", chi2 "<<e->chi2()<<". ";
                        tmpcnt2++;
        }

#ifndef NOT_UPDATE_GYRO_BIAS
        e->computeError();
        Vector9d err=e->error();
        for(int n=0;n<9;n++)
            dbgLBAPVRErr2<<err[n]<<" ";
        dbgLBAPVRErr2<<endl;
#endif
    }
        //cout<<endl<<"edge bias ns bad: ";
    for(size_t i=0, iend=vpEdgesNavStateBias.size(); i<iend; i++)
    {
        g2o::EdgeNavStateBias* e = vpEdgesNavStateBias[i];
        if(e->chi2()>16.812)
        {
                        cout<<"2 Bias edge "<<i<<", chi2 "<<e->chi2()<<". ";
                        tmpcnt2++;
        }

#ifndef NOT_UPDATE_GYRO_BIAS
        e->computeError();
        Vector6d err=e->error();
        for(int n=0;n<6;n++)
            dbgLBABiasErr2<<err[n]<<" ";
        dbgLBABiasErr2<<endl;
#endif
    }
    if(tmpcnt2>0)
        cout<<endl;
    //cout<<"pose-point chi2: "<<PosePointchi2<<", pose-pose chi2: "<<PosePosechi2<<endl;

    // Get Map Mutex
    unique_lock<mutex> lock(pMap->mMutexMapUpdate);

    if(!vToErase.empty())
    {
        for(size_t i=0;i<vToErase.size();i++)
        {
            KeyFrame* pKFi = vToErase[i].first;
            MapPoint* pMPi = vToErase[i].second;
            pKFi->EraseMapPointMatch(pMPi);
            pMPi->EraseObservation(pKFi);
        }
    }

    //Keyframes
    for(list<KeyFrame*>::const_iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;
        g2o::VertexNavStatePVR* vNSPVR = static_cast<g2o::VertexNavStatePVR*>(optimizer.vertex(2*pKFi->mnId));
        g2o::VertexNavStateBias* vNSBias = static_cast<g2o::VertexNavStateBias*>(optimizer.vertex(2*pKFi->mnId+1));
        // In optimized navstate, bias not changed, delta_bias not zero, should be added to bias
        const NavState& optPVRns = vNSPVR->estimate();
        const NavState& optBiasns = vNSBias->estimate();
        NavState primaryns = pKFi->GetNavState();
        // Update NavState
        pKFi->SetNavStatePos(optPVRns.Get_P());
        pKFi->SetNavStateVel(optPVRns.Get_V());
        pKFi->SetNavStateRot(optPVRns.Get_R());
        //if(optBiasns.Get_dBias_Acc().norm()<1e-2 && optBiasns.Get_BiasGyr().norm()<1e-4)
        //{
        pKFi->SetNavStateDeltaBg(optBiasns.Get_dBias_Gyr());
        pKFi->SetNavStateDeltaBa(optBiasns.Get_dBias_Acc());
        //}

        // Update pose Tcw
        pKFi->UpdatePoseFromNS(ConfigParam::GetMatTbc());
//        cv::Mat cvRwb = Converter::toCvMat(pKFi->GetNavState().Get_RotMatrix());
//        cv::Mat cvPwb = Converter::toCvMat(pKFi->GetNavState().Get_P());
//        cv::Mat cvRcw = cvRbc.t()*cvRwb.t();
//        cv::Mat cvPcw = -cvRcw*(cvRwb*cvPbc+cvPwb);
//        cv::Mat cvTcw = cv::Mat::eye(4,4,CV_32F);
//        cvRcw.copyTo(cvTcw.rowRange(0,3).colRange(0,3));
//        cvPcw.copyTo(cvTcw.rowRange(0,3).col(3));
//        pKFi->SetPose(cvTcw);

        // Test log
        if( (primaryns.Get_BiasGyr() - optPVRns.Get_BiasGyr()).norm() > 1e-6 || (primaryns.Get_BiasGyr() - optBiasns.Get_BiasGyr()).norm() > 1e-6 )
            cerr<<"gyr bias change in optimization?"<<endl;
        if( (primaryns.Get_BiasAcc() - optPVRns.Get_BiasAcc()).norm() > 1e-6 || (primaryns.Get_BiasAcc() - optBiasns.Get_BiasAcc()).norm() > 1e-6 )
            cerr<<"acc bias change in optimization?"<<endl;
        // Debug log
        //cout<<"updated delta bias gyr: "<<optns.Get_dBias_Gyr().transpose()<<endl;
        //cout<<"updated delta bias acc: "<<optns.Get_dBias_Acc().transpose()<<endl;
        //cout<<"before and after opt, navstate.P: "<<primaryns.Get_P().transpose()<<" vs "<<optns.Get_P().transpose()<<endl;
        //cout<<"before and after opt, navstate.V: "<<primaryns.Get_V().transpose()<<" vs "<<optns.Get_V().transpose()<<endl;
        //cout<<"before and after opt, navstate.R: "<<primaryns.Get_RotMatrix()<<" vs "<<optns.Get_RotMatrix()<<endl;

    }

    //Points
    for(list<MapPoint*>::const_iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        MapPoint* pMP = *lit;
        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mnId+maxKFid+1));
        pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
        pMP->UpdateNormalAndDepth();
    }

    if(pLM)
    {
        pLM->SetMapUpdateFlagInTracking(true);
    }

}


void Optimizer::LocalBundleAdjustmentNavState15DoF(KeyFrame *pCurKF, const std::list<KeyFrame*> &lLocalKeyFrames, bool* pbStopFlag, Map* pMap, cv::Mat& gw, LocalMapping* pLM)
{
    // Check current KeyFrame in local window
    if(pCurKF != lLocalKeyFrames.back())
        cerr<<"pCurKF != lLocalKeyFrames.back. check"<<endl;

    // Extrinsics
    Matrix4d Tbc = ConfigParam::GetEigTbc();
    Matrix3d Rbc = Tbc.topLeftCorner(3,3);
    Vector3d Pbc = Tbc.topRightCorner(3,1);
    // Gravity vector in world frame
    Vector3d GravityVec = Converter::toVector3d(gw);

    // All KeyFrames in Local window are optimized
    for(list<KeyFrame*>::const_iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;
        pKFi->mnBALocalForKF = pCurKF->mnId;
    }

    // Local MapPoints seen in Local KeyFrames
    list<MapPoint*> lLocalMapPoints;
    for(list<KeyFrame*>::const_iterator lit=lLocalKeyFrames.begin() , lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        vector<MapPoint*> vpMPs = (*lit)->GetMapPointMatches();
        for(vector<MapPoint*>::iterator vit=vpMPs.begin(), vend=vpMPs.end(); vit!=vend; vit++)
        {
            MapPoint* pMP = *vit;
            if(pMP)
                if(!pMP->isBad())
                    if(pMP->mnBALocalForKF!=pCurKF->mnId)
                    {
                        lLocalMapPoints.push_back(pMP);
                        pMP->mnBALocalForKF=pCurKF->mnId;
                    }
        }
    }

    // Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local Keyframes
    list<KeyFrame*> lFixedCameras;
    // Add the KeyFrame before local window.
    KeyFrame* pKFPrevLocal = lLocalKeyFrames.front()->GetPrevKeyFrame();
    if(pKFPrevLocal)
    {
        // Test log
        if(pKFPrevLocal->isBad()) cerr<<"KeyFrame before local window is bad?"<<endl;
        if(pKFPrevLocal->mnBAFixedForKF==pCurKF->mnId) cerr<<"KeyFrame before local, has been added to lFixedKF?"<<endl;
        if(pKFPrevLocal->mnBALocalForKF==pCurKF->mnId) cerr<<"KeyFrame before local, has been added to lLocalKF?"<<endl;

        pKFPrevLocal->mnBAFixedForKF = pCurKF->mnId;
        if(!pKFPrevLocal->isBad())
            lFixedCameras.push_back(pKFPrevLocal);
        else
            cerr<<"pKFPrevLocal is Bad?"<<endl;
    }
    // Test log
    else {cerr<<"pKFPrevLocal is NULL?"<<endl;}
    // Covisible KeyFrames
    for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        map<KeyFrame*,size_t> observations = (*lit)->GetObservations();
        for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFi = mit->first;

            if(pKFi->mnBALocalForKF!=pCurKF->mnId && pKFi->mnBAFixedForKF!=pCurKF->mnId)
            {
                pKFi->mnBAFixedForKF=pCurKF->mnId;
                if(!pKFi->isBad())
                    lFixedCameras.push_back(pKFi);
            }
        }
    }

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);

    //g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    //TODO: add terminate criteria for GaussNewton method.
    //To add two more files to g2o/core
    // Sparse_optimizer_terminate_action.h/.cpp
    //To add preIteration(-1) in SparseOptimizer.cpp
    //To add addPreInterationAction for optimizer

    if(pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    int gwId = KeyFrame::nNextId + MapPoint::nNextId + 1;
    g2o::VertexGravityW * vGw = new g2o::VertexGravityW();
    {
        vGw->setEstimate(GravityVec);
        vGw->setId(gwId);
        vGw->setFixed(true);
        optimizer.addVertex(vGw);
    }

    unsigned long maxKFid = 0;

    // Set Local KeyFrame vertices
    for(list<KeyFrame*>::const_iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;
        g2o::VertexNavState * vNS = new g2o::VertexNavState();
        vNS->setEstimate(pKFi->GetNavState());
        vNS->setId(pKFi->mnId);
        vNS->setFixed(pKFi->mnId==0);
        optimizer.addVertex(vNS);
        if(pKFi->mnId>maxKFid)
            maxKFid=pKFi->mnId;
        // Test log
        if(pKFi->mnId == 0) cerr<<"pKFi->mnId == 0, shouldn't in LocalBA of NavState"<<endl;
    }

    // Set Fixed KeyFrame vertices. Including the pKFPrevLocal.
    for(list<KeyFrame*>::iterator lit=lFixedCameras.begin(), lend=lFixedCameras.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;
        g2o::VertexNavState * vNS = new g2o::VertexNavState();
        vNS->setEstimate(pKFi->GetNavState());
        vNS->setId(pKFi->mnId);
        vNS->setFixed(true);
        optimizer.addVertex(vNS);
        if(pKFi->mnId>maxKFid)
            maxKFid=pKFi->mnId;
    }

    // Edges between KeyFrames in Local Window
    // and
    // Edges between 1st KeyFrame of Local Window and its previous (fixed)KeyFrame - pKFPrevLocal
    //vector<g2o::EdgeNavState*> vpEdgesNavState;
    vector<g2o::EdgeNavStateGw*> vpEdgesNavState;
    // Use chi2inv() in MATLAB to compute the value corresponding to 0.95/0.99 prob. w.r.t 15DOF: 24.9958/30.5779
    const float thHuberNavState = sqrt(30.5779);
    // Inverse covariance of bias random walk
    Matrix<double,6,6> InvCovBgaRW = Matrix<double,6,6>::Identity();
    InvCovBgaRW.topLeftCorner(3,3) = Matrix3d::Identity()/IMUData::getGyrBiasRW2();       // Gyroscope bias random walk, covariance INVERSE
    InvCovBgaRW.bottomRightCorner(3,3) = Matrix3d::Identity()/IMUData::getAccBiasRW2();   // Accelerometer bias random walk, covariance INVERSE

    for(list<KeyFrame*>::const_iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        KeyFrame* pKF1 = *lit;                      // Current KF, store the IMU pre-integration between previous-current
        KeyFrame* pKF0 = pKF1->GetPrevKeyFrame();   // Previous KF

        //g2o::EdgeNavState * e = new g2o::EdgeNavState();
        g2o::EdgeNavStateGw * e = new g2o::EdgeNavStateGw();
        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF0->mnId)));
        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF1->mnId)));
        e->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(gwId)));
        e->setMeasurement(pKF1->GetIMUPreInt());

        // Test log
        if(pKF1->GetIMUPreInt().getDeltaTime() < 1e-3)
        {
            cerr<<"IMU pre-integrator delta time between 2 KFs too small: "<<pKF1->GetIMUPreInt().getDeltaTime()<<endl;
            cerr<<"No EdgeNavState added"<<endl;
            continue;
        }

        Matrix9d CovPVR = pKF1->GetIMUPreInt().getCovPVPhi() /*+ 1e-20*Matrix9d::Identity()*/;  // Add 1e-20 in case computed cov is zero..
        Matrix<double,15,15> InfoMat = Matrix<double,15,15>::Identity();
        InfoMat.topLeftCorner(9,9) = CovPVR.inverse();
        InfoMat.bottomRightCorner(6,6) = InvCovBgaRW/pKF1->GetIMUPreInt().getDeltaTime();

        e->setInformation(InfoMat);

        //e->SetParams(GravityVec);

        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
        e->setRobustKernel(rk);
        rk->setDelta(thHuberNavState);

        optimizer.addEdge(e);
        vpEdgesNavState.push_back(e);

        if(lit == lLocalKeyFrames.begin())
        {
            // First KF in Local Window, link (fixed) pKFPrevLocal
            // Test log
            if(pKF0 != pKFPrevLocal) cerr<<"pKF0 != pKFPrevLocal for 1st KF in Local Window, id: "<<pKF0->mnId<<","<<pKFPrevLocal->mnId<<endl;
        }
        else
        {
            // KFs in Local Window, link another local KF
        }
    }

    // Set MapPoint vertices
    const int nExpectedSize = (lLocalKeyFrames.size()+lFixedCameras.size())*lLocalMapPoints.size();

    vector<g2o::EdgeNavStatePointXYZ*> vpEdgesMono;
    vpEdgesMono.reserve(nExpectedSize);

    vector<KeyFrame*> vpEdgeKFMono;
    vpEdgeKFMono.reserve(nExpectedSize);

    vector<MapPoint*> vpMapPointEdgeMono;
    vpMapPointEdgeMono.reserve(nExpectedSize);

    const float thHuberMono = sqrt(5.991);

    for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        MapPoint* pMP = *lit;
        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
        int id = pMP->mnId+maxKFid+1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);

        const map<KeyFrame*,size_t> observations = pMP->GetObservations();

        //Set edges
        // Edges between KeyFrame and MapPoint
        for(map<KeyFrame*,size_t>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFi = mit->first;

            if(!pKFi->isBad())
            {
                const cv::KeyPoint &kpUn = pKFi->mvKeysUn[mit->second];

                // Monocular observation
                if(pKFi->mvuRight[mit->second]<0)
                {
                    Eigen::Matrix<double,2,1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    g2o::EdgeNavStatePointXYZ* e = new g2o::EdgeNavStatePointXYZ();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(obs);
                    const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                    e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberMono);

                    e->SetParams(pKFi->fx,pKFi->fy,pKFi->cx,pKFi->cy,Rbc,Pbc);

                    optimizer.addEdge(e);
                    vpEdgesMono.push_back(e);
                    vpEdgeKFMono.push_back(pKFi);
                    vpMapPointEdgeMono.push_back(pMP);
                }
                else
                {
                    // Test log
                    cerr<<"Stereo not supported yet, why here?? check."<<endl;
                }
            }
        }
    }

    if(pbStopFlag)
        if(*pbStopFlag)
            return;

//    //Debug log
//    cout<<endl<<"first optimize: "<<endl;


    //std::cout<<std::fixed<<std::setprecision(3);
    // First try
    //optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(5);
    //std::cout.unsetf ( std::ios::showbase );                // deactivate showbase

    bool bDoMore= true;

    if(pbStopFlag)
        if(*pbStopFlag)
            bDoMore = false;

    if(bDoMore)
    {
    // Check inlier observations
    for(size_t i=0, iend=vpEdgesMono.size(); i<iend;i++)
    {
        g2o::EdgeNavStatePointXYZ* e = vpEdgesMono[i];
        MapPoint* pMP = vpMapPointEdgeMono[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>5.991/*15*/ || !e->isDepthPositive())
        {
            e->setLevel(1);
        }

        e->setRobustKernel(0);
    }

//    // Check inlier observations
//    cout<<"edge ns bad: ";
    for(size_t i=0, iend=vpEdgesNavState.size(); i<iend; i++)
    {
        //g2o::EdgeNavState* e = vpEdgesNavState[i];
        g2o::EdgeNavStateGw* e = vpEdgesNavState[i];
        if(e->chi2()>30.5779)
        {
            e->setLevel(1);
//            cout<<"edge "<<i<<", chi2 "<<e->chi2()<<". ";
        }
        e->setRobustKernel(0);
    }
//    cout<<endl;

//    //Debug log
//    cout<<endl<<"second optimize: "<<endl;

    //std::cout<<std::fixed<<std::setprecision(3);
    // Optimize again without the outliers
    optimizer.initializeOptimization(0);
    optimizer.optimize(10);
    //std::cout.unsetf ( std::ios::showbase );                // deactivate showbase

    }

    //
    vector<pair<KeyFrame*,MapPoint*> > vToErase;
    vToErase.reserve(vpEdgesMono.size());

    double PosePointchi2=0, PosePosechi2=0;
    // Check inlier observations
    for(size_t i=0, iend=vpEdgesMono.size(); i<iend;i++)
    {
        g2o::EdgeNavStatePointXYZ* e = vpEdgesMono[i];
        MapPoint* pMP = vpMapPointEdgeMono[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>5.991/*15*/ || !e->isDepthPositive())
        {
            KeyFrame* pKFi = vpEdgeKFMono[i];
            vToErase.push_back(make_pair(pKFi,pMP));
        }

        PosePointchi2 += e->chi2();
    }

    // Debug log
    //cout<<"edge navstate chi2: ";
    for(size_t i=0, iend=vpEdgesNavState.size(); i<iend; i++)
    {
        //g2o::EdgeNavState* e = vpEdgesNavState[i];
        g2o::EdgeNavStateGw* e = vpEdgesNavState[i];
        //cout<<e->chi2()<<", ";
        PosePosechi2 += e->chi2();
        //cout<<e->error().transpose()<<endl;
    }
    //cout<<endl;
    //cout<<"pose-point chi2: "<<PosePointchi2<<", pose-pose chi2: "<<PosePosechi2<<endl;

    // Get Map Mutex
    unique_lock<mutex> lock(pMap->mMutexMapUpdate);

    if(!vToErase.empty())
    {
        for(size_t i=0;i<vToErase.size();i++)
        {
            KeyFrame* pKFi = vToErase[i].first;
            MapPoint* pMPi = vToErase[i].second;
            pKFi->EraseMapPointMatch(pMPi);
            pMPi->EraseObservation(pKFi);
        }
    }

    // Recover optimized data

    //Gravity vector
    //g2o::VertexGravityW* vGwopt = static_cast<g2o::VertexGravityW*>(optimizer.vertex(gwId));
    //Vector3d gwopt = vGwopt->estimate();
    //gw = Converter::toCvMat(gwopt);
    //    //Debug log
    //    cout<<"gw, before opt:"<<GravityVec.transpose()<<endl;
    //    cout<<"gw, after  opt:"<<gwopt.transpose()<<endl;

    //Keyframes
    for(list<KeyFrame*>::const_iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;
        g2o::VertexNavState* vNS = static_cast<g2o::VertexNavState*>(optimizer.vertex(pKFi->mnId));
        // In optimized navstate, bias not changed, delta_bias not zero, should be added to bias
        NavState optns = vNS->estimate();
        NavState primaryns = pKFi->GetNavState();
        // Update NavState
        pKFi->SetNavStatePos(optns.Get_P());
        pKFi->SetNavStateVel(optns.Get_V());
        pKFi->SetNavStateRot(optns.Get_R());
        //if(optns.Get_dBias_Acc().norm()<1e-2 && optns.Get_BiasGyr().norm()<1e-4)
        //{
        pKFi->SetNavStateDeltaBg(optns.Get_dBias_Gyr());
        pKFi->SetNavStateDeltaBa(optns.Get_dBias_Acc());
        //}

        //        // Initial bias keep fixed.
        //        pKFi->SetNavStateBiasGyr(primaryns.Get_BiasGyr() + optns.Get_dBias_Gyr());
        //        pKFi->SetNavStateBiasAcc(primaryns.Get_BiasAcc() + optns.Get_dBias_Acc());
        //        //cout<<"updated delta bias gyr: "<<optns.Get_dBias_Gyr().transpose()<<endl;
        //        //cout<<"updated delta bias acc: "<<optns.Get_dBias_Acc().transpose()<<endl;
        //        // Need to re-compute pre-integration with new bias
        //        pKFi->ComputePreInt();

        // Update pose Tcw
        pKFi->UpdatePoseFromNS(ConfigParam::GetMatTbc());
//        cv::Mat cvRwb = Converter::toCvMat(pKFi->GetNavState().Get_RotMatrix());
//        cv::Mat cvPwb = Converter::toCvMat(pKFi->GetNavState().Get_P());
//        cv::Mat cvRcw = cvRbc.t()*cvRwb.t();
//        cv::Mat cvPcw = -cvRcw*(cvRwb*cvPbc+cvPwb);
//        cv::Mat cvTcw = cv::Mat::eye(4,4,CV_32F);
//        cvRcw.copyTo(cvTcw.rowRange(0,3).colRange(0,3));
//        cvPcw.copyTo(cvTcw.rowRange(0,3).col(3));
//        pKFi->SetPose(cvTcw);

        // Test log
        if( (primaryns.Get_BiasGyr() - optns.Get_BiasGyr()).norm() > 1e-6 ) cerr<<"gyr bias change in optimization?"<<endl;
        if( (primaryns.Get_BiasAcc() - optns.Get_BiasAcc()).norm() > 1e-6 ) cerr<<"acc bias change in optimization?"<<endl;
        // Debug log
        //cout<<"updated delta bias gyr: "<<optns.Get_dBias_Gyr().transpose()<<endl;
        //cout<<"updated delta bias acc: "<<optns.Get_dBias_Acc().transpose()<<endl;
        //cout<<"before and after opt, navstate.P: "<<primaryns.Get_P().transpose()<<" vs "<<optns.Get_P().transpose()<<endl;
        //cout<<"before and after opt, navstate.V: "<<primaryns.Get_V().transpose()<<" vs "<<optns.Get_V().transpose()<<endl;
        //cout<<"before and after opt, navstate.R: "<<primaryns.Get_RotMatrix()<<" vs "<<optns.Get_RotMatrix()<<endl;

    }

    //Points
    for(list<MapPoint*>::const_iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        MapPoint* pMP = *lit;
        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mnId+maxKFid+1));
        pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
        pMP->UpdateNormalAndDepth();
    }

    if(pLM)
    {
        pLM->SetMapUpdateFlagInTracking(true);
    }

//    //Debug log
//    cout<<"current KeyFrame gyr bias+dbias: "<<(pCurKF->GetNavState().Get_BiasGyr()+pCurKF->GetNavState().Get_dBias_Gyr()).transpose()<<endl;
//    cout<<"current KeyFrame acc bias+dbias: "<<(pCurKF->GetNavState().Get_BiasAcc()+pCurKF->GetNavState().Get_dBias_Acc()).transpose()<<endl;
}

void Optimizer::GlobalBundleAdjustmentNavStateWithGw(Map* pMap, cv::Mat& gw, int nIterations, bool* pbStopFlag)
{
    // Extrinsics
    Matrix4d Tbc = ConfigParam::GetEigTbc();
    Matrix3d Rbc = Tbc.topLeftCorner(3,3);
    Vector3d Pbc = Tbc.topRightCorner(3,1);
    // Gravity vector in world frame
    Vector3d GravityVec = Converter::toVector3d(gw);

    vector<KeyFrame*> vpAllKFs = pMap->GetAllKeyFrames();
    vector<MapPoint*> vpAllMPs = pMap->GetAllMapPoints();

    vector<bool> vbNotIncludedMP;
    vbNotIncludedMP.resize(vpAllMPs.size());

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);

    //g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    if(pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    // gravity vector
    int gwId = KeyFrame::nNextId + MapPoint::nNextId + 1;
    g2o::VertexGravityW * vGw = new g2o::VertexGravityW();
    {
        vGw->setEstimate(GravityVec);
        vGw->setId(gwId);
        vGw->setFixed(false);
        optimizer.addVertex(vGw);
    }

    unsigned long maxKFid = 0;

    // Set KeyFrame vertices
    for(vector<KeyFrame*>::const_iterator vit=vpAllKFs.begin(), vend=vpAllKFs.end(); vit!=vend; vit++)
    {
        KeyFrame* pKFi = *vit;
        g2o::VertexNavState * vNS = new g2o::VertexNavState();
        vNS->setEstimate(pKFi->GetNavState());
        vNS->setId(pKFi->mnId);
        vNS->setFixed(pKFi->mnId==0);
        optimizer.addVertex(vNS);
        if(pKFi->mnId>maxKFid)
            maxKFid=pKFi->mnId;
    }

    // Edges between KeyFrames in Local Window
    // and
    // Edges between 1st KeyFrame of Local Window and its previous (fixed)KeyFrame - pKFPrevLocal
    //vector<g2o::EdgeNavState*> vpEdgesNavState;
    vector<g2o::EdgeNavStateGw*> vpEdgesNavState;
    // Use chi2inv() in MATLAB to compute the value corresponding to 0.95/0.99 prob. w.r.t 15DOF: 24.9958/30.5779
    //const float thHuberNavState = sqrt(30.5779);
    // Inverse covariance of bias random walk
    Matrix<double,6,6> InvCovBgaRW = Matrix<double,6,6>::Identity();
    InvCovBgaRW.topLeftCorner(3,3) = Matrix3d::Identity()/IMUData::getGyrBiasRW2();       // Gyroscope bias random walk, covariance INVERSE
    InvCovBgaRW.bottomRightCorner(3,3) = Matrix3d::Identity()/IMUData::getAccBiasRW2();   // Accelerometer bias random walk, covariance INVERSE
    for(vector<KeyFrame*>::const_iterator vit=vpAllKFs.begin(), vend=vpAllKFs.end(); vit!=vend; vit++)
    {
        KeyFrame* pKF1 = *vit;                      // Current KF, store the IMU pre-integration between previous-current
        if(pKF1->mnId==0)
            continue;

        KeyFrame* pKF0 = pKF1->GetPrevKeyFrame();   // Previous KF
        // Test log
        if(!pKF0) cerr<<"pKF0 is NULL in global BA"<<endl;
        if(!pKF1) cerr<<"pKF1 is NULL in global BA"<<endl;
        if(pKF0->isBad() || pKF1->isBad()) cerr<<"pKF0/pKF1 is bad? "<<(int)pKF0->isBad()<<"/"<<(int)pKF1->isBad()<<endl;
        if(pKF1->GetIMUPreInt().getDeltaTime() < 1e-3)
        {
            cerr<<"IMU pre-integrator delta time between 2 KFs too small: "<<pKF1->GetIMUPreInt().getDeltaTime()<<endl;
            cerr<<"No EdgeNavState added"<<endl;
            continue;
        }

        //g2o::EdgeNavState * e = new g2o::EdgeNavState();
        g2o::EdgeNavStateGw * e = new g2o::EdgeNavStateGw();
        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF0->mnId)));
        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF1->mnId)));
        e->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(gwId)));
        e->setMeasurement(pKF1->GetIMUPreInt());

        Matrix9d CovPVR = pKF1->GetIMUPreInt().getCovPVPhi() /*+ 1e-20*Matrix9d::Identity()*/;  // Add 1e-20 in case computed cov is zero..
        Matrix<double,15,15> InfoMat = Matrix<double,15,15>::Identity();
        InfoMat.topLeftCorner(9,9) = CovPVR.inverse();
        InfoMat.bottomRightCorner(6,6) = InvCovBgaRW/pKF1->GetIMUPreInt().getDeltaTime();

        e->setInformation(InfoMat);

        //e->SetParams(GravityVec);

//        if(bRobust)
//        {
//            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
//            e->setRobustKernel(rk);
//            rk->setDelta(thHuberNavState);
//        }

        optimizer.addEdge(e);
        vpEdgesNavState.push_back(e);
    }

    // Set MapPoint vertices
    for(size_t i=0; i<vpAllMPs.size(); i++)
    {
        MapPoint* pMP = vpAllMPs[i];
        if(pMP->isBad())
            continue;
        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
        const int id = pMP->mnId+maxKFid+1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);

       const map<KeyFrame*,size_t> observations = pMP->GetObservations();

        int nEdges = 0;
        //SET EDGES
        for(map<KeyFrame*,size_t>::const_iterator mit=observations.begin(); mit!=observations.end(); mit++)
        {

            KeyFrame* pKFi = mit->first;
            if(pKFi->isBad() || pKFi->mnId>maxKFid)
                continue;

            nEdges++;

            const cv::KeyPoint &kpUn = pKFi->mvKeysUn[mit->second];

            // Monocular observation
            if(pKFi->mvuRight[mit->second]<0)
            {
                Eigen::Matrix<double,2,1> obs;
                obs << kpUn.pt.x, kpUn.pt.y;

                g2o::EdgeNavStatePointXYZ* e = new g2o::EdgeNavStatePointXYZ();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                e->setMeasurement(obs);
                const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

//                if(bRobust)
//                {
//                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
//                    e->setRobustKernel(rk);
//                    rk->setDelta(thHuberMono);
//                }

                e->SetParams(pKFi->fx,pKFi->fy,pKFi->cx,pKFi->cy,Rbc,Pbc);

                optimizer.addEdge(e);
            }
            else
            {
                // Test log
                cerr<<"Stereo not supported yet, why here?? check."<<endl;
            }
        }

        if(nEdges==0)
        {
            optimizer.removeVertex(vPoint);
            vbNotIncludedMP[i]=true;
        }
        else
        {
            vbNotIncludedMP[i]=false;
        }
    }

    // Optimize!
    optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(nIterations);

    // Recover optimized data

    //Gravity vector
    //g2o::VertexGravityW* vGwopt = static_cast<g2o::VertexGravityW*>(optimizer.vertex(gwId));
    //Vector3d gwopt = vGwopt->estimate();
    //gw = Converter::toCvMat(gwopt);
    //    //Debug log
    //    cout<<"gw, before opt:"<<GravityVec.transpose()<<endl;
    //    cout<<"gw, after  opt:"<<gwopt.transpose()<<endl;

    cv::Mat cvTbc = ConfigParam::GetMatTbc();
    cv::Mat cvRbc = cvTbc.rowRange(0,3).colRange(0,3);
    cv::Mat cvPbc = cvTbc.rowRange(0,3).col(3);
    //Keyframes
    for(vector<KeyFrame*>::const_iterator vit=vpAllKFs.begin(), vend=vpAllKFs.end(); vit!=vend; vit++)
    {
        KeyFrame* pKF = *vit;
        g2o::VertexNavState* vNS = static_cast<g2o::VertexNavState*>(optimizer.vertex(pKF->mnId));
        // In optimized navstate, bias not changed, delta_bias not zero, should be added to bias
        NavState optns = vNS->estimate();
        NavState primaryns = pKF->GetNavState();

        // Update NavState
        pKF->SetNavStatePos(optns.Get_P());
        pKF->SetNavStateVel(optns.Get_V());
        pKF->SetNavStateRot(optns.Get_R());
        pKF->SetNavStateDeltaBg(optns.Get_dBias_Gyr());
        pKF->SetNavStateDeltaBa(optns.Get_dBias_Acc());
        // Initial bias keep fixed.
        //pKF->SetNavStateBiasGyr(primaryns.Get_BiasGyr() + optns.Get_dBias_Gyr());
        //pKF->SetNavStateBiasAcc(primaryns.Get_BiasAcc() + optns.Get_dBias_Acc());

        //// Need to re-compute pre-integration with new bias
        //pKF->ComputePreInt();

        // Update pose Tcw
        cv::Mat cvRwb = Converter::toCvMat(pKF->GetNavState().Get_RotMatrix());
        cv::Mat cvPwb = Converter::toCvMat(pKF->GetNavState().Get_P());
        cv::Mat cvRcw = cvRbc.t()*cvRwb.t();
        cv::Mat cvPcw = -cvRcw*(cvRwb*cvPbc+cvPwb);
        cv::Mat cvTcw = cv::Mat::eye(4,4,CV_32F);
        cvRcw.copyTo(cvTcw.rowRange(0,3).colRange(0,3));
        cvPcw.copyTo(cvTcw.rowRange(0,3).col(3));
        pKF->SetPose(cvTcw);

        // Test log
        if( (primaryns.Get_BiasGyr() - optns.Get_BiasGyr()).norm() > 1e-6 ) cerr<<"gyr bias change in optimization?"<<endl;
        if( (primaryns.Get_BiasAcc() - optns.Get_BiasAcc()).norm() > 1e-6 ) cerr<<"acc bias change in optimization?"<<endl;
    }

    //Points
    for(size_t i=0; i<vpAllMPs.size(); i++)
    {
        if(vbNotIncludedMP[i])
            continue;
        MapPoint* pMP = vpAllMPs[i];
        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mnId+maxKFid+1));
        pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
        pMP->UpdateNormalAndDepth();
    }

//    // Recover optimized data

//    //Keyframes
//    for(size_t i=0; i<vpKFs.size(); i++)
//    {
//        KeyFrame* pKF = vpKFs[i];
//        if(pKF->isBad())
//            continue;
//        g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKF->mnId));
//        g2o::SE3Quat SE3quat = vSE3->estimate();
//        if(nLoopKF==0)
//        {
//            pKF->SetPose(Converter::toCvMat(SE3quat));
//        }
//        else
//        {
//            pKF->mTcwGBA.create(4,4,CV_32F);
//            Converter::toCvMat(SE3quat).copyTo(pKF->mTcwGBA);
//            pKF->mnBAGlobalForKF = nLoopKF;
//        }
//    }

//    //Points
//    for(size_t i=0; i<vpMP.size(); i++)
//    {
//        if(vbNotIncludedMP[i])
//            continue;

//        MapPoint* pMP = vpMP[i];

//        if(pMP->isBad())
//            continue;
//        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mnId+maxKFid+1));

//        if(nLoopKF==0)
//        {
//            pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
//            pMP->UpdateNormalAndDepth();
//        }
//        else
//        {
//            pMP->mPosGBA.create(3,1,CV_32F);
//            Converter::toCvMat(vPoint->estimate()).copyTo(pMP->mPosGBA);
//            pMP->mnBAGlobalForKF = nLoopKF;
//        }
//    }
}


Vector3d Optimizer::OptimizeInitialGyroBias(const std::vector<Frame> &vFrames)
{
    //size_t N = vpKFs.size();
    Matrix4d Tbc = ConfigParam::GetEigTbc();
    Matrix3d Rcb = Tbc.topLeftCorner(3,3).transpose();

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
    //g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    // Add vertex of gyro bias, to optimizer graph
    g2o::VertexGyrBias * vBiasg = new g2o::VertexGyrBias();
    vBiasg->setEstimate(Eigen::Vector3d::Zero());
    vBiasg->setId(0);
    optimizer.addVertex(vBiasg);

    // Add unary edges for gyro bias vertex
    for(size_t i=0; i<vFrames.size(); i++)
    {
        // Only 19 edges between 20 Frames
        if(i==0)
            continue;

        const Frame& Fi = vFrames[i-1];
        const Frame& Fj = vFrames[i];

        cv::Mat Tiw = Fi.mTcw;      // pose of previous KF
        Eigen::Matrix3d Rwci = Converter::toMatrix3d(Tiw.rowRange(0,3).colRange(0,3).t());
        cv::Mat Tjw = Fj.mTcw;      // pose of this KF
        Eigen::Matrix3d Rwcj = Converter::toMatrix3d(Tjw.rowRange(0,3).colRange(0,3).t());

        //
        IMUPreintegrator imupreint;
        Fj.ComputeIMUPreIntSinceLastFrame(&Fi,imupreint);

        g2o::EdgeGyrBias * eBiasg = new g2o::EdgeGyrBias();
        eBiasg->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
        // measurement is not used in EdgeGyrBias
        eBiasg->dRbij = imupreint.getDeltaR();
        eBiasg->J_dR_bg = imupreint.getJRBiasg();
        eBiasg->Rwbi = Rwci*Rcb;
        eBiasg->Rwbj = Rwcj*Rcb;
        eBiasg->setInformation(Eigen::Matrix3d::Identity());
        optimizer.addEdge(eBiasg);
    }

    // It's actualy a linear estimator, so 1 iteration is enough.
    //optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(1);

    g2o::VertexGyrBias * vBgEst = static_cast<g2o::VertexGyrBias*>(optimizer.vertex(0));

    return vBgEst->estimate();
}

Vector3d Optimizer::OptimizeInitialGyroBias(const std::list<KeyFrame *> &lLocalKeyFrames)
{
    return OptimizeInitialGyroBias(std::vector<KeyFrame*>(lLocalKeyFrames.begin(),lLocalKeyFrames.end()));
}

Vector3d Optimizer::OptimizeInitialGyroBias(const std::vector<KeyFrame *> &vpKFs)
{
    //size_t N = vpKFs.size();
    Matrix4d Tbc = ConfigParam::GetEigTbc();
    Matrix3d Rcb = Tbc.topLeftCorner(3,3).transpose();

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
    //g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    // Add vertex of gyro bias, to optimizer graph
    g2o::VertexGyrBias * vBiasg = new g2o::VertexGyrBias();
    vBiasg->setEstimate(Eigen::Vector3d::Zero());
    vBiasg->setId(0);
    optimizer.addVertex(vBiasg);

    // Add unary edges for gyro bias vertex
    KeyFrame* pPrevKF0 = vpKFs.front();
    for(std::vector<KeyFrame*>::const_iterator lit=vpKFs.begin(), lend=vpKFs.end(); lit!=lend; lit++)
    {
        KeyFrame* pKF = *lit;
        // Ignore the first KF
        if(pKF == vpKFs.front())
            continue;

        KeyFrame* pPrevKF = pKF->GetPrevKeyFrame();
        cv::Mat Twi = pPrevKF->GetPoseInverse();    // pose of previous KF
        Eigen::Matrix3d Rwci = Converter::toMatrix3d(Twi.rowRange(0,3).colRange(0,3));
        cv::Mat Twj = pKF->GetPoseInverse();        // pose of this KF
        Eigen::Matrix3d Rwcj = Converter::toMatrix3d(Twj.rowRange(0,3).colRange(0,3));

        //
        const IMUPreintegrator& imupreint = pKF->GetIMUPreInt();
        g2o::EdgeGyrBias * eBiasg = new g2o::EdgeGyrBias();
        eBiasg->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
        // measurement is not used in EdgeGyrBias
        eBiasg->dRbij = imupreint.getDeltaR();
        eBiasg->J_dR_bg = imupreint.getJRBiasg();
        eBiasg->Rwbi = Rwci*Rcb;
        eBiasg->Rwbj = Rwcj*Rcb;
        eBiasg->setInformation(Eigen::Matrix3d::Identity());
        optimizer.addEdge(eBiasg);

        // Test log
        if(pPrevKF0 != pPrevKF) cerr<<"pPrevKF in list != pKF->pPrevKF? in OptimizeInitialGyroBias"<<endl;
        pPrevKF0 = pKF;

        // Debug log
        //cout<<"dRbij in pre-int: "<<endl<<eBiasg->dRbij<<endl;
        //cout<<"Rwbi'*Rwbj by ORBSLAM: "<<endl<<eBiasg->Rwbi.transpose()*eBiasg->Rwbj<<endl;
    }

    // It's actualy a linear estimator, so 1 iteration is enough.
    //optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(1);

    g2o::VertexGyrBias * vBgEst = static_cast<g2o::VertexGyrBias*>(optimizer.vertex(0));

    return vBgEst->estimate();
}

Vector3d Optimizer::OptimizeInitialGyroBias(const vector<cv::Mat>& vTwc, const vector<IMUPreintegrator>& vImuPreInt)
{
    int N = vTwc.size(); if(vTwc.size()!=vImuPreInt.size()) cerr<<"vTwc.size()!=vImuPreInt.size()"<<endl;
    Matrix4d Tbc = ConfigParam::GetEigTbc();
    Matrix3d Rcb = Tbc.topLeftCorner(3,3).transpose();

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
    //g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    // Add vertex of gyro bias, to optimizer graph
    g2o::VertexGyrBias * vBiasg = new g2o::VertexGyrBias();
    vBiasg->setEstimate(Eigen::Vector3d::Zero());
    vBiasg->setId(0);
    optimizer.addVertex(vBiasg);

    // Add unary edges for gyro bias vertex
    //for(std::vector<KeyFrame*>::const_iterator lit=vpKFs.begin(), lend=vpKFs.end(); lit!=lend; lit++)
    for(int i=0; i<N; i++)
    {
        // Ignore the first KF
        if(i==0)
            continue;

        const cv::Mat& Twi = vTwc[i-1];    // pose of previous KF
        Matrix3d Rwci = Converter::toMatrix3d(Twi.rowRange(0,3).colRange(0,3));
        //Matrix3d Rwci = Twi.rotation_matrix();
        const cv::Mat& Twj = vTwc[i];        // pose of this KF
        Matrix3d Rwcj = Converter::toMatrix3d(Twj.rowRange(0,3).colRange(0,3));
        //Matrix3d Rwcj =Twj.rotation_matrix();

        const IMUPreintegrator& imupreint = vImuPreInt[i];

        g2o::EdgeGyrBias * eBiasg = new g2o::EdgeGyrBias();
        eBiasg->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
        // measurement is not used in EdgeGyrBias
        eBiasg->dRbij = imupreint.getDeltaR();
        eBiasg->J_dR_bg = imupreint.getJRBiasg();
        eBiasg->Rwbi = Rwci*Rcb;
        eBiasg->Rwbj = Rwcj*Rcb;
        //eBiasg->setInformation(Eigen::Matrix3d::Identity());
        eBiasg->setInformation(imupreint.getCovPVPhi().bottomRightCorner(3,3));
        optimizer.addEdge(eBiasg);
    }

    // It's actualy a linear estimator, so 1 iteration is enough.
    //optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(1);

    g2o::VertexGyrBias * vBgEst = static_cast<g2o::VertexGyrBias*>(optimizer.vertex(0));

    return vBgEst->estimate();
}


void Optimizer::LocalBundleAdjustment(KeyFrame *pKF, const std::list<KeyFrame*> &lLocalKeyFrames, bool* pbStopFlag, Map* pMap, LocalMapping* pLM)
{
    // Check current KeyFrame in local window
    if(pKF != lLocalKeyFrames.back())
        cerr<<"pKF != lLocalKeyFrames.back. check"<<endl;


    for(list<KeyFrame*>::const_iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;
        pKFi->mnBALocalForKF = pKF->mnId;
    }

    // Local MapPoints seen in Local KeyFrames
    list<MapPoint*> lLocalMapPoints;
    for(list<KeyFrame*>::const_iterator lit=lLocalKeyFrames.begin() , lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        vector<MapPoint*> vpMPs = (*lit)->GetMapPointMatches();
        for(vector<MapPoint*>::iterator vit=vpMPs.begin(), vend=vpMPs.end(); vit!=vend; vit++)
        {
            MapPoint* pMP = *vit;
            if(pMP)
                if(!pMP->isBad())
                    if(pMP->mnBALocalForKF!=pKF->mnId)
                    {
                        lLocalMapPoints.push_back(pMP);
                        pMP->mnBALocalForKF=pKF->mnId;
                    }
        }
    }

    // Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local Keyframes
    list<KeyFrame*> lFixedCameras;
    // Add the KeyFrame before local window
    KeyFrame* pKFPrevLocal = lLocalKeyFrames.front()->GetPrevKeyFrame();
    if(pKFPrevLocal)
    {
        // Test log
        if(pKFPrevLocal->isBad()) cerr<<"KeyFrame before local window is bad?"<<endl;
        if(pKFPrevLocal->mnBAFixedForKF==pKF->mnId) cerr<<"KeyFrame before local, has been added to lFixedKF?"<<endl;
        if(pKFPrevLocal->mnBALocalForKF==pKF->mnId) cerr<<"KeyFrame before local, has been added to lLocalKF?"<<endl;

        pKFPrevLocal->mnBAFixedForKF = pKF->mnId;
        if(!pKFPrevLocal->isBad())
            lFixedCameras.push_back(pKFPrevLocal);
        else
            cerr<<"pKFPrevLocal is Bad?"<<endl;
    }
    // Covisible KeyFrames
    for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        map<KeyFrame*,size_t> observations = (*lit)->GetObservations();
        for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFi = mit->first;

            if(pKFi->mnBALocalForKF!=pKF->mnId && pKFi->mnBAFixedForKF!=pKF->mnId)
            {
                pKFi->mnBAFixedForKF=pKF->mnId;
                if(!pKFi->isBad())
                    lFixedCameras.push_back(pKFi);
            }
        }
    }

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    if(pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    unsigned long maxKFid = 0;

    // Set Local KeyFrame vertices
    for(list<KeyFrame*>::const_iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
        vSE3->setId(pKFi->mnId);
        vSE3->setFixed(pKFi->mnId==0);
        optimizer.addVertex(vSE3);
        if(pKFi->mnId>maxKFid)
            maxKFid=pKFi->mnId;
    }

    // Set Fixed KeyFrame vertices
    for(list<KeyFrame*>::iterator lit=lFixedCameras.begin(), lend=lFixedCameras.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
        vSE3->setId(pKFi->mnId);
        vSE3->setFixed(true);
        optimizer.addVertex(vSE3);
        if(pKFi->mnId>maxKFid)
            maxKFid=pKFi->mnId;
    }

    // Set MapPoint vertices
    const int nExpectedSize = (lLocalKeyFrames.size()+lFixedCameras.size())*lLocalMapPoints.size();

    vector<g2o::EdgeSE3ProjectXYZ*> vpEdgesMono;
    vpEdgesMono.reserve(nExpectedSize);

    vector<KeyFrame*> vpEdgeKFMono;
    vpEdgeKFMono.reserve(nExpectedSize);

    vector<MapPoint*> vpMapPointEdgeMono;
    vpMapPointEdgeMono.reserve(nExpectedSize);

    vector<g2o::EdgeStereoSE3ProjectXYZ*> vpEdgesStereo;
    vpEdgesStereo.reserve(nExpectedSize);

    vector<KeyFrame*> vpEdgeKFStereo;
    vpEdgeKFStereo.reserve(nExpectedSize);

    vector<MapPoint*> vpMapPointEdgeStereo;
    vpMapPointEdgeStereo.reserve(nExpectedSize);

    const float thHuberMono = sqrt(5.991);
    const float thHuberStereo = sqrt(7.815);

    for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        MapPoint* pMP = *lit;
        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
        int id = pMP->mnId+maxKFid+1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);

        const map<KeyFrame*,size_t> observations = pMP->GetObservations();

        //Set edges
        for(map<KeyFrame*,size_t>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFi = mit->first;

            if(!pKFi->isBad())
            {
                const cv::KeyPoint &kpUn = pKFi->mvKeysUn[mit->second];

                // Monocular observation
                if(pKFi->mvuRight[mit->second]<0)
                {
                    Eigen::Matrix<double,2,1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(obs);
                    const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                    e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberMono);

                    e->fx = pKFi->fx;
                    e->fy = pKFi->fy;
                    e->cx = pKFi->cx;
                    e->cy = pKFi->cy;

                    optimizer.addEdge(e);
                    vpEdgesMono.push_back(e);
                    vpEdgeKFMono.push_back(pKFi);
                    vpMapPointEdgeMono.push_back(pMP);
                }
                else // Stereo observation
                {
                    Eigen::Matrix<double,3,1> obs;
                    const float kp_ur = pKFi->mvuRight[mit->second];
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                    g2o::EdgeStereoSE3ProjectXYZ* e = new g2o::EdgeStereoSE3ProjectXYZ();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(obs);
                    const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                    Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*invSigma2;
                    e->setInformation(Info);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberStereo);

                    e->fx = pKFi->fx;
                    e->fy = pKFi->fy;
                    e->cx = pKFi->cx;
                    e->cy = pKFi->cy;
                    e->bf = pKFi->mbf;

                    optimizer.addEdge(e);
                    vpEdgesStereo.push_back(e);
                    vpEdgeKFStereo.push_back(pKFi);
                    vpMapPointEdgeStereo.push_back(pMP);
                }
            }
        }
    }

    if(pbStopFlag)
        if(*pbStopFlag)
            return;

    optimizer.initializeOptimization();
    optimizer.optimize(5);

    bool bDoMore= true;

    if(pbStopFlag)
        if(*pbStopFlag)
            bDoMore = false;

    if(bDoMore)
    {

    // Check inlier observations
    for(size_t i=0, iend=vpEdgesMono.size(); i<iend;i++)
    {
        g2o::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
        MapPoint* pMP = vpMapPointEdgeMono[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>5.991 || !e->isDepthPositive())
        {
            e->setLevel(1);
        }

        e->setRobustKernel(0);
    }

    for(size_t i=0, iend=vpEdgesStereo.size(); i<iend;i++)
    {
        g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
        MapPoint* pMP = vpMapPointEdgeStereo[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>7.815 || !e->isDepthPositive())
        {
            e->setLevel(1);
        }

        e->setRobustKernel(0);
    }

    // Optimize again without the outliers

    optimizer.initializeOptimization(0);
    optimizer.optimize(10);

    }

    vector<pair<KeyFrame*,MapPoint*> > vToErase;
    vToErase.reserve(vpEdgesMono.size()+vpEdgesStereo.size());

    // Check inlier observations
    for(size_t i=0, iend=vpEdgesMono.size(); i<iend;i++)
    {
        g2o::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
        MapPoint* pMP = vpMapPointEdgeMono[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>5.991 || !e->isDepthPositive())
        {
            KeyFrame* pKFi = vpEdgeKFMono[i];
            vToErase.push_back(make_pair(pKFi,pMP));
        }
    }

    for(size_t i=0, iend=vpEdgesStereo.size(); i<iend;i++)
    {
        g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
        MapPoint* pMP = vpMapPointEdgeStereo[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>7.815 || !e->isDepthPositive())
        {
            KeyFrame* pKFi = vpEdgeKFStereo[i];
            vToErase.push_back(make_pair(pKFi,pMP));
        }
    }

    // Get Map Mutex
    unique_lock<mutex> lock(pMap->mMutexMapUpdate);

    if(!vToErase.empty())
    {
        for(size_t i=0;i<vToErase.size();i++)
        {
            KeyFrame* pKFi = vToErase[i].first;
            MapPoint* pMPi = vToErase[i].second;
            pKFi->EraseMapPointMatch(pMPi);
            pMPi->EraseObservation(pKFi);
        }
    }

    // Recover optimized data

    //Keyframes
    for(list<KeyFrame*>::const_iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        KeyFrame* pKF = *lit;
        g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKF->mnId));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        pKF->SetPose(Converter::toCvMat(SE3quat));
    }

    //Points
    for(list<MapPoint*>::const_iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        MapPoint* pMP = *lit;
        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mnId+maxKFid+1));
        pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
        pMP->UpdateNormalAndDepth();
    }

    if(pLM)
    {
        pLM->SetMapUpdateFlagInTracking(true);
    }
}


//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------

void Optimizer::GlobalBundleAdjustemnt(Map* pMap, int nIterations, bool* pbStopFlag, const unsigned long nLoopKF, const bool bRobust)
{
    vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
    vector<MapPoint*> vpMP = pMap->GetAllMapPoints();
    BundleAdjustment(vpKFs,vpMP,nIterations,pbStopFlag, nLoopKF, bRobust);
}


void Optimizer::BundleAdjustment(const vector<KeyFrame *> &vpKFs, const vector<MapPoint *> &vpMP,
                                 int nIterations, bool* pbStopFlag, const unsigned long nLoopKF, const bool bRobust)
{
    vector<bool> vbNotIncludedMP;
    vbNotIncludedMP.resize(vpMP.size());

    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    if(pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    long unsigned int maxKFid = 0;

    // Set KeyFrame vertices
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];
        if(pKF->isBad())
            continue;
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKF->GetPose()));
        vSE3->setId(pKF->mnId);
        vSE3->setFixed(pKF->mnId==0);
        optimizer.addVertex(vSE3);
        if(pKF->mnId>maxKFid)
            maxKFid=pKF->mnId;
    }

    const float thHuber2D = sqrt(5.99);
    const float thHuber3D = sqrt(7.815);

    // Set MapPoint vertices
    for(size_t i=0; i<vpMP.size(); i++)
    {
        MapPoint* pMP = vpMP[i];
        if(pMP->isBad())
            continue;
        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
        const int id = pMP->mnId+maxKFid+1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);

       const map<KeyFrame*,size_t> observations = pMP->GetObservations();

        int nEdges = 0;
        //SET EDGES
        for(map<KeyFrame*,size_t>::const_iterator mit=observations.begin(); mit!=observations.end(); mit++)
        {

            KeyFrame* pKF = mit->first;
            if(pKF->isBad() || pKF->mnId>maxKFid)
                continue;

            nEdges++;

            const cv::KeyPoint &kpUn = pKF->mvKeysUn[mit->second];

            if(pKF->mvuRight[mit->second]<0)
            {
                Eigen::Matrix<double,2,1> obs;
                obs << kpUn.pt.x, kpUn.pt.y;

                g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->mnId)));
                e->setMeasurement(obs);
                const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
                e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                if(bRobust)
                {
                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuber2D);
                }

                e->fx = pKF->fx;
                e->fy = pKF->fy;
                e->cx = pKF->cx;
                e->cy = pKF->cy;

                optimizer.addEdge(e);
            }
            else
            {
                Eigen::Matrix<double,3,1> obs;
                const float kp_ur = pKF->mvuRight[mit->second];
                obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                g2o::EdgeStereoSE3ProjectXYZ* e = new g2o::EdgeStereoSE3ProjectXYZ();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->mnId)));
                e->setMeasurement(obs);
                const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
                Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*invSigma2;
                e->setInformation(Info);

                if(bRobust)
                {
                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuber3D);
                }

                e->fx = pKF->fx;
                e->fy = pKF->fy;
                e->cx = pKF->cx;
                e->cy = pKF->cy;
                e->bf = pKF->mbf;

                optimizer.addEdge(e);
            }
        }

        if(nEdges==0)
        {
            optimizer.removeVertex(vPoint);
            vbNotIncludedMP[i]=true;
        }
        else
        {
            vbNotIncludedMP[i]=false;
        }
    }

    // Optimize!
    optimizer.initializeOptimization();
    optimizer.optimize(nIterations);

    // Recover optimized data

    //Keyframes
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];
        if(pKF->isBad())
            continue;
        g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKF->mnId));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        if(nLoopKF==0)
        {
            pKF->SetPose(Converter::toCvMat(SE3quat));
        }
        else
        {
            pKF->mTcwGBA.create(4,4,CV_32F);
            Converter::toCvMat(SE3quat).copyTo(pKF->mTcwGBA);
            pKF->mnBAGlobalForKF = nLoopKF;
        }
    }

    //Points
    for(size_t i=0; i<vpMP.size(); i++)
    {
        if(vbNotIncludedMP[i])
            continue;

        MapPoint* pMP = vpMP[i];

        if(pMP->isBad())
            continue;
        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mnId+maxKFid+1));

        if(nLoopKF==0)
        {
            pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
            pMP->UpdateNormalAndDepth();
        }
        else
        {
            pMP->mPosGBA.create(3,1,CV_32F);
            Converter::toCvMat(vPoint->estimate()).copyTo(pMP->mPosGBA);
            pMP->mnBAGlobalForKF = nLoopKF;
        }
    }

}

int Optimizer::PoseOptimization(Frame *pFrame)
{
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    //linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();
    linearSolver = new g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    //g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
    optimizer.setAlgorithm(solver);

    int nInitialCorrespondences=0;

    // Set Frame vertex
    g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
    vSE3->setEstimate(Converter::toSE3Quat(pFrame->mTcw));
    vSE3->setId(0);
    vSE3->setFixed(false);
    optimizer.addVertex(vSE3);

    // Set MapPoint vertices
    const int N = pFrame->N;

    vector<g2o::EdgeSE3ProjectXYZOnlyPose*> vpEdgesMono;
    vector<size_t> vnIndexEdgeMono;
    vpEdgesMono.reserve(N);
    vnIndexEdgeMono.reserve(N);

    vector<g2o::EdgeStereoSE3ProjectXYZOnlyPose*> vpEdgesStereo;
    vector<size_t> vnIndexEdgeStereo;
    vpEdgesStereo.reserve(N);
    vnIndexEdgeStereo.reserve(N);

    const float deltaMono = sqrt(5.991);
    const float deltaStereo = sqrt(7.815);


    {
    unique_lock<mutex> lock(MapPoint::mGlobalMutex);

    for(int i=0; i<N; i++)
    {
        MapPoint* pMP = pFrame->mvpMapPoints[i];
        if(pMP)
        {
            // Monocular observation
            if(pFrame->mvuRight[i]<0)
            {
                nInitialCorrespondences++;
                pFrame->mvbOutlier[i] = false;

                Eigen::Matrix<double,2,1> obs;
                const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
                obs << kpUn.pt.x, kpUn.pt.y;

                g2o::EdgeSE3ProjectXYZOnlyPose* e = new g2o::EdgeSE3ProjectXYZOnlyPose();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
                e->setMeasurement(obs);
                const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
                e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                e->setRobustKernel(rk);
                rk->setDelta(deltaMono);

                e->fx = pFrame->fx;
                e->fy = pFrame->fy;
                e->cx = pFrame->cx;
                e->cy = pFrame->cy;
                cv::Mat Xw = pMP->GetWorldPos();
                e->Xw[0] = Xw.at<float>(0);
                e->Xw[1] = Xw.at<float>(1);
                e->Xw[2] = Xw.at<float>(2);

                optimizer.addEdge(e);

                vpEdgesMono.push_back(e);
                vnIndexEdgeMono.push_back(i);
            }
            else  // Stereo observation
            {
                nInitialCorrespondences++;
                pFrame->mvbOutlier[i] = false;

                //SET EDGE
                Eigen::Matrix<double,3,1> obs;
                const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
                const float &kp_ur = pFrame->mvuRight[i];
                obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                g2o::EdgeStereoSE3ProjectXYZOnlyPose* e = new g2o::EdgeStereoSE3ProjectXYZOnlyPose();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
                e->setMeasurement(obs);
                const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
                Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*invSigma2;
                e->setInformation(Info);

                g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                e->setRobustKernel(rk);
                rk->setDelta(deltaStereo);

                e->fx = pFrame->fx;
                e->fy = pFrame->fy;
                e->cx = pFrame->cx;
                e->cy = pFrame->cy;
                e->bf = pFrame->mbf;
                cv::Mat Xw = pMP->GetWorldPos();
                e->Xw[0] = Xw.at<float>(0);
                e->Xw[1] = Xw.at<float>(1);
                e->Xw[2] = Xw.at<float>(2);

                optimizer.addEdge(e);

                vpEdgesStereo.push_back(e);
                vnIndexEdgeStereo.push_back(i);
            }
        }

    }
    }


    if(nInitialCorrespondences<3)
        return 0;

    // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
    // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
    const float chi2Mono[4]={5.991,5.991,5.991,5.991};
    const float chi2Stereo[4]={7.815,7.815,7.815, 7.815};
    const int its[4]={10,10,10,10};    

    int nBad=0;
    for(size_t it=0; it<4; it++)
    {

        vSE3->setEstimate(Converter::toSE3Quat(pFrame->mTcw));
        optimizer.initializeOptimization(0);
        optimizer.optimize(its[it]);

        nBad=0;
        for(size_t i=0, iend=vpEdgesMono.size(); i<iend; i++)
        {
            g2o::EdgeSE3ProjectXYZOnlyPose* e = vpEdgesMono[i];

            const size_t idx = vnIndexEdgeMono[i];

            if(pFrame->mvbOutlier[idx])
            {
                e->computeError();
            }

            const float chi2 = e->chi2();

            if(chi2>chi2Mono[it])
            {                
                pFrame->mvbOutlier[idx]=true;
                e->setLevel(1);
                nBad++;
            }
            else
            {
                pFrame->mvbOutlier[idx]=false;
                e->setLevel(0);
            }

            if(it==2)
                e->setRobustKernel(0);
        }

        for(size_t i=0, iend=vpEdgesStereo.size(); i<iend; i++)
        {
            g2o::EdgeStereoSE3ProjectXYZOnlyPose* e = vpEdgesStereo[i];

            const size_t idx = vnIndexEdgeStereo[i];

            if(pFrame->mvbOutlier[idx])
            {
                e->computeError();
            }

            const float chi2 = e->chi2();

            if(chi2>chi2Stereo[it])
            {
                pFrame->mvbOutlier[idx]=true;
                e->setLevel(1);
                nBad++;
            }
            else
            {                
                e->setLevel(0);
                pFrame->mvbOutlier[idx]=false;
            }

            if(it==2)
                e->setRobustKernel(0);
        }

        if(optimizer.edges().size()<10)
            break;
    }    

    // Recover optimized pose and return number of inliers
    g2o::VertexSE3Expmap* vSE3_recov = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
    g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
    cv::Mat pose = Converter::toCvMat(SE3quat_recov);
    pFrame->SetPose(pose);

//    //For Test
//    g2o::SparseBlockMatrixXd spinv;
//    bool flag = optimizer.computeMarginals(spinv,optimizer.vertex(0));
//    cout<<"spinv: "<<spinv<<endl;
//    if(!flag) cerr<<"computeMarginals failed. vertex hessian id: "<<optimizer.vertex(0)->hessianIndex()<<endl;

//    //For Test
//    static_cast<g2o::OptimizationAlgorithmGaussNewton*>(optimizer.solver())->solver()->vectorSize();
//    cout<<"vector size: "<<solver_ptr->vectorSize()<<endl;
//    MatrixXd Hpp = *(solver_ptr->getHpp()->block(0,0));
//    cout<<"Hpp: "<<Hpp<<endl;
//    cout<<"Hpp.inverse: "<<Hpp.inverse()<<endl;
//    Eigen::Map<VectorXd> bvec(solver_ptr->b(), solver_ptr->vectorSize());
//    cout<<"b: "<<bvec.transpose()<<endl;

    return nInitialCorrespondences-nBad;
}

void Optimizer::LocalBundleAdjustment(KeyFrame *pKF, bool* pbStopFlag, Map* pMap, LocalMapping* pLM)
{    
    // Local KeyFrames: First Breath Search from Current Keyframe
    list<KeyFrame*> lLocalKeyFrames;

    lLocalKeyFrames.push_back(pKF);
    pKF->mnBALocalForKF = pKF->mnId;

    const vector<KeyFrame*> vNeighKFs = pKF->GetVectorCovisibleKeyFrames();
    for(int i=0, iend=vNeighKFs.size(); i<iend; i++)
    {
        KeyFrame* pKFi = vNeighKFs[i];
        pKFi->mnBALocalForKF = pKF->mnId;
        if(!pKFi->isBad())
            lLocalKeyFrames.push_back(pKFi);
    }

    // Local MapPoints seen in Local KeyFrames
    list<MapPoint*> lLocalMapPoints;
    for(list<KeyFrame*>::iterator lit=lLocalKeyFrames.begin() , lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        vector<MapPoint*> vpMPs = (*lit)->GetMapPointMatches();
        for(vector<MapPoint*>::iterator vit=vpMPs.begin(), vend=vpMPs.end(); vit!=vend; vit++)
        {
            MapPoint* pMP = *vit;
            if(pMP)
                if(!pMP->isBad())
                    if(pMP->mnBALocalForKF!=pKF->mnId)
                    {
                        lLocalMapPoints.push_back(pMP);
                        pMP->mnBALocalForKF=pKF->mnId;
                    }
        }
    }

    // Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local Keyframes
    list<KeyFrame*> lFixedCameras;
    for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        map<KeyFrame*,size_t> observations = (*lit)->GetObservations();
        for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFi = mit->first;

            if(pKFi->mnBALocalForKF!=pKF->mnId && pKFi->mnBAFixedForKF!=pKF->mnId)
            {                
                pKFi->mnBAFixedForKF=pKF->mnId;
                if(!pKFi->isBad())
                    lFixedCameras.push_back(pKFi);
            }
        }
    }

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    if(pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    unsigned long maxKFid = 0;

    // Set Local KeyFrame vertices
    for(list<KeyFrame*>::iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
        vSE3->setId(pKFi->mnId);
        vSE3->setFixed(pKFi->mnId==0);
        optimizer.addVertex(vSE3);
        if(pKFi->mnId>maxKFid)
            maxKFid=pKFi->mnId;
    }

    // Set Fixed KeyFrame vertices
    for(list<KeyFrame*>::iterator lit=lFixedCameras.begin(), lend=lFixedCameras.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
        vSE3->setId(pKFi->mnId);
        vSE3->setFixed(true);
        optimizer.addVertex(vSE3);
        if(pKFi->mnId>maxKFid)
            maxKFid=pKFi->mnId;
    }

    // Set MapPoint vertices
    const int nExpectedSize = (lLocalKeyFrames.size()+lFixedCameras.size())*lLocalMapPoints.size();

    vector<g2o::EdgeSE3ProjectXYZ*> vpEdgesMono;
    vpEdgesMono.reserve(nExpectedSize);

    vector<KeyFrame*> vpEdgeKFMono;
    vpEdgeKFMono.reserve(nExpectedSize);

    vector<MapPoint*> vpMapPointEdgeMono;
    vpMapPointEdgeMono.reserve(nExpectedSize);

    vector<g2o::EdgeStereoSE3ProjectXYZ*> vpEdgesStereo;
    vpEdgesStereo.reserve(nExpectedSize);

    vector<KeyFrame*> vpEdgeKFStereo;
    vpEdgeKFStereo.reserve(nExpectedSize);

    vector<MapPoint*> vpMapPointEdgeStereo;
    vpMapPointEdgeStereo.reserve(nExpectedSize);

    const float thHuberMono = sqrt(5.991);
    const float thHuberStereo = sqrt(7.815);

    for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        MapPoint* pMP = *lit;
        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
        int id = pMP->mnId+maxKFid+1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);

        const map<KeyFrame*,size_t> observations = pMP->GetObservations();

        //Set edges
        for(map<KeyFrame*,size_t>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFi = mit->first;

            if(!pKFi->isBad())
            {                
                const cv::KeyPoint &kpUn = pKFi->mvKeysUn[mit->second];

                // Monocular observation
                if(pKFi->mvuRight[mit->second]<0)
                {
                    Eigen::Matrix<double,2,1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(obs);
                    const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                    e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberMono);

                    e->fx = pKFi->fx;
                    e->fy = pKFi->fy;
                    e->cx = pKFi->cx;
                    e->cy = pKFi->cy;

                    optimizer.addEdge(e);
                    vpEdgesMono.push_back(e);
                    vpEdgeKFMono.push_back(pKFi);
                    vpMapPointEdgeMono.push_back(pMP);
                }
                else // Stereo observation
                {
                    Eigen::Matrix<double,3,1> obs;
                    const float kp_ur = pKFi->mvuRight[mit->second];
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                    g2o::EdgeStereoSE3ProjectXYZ* e = new g2o::EdgeStereoSE3ProjectXYZ();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(obs);
                    const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                    Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*invSigma2;
                    e->setInformation(Info);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberStereo);

                    e->fx = pKFi->fx;
                    e->fy = pKFi->fy;
                    e->cx = pKFi->cx;
                    e->cy = pKFi->cy;
                    e->bf = pKFi->mbf;

                    optimizer.addEdge(e);
                    vpEdgesStereo.push_back(e);
                    vpEdgeKFStereo.push_back(pKFi);
                    vpMapPointEdgeStereo.push_back(pMP);
                }
            }
        }
    }

    if(pbStopFlag)
        if(*pbStopFlag)
            return;

    optimizer.initializeOptimization();
    optimizer.optimize(5);

    bool bDoMore= true;

    if(pbStopFlag)
        if(*pbStopFlag)
            bDoMore = false;

    if(bDoMore)
    {

    // Check inlier observations
    for(size_t i=0, iend=vpEdgesMono.size(); i<iend;i++)
    {
        g2o::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
        MapPoint* pMP = vpMapPointEdgeMono[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>5.991 || !e->isDepthPositive())
        {
            e->setLevel(1);
        }

        e->setRobustKernel(0);
    }

    for(size_t i=0, iend=vpEdgesStereo.size(); i<iend;i++)
    {
        g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
        MapPoint* pMP = vpMapPointEdgeStereo[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>7.815 || !e->isDepthPositive())
        {
            e->setLevel(1);
        }

        e->setRobustKernel(0);
    }

    // Optimize again without the outliers

    optimizer.initializeOptimization(0);
    optimizer.optimize(10);

    }

    vector<pair<KeyFrame*,MapPoint*> > vToErase;
    vToErase.reserve(vpEdgesMono.size()+vpEdgesStereo.size());

    // Check inlier observations       
    for(size_t i=0, iend=vpEdgesMono.size(); i<iend;i++)
    {
        g2o::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
        MapPoint* pMP = vpMapPointEdgeMono[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>5.991 || !e->isDepthPositive())
        {
            KeyFrame* pKFi = vpEdgeKFMono[i];
            vToErase.push_back(make_pair(pKFi,pMP));
        }
    }

    for(size_t i=0, iend=vpEdgesStereo.size(); i<iend;i++)
    {
        g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
        MapPoint* pMP = vpMapPointEdgeStereo[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>7.815 || !e->isDepthPositive())
        {
            KeyFrame* pKFi = vpEdgeKFStereo[i];
            vToErase.push_back(make_pair(pKFi,pMP));
        }
    }

    // Get Map Mutex
    unique_lock<mutex> lock(pMap->mMutexMapUpdate);

    if(!vToErase.empty())
    {
        for(size_t i=0;i<vToErase.size();i++)
        {
            KeyFrame* pKFi = vToErase[i].first;
            MapPoint* pMPi = vToErase[i].second;
            pKFi->EraseMapPointMatch(pMPi);
            pMPi->EraseObservation(pKFi);
        }
    }

    // Recover optimized data

    //Keyframes
    for(list<KeyFrame*>::iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        KeyFrame* pKF = *lit;
        g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKF->mnId));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        pKF->SetPose(Converter::toCvMat(SE3quat));
    }

    //Points
    for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        MapPoint* pMP = *lit;
        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mnId+maxKFid+1));
        pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
        pMP->UpdateNormalAndDepth();
    }

    if(pLM)
    {
        pLM->SetMapUpdateFlagInTracking(true);
    }
}


void Optimizer::OptimizeEssentialGraph(Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF,
                                       const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                       const LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                       const map<KeyFrame *, set<KeyFrame *> > &LoopConnections, const bool &bFixScale,
                                       LoopClosing* pLC)
{
    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);
    g2o::BlockSolver_7_3::LinearSolverType * linearSolver =
           new g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>();
    g2o::BlockSolver_7_3 * solver_ptr= new g2o::BlockSolver_7_3(linearSolver);
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

    solver->setUserLambdaInit(1e-16);
    optimizer.setAlgorithm(solver);

    const vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
    const vector<MapPoint*> vpMPs = pMap->GetAllMapPoints();

    const unsigned int nMaxKFid = pMap->GetMaxKFid();

    vector<g2o::Sim3,Eigen::aligned_allocator<g2o::Sim3> > vScw(nMaxKFid+1);
    vector<g2o::Sim3,Eigen::aligned_allocator<g2o::Sim3> > vCorrectedSwc(nMaxKFid+1);
    vector<g2o::VertexSim3Expmap*> vpVertices(nMaxKFid+1);

    const int minFeat = 100;

    // Set KeyFrame vertices
    for(size_t i=0, iend=vpKFs.size(); i<iend;i++)
    {
        KeyFrame* pKF = vpKFs[i];
        if(pKF->isBad())
            continue;
        g2o::VertexSim3Expmap* VSim3 = new g2o::VertexSim3Expmap();

        const int nIDi = pKF->mnId;

        LoopClosing::KeyFrameAndPose::const_iterator it = CorrectedSim3.find(pKF);

        if(it!=CorrectedSim3.end())
        {
            vScw[nIDi] = it->second;
            VSim3->setEstimate(it->second);
        }
        else
        {
            Eigen::Matrix<double,3,3> Rcw = Converter::toMatrix3d(pKF->GetRotation());
            Eigen::Matrix<double,3,1> tcw = Converter::toVector3d(pKF->GetTranslation());
            g2o::Sim3 Siw(Rcw,tcw,1.0);
            vScw[nIDi] = Siw;
            VSim3->setEstimate(Siw);
        }

        if(pKF==pLoopKF)
            VSim3->setFixed(true);

        VSim3->setId(nIDi);
        VSim3->setMarginalized(false);
        VSim3->_fix_scale = bFixScale;

        optimizer.addVertex(VSim3);

        vpVertices[nIDi]=VSim3;
    }


    set<pair<long unsigned int,long unsigned int> > sInsertedEdges;

    const Eigen::Matrix<double,7,7> matLambda = Eigen::Matrix<double,7,7>::Identity();

    // Set Loop edges
    for(map<KeyFrame *, set<KeyFrame *> >::const_iterator mit = LoopConnections.begin(), mend=LoopConnections.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;
        const long unsigned int nIDi = pKF->mnId;
        const set<KeyFrame*> &spConnections = mit->second;
        const g2o::Sim3 Siw = vScw[nIDi];
        const g2o::Sim3 Swi = Siw.inverse();

        for(set<KeyFrame*>::const_iterator sit=spConnections.begin(), send=spConnections.end(); sit!=send; sit++)
        {
            const long unsigned int nIDj = (*sit)->mnId;
            if((nIDi!=pCurKF->mnId || nIDj!=pLoopKF->mnId) && pKF->GetWeight(*sit)<minFeat)
                continue;

            const g2o::Sim3 Sjw = vScw[nIDj];
            const g2o::Sim3 Sji = Sjw * Swi;

            g2o::EdgeSim3* e = new g2o::EdgeSim3();
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDj)));
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
            e->setMeasurement(Sji);

            e->information() = matLambda;

            optimizer.addEdge(e);

            sInsertedEdges.insert(make_pair(min(nIDi,nIDj),max(nIDi,nIDj)));
        }
    }

    // Set normal edges
    for(size_t i=0, iend=vpKFs.size(); i<iend; i++)
    {
        KeyFrame* pKF = vpKFs[i];

        const int nIDi = pKF->mnId;

        g2o::Sim3 Swi;

        LoopClosing::KeyFrameAndPose::const_iterator iti = NonCorrectedSim3.find(pKF);

        if(iti!=NonCorrectedSim3.end())
            Swi = (iti->second).inverse();
        else
            Swi = vScw[nIDi].inverse();

        KeyFrame* pParentKF = pKF->GetParent();

        // Spanning tree edge
        if(pParentKF)
        {
            int nIDj = pParentKF->mnId;

            g2o::Sim3 Sjw;

            LoopClosing::KeyFrameAndPose::const_iterator itj = NonCorrectedSim3.find(pParentKF);

            if(itj!=NonCorrectedSim3.end())
                Sjw = itj->second;
            else
                Sjw = vScw[nIDj];

            g2o::Sim3 Sji = Sjw * Swi;

            g2o::EdgeSim3* e = new g2o::EdgeSim3();
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDj)));
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
            e->setMeasurement(Sji);

            e->information() = matLambda;
            optimizer.addEdge(e);
        }

        // Loop edges
        const set<KeyFrame*> sLoopEdges = pKF->GetLoopEdges();
        for(set<KeyFrame*>::const_iterator sit=sLoopEdges.begin(), send=sLoopEdges.end(); sit!=send; sit++)
        {
            KeyFrame* pLKF = *sit;
            if(pLKF->mnId<pKF->mnId)
            {
                g2o::Sim3 Slw;

                LoopClosing::KeyFrameAndPose::const_iterator itl = NonCorrectedSim3.find(pLKF);

                if(itl!=NonCorrectedSim3.end())
                    Slw = itl->second;
                else
                    Slw = vScw[pLKF->mnId];

                g2o::Sim3 Sli = Slw * Swi;
                g2o::EdgeSim3* el = new g2o::EdgeSim3();
                el->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pLKF->mnId)));
                el->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
                el->setMeasurement(Sli);
                el->information() = matLambda;
                optimizer.addEdge(el);
            }
        }

        // Covisibility graph edges
        const vector<KeyFrame*> vpConnectedKFs = pKF->GetCovisiblesByWeight(minFeat);
        for(vector<KeyFrame*>::const_iterator vit=vpConnectedKFs.begin(); vit!=vpConnectedKFs.end(); vit++)
        {
            KeyFrame* pKFn = *vit;
            if(pKFn && pKFn!=pParentKF && !pKF->hasChild(pKFn) && !sLoopEdges.count(pKFn))
            {
                if(!pKFn->isBad() && pKFn->mnId<pKF->mnId)
                {
                    if(sInsertedEdges.count(make_pair(min(pKF->mnId,pKFn->mnId),max(pKF->mnId,pKFn->mnId))))
                        continue;

                    g2o::Sim3 Snw;

                    LoopClosing::KeyFrameAndPose::const_iterator itn = NonCorrectedSim3.find(pKFn);

                    if(itn!=NonCorrectedSim3.end())
                        Snw = itn->second;
                    else
                        Snw = vScw[pKFn->mnId];

                    g2o::Sim3 Sni = Snw * Swi;

                    g2o::EdgeSim3* en = new g2o::EdgeSim3();
                    en->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFn->mnId)));
                    en->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
                    en->setMeasurement(Sni);
                    en->information() = matLambda;
                    optimizer.addEdge(en);
                }
            }
        }
    }

    // Optimize!
    optimizer.initializeOptimization();
    optimizer.optimize(20);

    unique_lock<mutex> lock(pMap->mMutexMapUpdate);

    cv::Mat Tbc = ConfigParam::GetMatTbc();

    // SE3 Pose Recovering. Sim3:[sR t;0 1] -> SE3:[R t/s;0 1]
    for(size_t i=0;i<vpKFs.size();i++)
    {
        KeyFrame* pKFi = vpKFs[i];

        const int nIDi = pKFi->mnId;

        g2o::VertexSim3Expmap* VSim3 = static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(nIDi));
        g2o::Sim3 CorrectedSiw =  VSim3->estimate();
        vCorrectedSwc[nIDi]=CorrectedSiw.inverse();
        Eigen::Matrix3d eigR = CorrectedSiw.rotation().toRotationMatrix();
        Eigen::Vector3d eigt = CorrectedSiw.translation();
        double s = CorrectedSiw.scale();

        eigt *=(1./s); //[R t/s;0 1]

        cv::Mat Tiw = Converter::toCvSE3(eigR,eigt);

        pKFi->SetPose(Tiw);

        // Update P/V/R in NavState
        pKFi->UpdateNavStatePVRFromTcw(Tiw,Tbc);
    }

    // Correct points. Transform to "non-optimized" reference keyframe pose and transform back with optimized pose
    for(size_t i=0, iend=vpMPs.size(); i<iend; i++)
    {
        MapPoint* pMP = vpMPs[i];

        if(pMP->isBad())
            continue;

        int nIDr;
        if(pMP->mnCorrectedByKF==pCurKF->mnId)
        {
            nIDr = pMP->mnCorrectedReference;
        }
        else
        {
            KeyFrame* pRefKF = pMP->GetReferenceKeyFrame();
            nIDr = pRefKF->mnId;
        }


        g2o::Sim3 Srw = vScw[nIDr];
        g2o::Sim3 correctedSwr = vCorrectedSwc[nIDr];

        cv::Mat P3Dw = pMP->GetWorldPos();
        Eigen::Matrix<double,3,1> eigP3Dw = Converter::toVector3d(P3Dw);
        Eigen::Matrix<double,3,1> eigCorrectedP3Dw = correctedSwr.map(Srw.map(eigP3Dw));

        cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);
        pMP->SetWorldPos(cvCorrectedP3Dw);

        pMP->UpdateNormalAndDepth();
    }

    if(pLC)
    {
        pLC->SetMapUpdateFlagInTracking(true);
    }
}

int Optimizer::OptimizeSim3(KeyFrame *pKF1, KeyFrame *pKF2, vector<MapPoint *> &vpMatches1, g2o::Sim3 &g2oS12, const float th2, const bool bFixScale)
{
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    // Calibration
    const cv::Mat &K1 = pKF1->mK;
    const cv::Mat &K2 = pKF2->mK;

    // Camera poses
    const cv::Mat R1w = pKF1->GetRotation();
    const cv::Mat t1w = pKF1->GetTranslation();
    const cv::Mat R2w = pKF2->GetRotation();
    const cv::Mat t2w = pKF2->GetTranslation();

    // Set Sim3 vertex
    g2o::VertexSim3Expmap * vSim3 = new g2o::VertexSim3Expmap();    
    vSim3->_fix_scale=bFixScale;
    vSim3->setEstimate(g2oS12);
    vSim3->setId(0);
    vSim3->setFixed(false);
    vSim3->_principle_point1[0] = K1.at<float>(0,2);
    vSim3->_principle_point1[1] = K1.at<float>(1,2);
    vSim3->_focal_length1[0] = K1.at<float>(0,0);
    vSim3->_focal_length1[1] = K1.at<float>(1,1);
    vSim3->_principle_point2[0] = K2.at<float>(0,2);
    vSim3->_principle_point2[1] = K2.at<float>(1,2);
    vSim3->_focal_length2[0] = K2.at<float>(0,0);
    vSim3->_focal_length2[1] = K2.at<float>(1,1);
    optimizer.addVertex(vSim3);

    // Set MapPoint vertices
    const int N = vpMatches1.size();
    const vector<MapPoint*> vpMapPoints1 = pKF1->GetMapPointMatches();
    vector<g2o::EdgeSim3ProjectXYZ*> vpEdges12;
    vector<g2o::EdgeInverseSim3ProjectXYZ*> vpEdges21;
    vector<size_t> vnIndexEdge;

    vnIndexEdge.reserve(2*N);
    vpEdges12.reserve(2*N);
    vpEdges21.reserve(2*N);

    const float deltaHuber = sqrt(th2);

    int nCorrespondences = 0;

    for(int i=0; i<N; i++)
    {
        if(!vpMatches1[i])
            continue;

        MapPoint* pMP1 = vpMapPoints1[i];
        MapPoint* pMP2 = vpMatches1[i];

        const int id1 = 2*i+1;
        const int id2 = 2*(i+1);

        const int i2 = pMP2->GetIndexInKeyFrame(pKF2);

        if(pMP1 && pMP2)
        {
            if(!pMP1->isBad() && !pMP2->isBad() && i2>=0)
            {
                g2o::VertexSBAPointXYZ* vPoint1 = new g2o::VertexSBAPointXYZ();
                cv::Mat P3D1w = pMP1->GetWorldPos();
                cv::Mat P3D1c = R1w*P3D1w + t1w;
                vPoint1->setEstimate(Converter::toVector3d(P3D1c));
                vPoint1->setId(id1);
                vPoint1->setFixed(true);
                optimizer.addVertex(vPoint1);

                g2o::VertexSBAPointXYZ* vPoint2 = new g2o::VertexSBAPointXYZ();
                cv::Mat P3D2w = pMP2->GetWorldPos();
                cv::Mat P3D2c = R2w*P3D2w + t2w;
                vPoint2->setEstimate(Converter::toVector3d(P3D2c));
                vPoint2->setId(id2);
                vPoint2->setFixed(true);
                optimizer.addVertex(vPoint2);
            }
            else
                continue;
        }
        else
            continue;

        nCorrespondences++;

        // Set edge x1 = S12*X2
        Eigen::Matrix<double,2,1> obs1;
        const cv::KeyPoint &kpUn1 = pKF1->mvKeysUn[i];
        obs1 << kpUn1.pt.x, kpUn1.pt.y;

        g2o::EdgeSim3ProjectXYZ* e12 = new g2o::EdgeSim3ProjectXYZ();
        e12->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id2)));
        e12->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
        e12->setMeasurement(obs1);
        const float &invSigmaSquare1 = pKF1->mvInvLevelSigma2[kpUn1.octave];
        e12->setInformation(Eigen::Matrix2d::Identity()*invSigmaSquare1);

        g2o::RobustKernelHuber* rk1 = new g2o::RobustKernelHuber;
        e12->setRobustKernel(rk1);
        rk1->setDelta(deltaHuber);
        optimizer.addEdge(e12);

        // Set edge x2 = S21*X1
        Eigen::Matrix<double,2,1> obs2;
        const cv::KeyPoint &kpUn2 = pKF2->mvKeysUn[i2];
        obs2 << kpUn2.pt.x, kpUn2.pt.y;

        g2o::EdgeInverseSim3ProjectXYZ* e21 = new g2o::EdgeInverseSim3ProjectXYZ();

        e21->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id1)));
        e21->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
        e21->setMeasurement(obs2);
        float invSigmaSquare2 = pKF2->mvInvLevelSigma2[kpUn2.octave];
        e21->setInformation(Eigen::Matrix2d::Identity()*invSigmaSquare2);

        g2o::RobustKernelHuber* rk2 = new g2o::RobustKernelHuber;
        e21->setRobustKernel(rk2);
        rk2->setDelta(deltaHuber);
        optimizer.addEdge(e21);

        vpEdges12.push_back(e12);
        vpEdges21.push_back(e21);
        vnIndexEdge.push_back(i);
    }

    // Optimize!
    optimizer.initializeOptimization();
    optimizer.optimize(5);

    // Check inliers
    int nBad=0;
    for(size_t i=0; i<vpEdges12.size();i++)
    {
        g2o::EdgeSim3ProjectXYZ* e12 = vpEdges12[i];
        g2o::EdgeInverseSim3ProjectXYZ* e21 = vpEdges21[i];
        if(!e12 || !e21)
            continue;

        if(e12->chi2()>th2 || e21->chi2()>th2)
        {
            size_t idx = vnIndexEdge[i];
            vpMatches1[idx]=static_cast<MapPoint*>(NULL);
            optimizer.removeEdge(e12);
            optimizer.removeEdge(e21);
            vpEdges12[i]=static_cast<g2o::EdgeSim3ProjectXYZ*>(NULL);
            vpEdges21[i]=static_cast<g2o::EdgeInverseSim3ProjectXYZ*>(NULL);
            nBad++;
        }
    }

    int nMoreIterations;
    if(nBad>0)
        nMoreIterations=10;
    else
        nMoreIterations=5;

    if(nCorrespondences-nBad<10)
        return 0;

    // Optimize again only with inliers

    optimizer.initializeOptimization();
    optimizer.optimize(nMoreIterations);

    int nIn = 0;
    for(size_t i=0; i<vpEdges12.size();i++)
    {
        g2o::EdgeSim3ProjectXYZ* e12 = vpEdges12[i];
        g2o::EdgeInverseSim3ProjectXYZ* e21 = vpEdges21[i];
        if(!e12 || !e21)
            continue;

        if(e12->chi2()>th2 || e21->chi2()>th2)
        {
            size_t idx = vnIndexEdge[i];
            vpMatches1[idx]=static_cast<MapPoint*>(NULL);
        }
        else
            nIn++;
    }

    // Recover optimized Sim3
    g2o::VertexSim3Expmap* vSim3_recov = static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(0));
    g2oS12= vSim3_recov->estimate();

    return nIn;
}


} //namespace ORB_SLAM
