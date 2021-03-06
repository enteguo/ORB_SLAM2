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


#ifndef TRACKING_H
#define TRACKING_H

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include"Viewer.h"
#include"FrameDrawer.h"
#include"Map.h"
#include"LocalMapping.h"
#include"LoopClosing.h"
#include"Frame.h"
#include "ORBVocabulary.h"
#include"KeyFrameDatabase.h"
#include"ORBextractor.h"
#include "Initializer.h"
#include "MapDrawer.h"
#include "System.h"

#include <mutex>

namespace ORB_SLAM2
{

class Viewer;
class FrameDrawer;
class Map;
class LocalMapping;
class LoopClosing;
class System;

class Tracking
{  

public:
    Tracking(System* pSys, ORBVocabulary* pVoc, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Map* pMap,
             KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor);

    // Preprocess the input and call Track(). Extract features and performs stereo matching.
    cv::Mat GrabImageStereo(const cv::Mat &imRectLeft,const cv::Mat &imRectRight, const double &timestamp);
    cv::Mat GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp);
    cv::Mat GrabImageMonocular(const cv::Mat &im, const double &timestamp);

    void SetLocalMapper(LocalMapping* pLocalMapper);
    void SetLoopClosing(LoopClosing* pLoopClosing);
    void SetViewer(Viewer* pViewer);

    // Load new settings
    // The focal lenght should be similar or scale prediction will fail when projecting points
    // TODO: Modify MapPoint::PredictScale to take into account focal lenght
    void ChangeCalibration(const string &strSettingPath);

    // Use this function if you have deactivated local mapping and you only want to localize the camera.
    void InformOnlyTracking(const bool &flag);


public:

    // Tracking states
    enum eTrackingState{
        SYSTEM_NOT_READY=-1,
        NO_IMAGES_YET=0,
        NOT_INITIALIZED=1,
        OK=2,
        LOST=3
    };

    eTrackingState mState;
    eTrackingState mLastProcessedState;

    // Input sensor
    int mSensor;

    // Current Frame
    Frame mCurrentFrame;
    cv::Mat mImGray;

    // Initialization Variables (Monocular)
    std::vector<int> mvIniLastMatches;
    std::vector<int> mvIniMatches;
    std::vector<cv::Point2f> mvbPrevMatched;
    std::vector<cv::Point3f> mvIniP3D;
    Frame mInitialFrame;

    // Lists used to recover the full camera trajectory at the end of the execution.
    // Basically we store the reference keyframe for each frame and its relative transformation
    list<cv::Mat> mlRelativeFramePoses;
    list<KeyFrame*> mlpReferences;
    list<double> mlFrameTimes;
    list<bool> mlbLost;

    // True if local mapping is deactivated and we are performing only localization
    bool mbOnlyTracking;

    void Reset();

protected:

    // Main tracking function. It is independent of the input sensor.
    void Track();

    // Map initialization for stereo and RGB-D
    void StereoInitialization();

    // Map initialization for monocular
    //函数功能：单目视觉的初始化
    //选取两个初始帧，通过单应矩阵和基本矩阵初始化pose
    //通过pose三角化mappoints
    //最后优化
    void MonocularInitialization();

    //CreateInitialMapMonocular
    //函数功能：1.注册关键帧。2.计算词袋。3.将KF和mappoints加入地图。4.计算某个KP最佳描述子
    //5.计算平均方向和深度。6.计算共视图。7.
    void CreateInitialMapMonocular();

    void CheckReplacedInLastFrame();
    //TrackReferenceKeyFrame()
    //函数功能：用上一帧pose，匹配参考KF的KP，并优化pose
    //通过BOW匹配参考KF上的KF，用上一帧pose作为当前pose
    //优化，去除outliner
    //返回是否有大于10个inliner，即为跟踪成功
    bool TrackReferenceKeyFrame();
    void UpdateLastFrame();

    //TrackWithMotionModel()
    //函数功能：假设运动状态不变，通过前面的运动状态推断出当前帧pose
    //通过当前帧pose使用投影搜索匹配kp，如果匹配点数过少，扩大搜索范围
    //优化，去除outliner，返回是否有大于10个inliner，即为跟踪成功
    bool TrackWithMotionModel();

    bool Relocalization();

    void UpdateLocalMap();

    //UpdateLocalPoints（）
    //函数功能：局部mappoints更新
    //从局部KF所有的mappoints（除了当前帧的mappoints）添加到局部mapoints
    void UpdateLocalPoints();

    //UpdateLocalKeyFrames()
    //功能：为当前帧选出KF加入局部KF
    //三个策略：1.和当前帧观测到同样mappoints的KF
    //         2.（1）中的KF的最大共视10帧，子KF,父KF
    //         3.与当前帧观测到mappoints点数最多的KF为参考KF
    void UpdateLocalKeyFrames();


    //TrackLocalMap()
    //函数功能：投影局部mappoints，并优化
    //1.如果刚刚重定位过，并且inliner特征点小于50，则失败
    //2.如果inliner特征点小于30则失败
    bool TrackLocalMap();

    //SearchLocalPoints()
    //函数功能：查找当前帧可视的mappoints，并通过投影匹配
    void SearchLocalPoints();

    //bool NeedNewKeyFrame();
    //函数功能：判断是否需要插入关键帧
    //三种情况不插入KF
    //1.只做跟踪
    //2.回环线程在工作
    //3.刚刚重定位不久或关键帧数超过最大关键帧数
    //得到参考关键帧跟踪到的MapPoints数量，
    //在执行上判断如果参考帧的MapPoints点被观测到的次数大于minObs，则认为该点被跟踪到，并递增计数器
    //插入关键帧的条件
    //1.离上一个KF较远
    //2.mapping线程空闲
    //3.跟踪的inliner小于参考帧跟踪的inliner的90%
    bool NeedNewKeyFrame();
    void CreateNewKeyFrame();

    // In case of performing only localization, this flag is true when there are no matches to
    // points in the map. Still tracking will continue if there are enough matches with temporal points.
    // In that case we are doing visual odometry. The system will try to do relocalization to recover
    // "zero-drift" localization to the map.
    bool mbVO;

    //Other Thread Pointers
    LocalMapping* mpLocalMapper;
    LoopClosing* mpLoopClosing;

    //ORB
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;
    ORBextractor* mpIniORBextractor;

    //BoW
    ORBVocabulary* mpORBVocabulary;
    KeyFrameDatabase* mpKeyFrameDB;

    // Initalization (only for monocular)
    Initializer* mpInitializer;

    //Local Map
    KeyFrame* mpReferenceKF;
    std::vector<KeyFrame*> mvpLocalKeyFrames;
    std::vector<MapPoint*> mvpLocalMapPoints;
    
    // System
    System* mpSystem;
    
    //Drawers
    Viewer* mpViewer;
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;

    //Map
    Map* mpMap;

    //Calibration matrix
    cv::Mat mK;
    cv::Mat mDistCoef;
    float mbf;

    //New KeyFrame rules (according to fps)
    //初始化为0
    int mMinFrames;
    //初始化为fps
    int mMaxFrames;

    // Threshold close/far points
    // Points seen as close by the stereo/RGBD sensor are considered reliable
    // and inserted from just one frame. Far points requiere a match in two keyframes.
    float mThDepth;

    // For RGB-D inputs only. For some datasets (e.g. TUM) the depthmap values are scaled.
    float mDepthMapFactor;

    //Current matches in frame
    int mnMatchesInliers;

    //Last Frame, KeyFrame and Relocalisation Info
    KeyFrame* mpLastKeyFrame;
    Frame mLastFrame;
    unsigned int mnLastKeyFrameId;
    unsigned int mnLastRelocFrameId;

    //Motion Model
    cv::Mat mVelocity;

    //Color order (true RGB, false BGR, ignored if grayscale)
    bool mbRGB;

    list<MapPoint*> mlpTemporalPoints;
};

} //namespace ORB_SLAM

#endif // TRACKING_H
