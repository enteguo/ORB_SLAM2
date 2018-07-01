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

#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H

#include "KeyFrame.h"
#include "Map.h"
#include "LoopClosing.h"
#include "Tracking.h"
#include "KeyFrameDatabase.h"

#include <mutex>


namespace ORB_SLAM2
{

class Tracking;
class LoopClosing;
class Map;

class LocalMapping
{
public:
    LocalMapping(Map* pMap, const float bMonocular);

    void SetLoopCloser(LoopClosing* pLoopCloser);

    void SetTracker(Tracking* pTracker);

    // Main function
    void Run();

    void InsertKeyFrame(KeyFrame* pKF);

    // Thread Synch
    void RequestStop();
    void RequestReset();
    bool Stop();
    void Release();
    bool isStopped();
    bool stopRequested();
    bool AcceptKeyFrames();

    //SetAcceptKeyFrames
    //函数功能：告诉Tracking，LocalMapping正处于繁忙状态
    void SetAcceptKeyFrames(bool flag);
    bool SetNotStop(bool flag);

    void InterruptBA();

    void RequestFinish();
    bool isFinished();

    int KeyframesInQueue(){
        unique_lock<std::mutex> lock(mMutexNewKFs);
        return mlNewKeyFrames.size();
    }

protected:

    //CheckNewKeyFrames
    //函数功能：检查mlNewKeyFrames是否为空（队列中是否还有KF）
    //输出：为空返回0，否则返回1
    bool CheckNewKeyFrames();

    //ProcessNewKeyFrame
    //函数功能：类似CreateInitialMapMonocular这个函数
    //1.对tracking的新KF上的kps与localmap中的mappoins关联，
    //2.计算词袋。3.计算某个KP最佳描述子
    //4.计算平均方向和深度。5.计算共视图。6，KF插入地图
    void ProcessNewKeyFrame();

    //CreateNewMapPoints
    //函数功能：三角化三维点
    //遍历20个最佳共视帧，检查基线是否大于场景深度中值的1%，
    //极线搜索匹配，三角化。检查深度是否大于0，重投影误差是否能接受
    void CreateNewMapPoints();

    //MapPointCulling
    //函数功能：对最近添加的Mappoints进行挑选
    //1.不是bad的点
    //2.被检测到比被重投影观测到的帧大于25%
    //3.被观测次数小于阈值，并且检测到该点的两个KF不能差2帧以上
    //4.检测到该点的两个KF不能差3帧以上
    void MapPointCulling();

    //SearchInNeighbors
    //函数功能：在融合目标帧中搜索将近似的mappoints融合
    //融合目标帧定义为：一级相邻帧和二级相邻帧，一级相邻帧为当前帧的最优20共视帧；二级相邻帧为共视帧的最优5个共视帧
    //遍历所有的融合目标帧，将当前帧的mappoints与目标帧的mappoints融合，
    //融合目标帧上的mappoints与当前帧的mappoints相融合
    //最后更新当前帧MapPoints的描述子，深度，观测主方向等属性，更新当前帧的MapPoints后更新与其它帧的连接关系。
    void SearchInNeighbors();

    //KeyFrameCulling
    //函数功能：对当前帧与其共视的关键帧进行剔除，90%以上的MapPoint能被其他共视关键帧所观测到，那么该帧就会被剔除。
    //
    void KeyFrameCulling();

    cv::Mat ComputeF12(KeyFrame* &pKF1, KeyFrame* &pKF2);

    cv::Mat SkewSymmetricMatrix(const cv::Mat &v);

    bool mbMonocular;

    void ResetIfRequested();
    bool mbResetRequested;
    std::mutex mMutexReset;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    Map* mpMap;

    LoopClosing* mpLoopCloser;
    Tracking* mpTracker;

    std::list<KeyFrame*> mlNewKeyFrames;

    KeyFrame* mpCurrentKeyFrame;

    std::list<MapPoint*> mlpRecentAddedMapPoints;

    std::mutex mMutexNewKFs;

    bool mbAbortBA;

    bool mbStopped;
    bool mbStopRequested;
    bool mbNotStop;
    std::mutex mMutexStop;

    bool mbAcceptKeyFrames;
    std::mutex mMutexAccept;
};

} //namespace ORB_SLAM

#endif // LOCALMAPPING_H
