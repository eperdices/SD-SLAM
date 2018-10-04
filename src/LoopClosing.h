/**
 *
 *  Copyright (C) 2017 Eduardo Perdices <eperdices at gsyc dot es>
 *
 *  The following code is a derivative work of the code from the ORB-SLAM2 project,
 *  which is licensed under the GNU Public License, version 3. This code therefore
 *  is also licensed under the terms of the GNU Public License, version 3.
 *  For more information see <https://github.com/raulmur/ORB_SLAM2>.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef SD_SLAM_LOOPCLOSING_H
#define SD_SLAM_LOOPCLOSING_H

#include <thread>
#include <mutex>
#include "KeyFrame.h"
#include "LocalMapping.h"
#include "Map.h"
#include "Tracking.h"
#include "extra/g2o/types/types_seven_dof_expmap.h"

namespace SD_SLAM {

class Tracking;
class LocalMapping;

class LoopClosing {
 public:

  typedef std::pair<std::set<KeyFrame*>, int> ConsistentGroup;
  typedef std::map<KeyFrame*,g2o::Sim3, std::less<KeyFrame*>,
    Eigen::aligned_allocator<std::pair<const KeyFrame*, g2o::Sim3> > > KeyFrameAndPose;

 public:
  LoopClosing(Map* pMap, const bool bFixScale);

  void SetTracker(Tracking* pTracker);

  void SetLocalMapper(LocalMapping* pLocalMapper);

  // Main function
  void Run();

  void InsertKeyFrame(KeyFrame *pKF);

  void RequestReset();

  // This function will run in a separate thread
  void RunGlobalBundleAdjustment(unsigned long nLoopKF);

  bool isRunningGBA() {
    std::unique_lock<std::mutex> lock(mMutexGBA);
    return mbRunningGBA;
  }
  bool isFinishedGBA() {
    std::unique_lock<std::mutex> lock(mMutexGBA);
    return mbFinishedGBA;
  }

  void RequestFinish();

  bool isFinished();

 protected:
  bool CheckNewKeyFrames();

  bool DetectLoop();

  bool ComputeSim3();

  void SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap);

  void CorrectLoop();

  void ResetIfRequested();
  bool mbResetRequested;
  std::mutex mMutexReset;

  bool CheckFinish();
  void SetFinish();
  bool mbFinishRequested;
  bool mbFinished;
  std::mutex mMutexFinish;

  Map* mpMap;
  Tracking* mpTracker;

  LocalMapping *mpLocalMapper;

  std::list<KeyFrame*> mlpLoopKeyFrameQueue;

  std::mutex mMutexLoopQueue;

  // Loop detector parameters
  float mnCovisibilityConsistencyTh;

  // Loop detector variables
  KeyFrame* mpCurrentKF;
  KeyFrame* mpMatchedKF;
  std::vector<ConsistentGroup> mvConsistentGroups;
  std::vector<KeyFrame*> mvpEnoughConsistentCandidates;
  std::vector<KeyFrame*> mvpCurrentConnectedKFs;
  std::vector<MapPoint*> mvpCurrentMatchedPoints;
  std::vector<MapPoint*> mvpLoopMapPoints;
  Eigen::Matrix4d mScw;
  g2o::Sim3 mg2oScw;

  long unsigned int mLastLoopKFid;

  // Variables related to Global Bundle Adjustment
  bool mbRunningGBA;
  bool mbFinishedGBA;
  bool mbStopGBA;
  std::mutex mMutexGBA;
  std::thread* mpThreadGBA;

  // Fix scale in the stereo/RGB-D case
  bool mbFixScale;

  int mnFullBAIdx;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace SD_SLAM

#endif  // SD_SLAM_LOOPCLOSING_H
