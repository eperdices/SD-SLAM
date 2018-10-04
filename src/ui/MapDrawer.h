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

#ifndef SD_SLAM_MAPDRAWER_H
#define SD_SLAM_MAPDRAWER_H

#include <mutex>
#include <pangolin/pangolin.h>
#include <Eigen/Dense>
#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"

namespace SD_SLAM {

class MapDrawer {
 public:
  MapDrawer(Map* pMap);

  Map* mpMap;

  void DrawMapPoints();
  void DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph);
  void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc);
  void SetCurrentCameraPose(const Eigen::Matrix4d &Tcw);
  bool GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M);

 private:
  Eigen::Matrix4d mCameraPose;

  std::mutex mMutexCamera;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace SD_SLAM

#endif  // SD_SLAM_MAPDRAWER_H
