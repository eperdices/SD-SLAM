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

#include "Frame.h"
#include <thread>
#include "ORBmatcher.h"
#include "Converter.h"

using std::vector;

namespace SD_SLAM {

long unsigned int Frame::nNextId = 0;
bool Frame::mbInitialComputations = true;
float Frame::cx, Frame::cy, Frame::fx, Frame::fy, Frame::invfx, Frame::invfy;
float Frame::mnMinX, Frame::mnMinY, Frame::mnMaxX, Frame::mnMaxY;
float Frame::mfGridElementWidthInv, Frame::mfGridElementHeightInv;

Frame::Frame() {
  mTcw.setZero();
}

// Copy Constructor
Frame::Frame(const Frame &frame): mpORBextractorLeft(frame.mpORBextractorLeft),
  mK(frame.mK), mDistCoef(frame.mDistCoef.clone()), mbf(frame.mbf), mb(frame.mb), mThDepth(frame.mThDepth),
  N(frame.N), mvKeys(frame.mvKeys), mvKeysUn(frame.mvKeysUn), mvuRight(frame.mvuRight), mvDepth(frame.mvDepth),
  mDescriptors(frame.mDescriptors.clone()), mvpMapPoints(frame.mvpMapPoints), mvbOutlier(frame.mvbOutlier),
  mnId(frame.mnId), mpReferenceKF(frame.mpReferenceKF), mnScaleLevels(frame.mnScaleLevels),
  mfScaleFactor(frame.mfScaleFactor), mfLogScaleFactor(frame.mfLogScaleFactor), mvScaleFactors(frame.mvScaleFactors),
  mvInvScaleFactors(frame.mvInvScaleFactors), mvLevelSigma2(frame.mvLevelSigma2),
  mvInvLevelSigma2(frame.mvInvLevelSigma2) {
  for (int i = 0; i < FRAME_GRID_COLS; i++)
    for (int j = 0; j < FRAME_GRID_ROWS; j++)
      mGrid[i][j] = frame.mGrid[i][j];

  SetPose(frame.mTcw);

  // Copy pyramid
  int size = frame.mvImagePyramid.size();
  mvImagePyramid.resize(size);
  for (int i = 0; i < size; i++)
    mvImagePyramid[i] = frame.mvImagePyramid[i].clone();

  mDepthImage = frame.mDepthImage.clone();
}


Frame::Frame(const cv::Mat &imGray, const cv::Mat &imDepth, ORBextractor* extractor,
  const Eigen::Matrix3d &K, cv::Mat &distCoef, const float &bf, const float &thDepth) :
  mpORBextractorLeft(extractor), mK(K), mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth) {
  // Frame ID
  mnId = nNextId++;

  mTcw.setZero();

  // Scale Level Info
  mnScaleLevels = mpORBextractorLeft->GetLevels();
  mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
  mfLogScaleFactor = log(mfScaleFactor);
  mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
  mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
  mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
  mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

  // ORB extraction
  ExtractORB(imGray);

  N = mvKeys.size();

  if (mvKeys.empty())
    return;

  UndistortKeyPoints();

  ComputeStereoFromRGBD(imDepth);
  mDepthImage = imDepth.clone();

  mvpMapPoints = vector<MapPoint*>(N, static_cast<MapPoint*>(NULL));
  mvbOutlier = vector<bool>(N, false);

  // This is done only for the first Frame
  if (mbInitialComputations) {
    ComputeImageBounds(imGray);

    mfGridElementWidthInv = static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
    mfGridElementHeightInv = static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

    fx = K(0, 0);
    fy = K(1, 1);
    cx = K(0, 2);
    cy = K(1, 2);
    invfx = 1.0f/fx;
    invfy = 1.0f/fy;

    mbInitialComputations = false;
  }

  mb = mbf/fx;

  AssignFeaturesToGrid();
}


Frame::Frame(const cv::Mat &imGray, ORBextractor* extractor, const Eigen::Matrix3d &K,
  cv::Mat &distCoef, const float &bf, const float &thDepth) :
  mpORBextractorLeft(extractor), mK(K), mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth) {
  // Frame ID
  mnId=nNextId++;

  mTcw.setZero();

  // Scale Level Info
  mnScaleLevels = mpORBextractorLeft->GetLevels();
  mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
  mfLogScaleFactor = log(mfScaleFactor);
  mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
  mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
  mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
  mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

  // ORB extraction
  ExtractORB(imGray);

  N = mvKeys.size();

  if (mvKeys.empty())
    return;

  UndistortKeyPoints();

  // Set no stereo information
  mvuRight = vector<float>(N, -1);
  mvDepth = vector<float>(N, -1);

  mvpMapPoints = vector<MapPoint*>(N, static_cast<MapPoint*>(NULL));
  mvbOutlier = vector<bool>(N, false);

  // This is done only for the first Frame (or after a change in the calibration)
  if (mbInitialComputations) {
    ComputeImageBounds(imGray);

    mfGridElementWidthInv = static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
    mfGridElementHeightInv = static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

    fx = K(0, 0);
    fy = K(1, 1);
    cx = K(0, 2);
    cy = K(1, 2);
    invfx = 1.0f/fx;
    invfy = 1.0f/fy;

    mbInitialComputations = false;
  }

  mb = mbf/fx;

  AssignFeaturesToGrid();
}

void Frame::AssignFeaturesToGrid() {
  int nReserve = 0.5f*N/(FRAME_GRID_COLS*FRAME_GRID_ROWS);
  for (unsigned int i = 0; i < FRAME_GRID_COLS; i++)
    for (unsigned int j = 0; j < FRAME_GRID_ROWS; j++)
      mGrid[i][j].reserve(nReserve);

  for (int i = 0; i < N; i++) {
    const cv::KeyPoint &kp = mvKeysUn[i];

    int nGridPosX, nGridPosY;
    if (PosInGrid(kp, nGridPosX, nGridPosY))
      mGrid[nGridPosX][nGridPosY].push_back(i);
  }
}

void Frame::ExtractORB(const cv::Mat &im) {
  (*mpORBextractorLeft)(im, cv::Mat(), mvKeys, mDescriptors, mvImagePyramid);
}

void Frame::SetPose(const Eigen::Matrix4d &Tcw) {
  Eigen::Matrix4d m = Tcw;  // Somehow it fixes problems with Eigen
  mTcw = m;
  UpdatePoseMatrices();
}

void Frame::UpdatePoseMatrices() {
  mRcw = mTcw.block<3, 3>(0, 0);
  mRwc = mRcw.transpose();
  mtcw = mTcw.block<3, 1>(0, 3);
  mOw = -mRwc*mtcw;

  mTwc.setIdentity();
  mTwc.block<3, 3>(0, 0) = mRwc;
  mTwc.block<3, 1>(0, 3) = mOw;
}

bool Frame::isInFrustum(MapPoint *pMP, float viewingCosLimit) {
  pMP->mbTrackInView = false;

  // 3D in absolute coordinates
  Eigen::Vector3d P = pMP->GetWorldPos();

  // 3D in camera coordinates
  Eigen::Vector3d Pc = mRcw*P+mtcw;
  const double PcX = Pc(0);
  const double PcY = Pc(1);
  const double PcZ = Pc(2);

  // Check positive depth
  if (PcZ < 0.0)
    return false;

  // Project in image and check it is not outside
  const float invz = 1.0f/PcZ;
  const float u = fx*PcX*invz+cx;
  const float v = fy*PcY*invz+cy;

  if (u < mnMinX || u > mnMaxX)
    return false;
  if (v < mnMinY || v > mnMaxY)
    return false;

  // Check distance is in the scale invariance region of the MapPoint
  const float maxDistance = pMP->GetMaxDistanceInvariance();
  const float minDistance = pMP->GetMinDistanceInvariance();
  Eigen::Vector3d PO = P-mOw;
  const float dist = PO.norm();

  if (dist<minDistance || dist>maxDistance)
    return false;

  // Check viewing angle
  Eigen::Vector3d Pn = pMP->GetNormal();
  const float viewCos = PO.dot(Pn)/dist;

  if (viewCos < viewingCosLimit)
    return false;

  // Predict scale in the image
  const int nPredictedLevel = pMP->PredictScale(dist, this);

  // Data used by the tracking
  pMP->mbTrackInView = true;
  pMP->mTrackProjX = u;
  pMP->mTrackProjXR = u - mbf*invz;
  pMP->mTrackProjY = v;
  pMP->mnTrackScaleLevel = nPredictedLevel;
  pMP->mTrackViewCos = viewCos;

  return true;
}

vector<size_t> Frame::GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel, const int maxLevel) const {
  vector<size_t> vIndices;
  vIndices.reserve(N);

  const int nMinCellX = std::max(0, static_cast<int>(floor((x-mnMinX-r)*mfGridElementWidthInv)));
  if (nMinCellX >= FRAME_GRID_COLS)
    return vIndices;

  const int nMaxCellX = std::min(static_cast<int>(FRAME_GRID_COLS-1),
                                 static_cast<int>(ceil((x-mnMinX+r)*mfGridElementWidthInv)));
  if (nMaxCellX < 0)
    return vIndices;

  const int nMinCellY = std::max(0, static_cast<int>(floor((y-mnMinY-r)*mfGridElementHeightInv)));
  if (nMinCellY >= FRAME_GRID_ROWS)
    return vIndices;

  const int nMaxCellY = std::min(static_cast<int>(FRAME_GRID_ROWS-1),
                                 static_cast<int>(ceil((y-mnMinY+r)*mfGridElementHeightInv)));
  if (nMaxCellY < 0)
    return vIndices;

  const bool bCheckLevels = (minLevel > 0) || (maxLevel >= 0);

  for (int ix = nMinCellX; ix <= nMaxCellX; ix++) {
    for (int iy = nMinCellY; iy <= nMaxCellY; iy++) {
      const vector<size_t> vCell = mGrid[ix][iy];
      if (vCell.empty())
        continue;

      for (size_t j = 0, jend=vCell.size(); j < jend; j++) {
        const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
        if (bCheckLevels) {
          if (kpUn.octave < minLevel)
            continue;
          if (maxLevel >= 0)
            if (kpUn.octave > maxLevel)
              continue;
        }

        const float distx = kpUn.pt.x-x;
        const float disty = kpUn.pt.y-y;

        if (fabs(distx) < r && fabs(disty) < r)
          vIndices.push_back(vCell[j]);
      }
    }
  }

  return vIndices;
}

bool Frame::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY) {
  posX = round((kp.pt.x-mnMinX)*mfGridElementWidthInv);
  posY = round((kp.pt.y-mnMinY)*mfGridElementHeightInv);

  // Keypoint's coordinates are undistorted, which could cause to go out of the image
  if (posX < 0 || posX >= FRAME_GRID_COLS || posY < 0 || posY >= FRAME_GRID_ROWS)
    return false;

  return true;
}


void Frame::UndistortKeyPoints() {
  if (mDistCoef.at<float>(0) == 0.0) {
    mvKeysUn = mvKeys;
    return;
  }

  // Fill matrix with points
  cv::Mat mat(N, 2, CV_32F);
  for (int i = 0; i < N; i++) {
    mat.at<float>(i, 0) = mvKeys[i].pt.x;
    mat.at<float>(i, 1) = mvKeys[i].pt.y;
  }

  // Undistort points
  mat = mat.reshape(2);
  cv::Mat mK_cv = Converter::toCvMat(mK);
  cv::undistortPoints(mat, mat, mK_cv, mDistCoef, cv::Mat(), mK_cv);
  mat = mat.reshape(1);

  // Fill undistorted keypoint vector
  mvKeysUn.resize(N);
  for (int i = 0; i < N; i++) {
    cv::KeyPoint kp = mvKeys[i];
    kp.pt.x = mat.at<float>(i, 0);
    kp.pt.y = mat.at<float>(i, 1);
    mvKeysUn[i] = kp;
  }
}

void Frame::ComputeImageBounds(const cv::Mat &imLeft) {
  if (mDistCoef.at<float>(0) != 0.0) {
    cv::Mat mat(4, 2, CV_32F);
    mat.at<float>(0, 0) = 0.0;
    mat.at<float>(0, 1) = 0.0;
    mat.at<float>(1, 0) = imLeft.cols;
    mat.at<float>(1, 1) = 0.0;
    mat.at<float>(2, 0) = 0.0;
    mat.at<float>(2, 1) = imLeft.rows;
    mat.at<float>(3, 0) = imLeft.cols;
    mat.at<float>(3, 1) = imLeft.rows;

    // Undistort corners
    mat = mat.reshape(2);
    cv::Mat mK_cv = Converter::toCvMat(mK);
    cv::undistortPoints(mat, mat, mK_cv, mDistCoef, cv::Mat(), mK_cv);
    mat = mat.reshape(1);

    mnMinX = std::min(mat.at<float>(0, 0), mat.at<float>(2, 0));
    mnMaxX = std::max(mat.at<float>(1, 0), mat.at<float>(3, 0));
    mnMinY = std::min(mat.at<float>(0, 1), mat.at<float>(1, 1));
    mnMaxY = std::max(mat.at<float>(2, 1), mat.at<float>(3, 1));

  } else {
    mnMinX = 0.0f;
    mnMaxX = imLeft.cols;
    mnMinY = 0.0f;
    mnMaxY = imLeft.rows;
  }
}

void Frame::ComputeStereoFromRGBD(const cv::Mat &imDepth) {
  mvuRight = vector<float>(N, -1);
  mvDepth = vector<float>(N, -1);

  for (int i = 0; i < N; i++) {
    const cv::KeyPoint &kp = mvKeys[i];
    const cv::KeyPoint &kpU = mvKeysUn[i];

    const float &v = kp.pt.y;
    const float &u = kp.pt.x;

    const float d = imDepth.at<float>(v, u);

    if (d > 0) {
      mvDepth[i] = d;
      mvuRight[i] = kpU.pt.x-mbf/d;
    }
  }
}

Eigen::Vector3d Frame::UnprojectStereo(const int &i) {
  const float z = mvDepth[i];
  if (z > 0) {
    const float u = mvKeysUn[i].pt.x;
    const float v = mvKeysUn[i].pt.y;
    const float x = (u-cx)*z*invfx;
    const float y = (v-cy)*z*invfy;
    Eigen::Vector3d x3Dc(x, y, z);
    return mRwc*x3Dc+mOw;
  } else {
    return Eigen::Vector3d::Zero();
  }
}

void Frame::Undistort(const cv::Mat& im, cv::Mat& im_out) {
  cv::Mat mK_cv = Converter::toCvMat(mK);
  cv::undistort(im, im_out, mK_cv, mDistCoef);
}

}  // namespace SD_SLAM
