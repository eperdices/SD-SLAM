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

#include "Viewer.h"
#include <pangolin/pangolin.h>
#include <unistd.h>
#include "Config.h"

using std::mutex;
using std::unique_lock;

namespace SD_SLAM {

Viewer::Viewer(System* pSystem, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer):
  mpSystem(pSystem), mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mbFinishRequested(false), mbFinished(true) {
}

void Viewer::Run() {
  int w, h, mw, iw, ih, ib;
  double fx, fy, cx, cy;

  mbFinished = false;
  iw = Config::Width();
  ih = Config::Height();
  fx = Config::fx();
  fy = Config::fy();
  cx = Config::cx();
  cy = Config::cy();
  mw = 200;
  w = iw+mw;
  h = ih;
  ib = 10;

  // Force even width
  if (iw % 2 != 0)
    iw -= 1;

  pangolin::CreateWindowAndBind("SD-SLAM", w, h);

  // 3D Mouse handler requires depth testing to be enabled
  glEnable(GL_DEPTH_TEST);

  // Issue specific OpenGl we might need
  glEnable (GL_BLEND);
  glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0,pangolin::Attach::Pix(mw));
  pangolin::Var<bool> menuReset("menu.Reset SD-SLAM", false, false);
  pangolin::Var<bool> menuStopAndSave("menu.Stop and Save", false, false);
  pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode", false, true);
  pangolin::Var<bool> menuShowMap("menu.Show Map", true, true);
	pangolin::Var<std::string> menuSeparatorMap("menu.## 3D Map parameters ##","", false);
  pangolin::Var<bool> menuFollowCamera("menu.Follow Camera", true, true);
  pangolin::Var<bool> menuShowPoints("menu.Show Points", true, true);
  pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames", true, true);
  pangolin::Var<bool> menuShowGraph("menu.Show Graph", true, true);
	pangolin::Var<std::string> menuSeparatorImage("menu.## Image parameters ##","", false);
  pangolin::Var<bool> menuUndistort("menu.Undistort Image", true, true);
  pangolin::Var<bool> menuShowFrames("menu.Show Thumbnail", true, true);
  pangolin::Var<double> menuImageScale("menu.Thumbnail Scale", 0.3, 0, 1, false);
	pangolin::Var<std::string> menuSeparatorAR("menu.## AR parameters ##","", false);
  pangolin::Var<bool> menuAddPlane("menu.Add AR Object", false, false);
  pangolin::Var<bool> menuClearAR("menu.Remove Objects", false, false);

  // Define Camera Render Object (for view / scene browsing)
  pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(w, h, Config::ViewpointF(), Config::ViewpointF(), w/2, h/2, 0.1, 1000),
        pangolin::ModelViewLookAt(Config::ViewpointX(), Config::ViewpointY(), Config::ViewpointZ(), 0, 0, 0, 0.0,-1.0, 0.0)
        );

  // Add named OpenGL viewport to window and provide 3D Handler
  pangolin::View& d_cam = pangolin::CreateDisplay()
      .SetBounds(0.0, 1.0, pangolin::Attach::Pix(mw), 1.0, -w/(float)h)
      .SetHandler(new pangolin::Handler3D(s_cam));

  pangolin::OpenGlMatrix Twc;
  Twc.SetIdentity();

  // Configure frame viewer
  pangolin::View& d_video = pangolin::Display("Frame");
  d_video.SetBounds(pangolin::Attach::Pix(ib), pangolin::Attach::Pix(ib+ih*menuImageScale),
                    pangolin::Attach::Pix(mw+ib), pangolin::Attach::Pix(mw+ib+iw*menuImageScale), (float)iw/ih);
  pangolin::GlTexture textVideo(iw, ih,GL_RGB, false, 0,GL_RGB,GL_UNSIGNED_BYTE);
  pangolin::OpenGlMatrixSpec P = pangolin::ProjectionMatrixRDF_TopLeft(iw,ih,fx,fy,cx,cy,0.001,1000);

  bool bFollow = false;
  bool bLocalizationMode = false;

  while (!pangolin::ShouldQuit()) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Check localization mode
    if(menuLocalizationMode && !bLocalizationMode) {
      mpSystem->ActivateLocalizationMode();
      bLocalizationMode = true;
    } else if(!menuLocalizationMode && bLocalizationMode) {
      mpSystem->DeactivateLocalizationMode();
      bLocalizationMode = false;
    }

    // Check stop
    if (menuStopAndSave) {
      mpSystem->RequestStop();
      menuStopAndSave = false;
    }

    // Set camera position
    bool validTwc = mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc);

    if (menuFollowCamera && bFollow) {
      s_cam.Follow(Twc);
    } else if (menuFollowCamera && !bFollow) {
      s_cam.SetModelViewMatrix(
        pangolin::ModelViewLookAt(Config::ViewpointX(), Config::ViewpointY(), Config::ViewpointZ(), 0, 0, 0, 0.0,-1.0, 0.0));
      s_cam.Follow(Twc);
      if (validTwc)
        bFollow = true;
    } else if (!menuFollowCamera && bFollow) {
      bFollow = false;
    }

    // Show 3D map and image thumbnails
    if (menuShowMap) {

      d_cam.Activate(s_cam);
      glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

      if (validTwc)
        mpMapDrawer->DrawCurrentCamera(Twc);
      if (menuShowKeyFrames || menuShowGraph)
        mpMapDrawer->DrawKeyFrames(menuShowKeyFrames, menuShowGraph);
      if (menuShowPoints)
        mpMapDrawer->DrawMapPoints();

      // Draw AR
      if (menuAddPlane) {
        mpFrameDrawer->AddPlane();
        menuAddPlane = false;
      }
      if (menuClearAR) {
        mpFrameDrawer->ClearAR();
        menuClearAR = false;
      }

      mpFrameDrawer->CheckPlanes(mpSystem->MapChanged());

      // Get frame and show
      if (menuShowFrames) {
        d_video.SetBounds(pangolin::Attach::Pix(ib), pangolin::Attach::Pix(ib+ih*menuImageScale),
                          pangolin::Attach::Pix(mw+ib), pangolin::Attach::Pix(mw+ib+iw*menuImageScale),(float)iw/ih);
        d_video.Activate();
        glColor3f(1.0,1.0,1.0);

        cv::Mat im, subim;
        mpFrameDrawer->SetUndistort(menuUndistort);
        im = mpFrameDrawer->DrawFrame();

        im(cv::Rect(0, 0, iw, ih)).copyTo(subim); // In case width is smaller
        textVideo.Upload(subim.data, GL_RGB, GL_UNSIGNED_BYTE);
        textVideo.RenderToViewportFlipY();
      }
    } else {
      // AR Mode
      d_video.SetBounds(0, 1.0f, pangolin::Attach::Pix(mw), 1.0f, (float)iw/ih);
      d_video.Activate();
      glColor3f(1.0,1.0,1.0);

      // Set image
      cv::Mat im, subim;
      mpFrameDrawer->SetUndistort(menuUndistort);
      im = mpFrameDrawer->DrawFrame();

      im(cv::Rect(0, 0, iw, ih)).copyTo(subim); // In case width is smaller
      textVideo.Upload(subim.data, GL_RGB, GL_UNSIGNED_BYTE);
      textVideo.RenderToViewportFlipY();

      glClear(GL_DEPTH_BUFFER_BIT);

      // Load camera projection
      glMatrixMode(GL_PROJECTION);
      P.Load();

      glMatrixMode(GL_MODELVIEW);

      // Load camera pose
      pangolin::OpenGlMatrix M;
      mpFrameDrawer->GetCurrentOpenGLCameraMatrix(M);
      M.Load();

      // Draw AR
      if (menuAddPlane) {
        mpFrameDrawer->AddPlane();
        menuAddPlane = false;
      }
      if (menuClearAR) {
        mpFrameDrawer->ClearAR();
        menuClearAR = false;
      }
      mpFrameDrawer->CheckPlanes(mpSystem->MapChanged());
    }

    pangolin::FinishFrame();

    if (menuReset) {
      menuShowMap = true;
      menuFollowCamera = true;
      menuShowPoints = true;
      menuShowKeyFrames = true;
      menuShowGraph = true;
      menuShowFrames = true;
      menuLocalizationMode = false;
      if(bLocalizationMode)
        mpSystem->DeactivateLocalizationMode();
      bLocalizationMode = false;
      bFollow = false;
      mpSystem->Reset();
      menuReset = false;
    }

    if (CheckFinish())
      break;
  }

  SetFinish();

	std::cout << "UI thread finished, exiting..." << std::endl;
}

void Viewer::RequestFinish() {
  unique_lock<mutex> lock(mMutexFinish);
  mbFinishRequested = true;
}

bool Viewer::CheckFinish() {
  unique_lock<mutex> lock(mMutexFinish);
  return mbFinishRequested;
}

void Viewer::SetFinish() {
  unique_lock<mutex> lock(mMutexFinish);
  mbFinished = true;
}

bool Viewer::isFinished() {
  unique_lock<mutex> lock(mMutexFinish);
  return mbFinished;
}

}
