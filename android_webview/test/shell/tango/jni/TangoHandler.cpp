/*
 * Copyright 2014 Google Inc. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <cstdlib>

#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLES3/gl3.h>

#include <cassert>

#include <cmath>

#include "TangoHandler.h"

#include <sstream>

#include <thread>

namespace {

const int kVersionStringLength = 128;
// The minimum Tango Core version required from this application.
const int kTangoCoreMinimumVersion = 9377;

void onTextureAvailable(void* context, TangoCameraId tangoCameraId)
{
  // Do nothing for now.
}

void onTangoEventAvailable(void* context, const TangoEvent* event) {
  tango_chromium::TangoHandler* tangoHandler =
      static_cast<tango_chromium::TangoHandler*>(context);
  tangoHandler->onTangoEventAvailable(event);
}

inline void multiplyMatrixWithVector(const float* m, const double* v, double* vr, bool addTranslation = true) {
  double v0 = v[0];
  double v1 = v[1];
  double v2 = v[2];
    vr[0] = m[ 0] * v0 + m[ 4] * v1 + m[ 8] * v2 + (addTranslation ? m[12] : 0);
    vr[1] = m[ 1] * v0 + m[ 5] * v1 + m[ 9] * v2 + (addTranslation ? m[13] : 0);
    vr[2] = m[ 2] * v0 + m[ 6] * v1 + m[10] * v2 + (addTranslation ? m[14] : 0);
}

inline void matrixInverse(const float* m, float* o)
{
    // based on http://www.euclideanspace.com/maths/algebra/matrix/functions/inverse/fourD/index.htm
    float* te = o;
    const float* me = m;

    float n11 = me[ 0 ], n21 = me[ 1 ], n31 = me[ 2 ], n41 = me[ 3 ],
      n12 = me[ 4 ], n22 = me[ 5 ], n32 = me[ 6 ], n42 = me[ 7 ],
      n13 = me[ 8 ], n23 = me[ 9 ], n33 = me[ 10 ], n43 = me[ 11 ],
      n14 = me[ 12 ], n24 = me[ 13 ], n34 = me[ 14 ], n44 = me[ 15 ],

      t11 = n23 * n34 * n42 - n24 * n33 * n42 + n24 * n32 * n43 - n22 * n34 * n43 - n23 * n32 * n44 + n22 * n33 * n44,
      t12 = n14 * n33 * n42 - n13 * n34 * n42 - n14 * n32 * n43 + n12 * n34 * n43 + n13 * n32 * n44 - n12 * n33 * n44,
      t13 = n13 * n24 * n42 - n14 * n23 * n42 + n14 * n22 * n43 - n12 * n24 * n43 - n13 * n22 * n44 + n12 * n23 * n44,
      t14 = n14 * n23 * n32 - n13 * n24 * n32 - n14 * n22 * n33 + n12 * n24 * n33 + n13 * n22 * n34 - n12 * n23 * n34;

    float det = n11 * t11 + n21 * t12 + n31 * t13 + n41 * t14;

    assert(det != 0);

    float detInv = 1.0 / det;

    te[ 0 ] = t11 * detInv;
    te[ 1 ] = ( n24 * n33 * n41 - n23 * n34 * n41 - n24 * n31 * n43 + n21 * n34 * n43 + n23 * n31 * n44 - n21 * n33 * n44 ) * detInv;
    te[ 2 ] = ( n22 * n34 * n41 - n24 * n32 * n41 + n24 * n31 * n42 - n21 * n34 * n42 - n22 * n31 * n44 + n21 * n32 * n44 ) * detInv;
    te[ 3 ] = ( n23 * n32 * n41 - n22 * n33 * n41 - n23 * n31 * n42 + n21 * n33 * n42 + n22 * n31 * n43 - n21 * n32 * n43 ) * detInv;

    te[ 4 ] = t12 * detInv;
    te[ 5 ] = ( n13 * n34 * n41 - n14 * n33 * n41 + n14 * n31 * n43 - n11 * n34 * n43 - n13 * n31 * n44 + n11 * n33 * n44 ) * detInv;
    te[ 6 ] = ( n14 * n32 * n41 - n12 * n34 * n41 - n14 * n31 * n42 + n11 * n34 * n42 + n12 * n31 * n44 - n11 * n32 * n44 ) * detInv;
    te[ 7 ] = ( n12 * n33 * n41 - n13 * n32 * n41 + n13 * n31 * n42 - n11 * n33 * n42 - n12 * n31 * n43 + n11 * n32 * n43 ) * detInv;

    te[ 8 ] = t13 * detInv;
    te[ 9 ] = ( n14 * n23 * n41 - n13 * n24 * n41 - n14 * n21 * n43 + n11 * n24 * n43 + n13 * n21 * n44 - n11 * n23 * n44 ) * detInv;
    te[ 10 ] = ( n12 * n24 * n41 - n14 * n22 * n41 + n14 * n21 * n42 - n11 * n24 * n42 - n12 * n21 * n44 + n11 * n22 * n44 ) * detInv;
    te[ 11 ] = ( n13 * n22 * n41 - n12 * n23 * n41 - n13 * n21 * n42 + n11 * n23 * n42 + n12 * n21 * n43 - n11 * n22 * n43 ) * detInv;

    te[ 12 ] = t14 * detInv;
    te[ 13 ] = ( n13 * n24 * n31 - n14 * n23 * n31 + n14 * n21 * n33 - n11 * n24 * n33 - n13 * n21 * n34 + n11 * n23 * n34 ) * detInv;
    te[ 14 ] = ( n14 * n22 * n31 - n12 * n24 * n31 - n14 * n21 * n32 + n11 * n24 * n32 + n12 * n21 * n34 - n11 * n22 * n34 ) * detInv;
    te[ 15 ] = ( n12 * n23 * n31 - n13 * n22 * n31 + n13 * n21 * n32 - n11 * n23 * n32 - n12 * n21 * n33 + n11 * n22 * n33 ) * detInv;
}

inline void matrixTranspose(const GLfloat* m, GLfloat* o)
{
    if (o == m)
    {
        GLfloat t;
        t = m[1];
        o[1] = m[4];
        o[4] = t;
        t = m[2];
        o[2] = m[8];
        o[8] = t;
        t = m[3];
        o[3] = m[12];
        o[12] = t;
        t = m[6];
        o[6] = m[9];
        o[9] = t;
        t = m[7];
        o[7] = m[13];
        o[13] = t;
        t = m[11];
        o[11] = m[14];
        o[14] = t;
    }
    else
    {
        o[1] = m[4];
        o[4] = m[1];
        o[2] = m[8];
        o[8] = m[2];
        o[3] = m[12];
        o[12] = m[3];
        o[6] = m[9];
        o[9] = m[6];
        o[7] = m[13];
        o[13] = m[7];
        o[11] = m[14];
        o[14] = m[11];
    }
    o[0] = m[0];
    o[5] = m[5];
    o[10] = m[10];
    o[15] = m[15];
}

inline double dot(const double* v1, const double* v2)
{
  return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

inline void transformPlane(const double* p, const float* m, double* pr)
{
  double pCopy[3] = { p[0], p[1], p[2] };

  pr[0] = p[0] * -p[3];
  pr[1] = p[1] * -p[3];
  pr[2] = p[2] * -p[3];

  multiplyMatrixWithVector(m, pr, pr);
  float mCopy[16];
  double normal[3];
  matrixInverse(m, mCopy);
  matrixTranspose(mCopy, mCopy);
  multiplyMatrixWithVector(mCopy, pCopy, normal, false);

  pr[3] = -dot(pr, normal);
  pr[0] = normal[0];
  pr[1] = normal[1];
  pr[2] = normal[2];
}

} // End anonymous namespace

namespace tango_chromium {

TangoHandler* TangoHandler::instance = 0;

TangoHandler* TangoHandler::getInstance()
{
  if (instance == 0)
  {
    instance = new TangoHandler();
  }
  return instance;
}

void TangoHandler::releaseInstance()
{
  delete instance;
  instance = 0;
}

TangoHandler::TangoHandler(): connected(false)
  , tangoConfig(nullptr)
  , lastTangoImageBufferTimestamp(0)
  , cameraImageWidth(0)
  , cameraImageHeight(0)
  , cameraImageTextureWidth(0)
  , cameraImageTextureHeight(0)
  , textureIdConnected(false)
{
}

TangoHandler::~TangoHandler()
{
  TangoConfig_free(tangoConfig);
  tangoConfig = nullptr;
}

void TangoHandler::onCreate(JNIEnv* env, jobject activity, int activityOrientation, int sensorOrientation)
{
  // Check the installed version of the TangoCore.  If it is too old, then
  // it will not support the most up to date features.
  int version = 0;
  TangoErrorType result;

  this->activityOrientation = activityOrientation;
  this->sensorOrientation = sensorOrientation;
}

void TangoHandler::onTangoServiceConnected(JNIEnv* env, jobject tango)
{
  TangoService_CacheTangoObject(env, tango);

  connect();
}


void TangoHandler::connect()
{
  TangoErrorType result;

  // TANGO_CONFIG_DEFAULT is enabling Motion Tracking and disabling Depth
  // Perception.
  tangoConfig = TangoService_getConfig(TANGO_CONFIG_DEFAULT);
  if (tangoConfig == nullptr)
  {
    LOGE("TangoHandler::connect, TangoService_getConfig error.");
    std::exit (EXIT_SUCCESS);
  }

  // Set auto-recovery for motion tracking as requested by the user.
  result = TangoConfig_setBool(tangoConfig, "config_enable_auto_recovery", true);
  if (result != TANGO_SUCCESS) {
    LOGE("TangoHandler::connect, config_enable_auto_recovery activation failed with error code: %d", result);
    std::exit(EXIT_SUCCESS);
  }

  // Enable color camera from config.
  result = TangoConfig_setBool(tangoConfig, "config_enable_color_camera", true);
  if (result != TANGO_SUCCESS) {
    LOGE("TangoHandler::connect, config_enable_color_camera() failed with error code: %d", result);
    std::exit(EXIT_SUCCESS);
  }

  // Note that it is super important for AR applications that we enable low
  // latency IMU integration so that we have pose information available as
  // quickly as possible. Without setting this flag, you will often receive
  // invalid poses when calling getPoseAtTime() for an image.
  result = TangoConfig_setBool(tangoConfig, "config_enable_low_latency_imu_integration", true);
  if (result != TANGO_SUCCESS) {
    LOGE("TangoHandler::connect, failed to enable low latency imu integration.");
    std::exit(EXIT_SUCCESS);
  }

  // Drift correction allows motion tracking to recover after it loses tracking.
  // The drift corrected pose is is available through the frame pair with
  // base frame AREA_DESCRIPTION and target frame DEVICE.
  result = TangoConfig_setBool(tangoConfig, "config_enable_drift_correction", true);
  if (result != TANGO_SUCCESS) {
    LOGE("TangoHandler::connect, enabling config_enable_drift_correction "
      "failed with error code: %d", result);
    std::exit(EXIT_SUCCESS);
  }

  // Enable Depth Perception.
  result = TangoConfig_setBool(tangoConfig, "config_enable_depth", true);
  if (result != TANGO_SUCCESS)
  {
    LOGE("TangoHandler::connect, config_enable_depth activation failed with error code: %d.", result);
    std::exit(EXIT_SUCCESS);
  }

  // Enabling experimental plane finding.
  result = TangoConfig_setBool(tangoConfig,
                            "config_experimental_enable_plane_detection", true);
  if (result != TANGO_SUCCESS) {
    LOGE("TangoHandler::connect, failed to enable experimental plane finding.");
    std::exit(EXIT_SUCCESS);
  }

  result = TangoConfig_setInt32(tangoConfig, "config_depth_mode",
                                TANGO_POINTCLOUD_XYZC);
  if (result != TANGO_SUCCESS) {
    LOGE(
        "TangoHandler::connect, failed to configure point cloud to "
        "XYZC.");
    std::exit(EXIT_SUCCESS);
  }

  // Get TangoCore version string from service.
  char tangoCoreVersionCharArray[kVersionStringLength];
  result = TangoConfig_getString(tangoConfig, "tango_service_library_version",
                              tangoCoreVersionCharArray, kVersionStringLength);
  if (result != TANGO_SUCCESS) {
    LOGE(
        "TangoHandler::connect, get tango core version failed with error"
        "code: %d",
        result);
    std::exit(EXIT_SUCCESS);
  }
  tangoCoreVersionString = tangoCoreVersionCharArray;

  // Connect some callbacks
  result = TangoService_connectOnTextureAvailable(TANGO_CAMERA_COLOR, this, ::onTextureAvailable);
  if (result != TANGO_SUCCESS)
  {
    LOGE("TangoHandler::connect, failed to connect texture callback with error code: %d", result);
    std::exit(EXIT_SUCCESS);
  }

  // Attach onEventAvailable callback.
  // The callback will be called after the service is connected.
  result = TangoService_connectOnTangoEvent(::onTangoEventAvailable);
  if (result != TANGO_SUCCESS) {
    LOGE(
        "TangoHandler::connect, failed to connect to event callback with error"
        "code: %d",
        result);
    std::exit(EXIT_SUCCESS);
  }

  // Connect the tango service.
  if (TangoService_connect(this, tangoConfig) != TANGO_SUCCESS)
  {
    LOGE("TangoHandler::connect, TangoService_connect error.");
    std::exit (EXIT_SUCCESS);
  }

  // Get the intrinsics for the color camera and pass them on to the depth
  // image.
  result = TangoService_getCameraIntrinsics(TANGO_CAMERA_COLOR, &tangoCameraIntrinsics);
  if (result != TANGO_SUCCESS) {
    LOGE("TangoHandler::connect: Failed to get the intrinsics for the color camera.");
    std::exit(EXIT_SUCCESS);
  }

  // By default, use the camera width and height retrieved from the tango camera intrinsics.
  cameraImageWidth = cameraImageTextureWidth = tangoCameraIntrinsics.width;
  cameraImageHeight = cameraImageTextureHeight = tangoCameraIntrinsics.height;

  // Initialize TangoSupport context.
  TangoSupport_initialize(TangoService_getPoseAtTime,
                          TangoService_getCameraIntrinsics);

  connected = true;
}

void TangoHandler::disconnect()
{
  TangoService_disconnect();

  cameraImageWidth = cameraImageHeight = 
    cameraImageTextureWidth = cameraImageTextureHeight = 0;

  textureIdConnected = false;

  connected = false;
}

void TangoHandler::onPause()
{
  disconnect();
}

void TangoHandler::onDeviceRotationChanged(int activityOrientation, int sensorOrientation)
{
  this->activityOrientation = activityOrientation;
  this->sensorOrientation = sensorOrientation;
}

void TangoHandler::onTangoEventAvailable(const TangoEvent* event) 
{
}

bool TangoHandler::isConnected() const
{
  return connected;
}

bool TangoHandler::getPose(TangoPoseData* tangoPoseData, bool* localized)
{
  bool result = connected;
  // *localized = false;
  // if (connected)
  // {
  //   bool lockNewTangoBufferId;
  //   tangoBufferIdsMutex.lock();
  //   lockNewTangoBufferId = tangoBufferIds.size() < MAX_NUMBER_OF_TANGO_BUFFER_IDS;
  //   tangoBufferIdsMutex.unlock();

  //   TangoBufferId tangoBufferId;
  //   if (lockNewTangoBufferId)
  //   {
  //     result = TangoService_lockCameraBuffer(TANGO_CAMERA_COLOR, &lastTangoImageBufferTimestamp, &tangoBufferId) == TANGO_SUCCESS;
  //   }
  
  //   if (result)
  //   {
  //     if (lockNewTangoBufferId)
  //     {
  //       tangoBufferIdsMutex.lock();
  //       tangoBufferIds.push(tangoBufferId);
  //       tangoBufferIdsMutex.unlock();
  //     }

  //     double timestamp = hasLastTangoImageBufferTimestampChangedLately() ? lastTangoImageBufferTimestamp : 0;

  //     if (lastEnabledADFUUID != "")
  //     {
  //       result = TangoSupport_getPoseAtTime(
  //         timestamp, TANGO_COORDINATE_FRAME_AREA_DESCRIPTION,
  //         TANGO_COORDINATE_FRAME_CAMERA_COLOR, TANGO_SUPPORT_ENGINE_OPENGL,
  //         TANGO_SUPPORT_ENGINE_OPENGL, 
  //         static_cast<TangoSupportRotation>(activityOrientation), tangoPoseData) == TANGO_SUCCESS;
  //       if (!result)
  //       {
  //         LOGE("TangoHandler::getPose: Failed to get the pose for area description.");
  //       }
  //       else if (tangoPoseData->status_code != TANGO_POSE_VALID)
  //       {
  //         LOGE("TangoHandler::getPose: Getting the Area Description pose did not work. Falling back to device pose estimation.");
  //       }
  //       else 
  //       {
  //         poseForMarkerDetectionMutex.lock();
  //         poseForMarkerDetectionIsCorrect = TangoSupport_getPoseAtTime(
  //           timestamp, TANGO_COORDINATE_FRAME_AREA_DESCRIPTION,
  //           TANGO_COORDINATE_FRAME_CAMERA_COLOR, TANGO_SUPPORT_ENGINE_OPENGL, 
  //           TANGO_SUPPORT_ENGINE_TANGO, ROTATION_IGNORED,
  //           &poseForMarkerDetection) == TANGO_SUCCESS;
  //         poseForMarkerDetectionMutex.unlock();
  //         *localized = true;
  //       }
  //     }

  //     if (lastEnabledADFUUID == "" || tangoPoseData->status_code != TANGO_POSE_VALID)
  //     {
  //       result = TangoSupport_getPoseAtTime(
  //         timestamp, TANGO_COORDINATE_FRAME_START_OF_SERVICE,
  //         TANGO_COORDINATE_FRAME_CAMERA_COLOR, TANGO_SUPPORT_ENGINE_OPENGL, 
  //         TANGO_SUPPORT_ENGINE_OPENGL, static_cast<TangoSupportRotation>(activityOrientation), tangoPoseData) == TANGO_SUCCESS;
  //       if (!result)
  //       {
  //         LOGE("TangoHandler::getPose: Failed to get the pose.");
  //       }
  //       else
  //       {
  //         poseForMarkerDetectionMutex.lock();
  //         poseForMarkerDetectionIsCorrect = TangoSupport_getPoseAtTime(
  //           timestamp, TANGO_COORDINATE_FRAME_START_OF_SERVICE,
  //           TANGO_COORDINATE_FRAME_CAMERA_COLOR, TANGO_SUPPORT_ENGINE_OPENGL, 
  //           TANGO_SUPPORT_ENGINE_TANGO, ROTATION_IGNORED, 
  //           &poseForMarkerDetection) == TANGO_SUCCESS;
  //         poseForMarkerDetectionMutex.unlock();
  //       }
  //     }
  //   }
  // }

  return result;
}

// bool TangoHandler::getPickingPointAndPlaneInPointCloud(float x, float y, double* point, double* plane)
// {
//   bool result = false;

//   if (connected)
//   {
//     double timestamp = hasLastTangoImageBufferTimestampChangedLately() ? lastTangoImageBufferTimestamp : 0.0;

//     TangoPoseData tangoPose;
//     if (TangoSupport_calculateRelativePose(
//       latestTangoPointCloud->timestamp, 
//       TANGO_COORDINATE_FRAME_CAMERA_DEPTH,
//       timestamp,
//       TANGO_COORDINATE_FRAME_CAMERA_COLOR,
//       &tangoPose) != TANGO_SUCCESS)
//     {
//       LOGE("%s: could not calculate relative pose", __func__);
//       return result;
//     }
//     float uv[] = {x, y};
//     double identity_translation[3] = {0.0, 0.0, 0.0};
//     double identity_orientation[4] = {0.0, 0.0, 0.0, 1.0};
//     if (TangoSupport_fitPlaneModelNearPoint(
//       latestTangoPointCloud, identity_translation, identity_orientation,
//       uv, static_cast<TangoSupportRotation>(activityOrientation),
//       tangoPose.translation,
//       tangoPose.orientation,
//       point, plane) != TANGO_SUCCESS)
//     {
//       LOGE("%s: could not calculate picking point and plane", __func__);
//       return result;
//     }
//     if (depthCameraMatrixTransform.status_code != TANGO_POSE_VALID) {
//       LOGE("TangoHandler::getPickingPointAndPlaneInPointCloud: Could not find a valid matrix transform at "
//       "time %lf for the depth camera.", latestTangoPointCloud->timestamp);
//       return result;
//     }
//     multiplyMatrixWithVector(depthCameraMatrixTransform.matrix, point, point);

//   //  LOGI("Before: %f, %f, %f, %f", plane[0], plane[1], plane[2], plane[3]);
//     transformPlane(plane, depthCameraMatrixTransform.matrix, plane);
//   //  LOGI("After: %f, %f, %f, %f", plane[0], plane[1], plane[2], plane[3]);

//     result = true;
//   }

//   return result;
// }

bool TangoHandler::getCameraImageSize(uint32_t* width, uint32_t* height)
{
  bool result = true;

  *width = cameraImageWidth;
  *height = cameraImageHeight;

  return result;
}

bool TangoHandler::getCameraImageTextureSize(uint32_t* width, uint32_t* height)
{
  bool result = true;

  *width = cameraImageTextureWidth;
  *height = cameraImageTextureHeight;

  return result;
}

bool TangoHandler::getCameraFocalLength(double* focalLengthX, double* focalLengthY)
{
  bool result = true;
  *focalLengthX = tangoCameraIntrinsics.fx;
  *focalLengthY = tangoCameraIntrinsics.fy;
  return result;
}

bool TangoHandler::getCameraPoint(double* x, double* y)
{
  bool result = true;
  *x = tangoCameraIntrinsics.cx;
  *y = tangoCameraIntrinsics.cy;
  return result;
}

bool TangoHandler::updateCameraImageIntoTexture(uint32_t textureId)
{
  if (!connected) return false;

  if (!textureIdConnected)
  {
    TangoErrorType result = TangoService_connectTextureId(TANGO_CAMERA_COLOR, textureId, nullptr, nullptr);
    if (result != TANGO_SUCCESS)
    {
      LOGE("TangoHandler::updateCameraImageIntoTexture: Failed to connect the texture id with error code: %d", result);
      return false;
    }
    textureIdConnected = true;
  }

  TangoErrorType result = TangoService_updateTextureExternalOes(TANGO_CAMERA_COLOR, textureId, &lastTangoImageBufferTimestamp);

  std::time(&lastTangoImagebufferTimestampTime);  

  return result == TANGO_SUCCESS;
}

int TangoHandler::getSensorOrientation() const
{
  return sensorOrientation;
}

bool TangoHandler::hasLastTangoImageBufferTimestampChangedLately()
{
  std::time_t currentTime;
  std::time(&currentTime);
  return std::difftime(currentTime, lastTangoImagebufferTimestampTime) < 1.0;
}

}  // namespace tango_chromium
