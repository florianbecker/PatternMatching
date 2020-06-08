/*
 * Copyright (c) 2020 Florian Becker <fb@vxapps.com> (VX APPS).
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* open cv header */
#include <opencv2/opencv.hpp>

/* pylon header */
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Weverything"
#include <pylon/PylonIncludes.h>
#include <pylon/BaslerUniversalInstantCamera.h>
using namespace Basler_UniversalCameraParams;
#pragma clang diagnostic pop

/* qt header */
#include <QPainter>
#include <QPen>
#include <QPixmap>

/* local header */
#include "Camera.h"
#include "LogoDetection.h"
#include "PatternMatching.h"
#include "Timer.h"

namespace VX {

  /** Number of images to be grabbed. 0 is inifinity. */
  constexpr int countOfImagesToGrab = 0;

  /** Timeout of grabbing image - like internet request timeout. */
  constexpr int timeoutGrabbingSeconds = 5;

  /** Factor from seconds to milliseconds */
  constexpr int secondsToMilliseconds = 1000;

  Camera::Camera( PatternMatching *_patternMatching )
    : m_patternMatching( _patternMatching ) {

    /* Camera loop */
    startCamera();
  }

  void Camera::startCamera() {

    cv::Mat frame;

    /* Before using any pylon methods, the pylon runtime must be initialized. */
    Pylon::PylonAutoInitTerm autoInitTerm;

    /* Create an instant camera object with the camera device found first. */
    Pylon::CBaslerUniversalInstantCamera camera( Pylon::CTlFactory::GetInstance().CreateFirstDevice() );

    Pylon::CImageFormatConverter formatConverter;
    formatConverter.OutputPixelFormat = Pylon::PixelType_BGR8packed;

    Pylon::CPylonImage pylonImage;

    /* Open the camera */
    camera.Open();

    /* START CAMERA SETTINGS */
    /* White Balanceing: Continues -> Off */
    /* Select auto function ROI 1 */
    camera.AutoFunctionROISelector.SetValue( AutoFunctionROISelector_ROI1 );

    /* Disable the Balance White Auto auto function */
    /* for the auto function ROI selected */
    camera.AutoFunctionROIUseWhiteBalance.SetValue( false );

    /* Disable Balance White Auto by setting the operating mode to Continuous */
    camera.BalanceWhiteAuto.SetValue( BalanceWhiteAuto_Off );

    /* Select the Frame Start trigger */
    camera.TriggerSelector.SetValue( TriggerSelector_FrameStart );
    /* Disable triggered image acquisition for the Frame Start trigger  */
    camera.TriggerMode.SetValue( TriggerMode_Off );
    /* Set the trigger source to Line 1 */
    camera.TriggerSource.SetValue( TriggerSource_Line1 );

    /* Set the Exposure Auto auto function to its minimum lower limit */
    /* and its maximum upper limit */
    double minLowerLimit = camera.AutoExposureTimeLowerLimit.GetMin();
    double maxUpperLimit = camera.AutoExposureTimeUpperLimit.GetMax();
    camera.AutoExposureTimeLowerLimit.SetValue( minLowerLimit );
    camera.AutoExposureTimeUpperLimit.SetValue( maxUpperLimit );

    /* Disable Exposure Auto by setting the operating mode to Continuous */
    camera.ExposureAuto.SetValue( ExposureAuto_Off );
    camera.ExposureTime.SetValue( 1000.0, true );

    /* Disable Gain Auto by setting the operating mode to Continuous */
    camera.GainAuto.SetValue( GainAuto_Off );

    /* Disable Gamma 1.0 */
    camera.Gamma.SetValue( 1.0, true );

    /* Set the width to the maximum value */
    int64_t maxWidth = camera.WidthMax.GetValue();
    int64_t maxHeight = camera.HeightMax.GetValue();
    camera.Width.SetValue( maxWidth );
    camera.Height.SetValue( maxHeight );
    /* Set the offset to 0 */
    camera.OffsetX.SetValue( 0 );
    camera.OffsetY.SetValue( 0 );

    /* Disable any light source preset, if you use your own lighting system */
//    camera.BslLightSourcePreset.SetValue( BslLightSourcePreset_Off );

    camera.BslLightSourcePreset.SetValue( BslLightSourcePreset_Daylight5000K );

    /* END CAMERA SETTINGS */

    /* This smart pointer will receive the grab result data. */
    Pylon::CGrabResultPtr ptrGrabResult;

    cv::Mat logo = cv::imread( "visa.png", cv::IMREAD_GRAYSCALE );
    LogoDetection logoDetection( logo );

    cv::Mat frameGray;

    /* Start the grabbing of countOfImagesToGrab images. */
    /* The camera device is parameterized with a default configuration which */
    /* sets up free-running continuous acquisition. */
    if ( countOfImagesToGrab > 0 ) {

      camera.StartGrabbing( countOfImagesToGrab, Pylon::GrabStrategy_LatestImageOnly );
    }
    /* Continues grabbing */
    else {

      camera.StartGrabbing( Pylon::GrabStrategy_LatestImageOnly );
    }

#ifdef WITH_TRIGGER
    /* Enable triggered image acquisition for the Frame Start trigger */
    camera.TriggerMode.SetValue( TriggerMode_On );
#endif

    cv::Rect logoRect;

    QImage image;

    int fps = 0;
    Timer fpsTimer = Timer();
    fpsTimer.setInterval( [&]() {

      std::cout << "Frames per seconds: " << fps << std::endl;
      fps = 0;
    }, 1 * secondsToMilliseconds );

    while ( camera.IsGrabbing() ) {

      /* Wait for an image and then retrieve it. A timeout of 5000 ms is used. */
      camera.RetrieveResult( timeoutGrabbingSeconds * secondsToMilliseconds, ptrGrabResult, Pylon::TimeoutHandling_ThrowException );

      /* Image grabbed successfully? */
      if ( ptrGrabResult->GrabSucceeded() ) {

        ++fps;

        formatConverter.Convert( pylonImage, ptrGrabResult );
        /* Create openCV image */
        frame = cv::Mat( static_cast<int>( pylonImage.GetHeight() ), static_cast<int>( pylonImage.GetWidth() ), CV_8UC3, static_cast<uint8_t *>( pylonImage.GetBuffer() ) );

        cv::cvtColor( frame, frameGray, cv::COLOR_BGR2GRAY );
        frame.release();

        /* Detection - Logo */
        cv::Rect found = logoDetection.area( frameGray );
        frameGray.release();

        if ( !found.empty() && logoRect.empty() ) {

          logoRect = found;
        }
        image = QImage( static_cast<uchar *>( pylonImage.GetBuffer() ), static_cast<int>( pylonImage.GetWidth() ), static_cast<int>( pylonImage.GetHeight() ), static_cast<int>( pylonImage.GetAllocatedBufferSize() / pylonImage.GetHeight() ), QImage::Format_RGB888 );
        image = image.rgbSwapped();

        QPainter painter( &image );
        QPen pen;
        pen.setWidth( 4 );
        pen.setColor( Qt::red );
        painter.setPen( pen );
        painter.drawRect( found.x, found.y, found.width, found.height );

        if ( m_patternMatching ) {

          m_patternMatching->updateStream( image );
        }
      }
#ifdef DEBUG
      else {

        std::cout << "Error: " << ptrGrabResult->GetErrorCode() << " " << ptrGrabResult->GetErrorDescription() << std::endl;
      }
#endif
    }
  }
}
