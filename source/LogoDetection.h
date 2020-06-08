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

#pragma once

/* open cv header */
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

/**
 * @brief VX (VX Apps) Namespace.
 */
namespace VX {

  /**
   * @brief Detecting logo.
   * @author Florian Becker <fb\@vxapps.com> (VX Apps)
   */
  class LogoDetection {

  public:
    /**
     * @brief Defacult constuctor for LogoDetection.
     * @param _logo Logo processing always the same.
     * @note USE GRAY IMAGE
     */
    LogoDetection( cv::Mat _logo );

    /**
     * @brief Detect the logo.
     * @param _image Detect the logo on this image.
     * @return Rectangle of the founded logo.
     * @note USE GRAY IMAGE
     */
    cv::Rect area( const cv::Mat &_image ) const;

  private:
    /**
     * @brief Member for logo.
     */
    cv::Mat m_logo = {};

    /**
     * @brief Corners from m_logo.
     */
    std::vector<cv::Point2f> m_objectCorners = {};

    /**
     * @brief Keypoints of m_logo.
     */
    std::vector<cv::KeyPoint> m_keypointsSource = {};

    /**
     * @brief Descitptors of m_logo.
     */
    cv::Mat m_descriptorsSource = {};

    /**
     * @brief Surf detector.
     */
    cv::Ptr<cv::xfeatures2d::SURF> m_detector = nullptr;

    /**
     * @brief Check if the points are intersecting.
     * @param _p1   Point 1.
     * @param _p2   Point 2.
     * @param _q1   Query point 1.
     * @param _q2   Query point 2.
     * @return True, if the points are intersecting - otherweise false.
     */
    bool isIntersecting( cv::Point2f &_p1, cv::Point2f &_p2, cv::Point2f &_q1, cv::Point2f &_q2 ) const;
  };
}
