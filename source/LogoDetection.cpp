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

/* local header */
#include "LogoDetection.h"

/* open cv header */
#include <opencv2/xfeatures2d.hpp>

namespace VX {

  /** Hessian threshold */
  const int hessianThreshold = 600;

  /**
   * Ratio threshold
   * @note 0.7f better results, 0.8f more false/positive, but even more perfect matches
   */
  const float ratioThreshold = 0.7f;

  LogoDetection::LogoDetection( cv::Mat _logo )
    : m_logo( std::move( _logo ) ) {

    /* Get the corners from the image_1 ( the object to be "detected" ) */
    m_objectCorners.emplace_back( cv::Point2f( 0, 0 ) );
    m_objectCorners.emplace_back( cv::Point2f( m_logo.cols, 0 ) );
    m_objectCorners.emplace_back( cv::Point2f( m_logo.cols, m_logo.rows ) );
    m_objectCorners.emplace_back( cv::Point2f( 0, m_logo.rows ) );

    /* Create detector SURF */
    m_detector = cv::xfeatures2d::SURF::create( hessianThreshold );

    /* Create feature set from source */
    m_detector->detectAndCompute( m_logo, cv::Mat(), m_keypointsSource, m_descriptorsSource );
  }

  cv::Rect LogoDetection::area( const cv::Mat &_image ) const {

    /* Create feature set from source */
    std::vector<cv::KeyPoint> keypointsDestination;
    cv::Mat descriptorDestination;
    m_detector->detectAndCompute( _image, cv::Mat(), keypointsDestination, descriptorDestination );

    /* Detect features FANN */
    std::vector<std::vector<cv::DMatch>> matches;
    try {

      cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create( cv::DescriptorMatcher::FLANNBASED );
      matcher->knnMatch( m_descriptorsSource, descriptorDestination, matches, 2 );
    }
    catch ( const cv::Exception &_exception ) {
#ifdef DEBUG
      std::cout << _exception.msg << std::endl;
#else
      ( void )_exception;
#endif
    }

    /* Filter matches using the Lowe's ratio test */
    std::vector<cv::DMatch> goodMatches;
    for ( const auto &match : matches ) {

      if ( match[0].distance < ratioThreshold * match[1].distance ) {

        goodMatches.emplace_back( match[0] );
      }
    }

    if ( goodMatches.size() <= 4 ) {

      return {};
    }

    std::vector<cv::Point2f> obj;
    std::vector<cv::Point2f> scene;
    for ( const auto &goodMatch : goodMatches ) {

      /* Get the keypoints from the good matches */
      obj.emplace_back( m_keypointsSource[static_cast<size_t>( goodMatch.queryIdx )].pt );
      scene.emplace_back( keypointsDestination[static_cast<size_t>( goodMatch.trainIdx )].pt );
    }

    cv::Mat homography = cv::findHomography( obj, scene, cv::RANSAC );

    /* There is not enough matching points, so there seams to be no logo */
    if ( homography.empty() ) {

      return {};
    }

    /* Get the scene corners */
    std::vector<cv::Point2f> sceneCorners( 4 );
    cv::perspectiveTransform( m_objectCorners, sceneCorners, homography );

    /* Do not return negative Values, these are not valid */
    for ( const cv::Point2f &sceneCorner : sceneCorners ) {

      if ( sceneCorner.x < 0 || sceneCorner.y < 0 ) {

#ifdef DEBUG
        std::cout << "Negative scene points" << std::endl;
#endif
        return {};
      }
    }

    /* Do not return intercepting lines */
    if ( isIntersecting( sceneCorners[0], sceneCorners[1], sceneCorners[2], sceneCorners[3] ) ) {

#ifdef DEBUG
      std::cout << "Intersecting points" << std::endl;
#endif
      return {};
    }
    return cv::boundingRect( sceneCorners );
  }

  bool LogoDetection::isIntersecting( cv::Point2f &_p1, cv::Point2f &_p2, cv::Point2f &_q1, cv::Point2f &_q2 ) const {

    return ( ( ( _q1.x - _p1.x ) * ( _p2.y - _p1.y ) - ( _q1.y - _p1.y ) * ( _p2.x - _p1.x ) )
             * ( ( _q2.x - _p1.x ) * ( _p2.y - _p1.y ) - ( _q2.y - _p1.y ) * ( _p2.x - _p1.x ) ) < 0 )
           &&
           ( ( ( _p1.x - _q1.x ) * ( _q2.y - _q1.y ) - ( _p1.y - _q1.y ) * ( _q2.x - _q1.x ) )
             * ( ( _p2.x - _q1.x ) * ( _q2.y - _q1.y ) - ( _p2.y - _q1.y ) * ( _q2.x - _q1.x ) ) < 0 );
  }
}
