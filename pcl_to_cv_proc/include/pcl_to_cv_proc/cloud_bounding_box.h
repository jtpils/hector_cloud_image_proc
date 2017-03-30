//=================================================================================================
// Copyright (c) 2016, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

// Code copied from https://aisgit.informatik.uni-freiburg.de/speck/shakey/blob/922b6c92a4dd3f56dc1b9fc4057bfb1b65055e88/shakey_object_recognition/src/segmentation3d.cpp

#ifndef CLOUD_BOUNDING_BOX_H___
#define CLOUD_BOUNDING_BOX_H___


// Upper part verified with cv 2.4, lower with 3.x
// Needs testing if the 3.x include also works on indigo
#if CV_MAJOR_VERSION == 2
  #include <opencv/cv.h>
#elif CV_MAJOR_VERSION == 3
  #include <opencv2/imgproc/imgproc.hpp>
#endif



#include <pcl/point_cloud.h>

namespace cloud_bounding_box{


template <typename PointT>  
std::vector<Eigen::Vector3f> minimal2DBoundingBox(boost::shared_ptr<pcl::PointCloud<PointT> > cloud_p,
      pcl::ModelCoefficients::Ptr coefficients)
  {
    std::vector<Eigen::Vector3f> plane_bbx;

    if (coefficients->values.size() == 0)
      return plane_bbx;

    // store the plane parameters
    Eigen::Vector3f plane_normal;
    plane_normal.x() = coefficients->values[0];
    plane_normal.y() = coefficients->values[1];
    plane_normal.z() = coefficients->values[2];
    // compute an orthogonal normal to the plane normal
    Eigen::Vector3f v = plane_normal.unitOrthogonal();
    // take the cross product of the two normals to get
    // a thirds normal, on the plane
    Eigen::Vector3f u = plane_normal.cross(v);
    // project the 3D point onto a 2D plane
    std::vector<cv::Point2f> points;
    // choose a point on the plane
    Eigen::Vector3f p0(cloud_p->points[0].x,
               cloud_p->points[0].y,
                       cloud_p->points[0].z);
    for(unsigned int i = 0; i < cloud_p->points.size(); i++)
    {
      Eigen::Vector3f p3d(cloud_p->points[i].x,
                          cloud_p->points[i].y,
                          cloud_p->points[i].z);
      // subtract all 3D points with a point in the plane
      // this will move the origin of the 3D coordinate system
      // onto the plane
      p3d = p3d - p0;
      cv::Point2f p2d;
      p2d.x = p3d.dot(u);
      p2d.y = p3d.dot(v);
      points.push_back(p2d);
    }
    cv::Mat points_mat(points);
    cv::RotatedRect rrect = cv::minAreaRect(points_mat);
    cv::Point2f rrPts[4];
    rrect.points(rrPts);
    //store the bounding points in a vector
    for(unsigned int i=0; i<4; i++)
    {
      Eigen::Vector3f pbbx(rrPts[i].x*u + rrPts[i].y*v + p0);
      plane_bbx.push_back(pbbx);
    }
    Eigen::Vector3f center(rrect.center.x*u + rrect.center.y*v + p0);
    plane_bbx.push_back(center);
    // 0-3: 4 corner points, 4: mean
    return plane_bbx;
  }
  
}

#endif
