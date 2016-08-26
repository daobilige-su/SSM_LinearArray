// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef G2O_EDGE_SE3_TRACKXYZ_LINEAR_H
#define G2O_EDGE_SE3_TRACKXYZ_LINEAR_H

//#include "g2o_types_slam3d_api.h"

#include "../core/base_binary_edge.h"
#include "vertex_se3.h"
#include "vertex_pointxyz.h"

#include "../stuff/misc.h"

namespace g2o {

  //namespace tutorial {

    /**
     * \brief 2D edge between two Vertex2, i.e., the odometry
     */
    class EdgeSE3PointXYZLinear : public BaseBinaryEdge<1, Matrix<double,1,1>, VertexSE3, VertexPointXYZ>
    {
      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        EdgeSE3PointXYZLinear();

        void computeError()
        {
          const VertexSE3* v1 = static_cast<const VertexSE3*>(_vertices[0]);
          const VertexPointXYZ* v2 = static_cast<const VertexPointXYZ*>(_vertices[1]);

          Eigen::Vector3d point3d_local_coord = (v1->estimate().inverse() * v2->estimate());
          double angle = atan2(point3d_local_coord[1], sqrt(pow(point3d_local_coord[0],2) + pow(point3d_local_coord[2],2)) );

          _error(0,0) = normalize_theta(_measurement(0,0) - angle );

	  //std::cout<<pow(point3d_local_coord[1],2)<<" , "<<pow(point3d_local_coord[3],2)<<std::endl;
          //std::cout<<pow(2,2)<<" , "<<pow(3,2)<<std::endl;
          //std::cout<<sqrt(pow(point3d_local_coord[1],2) + pow(point3d_local_coord[3],2))<<std::endl;
	  //std::cout<<"local_coord: "<<point3d_local_coord<<std::endl;
	  //std::cout<<"m: "<<_measurement<<", a: "<<angle<<std::endl;
	  //std::cout<<_error[0]<<std::endl;
          //_error[0]=0;
        }
  
        void setMeasurement(const Matrix<double,1,1>& m){
          _measurement = m;
        }

        virtual bool read(std::istream& is);
        virtual bool write(std::ostream& os) const;

      protected:
        Matrix<double,1,1> _inverseMeasurement;
    };

  //}

} // end namespace

#endif
