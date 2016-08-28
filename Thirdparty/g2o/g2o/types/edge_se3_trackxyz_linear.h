/**
* This file is part of the SSM_LinearArray (Sound Sources Mapping
* using a Linear Microphone Array)
* developed by Daobilige Su <daobilige DOT su AT student DOT uts DOT edu DOT au>
*  
* This file is under the GPLv3 licence. 
*/


#ifndef G2O_EDGE_SE3_TRACKXYZ_LINEAR_H
#define G2O_EDGE_SE3_TRACKXYZ_LINEAR_H

#include "../core/base_binary_edge.h"
#include "vertex_se3.h"
#include "vertex_pointxyz.h"

#include "../stuff/misc.h"

namespace g2o {
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
        }
  
        void setMeasurement(const Matrix<double,1,1>& m){
          _measurement = m;
        }

        virtual bool read(std::istream& is);
        virtual bool write(std::ostream& os) const;

      protected:
        Matrix<double,1,1> _inverseMeasurement;
    };

} // end namespace

#endif
