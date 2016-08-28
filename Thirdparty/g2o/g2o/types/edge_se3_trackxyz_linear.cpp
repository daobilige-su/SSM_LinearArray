/**
* This file is part of the SSM_LinearArray (Sound Sources Mapping
* using a Linear Microphone Array)
* developed by Daobilige Su <daobilige DOT su AT student DOT uts DOT edu DOT au>
*  
* This file is under the GPLv3 licence. 
*/


#include "edge_se3_trackxyz_linear.h"

namespace g2o {

    EdgeSE3PointXYZLinear::EdgeSE3PointXYZLinear() : BaseBinaryEdge<1, Matrix<double,1,1>, VertexSE3, VertexPointXYZ>()
    {
    }

    bool EdgeSE3PointXYZLinear::read(std::istream& is)
    {
      is >> _measurement(0,0) >> information()(0, 0);
      return true;
    }

    bool EdgeSE3PointXYZLinear::write(std::ostream& os) const
    {
      os << _measurement(0,0) << " " << information()(0,0);
      return os.good();
    }
} // end namespace
