//
// Copyright (c) 2014 CNRS
// Authors: Mylene Campana
//
// This file is part of hpp-core
// hpp-core is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-core is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-core  If not, see
// <http://www.gnu.org/licenses/>.

#ifndef HPP_RBPRM_PARABOLA_LIBRARY_HH
# define HPP_RBPRM_PARABOLA_LIBRARY_HH

# include <sstream>
# include <hpp/util/debug.hh>
//# include <hpp/fcl/math/transform.h>
# include <Eigen/Geometry>
# include <hpp/model/configuration.hh>
# include <hpp/model/device.hh>

namespace hpp {
  namespace rbprm {
    using model::displayConfig;
    using model::size_type;
    using core::value_type;


    /// Arrange robot orientation according to the surface normal direction
    /// So that the robot is "on" the surface, rotated
    inline core::Configuration_t setOrientation
    (const core::DevicePtr_t& robot, const core::Configuration_t& q) {
      core::Configuration_t qtest = q;
      //hppDout (info, "q: " << displayConfig (q));
      const core::JointPtr_t jointSO3 = robot->getJointVector () [1];
      const size_type indexSO3 = jointSO3->rankInConfiguration ();
      const size_type index = robot->configSize ()
	- robot->extraConfigSpace ().dimension ();

      const value_type nx = q [index];
      const value_type ny = q [index + 1];
      const value_type nz = q [index + 2];
      //const value_type theta = atan2 (2*qw*qz - 2*qx*qy, 1-2*qy*qy-2*qz*qz);
      const value_type theta = q [index + 3];
      //hppDout (info, "theta: " << theta);

      const int sign = -(theta > M_PI/2) - (theta < -M_PI/2) +
	(int) ((theta > -M_PI/2) && (theta < M_PI/2));
      
      value_type x12, y12, z12; // corresponding to R_r coordinates

      // See Matlab script or Latex doc for the different cases:
      if (theta != M_PI/2 && theta != -M_PI/2) {
	const value_type tantheta = tan(theta);
	if (nz != 0) {
	  x12= sign*nz/sqrt((1+tantheta*tantheta)
			    *nz*nz+(nx+ny*tantheta)*(nx+ny*tantheta));
	  y12 = x12*tantheta;
	  z12 = -x12*(nx+ny*tantheta)/nz;
	} else { // nz = 0 case
	  x12 = 0; y12 = 0; z12 = 1;
	}
      } else { // theta = +- pi/2
	x12 = 0;
	if (ny != 0) {
	  if (theta == M_PI/2)
	    z12 = 1/(1+(nz/ny)*(nz/ny));
	  else // theta = -pi/2
	    z12 = 1/(1+(nz/ny)*(nz/ny));
	  y12 = -nz/ny*z12;
	} else { // ny = 0
	  if (nx == 1) { // no matter y12 and z12)
	    y12 = 0.5;
	    z12 = 0.5;
	  } else { // nx = 1
	    y12 = 1;
	    z12 = 0;
	  }
	}
      }

      // cross-product
      const value_type zx = nz*x12-nx*z12;
      const value_type yz = ny*z12-nz*y12;
      const value_type xy = nx*y12-ny*x12;
      
      //fcl::Matrix3f A; // A: rotation matrix expressing R_r in R_0
      Eigen::Matrix<value_type,3,3> A;
      A (0,0) = x12; A (0,1) = yz; A (0,2) = nx;
      A (1,0) = y12; A (1,1) = zx; A (1,2) = ny;
      A (2,0) = z12; A (2,1) = xy; A (2,2) = nz;
      hppDout (info, "A: " << A);
      hppDout (info, "A.determinant (): " << A.determinant ());

      //fcl::Quaternion3f quat;
      Eigen::Quaternion<value_type> quat (A);
      //quat.fromRotation (A);
      //hppDout (info, "quat: " << quat);
	
      qtest [indexSO3] = quat.w (); // ORDER MAY BE WRONG
      qtest [indexSO3 + 1] = quat.x ();
      qtest [indexSO3 + 2] = quat.y ();
      qtest [indexSO3 + 3] = quat.z ();
      hppDout (info, "qtest: " << displayConfig (qtest));
      return qtest;
    }
  } //   namespace rbprm
} // namespace hpp

#endif // HPP_RBPRM_PARABOLA_LIBRARY_HH
