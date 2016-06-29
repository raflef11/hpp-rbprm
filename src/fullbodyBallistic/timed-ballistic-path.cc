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

#include <hpp/util/debug.hh>
#include <hpp/model/configuration.hh>
#include <hpp/model/device.hh>
#include <hpp/model/joint.hh>
#include <hpp/model/joint-configuration.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/rbprm/fullbodyBallistic/timed-ballistic-path.hh>
#include <hpp/core/straight-path.hh>

namespace hpp {
  namespace rbprm {
    using model::displayConfig;
    using core::value_type;
    using core::vector_t;
    using core::interval_t;
    using core::size_type;
    
    TimedBallisticPath::TimedBallisticPath (const core::DevicePtr_t& device,
                                            core::ConfigurationIn_t init,
                                            core::ConfigurationIn_t end,
                                            value_type length,
                                            value_type alpha, value_type theta, value_type v0, value_type z0) :
      parent_t (interval_t (0, length), device->configSize (),
                device->numberDof ()), device_ (device), initial_ (init),
      end_ (end), length_ (length),alpha_(alpha),theta_(theta),v0_(v0),z0_(z0),g_(9.81)
    {
      assert (device);
    }
    
    TimedBallisticPath::TimedBallisticPath (const rbprm::BallisticPathPtr_t ballisticPath) :
      parent_t (interval_t(0,ballisticPath->length()),ballisticPath->device()->configSize(),ballisticPath->device()->numberDof ()),device_(ballisticPath->device()),initial_(ballisticPath->initial()),end_(ballisticPath->end()),length_(ballisticPath->length())
    {
      // TODO : get coeffs
    }
    
    TimedBallisticPath::TimedBallisticPath (const TimedBallisticPath& path) :
      parent_t (path), device_ (path.device_), initial_ (path.initial_),
      end_ (path.end_),length_ (path.length_),alpha_(path.alpha_),theta_(path.theta_),v0_(path.v0_),z0_(path.z0_)
    {
    }
    
    bool TimedBallisticPath::impl_compute (core::ConfigurationOut_t result,
                                           value_type t) const
    {
      if (t == 0 || initial_(0) == end_(0)) {
        result = initial_;
        return true;
      }
      if (t >= length_) {
        result = end_;
        return true;
      }
      
      const value_type u = t/length_;
      const size_type nbConfig = device_->configSize();
      const size_type ecsDim = device_->extraConfigSpace ().dimension ();
      // TODO compute xyz for t
      /* Quaternions interpolation */
      const core::JointPtr_t SO3joint = device_->getJointByName ("base_joint_SO3");
      const std::size_t rank = SO3joint->rankInConfiguration ();
      const core::size_type dimSO3 = SO3joint->configSize ();
      SO3joint->configuration ()->interpolate
          (initial_, end_, u, rank, result);
      
      /* if robot has internal DoF (except freeflyer ones) */
      // translation dimension of freeflyer hardcoded...
      // min value (to reach for u = u_max) hardcoded...
      // manual interpolation since joint not available with index...
      const std::size_t freeflyerDim = 3 + dimSO3;
      const bool hasInternalDof = nbConfig > ecsDim + freeflyerDim;
      const value_type maxVal = 0; // because here initial_ = end_ ...
      if (hasInternalDof) {
        for (core::size_type i = freeflyerDim; i<nbConfig-ecsDim; i++) {
          /* monopod leg interpolation
      if (u <= u_max) {
      const value_type u_prime = u / u_max;
      result (i) = (1 - u_prime) * initial_ (i) + u_prime * maxVal;
    }
    else {
      const value_type u_prime = (u - u_max) / (1 - u_max);
      result (i) = (1 - u_prime) * maxVal + u_prime * end_ (i);
      }*/
          /* classical interpolation for robot trunk and limbs */
          result (i) = (1 - u) * initial_ (i) + u * end_ (i);
        }
      }
      
      /* Normal vector interpolation
   result (nbConfig-ecsDim) = (1 - u) *
   initial_(nbConfig-ecsDim) + u*end_(nbConfig-ecsDim);
   result (nbConfig-ecsDim+1) = (1 - u) *
   initial_(nbConfig-ecsDim+1) + u*end_(nbConfig-ecsDim+1);
   result (nbConfig-ecsDim+2) = (1 - u) *
   initial_(nbConfig-ecsDim+2) + u*end_(nbConfig-ecsDim+2);*/
      return true;
    }
    
    
    core::PathPtr_t TimedBallisticPath::extract (const interval_t& subInterval) const throw (hpp::core::projection_error)
    {
      bool success;
      core::Configuration_t q1 ((*this) (subInterval.first, success)); // straight
      core::Configuration_t q2 ((*this) (subInterval.second, success)); // straight
      core::PathPtr_t result = rbprm::TimedBallisticPath::create(device_,q1,q2,computeLength(q1,q2),alpha_,theta_,v0_,z0_);
      return result;
    }
    
    core::PathPtr_t TimedBallisticPath::reverse () const{
      hppDout(notice, "reverse path parabola !!!!!!!!!!!!!!!!!!!!!!!!");
      bool success;
      core::Configuration_t q1 ((*this) (length_, success));
      core::Configuration_t q2 ((*this) (0, success));
      core::PathPtr_t result = TimedBallisticPath::create (device_, q1, q2, length_,
                                                           alpha_,theta_,v0_,z0_);
      return result;
    }
    
    core::DevicePtr_t TimedBallisticPath::device () const
    {
      return device_;
    }
    
    value_type TimedBallisticPath::computeLength
    (const core::ConfigurationIn_t q1, const core::ConfigurationIn_t q2) const {
      value_type lenght;
      // TODO compute max time (tMax)
      return lenght;
    }
    
    
    
  } //   namespace rbprm
} // namespace hpp

