//
// Copyright (c) 2016 CNRS
// Authors: Mylene Campana, Steve Tonneau (mcampana@laas.fr)
//
// This file is part of hpp-rbprm.
// hpp-rbprm is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-rbprm is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-core  If not, see
// <http://www.gnu.org/licenses/>.

#ifndef HPP_BALLISTIC_INTERPOLATION_HH
# define HPP_BALLISTIC_INTERPOLATION_HH

# include <hpp/rbprm/config.hh>
# include <hpp/rbprm/rbprm-fullbody.hh>
# include <hpp/core/path-vector.hh>
# include <hpp/core/problem.hh>
# include <hpp/model/device.hh>

# include <vector>

namespace hpp {
  namespace rbprm {
    HPP_PREDEF_CLASS(BallisticInterpolation);

    /// Interpolation class for transforming a path computed by RB-PRM into
    /// a discrete sequence of balanced contact configurations.
    ///
    class BallisticInterpolation;
    typedef boost::shared_ptr <BallisticInterpolation>
    BallisticInterpolationPtr_t;

    class HPP_RBPRM_DLLAPI BallisticInterpolation
    {
    public:
      /// Creates a smart pointer to the Interpolation class
      ///
      /// \param path the path returned by RB-PRM computation
      /// \param robot the FullBody instance considered for extending the part
      /// \param start the start full body configuration of the problem
      /// \param end the end full body configuration of the problem
      /// \return a pointer to the created BallisticInterpolation instance
      static BallisticInterpolationPtr_t create
	(const core::Problem& problem, const RbPrmFullBodyPtr_t robot,
	 const State& start, const State& end,
	 const core::PathVectorConstPtr_t path = core::PathVectorConstPtr_t());

      ~BallisticInterpolation();

      /// Transforms the path computed by RB-PRM into
      /// a discrete sequence of configurations.
      /// Do not return states anymore since no contact or stability
      /// TODO: collision avoidance ??
      ///
      /// \param timeStep the discretization step of the path.
      /// \return a pointer to the created BallisticInterpolation instance
      /*std::vector<model::Configuration_t> InterpolateConfigs
	(const double timeStep = 0.01);*/

      /// On each subpath, tranform the trunk path and the start-goal 
      /// states so that:
      /// the trunk is still following the parabola path
      /// the limbs are linearly interpolated
      core::PathVectorPtr_t InterpolateFullPath
	(const model::ObjectVector_t &collisionObjects);

      /// Tranform the trunk path and the start-goal states so that:
      /// the trunk is still following the parabola path
      /// the limbs are linearly interpolated
      /// TODO: collision avoidance ??
      BallisticPathPtr_t Interpolate (const model::Configuration_t q1,
				      const model::Configuration_t q2,
				      const core::value_type length,
				      const core::vector_t coefficients);

      const core::PathVectorConstPtr_t path_;
      const State start_;
      const State end_;

    protected:
      BallisticInterpolation (const core::Problem& problem,
			      const RbPrmFullBodyPtr_t robot,
			      const State& start, const State& end,
			      const core::PathVectorConstPtr_t path);

      ///
      /// \brief Initialization.
      ///
      void init (const BallisticInterpolationWkPtr_t& weakPtr);

      /// Fill current config with 0 and copy ECS part
      core::Configuration_t fillConfiguration (core::ConfigurationIn_t config,
					       std::size_t configSize);

      /// Compute direction vector for EFORT relatively to a config
      /// It is basically the difference between the landing velocity to the 
      /// config and the takeoff from the config.
      fcl::Vec3f computeDir (const core::vector_t V0,
			     const core::vector_t Vimp);

    private:
      const core::Problem problem_;
      RbPrmFullBodyPtr_t robot_; // device of fullbody
      BallisticInterpolationWkPtr_t weakPtr_;
    }; // class BallisticInterpolation
  } // namespace rbprm
} // namespace hpp

#endif // HPP_BALLISTIC_INTERPOLATION_HH
