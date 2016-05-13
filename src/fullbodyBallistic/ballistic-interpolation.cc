// Copyright (c) 2016, LAAS-CNRS
// Authors: Mylene Campana (mcampana@laas.fr), Steve Tonneau
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
// hpp-rbprm. If not, see <http://www.gnu.org/licenses/>.

#include <hpp/util/debug.hh>
#include <hpp/model/configuration.hh>
#include <hpp/core/path.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/rbprm/rbprm-state.hh>
#include <hpp/rbprm/fullbodyBallistic/ballistic-path.hh>
#include <hpp/rbprm/fullbodyBallistic/ballistic-interpolation.hh>

namespace hpp {
  namespace rbprm {
    using model::displayConfig;
    using core::value_type;

    BallisticInterpolationPtr_t
    BallisticInterpolation::create (const hpp::rbprm::RbPrmFullBodyPtr_t robot,
				    const hpp::rbprm::State &start,
				    const hpp::rbprm::State &end,
				    const core::PathVectorConstPtr_t path) {
      BallisticInterpolation* rbprmDevice =
	new BallisticInterpolation(path, robot, start, end);
      BallisticInterpolationPtr_t res (rbprmDevice);
      res->init (res);
      return res;
    }

    BallisticInterpolation::~BallisticInterpolation()
    {
      // NOTHING
    }

    // ========================================================================

    namespace
    {
      core::Configuration_t configPosition
      (core::ConfigurationIn_t previous, const core::PathVectorConstPtr_t path,
       double i)
      {
        core::Configuration_t configuration = previous;
	bool success;
        const core::Configuration_t configPosition =
	  path->operator () (std::min(i, path->timeRange().second), success);
        configuration.head(configPosition.rows()) = configPosition;
        return configuration;
      }
    }
    
    core::Configuration_t fillConfiguration (core::ConfigurationIn_t config,
					     std::size_t configSize)
    {
      core::Configuration_t result (configSize);
      std::size_t trunkSize = config.size ();
      hppDout (info, "trunkSize= " << trunkSize);
      for (std::size_t j = 0; j < configSize; j++) {
	if (j < trunkSize)
	  result [j] = config [j];
	else
	  result [j] = 0;
      }
      hppDout (info, "filled config= " << displayConfig(config));
    }

    /*std::vector<model::Configuration_t>
    BallisticInterpolation::InterpolateConfigs (const double timeStep)
    {
      if(!path_) throw std::runtime_error ("Can not interpolate; not path given to interpolator ");
      std::vector<model::Configuration_t> configs;
      bool success;
      model::Configuration_t qStart = start_.configuration_;
      model::Configuration_t qEnd = end_.configuration_;
      core::DevicePtr_t robot = robot_->device_; // device of fullbody
      const std::size_t subPathNumber = path_->numberPaths ();
      hppDout (info, "number of sub-paths: " << subPathNumber);
      core::PathVectorPtr_t newPath = core::PathVector::create 
	(robot->configSize (), robot->numberDof ());

      for (std::size_t i=0; i<subPathNumber; i++) {
	core::PathPtr_t subpath = path_->pathAtRank (i);
	model::Configuration_t q1 = subpath->initial ();
	model::Configuration_t q2 = subpath->end ();
	BallisticPathPtr_t bp = Interpolate (q1, q2, subpath->length (),
					     subpath->coefficients ());
	const core::interval_t& range = path_->timeRange();
	for(value_type t = range.first; t< range.second; t += timeStep)
	  {
	    configs.push_back( bp->operator () (t, success));
	  }
      }
      return configs;
      }*/

    core::PathVectorPtr_t BallisticInterpolation::InterpolateFullPath
    (const model::ObjectVector_t &collisionObjects) {
      if(!path_) throw std::runtime_error ("Cannot interpolate; not path given to interpolator ");
      model::Configuration_t qStart = start_.configuration_;
      model::Configuration_t qEnd = end_.configuration_;
      core::DevicePtr_t robot = robot_->device_;
      const std::size_t subPathNumber = path_->numberPaths ();
      hppDout (info, "number of sub-paths: " << subPathNumber);
      core::PathVectorPtr_t newPath = core::PathVector::create 
	(robot->configSize (), robot->numberDof ());

      for (std::size_t i=0; i<subPathNumber; i++) {
	core::PathPtr_t subpath = path_->pathAtRank (i);
	core::Configuration_t q1 = fillConfiguration (subpath->initial (),
						      robot->configSize ());
	core::Configuration_t q2 = fillConfiguration (subpath->end (),
						      robot->configSize ());
	assert (q1.size () == robot->configSize ());
	fcl::Vec3f dir;
	dir [0] = 0; dir [1] = 0; dir [2] = 1;
	State state1 =ComputeContacts(robot_, q1, collisionObjects, dir);
	core::Configuration_t q1contact = state1.configuration_;
	State state2 =ComputeContacts(robot_, q2, collisionObjects, dir);
	core::Configuration_t q2contact = state2.configuration_;
	BallisticPathPtr_t bp = Interpolate (q1contact, q2contact,
					     subpath->length (),
					     subpath->coefficients ());
	newPath->appendPath (bp);
      }
      hppDout (info, "new ballistic path vector: " << *newPath);
      return newPath;
    }

    BallisticPathPtr_t BallisticInterpolation::Interpolate 
    (const model::Configuration_t q1, const model::Configuration_t q2,
     const core::value_type length, const core::vector_t coefficients) {
      if(!path_) throw std::runtime_error ("Can not interpolate; not path given to interpolator ");
      core::DevicePtr_t robot = robot_->device_;
      BallisticPathPtr_t bp = BallisticPath::create (robot, q1, q2, length,
						     coefficients);
      return bp;
    }

    void BallisticInterpolation::init(const BallisticInterpolationWkPtr_t& weakPtr)
    {
      weakPtr_ = weakPtr;
    }

    BallisticInterpolation::BallisticInterpolation (const core::PathVectorConstPtr_t path, const hpp::rbprm::RbPrmFullBodyPtr_t robot, const hpp::rbprm::State &start, const hpp::rbprm::State &end)
      : path_(path)
      , start_(start)
      , end_(end)
      , robot_(robot)
    {
      // TODO
    }
  } // model
} //hpp
