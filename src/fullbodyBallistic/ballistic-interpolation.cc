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
#include <hpp/rbprm/planner/parabola-path.hh>

namespace hpp {
  namespace rbprm {
    using model::displayConfig;
    using core::value_type;
    using core::vector_t;

    BallisticInterpolationPtr_t
    BallisticInterpolation::create (const core::Problem& problem,
				    const hpp::rbprm::RbPrmFullBodyPtr_t robot,
				    const hpp::rbprm::State &start,
				    const hpp::rbprm::State &end,
				    const core::PathVectorConstPtr_t path) {
      BallisticInterpolation* rbprmDevice =
	new BallisticInterpolation(problem, robot, start, end, path);
      BallisticInterpolationPtr_t res (rbprmDevice);
      res->init (res);
      return res;
    }

    BallisticInterpolation::BallisticInterpolation
    (const core::Problem& problem, const hpp::rbprm::RbPrmFullBodyPtr_t robot,
     const hpp::rbprm::State &start, const State& end,
     const core::PathVectorConstPtr_t path) : 
      path_(path), start_(start), end_(end), problem_ (problem), robot_(robot) 
	
    {
      // TODO
    }

    BallisticInterpolation::~BallisticInterpolation()
    {
      // NOTHING
    }

    // ========================================================================
    
    fcl::Vec3f BallisticInterpolation::computeDir
    (const vector_t V0, const vector_t Vimp) {
      fcl::Vec3f dir;
      for (int i = 0; i < 3; i++)
	dir [i] = V0 [i] - Vimp [i];
      return dir;
    }

    core::Configuration_t BallisticInterpolation::fillConfiguration
    (core::ConfigurationIn_t config, std::size_t configSize)
    {
      core::Configuration_t result (configSize);
      std::size_t trunkSize = config.size ();
      std::size_t ecsSize = robot_->device_->extraConfigSpace ().dimension ();
      hppDout (info, "original config= " << displayConfig (config));
      for (std::size_t j = 0; j < configSize - ecsSize; j++) {
	if (j < trunkSize)
	  result [j] = config [j];
	else
	  result [j] = 0;
      }
      // copy extra-configs at the end of the config
      for (std::size_t k = 0; k < ecsSize; k++)
	result [configSize - ecsSize + k] = config [trunkSize - ecsSize + k];
      hppDout (info, "filled config= " << displayConfig (result));
      return result;
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
    (const model::ObjectVector_t &co) {
      if(!path_) throw std::runtime_error ("Cannot interpolate; not path given to interpolator ");
      model::Configuration_t qStart = start_.configuration_;
      model::Configuration_t qEnd = end_.configuration_;
      core::DevicePtr_t robot = robot_->device_;
      core::Configuration_t q1 (robot->configSize ()), q2(robot->configSize ()),
	q1contact (robot->configSize ()), q2contact (robot->configSize ());
      const model::ObjectVector_t &collisionObjects =
	problem_.collisionObstacles();
      hppDout (info, "compare size of collision-objects from PS and problem:");
      hppDout (info, "problem_solver: " << co.size ());
      hppDout (info, "problem: " << collisionObjects.size ());
      const std::size_t subPathNumber = path_->numberPaths ();
      hppDout (info, "number of sub-paths: " << subPathNumber);
      core::PathVectorPtr_t newPath = core::PathVector::create 
	(robot->configSize (), robot->numberDof ());
      robot_->noStability_ = true; // disable stability for waypoints
      vector_t V0 (3), Vimp (3); fcl::Vec3f dir;
      core::PathPtr_t subpath = path_->pathAtRank (0);

      for (std::size_t i = 0; i < subPathNumber - 1; i++) {
	hppDout (info, "B-interp on path nb: " << i);
	core::PathPtr_t subpath_next = path_->pathAtRank (i+1);
	ParabolaPathPtr_t pp = 
	  boost::dynamic_pointer_cast<ParabolaPath>(subpath);
	ParabolaPathPtr_t pp_next = 
	  boost::dynamic_pointer_cast<ParabolaPath>(subpath_next);
	if (i == 0) { // keep qStart config which already has contacts
	  hppDout (info, "keep start config");
	  q1contact = qStart;
	}
	else {
	  q1contact = q2contact;
	}
	q2 = fillConfiguration (subpath->end (), robot->configSize ());
	V0 = pp_next->V0_; // V0_i+1
	Vimp = pp->Vimp_; // Vimp_i
	dir = computeDir (V0, Vimp);
	hppDout (info, "dir (Vimp-V0)= " << dir);
	State state2 = ComputeContacts(robot_, q2, collisionObjects, dir);
	q2contact = state2.configuration_;
	hppDout (info, "q2contact= " << displayConfig(q2contact));
	BallisticPathPtr_t bp = Interpolate (q1contact, q2contact,
					     subpath->length (),
					     subpath->coefficients ());
	newPath->appendPath (bp);
	subpath = subpath_next;
	
	if (i == subPathNumber - 2) { // subpath_next = final subpath
	  q1contact = q2contact;
	  q2contact = qEnd;
	  BallisticPathPtr_t bp = Interpolate (q1contact, q2contact,
					       subpath->length (),
					       subpath->coefficients ());
	  newPath->appendPath (bp);
	}
	
      }
      hppDout (info, "new ballistic path vector: " << *newPath);
      return newPath;
    }

    BallisticPathPtr_t BallisticInterpolation::Interpolate 
    (const model::Configuration_t q1, const model::Configuration_t q2,
     const core::value_type length, const core::vector_t coefficients) {
      BallisticPathPtr_t bp = BallisticPath::create (robot_->device_, q1, q2,
						     length,
						     coefficients);
      return bp;
    }

    void BallisticInterpolation::init(const BallisticInterpolationWkPtr_t& weakPtr)
    {
      weakPtr_ = weakPtr;
    }

  } // model
} //hpp
