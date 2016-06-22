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
#include <hpp/model/joint.hh>
#include <hpp/constraints/generic-transformation.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/core/locked-joint.hh>
#include <hpp/core/path.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/rbprm/rbprm-state.hh>
#include <hpp/rbprm/fullbodyBallistic/ballistic-path.hh>
#include <hpp/rbprm/fullbodyBallistic/ballistic-interpolation.hh>
#include <hpp/rbprm/stability/stability.hh>
#include <hpp/rbprm/ik-solver.hh>

namespace hpp {
  namespace rbprm {
    using model::displayConfig;
    using core::value_type;
    using core::vector_t;
    using core::Configuration_t;

    BallisticInterpolationPtr_t
    BallisticInterpolation::create (const core::Problem& problem,
				    const RbPrmFullBodyPtr_t robot,
				    const State &start,
				    const State &end,
				    const core::PathVectorConstPtr_t path) {
      BallisticInterpolation* rbprmDevice =
	new BallisticInterpolation(problem, robot, start, end, path);
      BallisticInterpolationPtr_t res (rbprmDevice);
      res->init (res);
      return res;
    }

    BallisticInterpolation::BallisticInterpolation
    (const core::Problem& problem, const RbPrmFullBodyPtr_t robot,
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
    
    fcl::Vec3f BallisticInterpolation::computeDir (const vector_t V0,
						   const vector_t Vimp) {
      fcl::Vec3f dir;
      for (int i = 0; i < 3; i++)
	dir [i] = V0 [i] - Vimp [i];
      return dir;
    }

    std::size_t BallisticInterpolation::computeLimbLength
    (const model::JointPtr_t limb, const model::JointPtr_t effector)
    {
      std::size_t start = limb->rankInConfiguration();
      std::size_t end = effector->rankInConfiguration()
	+ effector->neutralConfiguration().rows();
      return end - start;
    }

    Configuration_t BallisticInterpolation::fillConfiguration
    (Configuration_t config, std::size_t configSize)
    {
      Configuration_t result (configSize);
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
      hppDout (info, "ecs size in fillConfiguration= " << ecsSize);
      for (std::size_t k = 0; k < ecsSize; k++)
	result [configSize - ecsSize + k] = config [trunkSize - ecsSize + k];
      hppDout (info, "filled config= " << displayConfig (result));
      return result;
    }

    Configuration_t BallisticInterpolation::replaceLimbConfigsInFullConfig
    (const Configuration_t q_full, const Configuration_t q_contact,
     const std::vector <RbPrmLimbPtr_t> limbs) {
      Configuration_t result = q_full;
      const core::DevicePtr_t device = robot_->device_;
      for (std::size_t i = 0; i < limbs.size (); i++ ){
	  const RbPrmLimbPtr_t limb = limbs.at (i);
	  //model::JointPtr_t clone = device->getJointByName(limbName);
	  model::JointPtr_t effectorClone = device->getJointByName(limb->effector_->name ());
	  const std::size_t startRank (limb->limb_->rankInConfiguration());
	  const std::size_t length (computeLimbLength (limb->limb_, effectorClone));
	  hppDout (info, "startRank= " << startRank);
	  hppDout (info, "result (before replaceLimb)= " << displayConfig(result));
	  for (std::size_t i = 0; i < length; i++) {
	    std::size_t currentRank = startRank + i;
	    result (currentRank) = q_contact (currentRank);
	  }
	  hppDout (info, "result= " << displayConfig(result));
      }
      return result;
    }

    std::vector <RbPrmLimbPtr_t> BallisticInterpolation::activeLimbsFromROMs
    (const ParabolaPathPtr_t pp, const bool startConfig) {
      std::vector <RbPrmLimbPtr_t> activeLimbs;
      const T_Limb& limbs = robot_->GetLimbs();
      std::vector <std::string> ROMnames;
      if (startConfig)
	ROMnames = pp->initialROMnames_;
      else
	ROMnames = pp->endROMnames_;

      hppDout (info, "ROMnames size= " << ROMnames.size ());
      for (T_Limb::const_iterator it = limbs.begin(); it != limbs.end(); it++) {
	const RbPrmLimbPtr_t limb = it->second;
	const std::string effectorName = limb->effector_->name ();
	hppDout (info, "effectorName= " << effectorName);
	for (std::size_t k = 0; k < ROMnames.size (); k++) {
	  hppDout (info, "ROMnames.at(k)= " << ROMnames.at(k));
	  if (effectorName.compare (ROMnames.at(k)) == 0) {
	    hppDout (info, "found in common in name lists= " << effectorName);
	    activeLimbs.push_back (limb);
	  }
	}//for roms
      }//for limbs
      hppDout (info, "nb active limbs= " << activeLimbs.size ());
      return activeLimbs;
    }

// ========================================================================
    
    /*std::vector<Configuration_t>
    BallisticInterpolation::InterpolateConfigs (const double timeStep)
    {
      if(!path_) throw std::runtime_error ("Can not interpolate; not path given to interpolator ");
      std::vector<Configuration_t> configs;
      bool success;
      Configuration_t qStart = start_.configuration_;
      Configuration_t qEnd = end_.configuration_;
      core::DevicePtr_t robot = robot_->device_; // device of fullbody
      const std::size_t subPathNumber = path_->numberPaths ();
      hppDout (info, "number of sub-paths: " << subPathNumber);
      core::PathVectorPtr_t newPath = core::PathVector::create 
	(robot->configSize (), robot->numberDof ());

      for (std::size_t i=0; i<subPathNumber; i++) {
	core::PathPtr_t subpath = path_->pathAtRank (i);
	Configuration_t q1 = subpath->initial ();
	Configuration_t q2 = subpath->end ();
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
    (const core::value_type u_offset) {
      if(!path_) throw std::runtime_error ("Cannot interpolate; not path given to interpolator ");
      Configuration_t qStart = start_.configuration_;
      Configuration_t qEnd = end_.configuration_;
      core::DevicePtr_t robot = robot_->device_;
      Configuration_t q1 (robot->configSize ()), q2(robot->configSize ()),
	q1contact (robot->configSize ()), q2contact (robot->configSize ());
      Configuration_t q_trunk_offset1, q_trunk_offset2, qcof1, qcof2,
	q_contact_offset1, q_contact_offset2, q_contact_offset_full1,
	q_contact_offset_full2;
      BallisticPathPtr_t bp, bp1, bp2, bp3;
      const model::ObjectVector_t &collisionObjects =
	problem_.collisionObstacles();
      hppDout (info, "u_offset= " << u_offset);
      const std::size_t subPathNumber = path_->numberPaths ();
      hppDout (info, "number of sub-paths: " << subPathNumber);
      core::PathVectorPtr_t newPath = core::PathVector::create 
	(robot->configSize (), robot->numberDof ());
      robot_->noStability_ = true; // disable stability for waypoints
      vector_t V0 (3), Vimp (3); fcl::Vec3f dir;
      core::PathPtr_t subpath = path_->pathAtRank (0);
      State state, state1, state2;
      bool success;
      bool multipleBreaks, contactMaintained; //contacts broken
      value_type u_offset1 = u_offset, u_offset2 = 1-u_offset;

      std::map<std::string,core::CollisionValidationPtr_t> limbColVal = robot_->getLimbcollisionValidations ();

      for (std::size_t i = 0; i < subPathNumber - 1; i++) {
	hppDout (info, "B-interp on path nb: " << i);
	core::PathPtr_t subpath_next = path_->pathAtRank (i+1);
	ParabolaPathPtr_t pp = 
	  boost::dynamic_pointer_cast<ParabolaPath>(subpath);
	ParabolaPathPtr_t pp_next = 
	  boost::dynamic_pointer_cast<ParabolaPath>(subpath_next);
	const value_type pathLength = subpath->length ();
	
	if (i == 0) { // keep qStart config which already has contacts
	  hppDout (info, "keep start config");
	  q1contact = qStart;
	  state1 = start_;
	}
	else {
	  q1contact = q2contact;
	  state1 = state2;
	}
	q2 = fillConfiguration (subpath->end (), robot->configSize ());
	hppDout (info, "q2: " << displayConfig(q2));
	V0 = pp_next->V0_; // V0_i+1
	Vimp = pp->Vimp_; // Vimp_i
	dir = computeDir (V0, Vimp);
	hppDout (info, "dir (Vimp-V0)= " << dir);
	state2 = ComputeContacts(robot_, q2, collisionObjects, dir);
	q2contact = state2.configuration_;
	hppDout (info, "q2contact= " << displayConfig(q2contact));
	hppDout (info, "q1contact= " << displayConfig(q1contact));

	/* Offset contact positions */
	bool contact1_OK = false, contact2_OK = false;
	std::size_t iteration = 0;
	while (!contact1_OK && iteration < 10) { 
	  iteration++;
	  hppDout (info, "iteration_1= " << iteration);
	  hppDout (info, "u_offset1= " << u_offset1);
	  q_trunk_offset1 = (*subpath) (u_offset1*pathLength,success);
	  hppDout (info, "q_trunk_offset1= " << displayConfig (q_trunk_offset1));
	  dir = pp->evaluateVelocity (u_offset*pathLength);
	  // TODO: replace by something that keeps same contacts
	  //state = ComputeContacts(robot_, fillConfiguration (q_trunk_offset1, robot->configSize ()), collisionObjects, dir);
	  state = MaintainPreviousContacts (state1, limbColVal, fillConfiguration (q_trunk_offset1, robot->configSize ()), contactMaintained, multipleBreaks, 0);
	  //state = MaintainPreviousContacts (state1, limbColVal, q1contact, contactMaintained, multipleBreaks, 0); // DEBUG
	  q_contact_offset1 = state.configuration_;
	  hppDout (info, "q_contact_offset1= " << displayConfig (q_contact_offset1));
	  hppDout (info, "contactMaintained = " << contactMaintained);
	  hppDout (info, "multipleBreaks = " << multipleBreaks);
	  contact1_OK = !multipleBreaks;
	  if (!contact1_OK)
	    u_offset1 = u_offset1*0.8;
	}
	hppDout (info, "u_offset1= " << u_offset1);

	iteration = 0;
	while (!contact2_OK && iteration < 10) { 
	  iteration++;
	  hppDout (info, "iteration_2= " << iteration);
	  q_trunk_offset2 = (*subpath) (u_offset2*pathLength,success);
	  hppDout (info, "q_trunk_offset2= " << displayConfig (q_trunk_offset2));
	  dir = - pp->evaluateVelocity ((1-u_offset)*pathLength);
	  // TODO: replace by something that keeps same contacts
	  //state = ComputeContacts(robot_, fillConfiguration (q_trunk_offset2, robot->configSize ()), collisionObjects, dir);
	  state = MaintainPreviousContacts (state2, limbColVal, fillConfiguration (q_trunk_offset2, robot->configSize ()), contactMaintained, multipleBreaks, 0);
	  q_contact_offset2 = state.configuration_;
	  hppDout (info, "q_contact_offset2= " << displayConfig (q_contact_offset2));
	  hppDout (info, "contactMaintained = " << contactMaintained);
	  hppDout (info, "multipleBreaks = " << multipleBreaks);
	  contact2_OK = !multipleBreaks;
	  if (!contact2_OK) {
	    u_offset2 = 1-(1-u_offset2)*0.8;
	    hppDout (info, "u_offset2= " << u_offset2);
	  }
	}
	hppDout (info, "u_offset2= " << u_offset2);
	

	bp = Interpolate (q1contact, q2contact, pathLength, subpath->coefficients ());
	
	q_contact_offset_full1 = (*bp) (u_offset1*pathLength,success);
	q_contact_offset_full2 = (*bp) (u_offset2*pathLength,success);
	hppDout (info, "q_full_offset1= " << displayConfig (q_contact_offset_full1));
	hppDout (info, "q_full_offset2= " << displayConfig (q_contact_offset_full2));
	std::vector <RbPrmLimbPtr_t> activeLimbsStart = activeLimbsFromROMs
	  (pp, true);
	std::vector <RbPrmLimbPtr_t> activeLimbsEnd = activeLimbsFromROMs
	  (pp, false);
	qcof1 = replaceLimbConfigsInFullConfig (q_contact_offset_full1,
						q_contact_offset1,
						activeLimbsStart);
	qcof2 = replaceLimbConfigsInFullConfig (q_contact_offset_full2,
						q_contact_offset2,
						activeLimbsEnd);
	hppDout (info, "parabola length test, full= " << subpath->length () << ", partial1= " << pp->computeLength (q1contact, qcof1) << ", partial2= " << pp->computeLength (qcof1, qcof2) << ", partial3= " << pp->computeLength (qcof2, q2contact));

	bp1 = Interpolate (q1contact, qcof1,
			   pp->computeLength (q1contact, qcof1),
			   subpath->coefficients ());
	bp2 = Interpolate (qcof1, qcof2, pp->computeLength (qcof1, qcof2),
			   subpath->coefficients ());
	bp3 = Interpolate (qcof2, q2contact,
			   pp->computeLength (qcof2, q2contact),
			   subpath->coefficients ());

	newPath->appendPath (bp1);
	newPath->appendPath (bp2);
	newPath->appendPath (bp3);
	subpath = subpath_next;
	pp = boost::dynamic_pointer_cast<ParabolaPath>(subpath);
	
	if (i == subPathNumber - 2) { // subpath_next = final subpath
	  q1contact = q2contact;
	  state1 = state2;
	  q2contact = qEnd;
	  state2 = end_;

	  /* Offset contact positions */
	  iteration = 0;
	  while (!contact1_OK && iteration < 10) { 
	    iteration++;
	    hppDout (info, "iteration_1= " << iteration);
	    hppDout (info, "u_offset1= " << u_offset1);
	    q_trunk_offset1 = (*subpath) (u_offset1*pathLength,success);
	    hppDout (info, "q_trunk_offset1= " << displayConfig (q_trunk_offset1));
	    dir = pp->evaluateVelocity (u_offset*pathLength);
	    // TODO: replace by something that keeps same contacts
	    //state = ComputeContacts(robot_, fillConfiguration (q_trunk_offset1, robot->configSize ()), collisionObjects, dir);
	    state = MaintainPreviousContacts (state1, limbColVal, fillConfiguration (q_trunk_offset1, robot->configSize ()), contactMaintained, multipleBreaks, 0);
	    //state = MaintainPreviousContacts (state1, limbColVal, q1contact, contactMaintained, multipleBreaks, 0); // DEBUG
	    q_contact_offset1 = state.configuration_;
	    hppDout (info, "q_contact_offset1= " << displayConfig (q_contact_offset1));
	    hppDout (info, "contactMaintained = " << contactMaintained);
	    hppDout (info, "multipleBreaks = " << multipleBreaks);
	    contact1_OK = !multipleBreaks;
	    if (!contact1_OK)
	      u_offset1 = u_offset1*0.8;
	  }
	  hppDout (info, "u_offset1= " << u_offset1);

	  iteration = 0;
	  while (!contact2_OK && iteration < 10) { 
	    iteration++;
	    hppDout (info, "iteration_2= " << iteration);
	    q_trunk_offset2 = (*subpath) (u_offset2*pathLength,success);
	    hppDout (info, "q_trunk_offset2= " << displayConfig (q_trunk_offset2));
	    dir = - pp->evaluateVelocity ((1-u_offset)*pathLength);
	    // TODO: replace by something that keeps same contacts
	    //state = ComputeContacts(robot_, fillConfiguration (q_trunk_offset2, robot->configSize ()), collisionObjects, dir);
	    state = MaintainPreviousContacts (state2, limbColVal, fillConfiguration (q_trunk_offset2, robot->configSize ()), contactMaintained, multipleBreaks, 0);
	    q_contact_offset2 = state.configuration_;
	    hppDout (info, "q_contact_offset2= " << displayConfig (q_contact_offset2));
	    hppDout (info, "contactMaintained = " << contactMaintained);
	    hppDout (info, "multipleBreaks = " << multipleBreaks);
	    contact2_OK = !multipleBreaks;
	    if (!contact2_OK) {
	      u_offset2 = 1-(1-u_offset2)*0.8;
	      hppDout (info, "u_offset2= " << u_offset2);
	    }
	  }
	  hppDout (info, "u_offset2= " << u_offset2);

	  bp = Interpolate (q1contact, q2contact, subpath->length (),
			    subpath->coefficients ());
	  q_contact_offset_full1 = (*bp) (u_offset*pathLength,success);
	  q_contact_offset_full2 = (*bp) ((1-u_offset)*pathLength,success);
	  std::vector <RbPrmLimbPtr_t> activeLimbsStart = activeLimbsFromROMs
	    (pp, true);
	  std::vector <RbPrmLimbPtr_t> activeLimbsEnd = activeLimbsFromROMs
	    (pp, false);
	  qcof1 = replaceLimbConfigsInFullConfig (q_contact_offset_full1,
						  q_contact_offset1,
						  activeLimbsStart);
	  qcof2 = replaceLimbConfigsInFullConfig (q_contact_offset_full2,
						  q_contact_offset2,
						  activeLimbsEnd);
	  hppDout (info, "parabola length test, full= " << subpath->length () << ", partial1= " << pp->computeLength (q1contact, qcof1) << ", partial2= " << pp->computeLength (qcof1, qcof2) << ", partial3= " << pp->computeLength (qcof2, q2contact));

	  bp1 = Interpolate (q1contact, qcof1,
			     pp->computeLength (q1contact, qcof1),
			     subpath->coefficients ());
	  bp2 = Interpolate (qcof1, qcof2, pp->computeLength (qcof1, qcof2),
			     subpath->coefficients ());
	  bp3 = Interpolate (qcof2, q2contact,
			     pp->computeLength (qcof2, q2contact),
			     subpath->coefficients ());

	  newPath->appendPath (bp1);
	  newPath->appendPath (bp2);
	  newPath->appendPath (bp3);
	}//if final subpath
      }// for subpaths
      //assert (false);
      return newPath;
    }

    core::PathVectorPtr_t BallisticInterpolation::InterpolateDirectPath
    (const core::value_type u_offset) {
      if(!path_) throw std::runtime_error ("Cannot interpolate; not path given to interpolator ");
      hppDout (info, "direct B-interpolation");
      Configuration_t qStart = start_.configuration_;
      Configuration_t qEnd = end_.configuration_;
      core::DevicePtr_t robot = robot_->device_;
      Configuration_t q1 (robot->configSize ()), q2(robot->configSize ()),
	q1contact (robot->configSize ()), q2contact (robot->configSize ());
      Configuration_t q_trunk_offset1, q_trunk_offset2, qcof1, qcof2,
	q_contact_offset1, q_contact_offset2, q_contact_offset_full1,
	q_contact_offset_full2;
      BallisticPathPtr_t bp, bp1, bp2, bp3;
      const model::ObjectVector_t &collisionObjects =
	problem_.collisionObstacles();
      hppDout (info, "u_offset= " << u_offset);
      const std::size_t subPathNumber = path_->numberPaths ();
      hppDout (info, "number of sub-paths: " << subPathNumber);
      core::PathVectorPtr_t newPath = core::PathVector::create 
	(robot->configSize (), robot->numberDof ());
      robot_->noStability_ = true; // disable stability for waypoints
      vector_t V0 (3), Vimp (3); fcl::Vec3f dir;
      State state;
      bool success;

      const core::PathPtr_t path = path_->pathAtRank (0);
      const ParabolaPathPtr_t pp = 
	boost::dynamic_pointer_cast<ParabolaPath>(path);
      const value_type pathLength = path->length ();
      const vector_t pathCoefs = path->coefficients ();
      q_trunk_offset1 = (*path) (u_offset*pathLength,success);
      q_trunk_offset2 = (*path) ((1-u_offset)*pathLength,success);
      hppDout (info, "q_trunk_offset1= " << displayConfig (q_trunk_offset1));
      hppDout (info, "q_trunk_offset2= " << displayConfig (q_trunk_offset2));
      dir = pp->evaluateVelocity (u_offset*pathLength);
      state = ComputeContacts(robot_, fillConfiguration (q_trunk_offset1, robot->configSize ()), collisionObjects, dir);
      q_contact_offset1 = state.configuration_;
      hppDout (info, "q_contact_offset1= " << displayConfig (q_contact_offset1));
      dir = - pp->evaluateVelocity ((1-u_offset)*pathLength);		
      state = ComputeContacts(robot_, fillConfiguration (q_trunk_offset2, robot->configSize ()), collisionObjects, dir);
      q_contact_offset2 = state.configuration_;
      hppDout (info, "q_contact_offset2= " << displayConfig (q_contact_offset2));

      bp = Interpolate (qStart, qEnd, pathLength, pathCoefs);
      q_contact_offset_full1 = (*bp) (u_offset*pathLength,success);
      q_contact_offset_full2 = (*bp) ((1-u_offset)*pathLength,success);
      hppDout (info, "q_full_offset1= " << displayConfig (q_contact_offset_full1));
      hppDout (info, "q_full_offset2= " << displayConfig (q_contact_offset_full2));
      std::vector <RbPrmLimbPtr_t> activeLimbsStart = activeLimbsFromROMs
	  (pp, true);
      std::vector <RbPrmLimbPtr_t> activeLimbsEnd = activeLimbsFromROMs
	(pp, false);
      qcof1 = replaceLimbConfigsInFullConfig (q_contact_offset_full1,
					      q_contact_offset1,
					      activeLimbsStart);
      qcof2 = replaceLimbConfigsInFullConfig (q_contact_offset_full1,
					      q_contact_offset1,
					      activeLimbsEnd);
      hppDout (info, "parabola length test, full= " << path->length () << ", partial1= " << pp->computeLength (qStart, qcof1) << ", partial2= " << pp->computeLength (qcof1, qcof2) << ", partial3= " << pp->computeLength (qcof2, qEnd));

	bp1 = Interpolate (qStart, qcof1, pp->computeLength (qStart, qcof1),
			   pathCoefs);
	bp2 = Interpolate (qcof1, qcof2, pp->computeLength (qcof1, qcof2),
			   pathCoefs);
	bp3 = Interpolate (qcof2, qEnd, pp->computeLength (qcof2, qEnd),
			   pathCoefs);

	newPath->appendPath (bp1);
	newPath->appendPath (bp2);
	newPath->appendPath (bp3);

      return newPath;
    }

    BallisticPathPtr_t BallisticInterpolation::Interpolate 
    (const Configuration_t q1, const Configuration_t q2,
     const core::value_type length, const core::vector_t coefficients) {
      BallisticPathPtr_t bp = BallisticPath::create (robot_->device_, q1, q2,
						     length,
						     coefficients);
      return bp;
    }

    // ========================================================================
    
    // assumes unit direction
    std::vector<bool> BallisticInterpolation::setMaintainRotationConstraints(const fcl::Vec3f&) // direction)
    {
      std::vector<bool> res;
      for(std::size_t i =0; i <3; ++i)
        {
	  res.push_back(true);
        }
      return res;
    }

    std::vector<bool> BallisticInterpolation::setTranslationConstraints(const fcl::Vec3f&)// normal)
    {
      std::vector<bool> res;
      for(std::size_t i =0; i <3; ++i)
        {
	  res.push_back(true);
        }
      return res;
    }

    void BallisticInterpolation::LockJointRec
    (const std::string& limb, const model::JointPtr_t joint,
     core::ConfigProjectorPtr_t& projector)
    {
      if(joint->name() == limb) return;
      const core::Configuration_t& c = joint->robot()->currentConfiguration();
      core::size_type rankInConfiguration (joint->rankInConfiguration ());
      projector->add(core::LockedJoint::create(joint,c.segment(rankInConfiguration, joint->configSize())));
      for(std::size_t i=0; i< joint->numberChildJoints(); ++i)
        {
	  LockJointRec(limb,joint->childJoint(i), projector);
        }
    }

    bool BallisticInterpolation::ComputeCollisionFreeConfiguration
    (State& current, core::CollisionValidationPtr_t validation,
     const hpp::rbprm::RbPrmLimbPtr_t& limb,
     model::ConfigurationOut_t configuration,
     const double robustnessTreshold, bool stability)
    {
      for(std::vector<sampling::Sample>::const_iterator cit = limb->sampleContainer_.samples_.begin();
	  cit != limb->sampleContainer_.samples_.end(); ++cit)
        {
	  sampling::Load(*cit, configuration);
	  hpp::core::ValidationReportPtr_t valRep (new hpp::core::CollisionValidationReport);
	  if(validation->validate(configuration, valRep) && (!stability || stability::IsStable(robot_,current) >=robustnessTreshold))
            {
	      current.configuration_ = configuration;
	      return true;
            }
        }
      return false;
    }

    State BallisticInterpolation::MaintainPreviousContacts
    (const State& previous,
     std::map<std::string,core::CollisionValidationPtr_t>& limbValidations,
     model::ConfigurationIn_t configuration, bool& contactMaintained,
     bool& multipleBreaks, const double robustnessTreshold)
    {
      multipleBreaks = false;
      contactMaintained = true;
      std::vector<std::string> brokenContacts;
      // iterate over every existing contact and try to maintain them
      State current;
      current.configuration_ = configuration;
      model::Configuration_t config = configuration;
      core::ConfigurationIn_t save = robot_->device_->currentConfiguration();
      // iterate over contact filo list
      std::queue<std::string> previousStack = previous.contactOrder_;
      while(!previousStack.empty())
        {
	  const std::string name = previousStack.front();
	  previousStack.pop();
	  const RbPrmLimbPtr_t limb = robot_->GetLimbs().at(name);
	  // try to maintain contact
	  const fcl::Vec3f& ppos = previous.contactPositions_.at(name);
	  const fcl::Vec3f& normal = previous.contactNormals_.at(name);
	  core::ConfigProjectorPtr_t proj = core::ConfigProjector::create(robot_->device_,"proj", 1e-2, 30);
	  LockJointRec(limb->limb_->name(), robot_->device_->rootJoint(), proj);
	  const fcl::Vec3f z = limb->effector_->currentTransformation().getRotation() * limb->normal_;
	  const fcl::Matrix3f& rotation = previous.contactRotation_.at(name);
	  //proj->add(core::NumericalConstraint::create (constraints::Position::create("",robot_->device_, limb->effector_,fcl::Vec3f(0,0,0), ppos)));
	  fcl::Transform3f localFrame, globalFrame;
	  localFrame.setTranslation(ppos);
	  proj->add(core::NumericalConstraint::create (constraints::Position::create("",robot_->device_, limb->effector_,globalFrame, localFrame, setTranslationConstraints(normal))));
	  if(limb->contactType_ == hpp::rbprm::_6_DOF)
            {
	      proj->add(core::NumericalConstraint::create (constraints::Orientation::create("", robot_->device_, limb->effector_,rotation,setMaintainRotationConstraints(z))));
            }
	  if(proj->apply(config))
            {
	      hpp::core::ValidationReportPtr_t valRep (new hpp::core::CollisionValidationReport);
	      if(limbValidations.at(name)->validate(config, valRep))
                {
		  hppDout (info, "contact is valid");
		  hppDout (info, "contact name= " << name);
		  current.contacts_[name] = true;
		  current.contactPositions_[name] = previous.contactPositions_.at(name);
		  current.contactNormals_[name] = previous.contactNormals_.at(name);
		  current.contactRotation_[name] = previous.contactRotation_.at(name);
		  current.contactOrder_.push(name);
		  current.configuration_ = config;
                }
	      else
                {
		  hppDout (info, "contact is not valid");
		  contactMaintained = false;
		  ComputeCollisionFreeConfiguration(current,limbValidations.at(name),limb,current.configuration_,robustnessTreshold,false);
		  brokenContacts.push_back(name);
		  hppDout (info, "brokenContacts name= " << name);
                }
            }
	  else
            {
	      hppDout (info, "projection cannot be applied");
	      contactMaintained = false;
	      ComputeCollisionFreeConfiguration(current,limbValidations.at(name),limb,current.configuration_,robustnessTreshold,false);
	      brokenContacts.push_back(name);
	      hppDout (info, "brokenContacts name= " << name);
            }
        }
      // reload previous configuration
      robot_->device_->currentConfiguration(save);
      hppDout (info, "brokenContacts.size()= " << brokenContacts.size());
      if(brokenContacts.size() > 1)
        {
	  contactMaintained = false;
	  multipleBreaks = true;
        }
      return current;
      }

  } // model
} //hpp
