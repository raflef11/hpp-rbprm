// Copyright (c) 2014, LAAS-CNRS
// Authors: Steve Tonneau (steve.tonneau@laas.fr)
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

#include <hpp/rbprm/rbprm-path-interpolation.hh>

namespace hpp {
  namespace rbprm {

    RbPrmInterpolationPtr_t RbPrmInterpolation::create (const core::PathVectorConstPtr_t path, const hpp::rbprm::RbPrmFullBodyPtr_t robot, const hpp::rbprm::State &start, const hpp::rbprm::State &end)
    {
        RbPrmInterpolation* rbprmDevice = new RbPrmInterpolation(path, robot, start, end);
        RbPrmInterpolationPtr_t res (rbprmDevice);
        res->init (res);
        return res;
    }

    RbPrmInterpolation::~RbPrmInterpolation()
    {
        // NOTHING
    }

    // ========================================================================

    std::vector<State> RbPrmInterpolation::Interpolate(const model::ObjectVector_t &collisionObjects, const double timeStep)
    {
        std::vector<State> states;
        states.push_back(this->start_);
        const core::interval_t& range = path_->timeRange();
        for(double i = range.first; i< range.second; i+= timeStep)
        {
            const State& previous = states.back();
            core::Configuration_t configuration = previous.configuration_;
            const core::Configuration_t configPosition = path_->operator ()(i);
            Eigen::Vector3d dir = configPosition.head<3>() - previous.configuration_.head<3>();
            fcl::Vec3f direction(dir[0], dir[1], dir[2]);
            bool nonZero(false);
            direction.normalize(&nonZero);
            if(!nonZero) direction = fcl::Vec3f(0,0,1.);
            configuration.head(configPosition.rows()) = configPosition;
            std::cout << "position " << configuration.head<3>() << std::endl;
            states.push_back(ComputeContacts(robot_,configuration,collisionObjects,direction));
        }
        states.push_back(this->end_);
        return states;
    }

    void RbPrmInterpolation::init(const RbPrmInterpolationWkPtr_t& weakPtr)
    {
        weakPtr_ = weakPtr;
    }

    RbPrmInterpolation::RbPrmInterpolation (const core::PathVectorConstPtr_t path, const hpp::rbprm::RbPrmFullBodyPtr_t robot, const hpp::rbprm::State &start, const hpp::rbprm::State &end)
        : path_(path)
        , start_(start)
        , end_(end)
        , robot_(robot)
    {
        // TODO
    }
  } // model
} //hpp
