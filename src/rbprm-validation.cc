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

#include <hpp/rbprm/rbprm-validation.hh>
#include <hpp/core/collision-validation.hh>
#include <hpp/rbprm/rbprm-validation-report.hh>

namespace
{
    hpp::core::CollisionValidationPtr_t tuneFclValidation(const hpp::model::RbPrmDevicePtr_t& robot)
    {
        hpp::core::CollisionValidationPtr_t validation = hpp::core::CollisionValidation::create(robot);
        validation->collisionRequest_.enable_contact = true;
        return validation;
    }

    hpp::rbprm::T_RomValidation createRomValidations(const hpp::model::RbPrmDevicePtr_t& robot,
                                                     const std::map<std::string, hpp::rbprm::NormalFilter>& normalFilters)
    {
        hpp::rbprm::T_RomValidation result;
        for(hpp::model::T_Rom::const_iterator cit = robot->robotRoms_.begin();
            cit != robot->robotRoms_.end(); ++cit)
        {
            std::map<std::string, hpp::rbprm::NormalFilter>::const_iterator cfit = normalFilters.find(cit->first);
            if(cfit != normalFilters.end())
            {
                result.insert(std::make_pair(cit->first, hpp::rbprm::RbPrmRomValidation::create(cit->second, cfit->second)));
            }
            else
            {
                result.insert(std::make_pair(cit->first, hpp::rbprm::RbPrmRomValidation::create(cit->second)));
            }
        }
        return result;
    }
}

namespace hpp {
  using namespace core;
  namespace rbprm {

    RbPrmValidationPtr_t RbPrmValidation::create
    (const model::RbPrmDevicePtr_t& robot, const std::vector<std::string>& filter, const std::map<std::string, rbprm::NormalFilter>& normalFilters)
    {
      RbPrmValidation* ptr = new RbPrmValidation (robot, filter, normalFilters);
      return RbPrmValidationPtr_t (ptr);
    }

    RbPrmValidation::RbPrmValidation (const model::RbPrmDevicePtr_t& robot
                                      , const std::vector<std::string>& filter, const std::map<std::string, rbprm::NormalFilter>& normalFilters)
        : trunkValidation_(tuneFclValidation(robot))
        , romValidations_(createRomValidations(robot, normalFilters))
        , defaultFilter_(filter)
    {
        for(std::vector<std::string>::const_iterator cit = defaultFilter_.begin();
            cit != defaultFilter_.end(); ++cit)
        {
            if(romValidations_.find(*cit) == romValidations_.end())
            {
                std::cout << "warning: default filter impossible to match in rbprmshooter" << std::endl;
            }
        }
    }

    bool RbPrmValidation::validateRoms(const core::Configuration_t& config,
                      const std::vector<std::string>& filter, ValidationReportPtr_t& validationReport,
                      bool throwIfInValid)
    {
      /*  hppDout(notice, "Filters = ");
        for(std::vector<std::string>::const_iterator it = filter.begin() ; it != filter.end() ; ++it)
          hppDout(notice,"filter : "<<*it);*/

        RbprmValidationReportPtr_t rbprmReport(new RbprmValidationReport);
        if(validationReport){// if the trunk is in collision, we copy the informations in the new report
          CollisionValidationReportPtr_t colReport = boost::dynamic_pointer_cast<CollisionValidationReport>(validationReport);
          if(colReport->result.isCollision()){
            rbprmReport->object1 = colReport->object1;
            rbprmReport->object2 = colReport->object2;
            rbprmReport->result = colReport->result;
            rbprmReport->trunkInCollision = true;
          }else{
            rbprmReport->trunkInCollision=false;
          }
        }else{
          rbprmReport->trunkInCollision=false;
        }

        unsigned int filterMatch(0);
        for(T_RomValidation::const_iterator cit = romValidations_.begin();
            cit != romValidations_.end() && (filterMatch < 1 || filterMatch < filter.size()); ++cit)
        {
            if((filter.empty() || std::find(filter.begin(), filter.end(), cit->first) != filter.end())
                    && cit->second->validate(config, throwIfInValid))
            {
                ++filterMatch;
            }
        }
        if(filterMatch >= filter.size())
          rbprmReport->romsValid=true;
        else
          rbprmReport->romsValid=false;
        validationReport = rbprmReport;
        //std::string tr = (filterMatch >= filter.size()) ? "true" : "false";
        //hppDout(notice," validate romes ?" << filterMatch << " " <<  tr );
        return filterMatch >= filter.size();
    }

    bool RbPrmValidation::validateRoms(const core::Configuration_t& config, ValidationReportPtr_t& validationReport,bool throwIfInValid)
    {
        return validateRoms(config,defaultFilter_,validationReport,throwIfInValid);
    }

    bool RbPrmValidation::validateRoms(const core::Configuration_t& config,const std::vector<std::string>& filter, bool throwIfInValid)
    {
        ValidationReportPtr_t unusedReport;
        return validateRoms(config,filter,unusedReport,throwIfInValid);
    }

    bool RbPrmValidation::validateRoms(const core::Configuration_t& config,bool throwIfInValid ){
      ValidationReportPtr_t unusedReport;
      return validateRoms(config,defaultFilter_,unusedReport,throwIfInValid);
    }

    /*bool RbPrmValidation::validate (const Configuration_t& config)
    {
        return trunkValidation_->validate(config)
             && validateRoms(config, defaultFilter_);
    }*/

    //needed (pure virtual in base class), deprecated
    bool RbPrmValidation::validate (const Configuration_t& config,bool /*throwIfInValid*/)
    {
      return trunkValidation_->validate(config)
           && validateRoms(config, defaultFilter_);
    }

    bool RbPrmValidation::validate (const core::Configuration_t& config, core::ValidationReport& validationReport,bool /*throwIfInValid */){
      ValidationReportPtr_t reportPtr(&validationReport);
      return validate(config,reportPtr);
    }


    bool RbPrmValidation::validate (const Configuration_t& config,
               ValidationReportPtr_t& validationReport)
    {
        return trunkValidation_->validate(config, validationReport)
                && validateRoms(config, defaultFilter_,validationReport);
    }

    bool RbPrmValidation::validate (const Configuration_t& config,
                    ValidationReportPtr_t& validationReport,
                    const std::vector<std::string>& filter)
    {
        return trunkValidation_->validate(config, validationReport)
                && validateRoms(config, filter,validationReport);
    }

    void RbPrmValidation::addObstacle (const CollisionObjectPtr_t& object)
    {
        trunkValidation_->addObstacle(object);
        for(T_RomValidation::const_iterator cit = romValidations_.begin();
            cit != romValidations_.end(); ++cit)
        {
            cit->second->addObstacle(object);
        }
    }

    void RbPrmValidation::removeObstacleFromJoint
    (const JointPtr_t& joint, const CollisionObjectPtr_t& obstacle)
    {
        trunkValidation_->removeObstacleFromJoint(joint, obstacle);
        for(T_RomValidation::const_iterator cit = romValidations_.begin();
            cit != romValidations_.end(); ++cit)
        {
            cit->second->removeObstacleFromJoint(joint, obstacle);
        }
    }

  }// namespace rbprm
}// namespace hpp
