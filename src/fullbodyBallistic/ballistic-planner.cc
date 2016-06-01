//
// Copyright (c) 2016 CNRS
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
#include <hpp/core/basic-configuration-shooter.hh>
#include <hpp/core/connected-component.hh>
#include <hpp/core/node.hh>
#include <hpp/core/edge.hh>
#include <hpp/core/path.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/roadmap.hh>
#include <hpp/rbprm/rbprm-state.hh>
#include <hpp/rbprm/fullbodyBallistic/ballistic-planner.hh>
#include <polytope/stability_margin.h>
#include "utils/algorithms.h"

#include <hpp/core/path-validation-report.hh>
#include <hpp/rbprm/rbprm-path-validation.hh>
#include <hpp/rbprm/rbprm-validation-report.hh>
#include <hpp/core/config-validations.hh>

namespace hpp {
  namespace rbprm {
    using model::displayConfig;
    using core::value_type;
    using core::vector_t;
    using model::size_type;

    BallisticPlanner::BallisticPlanner (const core::Problem& problem):
      PathPlanner (problem),
      configurationShooter_ (problem.configurationShooter()),
      qProj_ (problem.robot ()->configSize ()),
      smParabola_(rbprm::SteeringMethodParabola::create((core::ProblemPtr_t(&problem)))),
      rbRoadmap_(core::RbprmRoadmap::create (problem.distance (),problem.robot())), roadmap_(boost::dynamic_pointer_cast<core::Roadmap>(rbRoadmap_)),
      fullRobot_ (RbPrmFullBody::create(problem.robot ()))
    {
      hppDout(notice,"Constructor ballistic-planner");
    }

    BallisticPlanner::BallisticPlanner (const core::Problem& problem,
			    const core::RoadmapPtr_t& roadmap) :
      PathPlanner (problem, roadmap),
      configurationShooter_ (problem.configurationShooter()),
      qProj_ (problem.robot ()->configSize ()),
      smParabola_(rbprm::SteeringMethodParabola::create((core::ProblemPtr_t(&problem)))),
      rbRoadmap_(core::RbprmRoadmap::create (problem.distance (),problem.robot())), roadmap_(roadmap), fullRobot_ (RbPrmFullBody::create(problem.robot ()))
    {
      hppDout(notice,"Constructor ballistic-planner with Roadmap");
    }

    void BallisticPlanner::oneStep ()
    {
      core::DevicePtr_t robot (problem ().robot ());
      core::PathPtr_t localPath;
      DelayedEdge_t fwdDelayedEdge, bwdDelayedEdge;
      DelayedEdges_t fwdDelayedEdges;

      fcl::Vec3f dir; dir [0] = 0; dir [1] = 0; dir [2] = 1;
      model::ObjectVector_t collObjs = problem ().collisionObstacles ();
      
      // shoot a RB-valid random configuration using rbprm-shooter
      core::ConfigurationPtr_t q_rand;
      hppDout(notice,"# oneStep BEGIN");
      q_rand = configurationShooter_->shoot ();
      hppDout (info, "q_rand: " << displayConfig (*q_rand));

      // TODO: update cone-normal from contact areas  -> GWIC Pierre !!
      // TODO: dir = cone direction (we don't know the velocities yet..)
      
      // compute non-stable contact position from q_rand
      // this could be used to say "see, the cone is not that different"
      // so... can be done after planning !
      fullRobot_->noStability_ = true;
      State state = ComputeContacts (fullRobot_, *q_rand, collObjs, dir);
      core::ConfigurationPtr_t qcontact ( new core::Configuration_t (state.configuration_));
      hppDout (info, "qcontact: " << displayConfig (*qcontact)); // DON'T USE IT

      // TODO: update cone-normal from contact areas  -> GWIC Pierre !!

      // Add q_rand as a new node: here for the parabola, as the impact node
      core::NodePtr_t impactNode = roadmap ()->addNode (q_rand);
      impactNode->indexInRM (roadmap ()->nodeIndex_);
      roadmap ()->nodeIndex_++;
      core::RbprmNodePtr_t x_new = rbprmRoadmap()->addNode(q_rand);
      computeGIWC(x_new);

      // try to connect the random configuration to each connected component
      // of the roadmap.
      for (core::ConnectedComponents_t::const_iterator itcc =
	     roadmap ()->connectedComponents ().begin ();
	   itcc != roadmap ()->connectedComponents ().end (); ++itcc) {
	core::ConnectedComponentPtr_t cc = *itcc;
	// except its own connected component of course
	if (cc != impactNode->connectedComponent ()) {

	  // iteration on each node of the current connected-component
	  for (core::NodeVector_t::const_iterator n_it = cc->nodes ().begin (); 
	       n_it != cc->nodes ().end (); ++n_it){
	    core::ConfigurationPtr_t qCC = (*n_it)->configuration ();
	    hppDout (info, "qCC: " << displayConfig (*qCC));

	    // Create forward local path from qCC to q_rand
	    localPath = (*smParabola_) (*qCC, *q_rand);

	    // if a forward path is returned, it is valid
	    if (localPath) {
	      // Save forward & backward delayed edges
	      fwdDelayedEdge = DelayedEdge_t (*n_it, impactNode, localPath);
	      fwdDelayedEdges.push_back (fwdDelayedEdge);
		
	      // Assuming that SM is symmetric (V0max = Vfmax)
	      // WARN: I had to reverse *n_it - impNode HERE
	      // to add edges consecutively to same vector fwdDelayedEdges
	      bwdDelayedEdge = DelayedEdge_t (impactNode, *n_it,
					      localPath->reverse ());
	      fwdDelayedEdges.push_back (bwdDelayedEdge);
	    } //if SM has returned a non-empty path
	  }//for nodes in cc
	}//avoid impactNode cc
      }//for cc in roadmap

      // Insert in roadmap all forward delayed edges (DE)
      bool avoidAddIdenticalEdge = true;
      for (DelayedEdges_t::const_iterator itEdge = fwdDelayedEdges.begin ();
	   itEdge != fwdDelayedEdges.end (); ++itEdge) {
	const core::NodePtr_t& nodeDE = itEdge-> get <0> ();
	const core::NodePtr_t& node2DE = itEdge-> get <1> ();
	const core::PathPtr_t& pathDE = itEdge-> get <2> ();
	core::EdgePtr_t edge = roadmap ()->addEdge (nodeDE, node2DE, pathDE);
	hppDout(info, "connection between q1: " 
		<< displayConfig (*(nodeDE->configuration ()))
		<< "and q2: "
		<< displayConfig (*(node2DE->configuration ())));
	edge->indexInRM (roadmap ()->edgeIndex_);
	// assure that forward and backward edges have same edgeIndex
	if (!avoidAddIdenticalEdge) {
	  roadmap ()->edgeIndex_++;
	  avoidAddIdenticalEdge = true;
	} else
	  avoidAddIdenticalEdge = false;
      }
    }

    void BallisticPlanner::tryDirectPath ()
    {
      // call steering method here to build a direct conexion
      core::PathPtr_t path;
      std::vector<std::string> filter;
      core::NodePtr_t initNode = roadmap ()->initNode();
      for (core::Nodes_t::const_iterator itn = roadmap ()->goalNodes ().begin();itn != roadmap ()->goalNodes ().end (); ++itn) {
        core::ConfigurationPtr_t q1 ((initNode)->configuration ());
        core::ConfigurationPtr_t q2 ((*itn)->configuration ());
        assert (*q1 != *q2);
        path = (*smParabola_) (*q1, *q2);
        if (path) { // has no collision
	  hppDout(notice, "#### direct parabola path is valid !");
	  roadmap ()->addEdge (initNode, *itn, path);
	  roadmap ()->addEdge (*itn, initNode, path->reverse());
	} else {
	  hppDout(notice, "#### direct parabola path not valid !");
	}
      } //for qgoals
    }

    void BallisticPlanner::computeGIWC(const core::RbprmNodePtr_t x){
      core::ValidationReportPtr_t report;
      problem().configValidations()->validate(*(x->configuration()),report);
      computeGIWC(x,report);
    }

    void BallisticPlanner::computeGIWC(const core::RbprmNodePtr_t node,
				       core::ValidationReportPtr_t report){
      hppDout(notice,"## compute GIWC");
      core::ConfigurationPtr_t q = node->configuration();
      // fil normal information in node
      if(node){
        size_t cSize = problem().robot()->configSize();
        hppDout(info,"~~ NODE cast correctly");
        node->normal((*q)[cSize-3],(*q)[cSize-2],(*q)[cSize-1]);
        hppDout(info,"~~ normal = "<<node->getNormal());
        
      }else{
        hppDout(error,"~~ NODE cannot be cast");
        return;
      }
      hppDout(info,"~~ q = "<<displayConfig(*q));
      
      core::RbprmValidationReportPtr_t rbReport =
	boost::dynamic_pointer_cast<core::RbprmValidationReport> (report);
      // checks :
      if(!rbReport)
	{
	  hppDout(error,"~~ Validation Report cannot be cast");
	  return;
	}
      if(rbReport->trunkInCollision)
	{
	  hppDout(warning,"~~ ComputeGIWC : trunk is in collision"); // shouldn't happen
	}
      if(!rbReport->romsValid)
	{
	  hppDout(warning,"~~ ComputeGIWC : roms filter not respected"); // shouldn't happen
	}
      
      //TODO
      polytope::T_rotation_t rotContact(3*rbReport->ROMReports.size(),3);
      polytope::vector_t posContact(3*rbReport->ROMReports.size());
      
      
      // get the 2 object in contact for each ROM :
      hppDout(info,"~~ Number of roms in collision : "<<rbReport->ROMReports.size());
      size_t indexRom = 0 ;
      for(std::map<std::string,core::CollisionValidationReportPtr_t>::const_iterator it = rbReport->ROMReports.begin() ; it != rbReport->ROMReports.end() ; ++it)
	{
	  hppDout(info,"~~ for rom : "<<it->first);
	  core::CollisionObjectPtr_t obj1 = it->second->object1;
	  core::CollisionObjectPtr_t obj2 = it->second->object2;
	  hppDout(notice,"~~ collision between : "<<obj1->name() << " and "<<obj2->name());
	  fcl::CollisionResult result = it->second->result;
	  /* size_t numContact =result.numContacts();
	     hppDout(notice,"~~ number of contact : "<<numContact);
	     std::ostringstream ss;
	     ss<<"[";
	     for(size_t i = 0 ; i < numContact ; i++)
	     { // print with python formating :
	     ss<<"["<<result.getContact(i).pos[0]<<","<<result.getContact(i).pos[1]<<","<<result.getContact(i).pos[2]<<"]";
	     if(i< (numContact-1))
             ss<<",";
	     }
	     ss<<"]";
	     std::cout<<ss.str()<<std::endl;
	  */
        
	  // get intersection between the two objects :
	  obj1->fcl();
	  geom::T_Point vertices1;
	  geom::BVHModelOBConst_Ptr_t model1 =  geom::GetModel(obj1->fcl());
	  hppDout(info,"vertices obj1 : "<<obj1->name()<< " ( "<<model1->num_vertices<<" ) ");
	  std::ostringstream ss1;
	  ss1<<"[";
	  for(int i = 0 ; i < model1->num_vertices ; ++i)
	    {
	      vertices1.push_back(Eigen::Vector3d(model1->vertices[i][0], model1->vertices[i][1], model1->vertices[i][2]));
	      //hppDout(notice,"vertices : "<<model1->vertices[i]);
	      ss1<<"["<<model1->vertices[i][0]<<","<<model1->vertices[i][1]<<","<<model1->vertices[i][2]<<"]";
	      if(i< (model1->num_vertices-1))
		ss1<<",";
	    }
	  ss1<<"]";
	  //std::cout<<ss1.str()<<std::endl;
        
        
	  obj2->fcl();
	  geom::T_Point vertices2;
	  geom::BVHModelOBConst_Ptr_t model2 =  geom::GetModel(obj2->fcl());
	  hppDout(info,"vertices obj2 : "<<obj2->name()<< " ( "<<model2->num_vertices<<" ) ");
	  std::ostringstream ss2;
	  ss2<<"[";
	  for(int i = 0 ; i < model2->num_vertices ; ++i)
	    {
	      vertices2.push_back(Eigen::Vector3d(model2->vertices[i][0], model2->vertices[i][1], model2->vertices[i][2]));
	      // hppDout(notice,"vertices : "<<model2->vertices[i]);
	      ss2<<"["<<model2->vertices[i][0]<<","<<model2->vertices[i][1]<<","<<model2->vertices[i][2]<<"]";
	      if(i< (model2->num_vertices -1))
		ss2<<",";
          
	    }
	  ss2<<"]";
	  //std::cout<<ss2.str()<<std::endl;
        
	  geom::T_Point hull = geom::intersectPolygonePlane(model1,model2,fcl::Vec3f(0,0,1),geom::ZJUMP,result);
        
	  // todo : compute center point of the hull
	  polytope::vector3_t normal,tangent0,tangent1;
	  geom::Point center = geom::center(hull.begin(),hull.end());
	  posContact.segment<3>(indexRom*3) = center;
	  //std::cout<<center<<std::endl<<std::endl;
	  polytope::rotation_t rot; 
	  normal = -result.getContact(0).normal;
	  hppDout(notice," !!! normal for GIWC : "<<normal);
	  // compute tangent vector : 
	  tangent0 = normal.cross(polytope::vector3_t(1,0,0));
	  if(tangent0.dot(tangent0)<0.001)
	    tangent0 = normal.cross(polytope::vector3_t(0,1,0)); 
	  tangent1 = normal.cross(tangent0);
	  rot(0,0) = tangent0(0) ; rot(0,1) = tangent1(0) ; rot(0,2) = normal(0);
	  rot(1,0) = tangent0(1) ; rot(1,1) = tangent1(1) ; rot(1,2) = normal(1);
	  rot(2,0) = tangent0(2) ; rot(2,1) = tangent1(2) ; rot(2,2) = normal(2);
        
	  rotContact.block<3,3>(indexRom*3,0) = rot;
	  //std::cout<<rot<<std::endl<<std::endl;
        
	  indexRom++;
	} // for each ROMS
      
      polytope::vector_t x(rbReport->ROMReports.size());
      polytope::vector_t y(rbReport->ROMReports.size());
      polytope::vector_t nu(rbReport->ROMReports.size());
      for(size_t k = 0 ; k<rbReport->ROMReports.size() ; ++k){
        x(k) = 0.25; // approx size of foot
        y(k) = 0.15; 
        nu(k) = 0.5;
      }
      // save giwc in node structure
      node->giwc(polytope::U_stance(rotContact,posContact,nu,x,y));
    }// computeGIWC

  } // namespace core
} // namespace hpp
