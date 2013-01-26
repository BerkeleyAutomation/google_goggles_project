//######################################################################
//
// GraspIt!
// Copyright (C) 2002-2009	Columbia University in the City of New York.
// All rights reserved.
//
// GraspIt! is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// GraspIt! is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with GraspIt!.	If not, see <http://www.gnu.org/licenses/>.
//
// Author(s):	Matei T. Ciocarlie
//
// $Id: taskDispatcher.cpp,v 1.8 2010/09/01 23:54:20 cmatei Exp $
//
//######################################################################

#include "graspit_dbase_tasks/dbTaskDispatcher.h"

#include <iostream>
#include <sstream>
#include <cassert>
#include <cstdlib>
#include <cstdio>
#include <new>

//#define GRASPITDBG

#include "DBPlanner/ros_database_manager.h"
#include "DBPlanner/inmemory_database_manager.h"

#include <Inventor/nodes/SoScale.h>
#include <src/Collision/collisionStructures.h>
#include <include/EGPlanner/searchEnergy.h>
#include <include/mytools.h>
#include <include/world.h>
#include <include/body.h>
#include <include/graspitGUI.h>
#include <include/ivmgr.h>
#include <include/scanSimulator.h>
#include <include/pr2Gripper.h>

#include "graspit_db_model.h"
#include "debug.h"

#include "graspit_dbase_tasks/dbTask.h"

#include <ros/callback_queue.h>

#include <object_manipulation_msgs/Grasp.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/point_cloud_conversion.h>

using namespace std;

namespace graspit_dbase_tasks {

bool meshFromROS(pcl::PolygonMesh& object, std::vector<double>& v, std::vector<int>& t) {
	sensor_msgs::PointCloud2 pc = object.cloud;
	std::vector<pcl::Vertices> polygons = object.polygons;

	if (pc.height != 1) {
		ROS_ERROR("2D point cloud!");
		return false;
	} else if (pc.width <= 0) {
		ROS_ERROR("Invalid width!");
		return false;
	}

	ROS_INFO("converting vertices");
	for (uint32_t i=0;i<pc.width;i++) {
		if (pc.fields[0].name != "x" || pc.fields[1].name != "y" || pc.fields[2].name != "z") {
			ROS_ERROR("point cloud has bad fields!");
			return false;
		} else if (pc.fields[0].datatype != sensor_msgs::PointField::FLOAT32) {
			ROS_ERROR("point cloud is not float32!");
			return false;
		}
		uint32_t offset = pc.point_step * i;
		for (int j=0;j<3;j++) {
			float val;
			char data[4];
			for (int k=0;k<4;k++) { data[k] = pc.data[offset+j*4+k]; }
			memcpy(&val,data,4);
			v.push_back(val);
		}

	}

	ROS_INFO("converting triangles");
	for (size_t i=0;i<polygons.size();i++) {
		t.push_back(polygons[i].vertices[0]);
		t.push_back(polygons[i].vertices[1]);
		t.push_back(polygons[i].vertices[2]);
	}
	
	return true;
}

void getMesh(std::vector<double>& v, std::vector<int>& t,double half_extent=0.025,double z_factor=1) {
	static int num_calls = 0;
	num_calls++;
	
	//double z_factor = flat_z ? 0.1 : 1;
	
	double x_offset = 0;
	double y_offset = 0;
	double z_offset = z_factor * half_extent;
	
	v.push_back( half_extent + x_offset); v.push_back( half_extent + y_offset); v.push_back(z_factor *  half_extent + z_offset);
	v.push_back(-half_extent + x_offset); v.push_back( half_extent + y_offset); v.push_back(z_factor *  half_extent + z_offset);
	v.push_back(-half_extent + x_offset); v.push_back(-half_extent + y_offset); v.push_back(z_factor *  half_extent + z_offset);
	v.push_back( half_extent + x_offset); v.push_back(-half_extent + y_offset); v.push_back(z_factor *  half_extent + z_offset);
	v.push_back( half_extent + x_offset); v.push_back( half_extent + y_offset); v.push_back(z_factor * -half_extent + z_offset);
	v.push_back(-half_extent + x_offset); v.push_back( half_extent + y_offset); v.push_back(z_factor * -half_extent + z_offset);
	v.push_back(-half_extent + x_offset); v.push_back(-half_extent + y_offset); v.push_back(z_factor * -half_extent + z_offset);
	v.push_back( half_extent + x_offset); v.push_back(-half_extent + y_offset); v.push_back(z_factor * -half_extent + z_offset);

	t.push_back(0); t.push_back(1); t.push_back(2);
	t.push_back(0); t.push_back(2); t.push_back(3);

	t.push_back(4); t.push_back(5); t.push_back(6);
	t.push_back(4); t.push_back(6); t.push_back(7);

	t.push_back(0); t.push_back(4); t.push_back(5);
	t.push_back(0); t.push_back(1); t.push_back(5);

	t.push_back(1); t.push_back(5); t.push_back(6);
	t.push_back(1); t.push_back(2); t.push_back(6);

	t.push_back(2); t.push_back(6); t.push_back(7);
	t.push_back(2); t.push_back(3); t.push_back(7);

	t.push_back(3); t.push_back(7); t.push_back(4);
	t.push_back(3); t.push_back(0); t.push_back(4);
}

 void modelFromMesh(int id,const std::string& name,const std::vector<double>& vertices, const std::vector<int>& triangles,GraspitDBModel* model) {
	//Model* model = model_allocator_->Get();
	
	model->SetModelId( id );
	model->SetScale( 1 );
	model->SetRescaleFactor( 1.0 );
	model->SetModelName( name );
	//model->SetTags(	);
	
	std::vector<double> m_vertices;
	std::vector<double>::const_iterator itr;
	for (itr=vertices.begin();itr!=vertices.end();itr++) {
		m_vertices.push_back(1000. * (*itr));
	}

	model->SetGeometry( m_vertices, triangles );
	
	std::cout << "Created model " << name << std::endl;
}

void modelFromMesh(const std::vector<double>& vertices, const std::vector<int>& triangles,GraspitDBModel* model) {
	int id = rand() % 1000;
	std::stringstream name;
	name << "model" << id;
	modelFromMesh(id,name.str(),vertices,triangles,model);
}

transf poseToTransf(const geometry_msgs::Pose& pose) {
  return transf(Quaternion(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z),
                vec3(1000 * pose.position.x, 1000 * pose.position.y, 1000 * pose.position.z));
}

geometry_msgs::Pose transfToPose(const transf& tr) {
  geometry_msgs::Pose pose;
  return pose;
}

DBTaskDispatcher::DBTaskDispatcher() : mDBMgr(NULL) , mCurrentTask(NULL), mStatus(READY), testGraspsRequest_(NULL), testGraspsResponse_(NULL)
{
	//hard-coded for now
	mMaxTasks = -1;
	mCompletedTasks = 0;
}

DBTaskDispatcher::~DBTaskDispatcher()
{
	if (mCurrentTask) {
		//this should not really happen
		DBGA("Dispatcher: deleting current task on cleanup");
		delete mCurrentTask;
	}
	delete mDBMgr;
	ros::shutdown();
}

int DBTaskDispatcher::connect(std::string host, int port, std::string username, 
															std::string password, std::string database) 
{
	delete mDBMgr;
	std::ostringstream port_str;
	port_str << port;

	std::cout << "DBTaskDispatcher: starting database manager" << std::endl;
	//careful: we pass a null for the grasp allocator as we don't know yet which hand we'll be using
	//mDBMgr = new db_planner::RosDatabaseManager(host, port_str.str(), username, password, database, NULL, NULL);
	mDBMgr = new db_planner::InMemoryDatabaseManager(NULL, NULL);
	
	std::cout << "DBTaskDispatcher: database manager created" << std::endl;
	//use the special allocator for models that get geometry directly from the database	
	GeomGraspitDBModelAllocator* allocator = new GeomGraspitDBModelAllocator(mDBMgr);
	mDBMgr->SetModelAllocator(allocator);
	
	if (!mDBMgr->isConnected()) {
		DBGA("DBase operator: Connection failed");
		delete mDBMgr; mDBMgr = NULL;
		return -1;
	}
	
	std::cout << "DBTaskDispatcher: database manager ready" << std::endl;
	return 0;
}

int DBTaskDispatcher::init(int argc, char **argv)
{
	std::cout << "Initializing DBTaskDispatcher" << std::endl;
	//in the future, we'll get the connection params from arguments
	int connect_result = connect("10.0.0.81",5432,"willow","willow","household_objects");
	
	//init ros
	ros::init(argc, argv, "graspit");
	//ROS_INFO("Using node name %s", node_name.c_str());
	
	//init node handles
	root_nh_ = new ros::NodeHandle("");
	priv_nh_ = new ros::NodeHandle("~");
	
	//std::cout << "here183" << std::endl;
	cb_queue_ = new ros::CallbackQueue();
	//std::cout << "here185" << std::endl;
	root_nh_->setCallbackQueue(cb_queue_);
	//std::cout << "here187" << std::endl;
	priv_nh_->setCallbackQueue(cb_queue_);
	//std::cout << "here189" << std::endl;
	
	
	generate_grasps_srv_ = root_nh_->advertiseService("generate_grasps", &DBTaskDispatcher::generateGraspsCB, this);
	test_grasps_srv_ = root_nh_->advertiseService("test_grasps", &DBTaskDispatcher::testGraspsCB, this);
	
	
	spinner_ = new ros::AsyncSpinner(1,cb_queue_);
	//std::cout << "here195" << std::endl;
	spinner_->start();
	//std::cout << "here197" << std::endl;

	//ROS_INFO("Loading gripper");
	//loadGripper();
	
	
	//runTest();
	
	//runTest();
	
	World* world = graspItGUI->getIVmgr()->getWorld();
	
	printf("num bodies %d\n",world->getNumBodies());
	
	std::vector<double> v;
	std::vector<int> t;

	if (true) {
		ROS_INFO("Adding table");
		double sz = 200;
		double z_factor = 0.5;
		getMesh(v,t,sz,z_factor);
		
		Body* body = new Body(world, "table");
		//material is default
		body->setMaterial(world->getMaterialIdx("plastic"));

		//PSB objects have a scale of their own. To get to "graspable size"
		//we manually set a scaling factor for each of them, which is in the
		//database as well. We need to scale the geometry appropriately
		SoScale* scale = new SoScale();
		scale->scaleFactor.setValue(1.0, 1.0, 1.0);
		body->getIVGeomRoot()->addChild(scale);

		std::vector<position> vertices;
		for (size_t i=0; i<v.size()/3; i++) {
			vertices.push_back( position(v.at(3*i+0), v.at(3*i+1), v.at(3*i+2)) );
		}

		body->loadGeometryMemory(vertices, t);
		body->addIVMat();
		
		
		body->addToIvc();
		world->addBody(body);
		
		transf tf(Quaternion(1, 0,0,0),
					vec3(0, 0, -2 * z_factor * sz ));

		//set the transform
		body->setTran(tf);
		
		printf("num bodies %d\n",world->getNumBodies());
	}
	return 0;
}

/*! Gets a new task from the database and starts it. Possible outcomes:
	- no more tasks in database; sets status to NO_TASK
	- max number of tasks exceeded; sets status to DONE
	- error in reading the task; sets status to ERROR
	- error in starting the task; sets status to READY
	- task has started and needs us to surrender control; sets status to RUNNING
	- task is finished in one shot; sets status to READY
*/
void DBTaskDispatcher::startNewTask()
{
	assert(!mCurrentTask);
	// check if we have completed the max number of tasks
	if (mMaxTasks >= 0 && mCompletedTasks >= mMaxTasks) {
		mStatus = DONE;
		return;
	}
	db_planner::TaskRecord rec;
	if (!mDBMgr->AcquireNextTask(&rec, mAcceptedTaskTypes)) {
		DBGA("Dispatcher: error reading next task");
		mStatus = ERROR;
		return;
	}
	//task type 0 is reserved for no task to do
	if (rec.taskType.empty()) {
		DBGP("Dispatcher: no tasks to be executed");
		mStatus = NO_TASK;
		return;
	}
	//check if we have a creator for tasks of this type
	std::map<std::string,DBTaskCreator*>::iterator it = mTaskCreators.find(rec.taskType);
	if (it==mTaskCreators.end()) 
	{
		DBGA("Dispatcher: no creator available for task type: " << rec.taskType);
		mStatus = ERROR;
		return;
	}
	//create the new task
	mCurrentTask = (*(it->second))(this, mDBMgr,rec);
	if (!mCurrentTask) {
		DBGA("Dispatcher: could not start task of type: " << rec.taskType);
		mStatus = ERROR;
		return;
	}
	//start the next task
	mCurrentTask->start();
	mStopCurrentTask = false;
}

/*! Checks on the current task; if it is finished, cleans up after it and marks the 
	result in the database.

	If task is finished, sets status to READY, unless there is an error marking the 
	finished task in the database, in which case status is set to ERROR.

	Nore that even if the task finishes with an error, the dispatcher will be READY
	for the next task (not abort altogether). The task that had an error is marked
	as such in the database. However, if there is an error in communicating with 
	the database, the dispatcher will abort altogether.
*/
void DBTaskDispatcher::checkCurrentTask()
{
	if (mStopCurrentTask) {
		mCurrentTask->stop();
		mStopCurrentTask = false;
	}
	assert(mCurrentTask);
	switch (mCurrentTask->getStatus()) {
	case DBTask::RUNNING:
		DBGP("Task running");
		mStatus = RUNNING;
		mCurrentTask->mainLoop();
		break;
	case DBTask::ERROR:
		DBGA("Task error");
		mStatus = READY;
		if (!mDBMgr->SetTaskStatus(mCurrentTask->getRecord().taskId,"ERROR")) {
			DBGA("Dispatcher: error marking completed task");
			mStatus = ERROR;
		}
		delete mCurrentTask; mCurrentTask = NULL;
		break;
	case DBTask::DONE:
		DBGA("Task done");
		mStatus = READY;
		mCompletedTasks++;
		if (!mDBMgr->SetTaskStatus(mCurrentTask->getRecord().taskId,"COMPLETED")) {
			DBGA("Dispatcher: error marking completed task");
			mStatus = ERROR;
		}
		delete mCurrentTask; mCurrentTask = NULL;
		break;
	default:
		DBGA("Dispatcher: Unknown task state");
		mStatus = ERROR;
	}
}

/*! Will start tasks as long as there are tasks to be run. If the tasks are of the one-shot type,
	it just loops in here as long as it has tasks. If the task is event-based and needs us to 
	surrender control, it will surrender control but schedule the timer to come back here and 
	check on the task later.
*/
int DBTaskDispatcher::mainLoop()
{
	//static bool printed = false;
	static size_t prev_size = 0;
	
	if(mCurrentTask) {
		checkCurrentTask();
	}
	if (mStatus != RUNNING) {
		startNewTask();
	}
	
	if (testGraspsRequest_) {
		ROS_INFO("Testing grasps!");
		graspit_srvs::TestGrasps::Response* response = new graspit_srvs::TestGrasps::Response();
		testGrasps(*testGraspsRequest_,*response);
		ROS_INFO("Done testing grasps");
		testGraspsResponse_ = response;
		testGraspsRequest_ = 0;
	}
	
	switch(mStatus) {
	case DONE:
		DBGA("Status is DONE");
		//graspItGUI->exitMainLoop();		
		return 0;
	case ERROR:
		DBGA("Status is ERROR");
		//graspItGUI->exitMainLoop();		
		return 1;
	case NO_TASK:
		//graspItGUI->exitMainLoop();		
		return 0;
	case RUNNING:
		break;
	case READY:
		break;
	}
	return 0;
}

bool DBTaskDispatcher::generateGraspsCB(
		graspit_srvs::GenerateGrasps::Request &request,
		graspit_srvs::GenerateGrasps::Response &response) {
	ROS_INFO("generateGraspsCB()!");
	
	ros::Time call_time = ros::Time::now();
	response.grasp_poses.header.stamp = call_time;
	
	//GeomGraspitDBModel* model = (GeomGraspitDBModel*) runTest2();
	std::vector<double> v;
	std::vector<int> t;

	//getMesh(v,t);
	
	meshFromROS(request.object, v, t);

	ROS_INFO("Adding model");
	GeomGraspitDBModel* model = new GeomGraspitDBModel(mDBMgr);
	modelFromMesh(v,t,model);
	
	bool result = mDBMgr->AddModel(model);
		
	std::vector<db_planner::Grasp*> grasp_list;
		
	mDBMgr->GetGrasps(*model,"WILLOW_GRIPPER_2010",&grasp_list);
	
	ROS_INFO("Adding task");
	//create task and dispatch
	db_planner::PlanningTaskRecord* task = new db_planner::PlanningTaskRecord();
	task->taskType = "GRASP_PLANNING";
	task->taskId = rand() % 1000;
	task->model = model;
	task->handName = "WILLOW_GRIPPER_2010";
	task->taskTime = -1;
	
	mDBMgr->AddTask(task);
	
	size_t prev_size = 0;
	ros::Rate rate(1);
	do {
		grasp_list.clear();
		mDBMgr->GetGrasps(*model,"WILLOW_GRIPPER_2010",&grasp_list);
		
		size_t curr_size = grasp_list.size();
		
		for (size_t i=0;i<grasp_list.size();i++) {
			delete grasp_list[i];
		}
		
		if (curr_size != prev_size) {
			ROS_INFO_STREAM("runTest2 Found " << curr_size << " grasps");
			prev_size = curr_size;
			
			if (curr_size >= 15) {
				if (mCurrentTask) {
					ROS_INFO("Stopping task");
					//mCurrentTask->stop();
					mStopCurrentTask = true;
				}
				//break;
			}
		}
		rate.sleep();
	} while (ros::ok() && mCurrentTask && mCurrentTask->getRecord().taskId == task->taskId);
	
	ROS_INFO("Test complete");
	
	grasp_list.clear();
	mDBMgr->GetGrasps(*model,"WILLOW_GRIPPER_2010",&grasp_list);
	ROS_INFO_STREAM("generateGraspsCB Found " << grasp_list.size() << " grasps");
		
	for (size_t i=0;i<grasp_list.size();i++) {
		db_planner::Grasp* grasp_ptr = grasp_list.at(i);
		std::vector<double> pre_joints = grasp_ptr->GetPregraspJoints();
		std::vector<double> joints = grasp_ptr->GetFinalgraspJoints();
		std::vector<double> pos = grasp_ptr->GetFinalgraspPosition();
		
		object_manipulation_msgs::Grasp grasp;
		
		sensor_msgs::JointState pre_grasp_posture;
		if (!pre_joints.empty()) {
			pre_grasp_posture.name.push_back("gripper_joint");
			pre_grasp_posture.position.push_back(pre_joints[0]);
		}
		
		sensor_msgs::JointState grasp_posture;
		if (!joints.empty()) {
			grasp_posture.name.push_back("gripper_joint");
			grasp_posture.position.push_back(joints[0]);
		}
		
		geometry_msgs::Pose pose;
		if (pos.size() >= 7) {
			pose.position.x = pos[0]/1000;
			pose.position.y = pos[1]/1000;
			pose.position.z = pos[2]/1000;
			
			pose.orientation.w = pos[3];
			pose.orientation.x = pos[4];
			pose.orientation.y = pos[5];
			pose.orientation.z = pos[6];
		}
		
		grasp.pre_grasp_posture = pre_grasp_posture;
		grasp.grasp_posture = grasp_posture;
		grasp.grasp_pose = pose;
		
		grasp.success_probability = grasp_ptr->Energy();
		
		//std::cout << grasp << std::endl;
		
		response.grasps.push_back(grasp);
		response.grasp_poses.poses.push_back(grasp.grasp_pose);
	}
	
	for (size_t i=0;i<grasp_list.size();i++) {
		delete grasp_list[i];
	}
	
	return true;
}

bool DBTaskDispatcher::testGraspsCB(
		graspit_srvs::TestGrasps::Request &request,
		graspit_srvs::TestGrasps::Response &response) {
	ROS_INFO("testGraspsCB called");
	testGraspsRequest_ = &request;
	
	ros::Rate rate(1);
	while (ros::ok() && !testGraspsResponse_) {
		rate.sleep();
	}
	
	response = *testGraspsResponse_;
	delete testGraspsResponse_;
	testGraspsResponse_ = 0;
	return true;
}

bool DBTaskDispatcher::testGrasps(
		graspit_srvs::TestGrasps::Request &request,
		graspit_srvs::TestGrasps::Response &response) {
	ROS_INFO("testGrasps called");
	
	std::vector<double> v;
	std::vector<int> t;

	//getMesh(v,t);
	
	meshFromROS(request.object, v, t);

	ROS_INFO("Adding model");
	GeomGraspitDBModel* model = new GeomGraspitDBModel(mDBMgr);
	modelFromMesh(v,t,model);
	
	bool result = mDBMgr->AddModel(model);
	
	World* world = graspItGUI->getIVmgr()->getWorld();
	
	model->load(world);
	
	GraspableBody* object = model->getGraspableBody();
	object->addToIvc();
	world->addBody(object);
	
	Pr2Gripper2010* gripper_ = dynamic_cast<Pr2Gripper2010*>(world->getCurrentHand());
	
	response.grasps = request.grasps;
	response.qualities.resize(request.grasps.size(),-1);
	
	for (int gInd=0;gInd<request.grasps.size();gInd++) {
		std::cout << "testing grasp " << gInd << std::endl;
		//printf("sleeping\n");
		//ros::Duration(1).sleep();
		printf("setting transform\n");
		//place the hand in the right pose
		gripper_->setTran(poseToTransf(request.grasps[gInd].grasp_pose));
		printf("forcing dof val\n");
		//set the gripper dof (for the moment all the way open)
		gripper_->forceDOFVal(0, 0.523);//request.grasp.grasp_posture.position[0]);
		
		//continue;
		
		//gripper_->autoGrasp(false, 0.5, true);
		//ros::Duration(1).sleep();
		
		//check for collisions
		
		printf("getting interest_list\n");
		bool hand_environment_collision = false;
		bool hand_object_collision = false;
		std::vector<DynamicBody*> gripper_link_list;
		gripper_->getAllLinks(gripper_link_list);
		std::vector<Body*> interest_list;
		for (size_t i = 0; i < gripper_link_list.size(); i++) {
			interest_list.push_back(gripper_link_list[i]);
		}
		CollisionReport collision_report;
		printf("getting collision report\n");
		world->getCollisionReport(&collision_report, &interest_list);
		for (size_t i = 0; i < collision_report.size(); i++) {
			//figure out what the other body we are colliding with is
			printf("checking body\n");
			Body *collision_body = collision_report[i].first;
			if (collision_body->getOwner() == gripper_) {
				collision_body = collision_report[i].second;
			}
			if (collision_body == object)
			hand_object_collision = true;
			else
			hand_environment_collision = true;
		}
		printf("done checking bodies\n");
		
		//if we find a collision, we are done
		if (hand_object_collision || hand_environment_collision) {
			std::cout << "collision!" << std::endl;
		} else {
			printf("closing gripper\n");
			gripper_->compliantClose();
			//printf("sleeping\n");
			//ros::Duration(1).sleep();
			
			//compute the energy value
			//computeEnergy(object, response);
			SearchEnergy energy_calculator;
			energy_calculator.setType(ENERGY_CONTACT);
			energy_calculator.setContactType(CONTACT_PRESET);
			bool legal;
			double energy;
			printf("calculating energy\n");
			energy_calculator.analyzeCurrentPosture(gripper_, object, legal, energy, false);
			if (!legal) {
				//this should not really happen, as we've checked for collisions already
				ROS_WARN("Energy calculator reports illegal state");
				//response.test_result = response.HAND_COLLISION;
				//response.energy_value = -1.0;
				//return;
			} else {
				//response.energy_value = energy;
				//hard-coded threshold for now, will make it into a parameter
				if (energy < 10.0) {
					std::cout << "SUCCESS: " << energy << std::endl;
					//response.test_result = response.GRASP_SUCCESS;
				} else {
					std::cout << "FAILURE: " << energy << std::endl;
					//response.test_result = response.GRASP_FAILURE;
				}
				response.grasps[gInd].success_probability = energy;
				response.qualities[gInd] = energy;
			}
		}
	}
}

bool DBTaskDispatcher::generateGraspsFromMesh(std::vector<double>& v, std::vector<int>& t) {
	GeomGraspitDBModel* model = new GeomGraspitDBModel(mDBMgr);
	modelFromMesh(v,t,model);
	
	bool result = mDBMgr->AddModel(model);
	
	db_planner::PlanningTaskRecord* task = new db_planner::PlanningTaskRecord();
	task->taskType = "GRASP_PLANNING";
	task->taskId = rand() % 1000;
	task->model = model;
	task->handName = "WILLOW_GRIPPER_2010";
	task->taskTime = -1;
	
	mDBMgr->AddTask(task);
	
	std::vector<db_planner::Grasp*> grasp_list;
	
	ros::Rate rate(10);
	while (ros::ok()) {
		mDBMgr->GetGrasps(*(task->model),task->handName,&grasp_list);
		if (!grasp_list.empty()) {
			ROS_INFO_STREAM("Found " << grasp_list.size() << " grasps");
			break;
		}
		rate.sleep();
	}
	
	return true;
}
	

void DBTaskDispatcher::runTest() {
	ROS_INFO("Running test");
	std::vector<double> v;
	std::vector<int> t;

	getMesh(v,t);

	ROS_INFO("Adding model");
	GeomGraspitDBModel* model = new GeomGraspitDBModel(mDBMgr);
	modelFromMesh(v,t,model);
	
	bool result = mDBMgr->AddModel(model);
	
	std::vector<db_planner::Model*> models;
	mDBMgr->ModelList(&models);
	ROS_INFO_STREAM("Num models: " << models.size());
	
	ROS_INFO("Adding task");
	//create task and dispatch
	db_planner::PlanningTaskRecord* task = new db_planner::PlanningTaskRecord();
	task->taskType = "GRASP_PLANNING";
	task->taskId = rand() % 1000;
	task->model = model;
	task->handName = "WILLOW_GRIPPER_2010";
	task->taskTime = -1;
	
	mDBMgr->AddTask(task);
	
	ROS_INFO("Test complete");
}

void* DBTaskDispatcher::runTest2() {
	ROS_INFO("Running test");
	std::vector<double> v;
	std::vector<int> t;

	getMesh(v,t);

	ROS_INFO("Adding model");
	GeomGraspitDBModel* model = new GeomGraspitDBModel(mDBMgr);
	modelFromMesh(v,t,model);
	
	bool result = mDBMgr->AddModel(model);
	
	/*
	ROS_INFO("Adding obstacle");
	World* world = graspItGUI->getIVmgr()->getWorld();
	Body* body = new Body(world, "table");
	body->setMaterial(world->getMaterialIdx("wood"));
	
	SoScale* scale = new SoScale();
	scale->scaleFactor.setValue(1,1,1);
	body->getIVGeomRoot()->addChild(scale);
	
	std::vector<position> vertices;
	for (size_t i=0; i<v.size()/3; i++) {
		vertices.push_back( position(v.at(3*i+0), v.at(3*i+1), v.at(3*i+2)) );
	}

	body->loadGeometryMemory(vertices,t);
	
	body->addIVMat();
	
	transf tf(Quaternion(1, 0,0,0),
                vec3(1000 * 0, 1000 * 0, 1000 * -0.05));

	//set the correct transform
	body->setTran(tf);
	
	world->addBody(body);
	*/
	
	
	ROS_INFO("Adding task");
	//create task and dispatch
	db_planner::PlanningTaskRecord* task = new db_planner::PlanningTaskRecord();
	task->taskType = "GRASP_PLANNING";
	task->taskId = rand() % 1000;
	task->model = model;
	task->handName = "WILLOW_GRIPPER_2010";
	task->taskTime = -1;
	
	mDBMgr->AddTask(task);
	
	
	size_t prev_size = 0;
	ros::Rate rate(1);
	do {
		std::vector<db_planner::Grasp*> grasp_list;
		
		mDBMgr->GetGrasps(*model,"WILLOW_GRIPPER_2010",&grasp_list);
		
		size_t curr_size = grasp_list.size();
		
		for (size_t i=0;i<grasp_list.size();i++) {
			delete grasp_list[i];
		}
		
		if (curr_size != prev_size) {
			ROS_INFO_STREAM("runTest2 Found " << curr_size << " grasps");
			prev_size = curr_size;
			
			if (curr_size >= 15) {
				if (mCurrentTask) {
					ROS_INFO("Stopping task");
					//mCurrentTask->stop();
					mStopCurrentTask = true;
				}
				//break;
			}
		}
		rate.sleep();
	} while (ros::ok() && mCurrentTask && mCurrentTask->getRecord().taskId == task->taskId);
	
	ROS_INFO("Test complete");
	
	return model;
}

} //namespace
