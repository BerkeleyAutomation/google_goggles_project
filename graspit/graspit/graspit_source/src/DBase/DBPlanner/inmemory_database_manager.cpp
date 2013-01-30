//######################################################################
//
// GraspIt!
// Copyright (C) 2002-2009  Columbia University in the City of New York.
// All rights reserved.
//
// GraspIt! is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// GraspIt! is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with GraspIt!.  If not, see <http://www.gnu.org/licenses/>.
//
// Author(s):  Matei T. Ciocarlie
//
// $Id: ros_database_manager.cpp,v 1.9 2010/09/01 23:54:20 cmatei Exp $
//
//######################################################################

#include "inmemory_database_manager.h"

#include <boost/shared_ptr.hpp>
#include <iostream>
#include <sstream>
#include <cstdlib>

namespace db_planner {

InMemoryDatabaseManager::InMemoryDatabaseManager(ModelAllocator *model_allocator, GraspAllocator* grasp_allocator)
{
  model_allocator_ = model_allocator;
  grasp_allocator_ = grasp_allocator;
}

InMemoryDatabaseManager::~InMemoryDatabaseManager()
{
  delete model_allocator_;
  delete grasp_allocator_;
}


void InMemoryDatabaseManager::modelFromDBModel(Model* model, const Model* db_model) const {
	model->SetModelId( db_model->ModelId() );
	model->SetScale( db_model->Scale() );
	model->SetRescaleFactor( db_model->RescaleFactor() );
	model->SetModelName( db_model->ModelName() );
	model->SetTags( const_cast<Model*>(db_model)->Tags().begin(), const_cast<Model*>(db_model)->Tags().end() );

	model->SetGeometry( db_model->GetVertices(), db_model->GetTriangles() );
//    //see what kind of files we have in the database for this model
//    std::vector< boost::shared_ptr<household_objects_database::DatabaseFilePath> > paths;
//    std::stringstream id;
//    id << db_model.original_model_id_.data();
//    std::string where_clause( "original_model_id=" + id.str());
//    if (!database_->getList<household_objects_database::DatabaseFilePath>(paths, where_clause)) {
//      std::cerr << "Failed to load file paths from database\n";
//      return;
//    }
//    for (size_t i=0; i<paths.size(); i++) {
//      if (paths[i]->file_type_.data() == "THUMBNAIL_BINARY_PNG") {
//        model->SetThumbnailPath(model_root + paths[i]->file_path_.data());
//      } else if (paths[i]->file_type_.data() == "GEOMETRY_BINARY_PLY") {
//        model->SetGeometryPath(model_root + paths[i]->file_path_.data());
//      }
//    }
}

void InMemoryDatabaseManager::DBModelFromModel(Model* db_model, const Model* model) const {
	db_model->SetModelId( model->ModelId() );
	db_model->SetScale( model->Scale() );
	db_model->SetRescaleFactor( model->RescaleFactor() );
	db_model->SetModelName( model->ModelName() );
	db_model->SetTags( const_cast<Model*>(model)->Tags().begin(), const_cast<Model*>(model)->Tags().end() );

	db_model->SetGeometry( model->GetVertices(), model->GetTriangles() );
}

void InMemoryDatabaseManager::graspFromDBGrasp(Grasp* grasp, const Grasp* db_grasp) const {
	grasp->SetGraspId(db_grasp->GraspId());
	grasp->SetEnergy(db_grasp->Energy());
	grasp->SetEpsilonQuality(0.0);
	grasp->SetVolumeQuality(0.0);
	grasp->SetClearance(db_grasp->Clearance());
	grasp->SetClusterRep(db_grasp->ClusterRep());
	grasp->SetTableClearance(db_grasp->TableClearance());
	grasp->SetCompliantCopy(db_grasp->CompliantCopy());
	grasp->SetCompliantOriginalId(db_grasp->CompliantOriginalId());
	
	std::vector<double> pregrasp_joints = db_grasp->GetPregraspJoints();
	std::vector<double> pregrasp_position = db_grasp->GetPregraspPosition();
	std::vector<double> grasp_joints = db_grasp->GetFinalgraspJoints();
	std::vector<double> grasp_position = db_grasp->GetFinalgraspPosition();
	
	grasp->SetGraspParameters(pregrasp_joints, 
							pregrasp_position, 
							grasp_joints, 
							grasp_position);
	grasp->SetPregraspJoints(pregrasp_joints);
	grasp->SetPregraspPosition(pregrasp_position);
	grasp->SetFinalgraspJoints(grasp_joints);
	grasp->SetFinalgraspPosition(grasp_position);
}

void InMemoryDatabaseManager::DBGraspFromGrasp(Grasp* db_grasp, const Grasp* grasp) const {
	db_grasp->SetGraspId(grasp->GraspId());
	db_grasp->SetEnergy(grasp->Energy());
	db_grasp->SetEpsilonQuality(0.0);
	db_grasp->SetVolumeQuality(0.0);
	db_grasp->SetClearance(grasp->Clearance());
	db_grasp->SetClusterRep(grasp->ClusterRep());
	db_grasp->SetTableClearance(grasp->TableClearance());
	db_grasp->SetCompliantCopy(grasp->CompliantCopy());
	db_grasp->SetCompliantOriginalId(grasp->CompliantOriginalId());
	
	std::vector<double> pregrasp_joints = grasp->GetPregraspJoints();
	std::vector<double> pregrasp_position = grasp->GetPregraspPosition();
	std::vector<double> grasp_joints = grasp->GetFinalgraspJoints();
	std::vector<double> grasp_position = grasp->GetFinalgraspPosition();
	
	db_grasp->SetGraspParameters(pregrasp_joints, 
							pregrasp_position, 
							grasp_joints, 
							grasp_position);
	db_grasp->SetPregraspJoints(pregrasp_joints);
	db_grasp->SetPregraspPosition(pregrasp_position);
	db_grasp->SetFinalgraspJoints(grasp_joints);
	db_grasp->SetFinalgraspPosition(grasp_position);
}

bool InMemoryDatabaseManager::ModelList(vector<Model*>* model_list, FilterList::FilterType filter) const {
	ModelMap::const_iterator itr;
	for (itr=models_.begin(); itr!=models_.end(); itr++) {
		Model* model = model_allocator_->Get();
		modelFromDBModel(model, itr->second);
		model_list->push_back(model);
	}
	return true;
}

bool InMemoryDatabaseManager::ScaledModel(Model* &model, int scaled_model_id) const {
	ModelMap::const_iterator db_model_itr = models_.find(scaled_model_id);
	if (db_model_itr == models_.end()) {
		return false;
	} else {
		model = model_allocator_->Get();
		modelFromDBModel(model,db_model_itr->second);
		return true;
	}
}

bool InMemoryDatabaseManager::AcquireNextTask(TaskRecord *rec, std::vector<std::string> accepted_types) {
	if (tasks_.empty()) {
		rec->taskType = "";
		rec->taskId = -1;
	} else {
		TaskList::iterator itr;
		for (itr=tasks_.begin();itr!=tasks_.end();itr++) {
			//std::cout << "task " << (*itr)->taskId << " status " << getTaskStatus((*itr)->taskId) << std::endl;
			if (getTaskStatus((*itr)->taskId) == "READY") {
				*rec = **itr;
				break;
			}
		}
		//*rec = *tasks_.front();
		//tasks_.pop();
	}
	return true;
//   std::vector< boost::shared_ptr<household_objects_database::DatabaseTask> > db_task;
//   if (!database_->acquireNextTask(db_task, accepted_types)) return false;
//   if (db_task.empty()) 
//   {
//     //no task to be run
//     rec->taskType = "";
//     return true;    
//   }
//   //populate the entry
//   rec->taskType = db_task[0]->type_.data();
//   rec->taskId = db_task[0]->id_.data();
//   return true;
}

bool InMemoryDatabaseManager::SetTaskStatus(int task_id, const string &status) {
	//std::cout << "Setting status for task " << task_id << ": " << status << std::endl;
	taskStatuses_[task_id] = status;
	return true;
//   household_objects_database::DatabaseTask db_task;
//   db_task.id_.data() = task_id;
//   db_task.outcome_name_.data() = status;
//   if ( !database_->saveToDatabase(&(db_task.outcome_name_)) ) return false;
//   return true;
}

bool InMemoryDatabaseManager::GetPlanningTaskRecord(int task_id, PlanningTaskRecord* rec) {
	TaskList::iterator itr;
	for (itr=tasks_.begin();itr!=tasks_.end();itr++) {
		if ((*itr)->taskId == task_id) {
			TaskRecord* dbrec = *itr;
			*rec = *(static_cast<PlanningTaskRecord*>(dbrec));
			return true;
		}
	}
	return false;
	
	/*
	if (tasks_.front()->taskId != task_id) {
		return false;
	} else {
		TaskRecord* dbrec = tasks_.front();
		std::cout << "db hand :" << static_cast<PlanningTaskRecord*>(dbrec)->handName << std::endl;
		*rec = *(static_cast<PlanningTaskRecord*>(dbrec));
		std::cout << "   hand: " << rec->handName << std::endl;
		tasks_.pop();
	}
	
	return true;
	*/
	
//   household_objects_database::DatabasePlanningTask planningTask;
//   planningTask.id_.data() = task_id;
//   if ( !database_->loadFromDatabase(&(planningTask.scaled_model_id_)) ||
//        !database_->loadFromDatabase(&(planningTask.hand_name_)) ||
//        !database_->loadFromDatabase(&(planningTask.time_)) )
//   {
//     std::cout << "Failed to load planning task for task id " << task_id << "\n";
//     return false;
//   }
//   rec->taskId = task_id;
//   rec->taskTime = planningTask.time_.data();
//   rec->handName = planningTask.hand_name_.data();
//   //load the actual model
//   //a fairly hacky way to do it for now, field-by-field
//   household_objects_database::DatabaseScaledModel db_model;
//   db_model.id_.data() = planningTask.scaled_model_id_.data();
//   //first we need to read the original model id so we can then read the rest
//   if ( !database_->loadFromDatabase(&(db_model.original_model_id_)) ) return false;
//   if ( !database_->loadFromDatabase(&(db_model.scale_)) ||
//        !database_->loadFromDatabase(&(db_model.tags_)) )
//   {
//     return false;
//   }
//   Model *model = model_allocator_->Get();
//   std::string model_root;
//   if (!database_->getModelRoot(model_root)) return false;
//   modelFromDBModel(model, db_model, model_root);
//   rec->model = model;
//   return true;
}

bool InMemoryDatabaseManager::GetOptimizationTaskRecord(int task_id, OptimizationTaskRecord *rec) {
	return false;
//   household_objects_database::DatabaseOptimizationTask optTask;
//   optTask.dbase_task_id_.data() = task_id;
//   if ( !database_->loadFromDatabase(&(optTask.parameters_)) || 
//        !database_->loadFromDatabase(&(optTask.hand_name_)) )
//   {
//     std::cout << "Failed to load optimization task for task id " << task_id << "\n";
//     return false;
//   }
//   rec->taskId = task_id;
//   rec->parameters = optTask.parameters_.data();
//   rec->hand_name = optTask.hand_name_.data();
//   return true;
}

bool InMemoryDatabaseManager::SaveOptimizationResults(
		const OptimizationTaskRecord &rec,
		const std::vector<double>& parameters,
		const std::vector<double>& results,
		const std::vector<double>& seed ) {
	return false;
//   household_objects_database::DatabaseOptimizationResult optResult;
//   optResult.dbase_task_id_.data() = rec.taskId;
//   optResult.parameters_.data() = parameters;
//   optResult.hand_name_.data() = rec.hand_name;
//   optResult.results_.data() = results;
//   optResult.seed_.data() = seed;
//   
//   if (!database_->insertIntoDatabase(&optResult))
//   {
//     std::cout << "Failed to save optimization results for task id " << rec.taskId << "\n";
//     return false;
//   }
//   return true;
}

bool InMemoryDatabaseManager::GraspTypeList(vector<string>* type_list) const {
	type_list->push_back("ALL");
	return true;
}

bool InMemoryDatabaseManager::GetGrasps(
		const Model& model,
		const string& hand_name, 
		vector<Grasp*>* grasp_list) const {
	GraspMap::const_iterator itr1 = grasps_.find(model.ModelId());
	if (itr1 == grasps_.end()) {
		//std::cout << "didn't find model " << model.ModelId() << std::endl;
		return false;
	}
	HandGraspMap::const_iterator itr2 = itr1->second.find(hand_name);
	if (itr2 == itr1->second.end()) {
		//std::cout << "didn't find hand " << hand_name << std::endl;
		return false;
	}
	
	GraspList::const_iterator db_grasp;
	for (db_grasp=itr2->second.begin(); db_grasp!=itr2->second.end(); db_grasp++) {
		Grasp* grasp;
		try {
		grasp = grasp_allocator_->Get();
		} catch (const std::bad_alloc& e) {
			std::cerr << "error ga-get " << e.what() << std::endl;
		}
		grasp->SetSourceModel(model);
		grasp->SetHandName(hand_name);

		try {
		graspFromDBGrasp(grasp,*db_grasp);
		} catch (const std::bad_alloc& e) {
			std::cerr << "error gfromdb " << e.what() << std::endl;
		}
		
		grasp_list->push_back(grasp);
	}
	return true;
}

bool InMemoryDatabaseManager::SaveGrasp(const Grasp* grasp) const {
	Grasp* db_grasp = grasp_allocator_->Get();
	
	DBGraspFromGrasp(db_grasp,grasp);
	
	const_cast<InMemoryDatabaseManager*>(this)->grasps_[grasp->SourceModel().ModelId()][grasp->HandName()].push_back(db_grasp);
	return true;
}

bool InMemoryDatabaseManager::DeleteGrasp(Grasp* grasp) const
{
	GraspMap::iterator itr1 = const_cast<InMemoryDatabaseManager*>(this)->grasps_.find(grasp->SourceModel().ModelId());
	if (itr1 == grasps_.end()) {
		return false;
	}
	HandGraspMap::iterator itr2 = itr1->second.find(grasp->HandName());
	if (itr2 == itr1->second.end()) {
		return false;
	}
	
	GraspList::iterator db_grasp;
	for (db_grasp=itr2->second.begin(); db_grasp!=itr2->second.end(); db_grasp++) {
		if ((*db_grasp)->GraspId() == grasp->GraspId()) {
			itr2->second.erase(db_grasp);
			return true;
		}
	}
	return false;
}

bool InMemoryDatabaseManager::SetGraspClusterRep(Grasp *grasp, bool rep) const
{
	//TODO: implement
	std::cout << "SetGraspClusterRep: returning false" << std::endl;
	return false;
//   household_objects_database::DatabaseGrasp db_grasp;
//   db_grasp.id_.get() = grasp->GraspId();
//   db_grasp.cluster_rep_.get() = rep;
//   if ( !database_->saveToDatabase(&db_grasp.cluster_rep_) ) return false;
//   return true;
}
bool InMemoryDatabaseManager::SetGraspTableClearance(Grasp *grasp, double clearance) const
{
	//TODO: implement
	std::cout << "SetGraspTableClearance: returning false" << std::endl;
	return false;
//   household_objects_database::DatabaseGrasp db_grasp;
//   db_grasp.id_.get() = grasp->GraspId();
//   db_grasp.table_clearance_.get() = clearance;
//   if ( !database_->saveToDatabase(&db_grasp.table_clearance_) ) return false;
//   return true;
}

bool InMemoryDatabaseManager::InsertGraspPair(const Grasp *grasp1, const Grasp *grasp2) const
{
  //grasp pair not used anymore
  return false;
  /*
  household_objects_database::DatabaseGraspPair db_pair;
  db_pair.grasp1_id_.get() = grasp1->GraspId();
  db_pair.grasp2_id_.get() = grasp2->GraspId();
  if ( !database_->insertIntoDatabase(&db_pair) ) return false;
  return true;
  */
}

bool InMemoryDatabaseManager::LoadModelGeometry(Model* model) const
{
	std::cout << "LoadModelGeometry: returning false" << std::endl;
	return false;
//   household_objects_database::DatabaseMesh mesh;
//   if (!database_->getScaledModelMesh(model->ModelId(), mesh)) return false;
//   //this gets the geometry in ROS units, which are meters
//   //scale to mm
//   for (size_t i=0; i<mesh.vertices_.data().size(); i++) {
//     mesh.vertices_.data().at(i) = 1000 * mesh.vertices_.data().at(i);
//   }
//   model->SetGeometry(mesh.vertices_.data(), mesh.triangles_.data());
//   return true;
}

bool InMemoryDatabaseManager::AddModel(const Model* model) {
	Model* db_model = model_allocator_->Get();
	DBModelFromModel(db_model,model);
	models_[model->ModelId()] = db_model;
	std::cout << "AddModel: Models size " << models_.size() << std::endl;
	return true;
}

Model* InMemoryDatabaseManager::AddModelFromMesh(int id,const std::string& name,const std::vector<double>& vertices, const std::vector<int>& triangles) {
	Model* model = model_allocator_->Get();
	
	model->SetModelId( id );
	model->SetScale( 1 );
	model->SetRescaleFactor( 1.0 );
	model->SetModelName( name );
	//model->SetTags(  );
	
	std::vector<double> m_vertices;
	std::vector<double>::const_iterator itr;
	for (itr=vertices.begin();itr!=vertices.end();itr++) {
		m_vertices.push_back(1000. * (*itr));
	}

	model->SetGeometry( m_vertices, triangles );
	
	models_[id] = model;
	
	std::cout << "Created model " << name << std::endl;
	
	Model* return_model = model_allocator_->Get();
	
	modelFromDBModel(return_model,model);
	
	return return_model;
}

Model* InMemoryDatabaseManager::AddModelFromMesh(const std::vector<double>& vertices, const std::vector<int>& triangles) {
	int id = rand() % 1000;
	std::stringstream name;
	name << "model" << id;
	return AddModelFromMesh(id,name.str(),vertices,triangles);
}

bool InMemoryDatabaseManager::AddTask(TaskRecord* rec) {
	tasks_.push_back(rec);
	printf("Task added %d\n",(int)tasks_.size());
	return true;
}

} //namespace db_planner
