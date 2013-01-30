#ifndef INMEMORY_DATABASE_MANAGER_H
#define INMEMORY_DATABASE_MANAGER_H

#include <string>
#include <vector>
#include <list>
#include <map>
#include <utility>

#include "db_manager.h"
#include "model.h"
#include "task.h"
#include "grasp.h"

using std::pair;
using std::string;
using std::vector;
using std::map;

namespace db_planner {

typedef int ModelId;
typedef string HandName;
typedef vector<Grasp*> GraspList;

typedef map<int,Model*> ModelMap; //model id
typedef map<HandName,GraspList > HandGraspMap; //hand name
typedef map<ModelId,HandGraspMap> GraspMap; //model id

typedef std::list<TaskRecord*> TaskList;
typedef map<int,string> TaskStatusMap; //task id

//! Pure virtual base class for interfacing GraspIt with an unspecified Grasp Database.
class InMemoryDatabaseManager : public DatabaseManager 
{
 protected:
	 
	 ModelMap models_;
	 GraspMap grasps_;
	 
	 TaskList tasks_;
	 TaskStatusMap taskStatuses_;
	 string getTaskStatus(int task_id) { return taskStatuses_.find(task_id)!=taskStatuses_.end()?taskStatuses_[task_id]:"READY"; }

  //! Initializes a graspit model from a raw database model. Helper for ModelList
  void modelFromDBModel(Model* model,const Model* db_model) const;
  void DBModelFromModel(Model* db_model, const Model* model) const;
  void graspFromDBGrasp(Grasp* grasp,const Grasp* db_grasp) const;
  void DBGraspFromGrasp(Grasp* db_grasp,const Grasp* grasp) const;
 public:
  InMemoryDatabaseManager(ModelAllocator *model_allocator, GraspAllocator* grasp_allocator);
  ~InMemoryDatabaseManager();

  //! Sets a new grasp allocator; deletes the old one
  void SetGraspAllocator(GraspAllocator* allocator) {
    delete grasp_allocator_;
    grasp_allocator_ = allocator;
  }
  //! Sets a new model allocator; deletes the old one
  void SetModelAllocator(ModelAllocator* allocator) {
    delete model_allocator_;
    model_allocator_ = allocator;
  }

  //! Not implemented
  virtual bool GetAlignment(const Model&, const Model&, const string&, float*) const {return false;}
  //! Not implemented
  virtual bool SaveAlignment(const Model&, const Model&, const string&, const float*) const {return false;}
  //! Not implemented
  virtual bool GetNeighbors(const Model&, const string&, const int, 
			    vector<pair<Model*, double> >*) const {return false;}
  //! Not implemented
  virtual bool SaveNeighbors(const Model&, const string&,
			     const vector<pair<Model*, double> >&) const {return false;}
  //! Not implemented
  virtual bool DistanceFunctionList(vector<string>*) const {return false;}
  //! Not implemented
  virtual bool AlignmentMethodList(vector<string>*) const {return false;}

  //! Returns true, there is connection
  virtual bool isConnected() const { return true; }

  //! Get a list of models in the database
  virtual bool ModelList(vector<Model*>* model_list, 
			 FilterList::FilterType filter = FilterList::NONE) const;
  //! Gets one individual model from the database based on scaled model id
  virtual bool ScaledModel(Model* &model, int scaled_model_id) const;

  //! Acquires the next experiment to be executed from the list of tasks in the database
  /*! If accepted_types is not empty, it will only get tasks of one of that types. If it's empty,
    it will get any task. Also marks any task it gets as RUNNING in an atomic fashion, so that it 
    is not acquired by another process.*/
  virtual bool AcquireNextTask(TaskRecord *rec, std::vector<std::string> accepted_types);
  //! Change the status of a task in the database (e.g. mark it as COMPLETED)
  virtual bool SetTaskStatus(int task_id, const string &status);
  //! Fills in the details for a planning task based on the task id
  virtual bool GetPlanningTaskRecord(int task_id, PlanningTaskRecord *rec);
  //! Fills in the details for an optimzation task based on the task id
  virtual bool GetOptimizationTaskRecord(int task_id, OptimizationTaskRecord *rec);
  //! Saves the results of an optimization in the database
  virtual bool SaveOptimizationResults(const OptimizationTaskRecord &rec,
                                       const std::vector<double>& parameters,
                                       const std::vector<double>& results,
				       const std::vector<double>& seed );
  //! Get a list of grasp types available in the database
  virtual bool GraspTypeList(vector<string>* type_list) const;
  //! Get a list of the Grasps for a Model.
  virtual bool GetGrasps(const Model& model, const string& hand_name, vector<Grasp*>* grasp_list) const;

  //! Save a grasp into the database
  virtual bool SaveGrasp(const Grasp*) const;
  //! Delete a grasp from the database
  virtual bool DeleteGrasp(Grasp* grasp) const;
  //! Sets the cluster_rep field of the grasp in the database  
  virtual bool SetGraspClusterRep(Grasp *grasp, bool rep) const;
  //! Sets the table_clearance field of the grasp in the database
  virtual bool SetGraspTableClearance(Grasp *grasp, double clearance) const;
  //! Inserts into the database info that a pair of existing db grasps can be executed simultaneously
  virtual bool InsertGraspPair(const Grasp *grasp1, const Grasp *grasp2) const;
  //! Loads the model's mesh directly from the database
  virtual bool LoadModelGeometry(Model* model) const;
  
  virtual bool AddModel(const Model* model);
  
  virtual Model* AddModelFromMesh(int id,const std::string& name,const std::vector<double>& vertices, const std::vector<int>& triangles);
  virtual Model* AddModelFromMesh(const std::vector<double>& vertices, const std::vector<int>& triangles);
  
  virtual bool AddTask(TaskRecord* rec);
};

}
#endif  // ROS_DATABASE_MANAGER_H
