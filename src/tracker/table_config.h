#include "config.h"

struct TableConfig : public Config {
  static float MIN_HEIGHT;
  static float MAX_HEIGHT;
  static float HIST_RES;
  static float TABLE_POINTS_TOLERANCE;
  static float VOXEL_SIZE;
  static float TABLE_CLUSTERING_TOLERANCE;
  static float OBJECT_CLUSTERING_TOLERANCE;
  static int OBJECT_CLUSTER_MIN_SIZE;
  static float OBJECT_MATCH_TOLERANCE;
  static float ABOVE_TABLE_CUTOFF;
  
  static float MAX_X_DIST;
  static float MAX_Y_DIST;

  TableConfig() : Config() {
    params.push_back(new Parameter<float>("min_height", &MIN_HEIGHT, ""));
    params.push_back(new Parameter<float>("max_height", &MAX_HEIGHT, ""));
    params.push_back(new Parameter<float>("hist_res", &HIST_RES, ""));
    params.push_back(new Parameter<float>("table_pts_tol", &TABLE_POINTS_TOLERANCE, ""));
    params.push_back(new Parameter<float>("voxel_size", &VOXEL_SIZE, ""));
    params.push_back(new Parameter<float>("table_clu_tol", &TABLE_CLUSTERING_TOLERANCE, ""));
    params.push_back(new Parameter<float>("obj_clu_tol", &OBJECT_CLUSTERING_TOLERANCE, ""));
    params.push_back(new Parameter<int>("obj_clu_min_size", &OBJECT_CLUSTER_MIN_SIZE, ""));
    params.push_back(new Parameter<float>("obj_match_tol", &OBJECT_MATCH_TOLERANCE, ""));
    params.push_back(new Parameter<float>("above_table_cutoff", &ABOVE_TABLE_CUTOFF, ""));
	
	 params.push_back(new Parameter<float>("max_x_dist", &MAX_X_DIST, ""));
	 params.push_back(new Parameter<float>("max_y_dist", &MAX_Y_DIST, ""));
  }

};

