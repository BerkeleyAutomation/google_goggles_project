#pragma once
#include <vector>
#include <Eigen/Dense>
using namespace std;

class btVector3;

Eigen::MatrixXf pairwiseSquareDist(const Eigen::MatrixXf& x_m3, const Eigen::MatrixXf& y_n3);
vector<int> argminAlongRows(const Eigen::MatrixXf& d_mn);
