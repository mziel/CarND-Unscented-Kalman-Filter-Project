#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include <cmath>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

class Tools {
public:
  /**
  * Constructor.
  */
  Tools();

  /**
  * Destructor.
  */
  virtual ~Tools();

  /**
  * A helper method to calculate RMSE.
  */
  VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);

/**
  * A helper method to process function h(x) for radar data. (convert carthesian coordinates to polar)
  * Convert carthesian coordinates to polar.
  */
  VectorXd CarthesianToPolar(const VectorXd& x_state);

/**
  * A helper method for processing input data.
  * Convert polar coordinates to carthesian.
  */
  VectorXd PolarToCarthesian(const VectorXd& x_state);

/**
  * A helper method to normalize atan difference between (-M_PI, M_PI)
  */
  float NormalizeAtanDiff(const float atan_diff);
};

#endif /* TOOLS_H_ */
