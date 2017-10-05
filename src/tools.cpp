#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {

  VectorXd rmse(4);
  rmse << 0,0,0,0;
  
  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if(estimations.size() != ground_truth.size()
          || estimations.size() == 0){
      cout << "Invalid estimation or ground_truth data" << endl;
      return rmse;
  }
  
  //accumulate squared residuals
  for(unsigned int i=0; i < estimations.size(); ++i){
  
      VectorXd residual = estimations[i] - ground_truth[i];
  
      //coefficient-wise multiplication
      residual = residual.array()*residual.array();
      rmse += residual;
  }
  
  //calculate the mean
  rmse = rmse/estimations.size();
  
  //calculate the squared root
  rmse = rmse.array().sqrt();
  
  //return the result
  return rmse;
}


VectorXd Tools::CarthesianToPolar(const VectorXd& x_input) {

  VectorXd x_out = VectorXd(3);

  float p_x = x_input[0];
  float p_y = x_input[1];
  float v_x = x_input[2];
  float v_y = x_input[3];

  float len_r = sqrt(p_x*p_x + p_y*p_y);
  float rate;
  if (len_r < 0.00001)
  {
    rate = 0.0;
  }
  else
  {
    rate = (p_x*v_x + p_y*v_y) / len_r;
  }

  x_out << len_r, atan2(p_y, p_x), rate;

  return x_out;
}


VectorXd Tools::PolarToCarthesian(const VectorXd& x_input) {

  VectorXd x_out = VectorXd(4);

  float rho = x_input[0];
  float phi = x_input[1];
  float rho_rate = x_input[2];

  float p_x = rho * cos(phi);
  float p_y = rho * sin(phi);
  float v_x = rho_rate * cos(phi);
  float v_y = rho_rate * sin(phi);

  x_out << p_x, p_y, v_x, v_y;

  return x_out;
}

float Tools::NormalizeAtanDiff(const float atan_diff) {
  float PI_2 = 2 * M_PI;
    if (atan_diff > PI_2) {
      return atan_diff - PI_2;
    } else if ( atan_diff < -1 * PI_2) {
      return atan_diff + PI_2;
    } else {
      return atan_diff;
    }
}

