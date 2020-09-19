#ifndef POLYNOMIALS_HPP_
#define POLYNOMIALS_HPP_

#include <math.h>
#include <eigen3/Eigen/Dense>
#include <cmath>

using namespace Eigen;

class quintic{
private:
  double xs, vxs, axs, xe, vxe, axe, a0, a1, a2, a3, a4, a5;
public:
  quintic(double, double, double, double, double, double, double);
  double calc_point(double);
  double calc_first_derivative(double);
  double calc_second_derivative(double);
  double calc_third_derivative(double);
};

class quartic{
private:
  double xs, vxs, axs, vxe, axe, a0, a1, a2, a3, a4;
public:
  quartic(double, double, double, double, double, double);
  double calc_point(double);
  double calc_first_derivative(double);
  double calc_second_derivative(double);
  double calc_third_derivative(double);
};

#endif  // POLYNOMIALS_HPP_
