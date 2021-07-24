#include "../include/polynomials.hpp"

quintic::quintic(double xs_t, double vxs_t, double axs_t, double xe_t, double vxe_t, double axe_t,
double T)
{
  xs = xs_t;
  vxs = vxs_t;
  axs = axs_t;
  xe = xe_t;
  vxe = vxe_t;
  axe = axe_t;

  a0 = xs;
  a1 = vxs;
  a2 = axs / 2.0;

  MatrixXd A(3, 3);
  MatrixXd B(3, 1);
  MatrixXd X(3, 1);

  A << pow(T, 3), pow(T, 4), pow(T, 5),
       3*pow(T, 2), 4*pow(T, 3), 5*pow(T, 4),
       6*T, 12*T*T, 20*pow(T, 3);
  B << xe - a0 - a1*T - a2*T*T,
       vxe - a1 - 2*a2*T,
       axe - 2*a2;

  X = A.inverse()*B;

  a3 = X(0, 0);
  a4 = X(1, 0);
  a5 = X(2, 0);
}

double quintic::calc_point(double t)
{
  double xt = a0 + a1*t + a2*t*t + a3*t*t*t + a4*t*t*t*t + a5*t*t*t*t*t;
  return xt;
}

double quintic::calc_first_derivative(double t)
{
  double xt = a1 + 2*a2*t + 3*a3*t*t + 4*a4*t*t*t + 5*a5*t*t*t*t;
  return xt;
}
double quintic::calc_second_derivative(double t)
{
  double xt = 2*a2 + 6*a3*t + 12*a4*t*t + 20*a5*t*t*t;
  return xt;
}

double quintic::calc_third_derivative(double t)
{
  double xt = 6*a3 + 24*a4*t + 60*a5*t*t;
  return xt;
}


quartic::quartic(double xs_t, double vxs_t, double axs_t, double vxe_t, double axe_t, double T)
{
  xs = xs_t;
  vxs = vxs_t;
  axs = axs_t;
  vxe = vxe_t;
  axe = axe_t;

  a0 = xs;
  a1 = vxs;
  a2 = axs / 2.0;

  MatrixXd A(2, 2);
  MatrixXd B(2, 1);
  MatrixXd X(2, 1);

  A << 3*pow(T, 2), 4*pow(T, 3),
       6*T, 12*T*T ;

  B << vxe - a1 - 2*a2*T,
       axe - 2*a2;

  X = A.inverse()*B;

  a3 = X(0, 0);
  a4 = X(1, 0);
}

double quartic::calc_point(double t)
{
  double xt = a0 + a1*t + a2*t*t + a3*t*t*t + a4*t*t*t*t;
  return xt;
}

double quartic::calc_first_derivative(double t)
{
  double xt = a1 + 2*a2*t + 3*a3*t*t + 4*a4*t*t*t;
  return xt;
}

double quartic::calc_second_derivative(double t)
{
  double xt = 2*a2 + 6*a3*t + 12*a4*t*t;
  return xt;
}

double quartic::calc_third_derivative(double t)
{
  double xt = 6*a3 + 24*a4*t;
  return xt;
}
