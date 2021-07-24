#ifndef FRENET_OPTIMAL_TRAJECTORY_HPP_
#define FRENET_OPTIMAL_TRAJECTORY_HPP_

#include "../include/polynomials.hpp"
#include "../include/parameters.hpp"
#include "../include/cubic_spline_planner.hpp"

class FrenetPath
{
  private :
    vecD t, d, d_d, d_dd, d_ddd, s, s_d, s_dd, s_ddd, x, y, yaw, ds, c;
    /*ye c kya hain radius of curvature hain kya, agr haa to frenet_ros main iska use na krke abhi
    local variable ka use ho rha hain initial conditions() main */
    double Js, Jp, cd, cv, cf;
  /*
      t is the vector comprising of the discrete points in time at which other translational variables are calculated
      d is the vector comprising of the lateral offset(distance) from the expected global path at the discrete points in time
      d_d is the vector comprising of the first derivative of d with respect to t
      d_dd is the vector comprising of the second derivative of d with respect to t
      d_ddd is the vector comprising of the third derivative of d with respect to t
      s is the vector comprising of the longitudnal arc length covered at the discrete points in time.
      s_d is the vector comprising of the first derivative of s with respect to t
      s_dd is the vector comprising of the second derivative of s with respect to t
      s_ddd is the vector comprising of the third derivative of s with respect to t
      Js is the summation comprising of the square of all values of d_ddd (latitudnal jerk)
      Jp is the summation comprising of the square of all values of s_ddd (longitusnal jerk)
      cd is the cost of the lateral trajectory in the frenet path
      cv if the cost of the longitudnal trajectory in the frenet path
      cf is the total cost of the frenet path(comprises of weighted values of cd and cv)
      
  */
  public :
    vecD get_t();
    vecD get_d();
    vecD get_d_d();
    vecD get_d_dd();
    vecD get_d_ddd();
    vecD get_s();
    vecD get_s_d();
    vecD get_s_dd();
    vecD get_s_ddd();
    vecD get_x();
    vecD get_y();
    vecD get_yaw();
    vecD get_ds();
    vecD get_c();
    double get_cf();
    void set_Jp(double);
    void set_Js(double);

    double get_Jp();
    double get_Js();

    void calc_lat_paths(double , double , double , double , double, double );
    void calc_lon_paths(double , double , double , double);
    void calc_lon_paths_quintic_poly(double , double , double , double , double  );
    void adding_global_path(Spline2D );
    bool check_collision(double);
    void plot_path();
    void plot_velocity_profile();
    friend ostream& operator<< (ostream& os, const FrenetPath& fp);
};
void get_limits_d(FrenetPath , double* , double *);
vector<FrenetPath> check_path(vector<FrenetPath>&, double, double, double);
vector<FrenetPath> calc_frenet_paths(double, double, double, double, double, FrenetPath );
vector<FrenetPath> calc_global_paths(vector<FrenetPath> &, double);
FrenetPath frenet_optimal_planning(Spline2D, double, double, double, double, double, FrenetPath,
double);

/*******for printing the frenet path***********/
template<class T> ostream& operator<<(ostream &os, vector<T> V) { os << "[ "; for(auto v : V) os <<
v << " "; return os << "]"; }
// #define trace(...) __f(#__VA_ARGS__, __VA_ARGS__)
// template <typename Arg1>
// void __f(const char* name, Arg1&& arg1){cerr << name << " : " << arg1 << endl;}
// template <typename Arg1, typename... Args>
// void __f(const char* names, Arg1&& arg1, Args&&... args){ const char* comma = strchr(names + 1,
//','); cerr.write(names, comma - names) << " : " << arg1<<" | "; __f(comma+1, args...); }
#define trace(...) 42
inline ostream& operator<<(ostream& os, const FrenetPath& fp)
{
  trace(fp.t);
  trace(fp.d);
  trace(fp.d_d);
  trace(fp.d_dd);
  trace(fp.d_ddd);
  trace(fp.s);
  trace(fp.s_d);
  trace(fp.s_dd);
  trace(fp.s_ddd);
  trace(fp.x);
  trace(fp.y);
  trace(fp.yaw);
  trace(fp.ds);
  return os;
}
/***********************end******************/
inline vecD FrenetPath::get_t()
{
  return t;
}
inline vecD FrenetPath::get_d()
{
  return d;
}
inline vecD FrenetPath::get_d_d()
{
  return d_d;
}
inline vecD FrenetPath::get_d_dd()
{
  return d_dd;
}
inline vecD FrenetPath::get_d_ddd()
{
  return d_ddd;
}
inline vecD FrenetPath::get_s()
{
  return s;
}
inline vecD FrenetPath::get_s_d()
{
  return s_d;
}
inline vecD FrenetPath::get_s_dd()
{
  return s_dd;
}
inline vecD FrenetPath::get_s_ddd()
{
  return s_ddd;
}
inline vecD FrenetPath::get_x()
{
  return x;
}
inline vecD FrenetPath::get_y()
{
  return y;
}
inline vecD FrenetPath::get_yaw()
{
  return yaw;
}
inline vecD FrenetPath::get_ds()
{
  return ds;
}
inline vecD FrenetPath::get_c()
{
  return c;
}
inline double FrenetPath::get_cf()
{
  return cf;
}
inline void FrenetPath::set_Jp(double jp)
{
  Jp = jp;
}
inline void FrenetPath::set_Js(double js)
{
  Js = js;
}
inline double FrenetPath::get_Jp()
{
  return Jp;
}
inline double FrenetPath::get_Js()
{
  return Js;
}
#endif  // FRENET_OPTIMAL_TRAJECTORY_HPP_
