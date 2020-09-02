
#include <vector>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <cmath>

#define NONE -1e9
using namespace std;
using namespace Eigen;

using vecD =  vector<double>;
class Spline
{
  private:
    vecD a, b, c, d, w;
    vecD x, y;
    int nx;

    // Functions
    MatrixXd calc_A(vecD h);
    MatrixXd calc_B(vecD h);
    int search_index(double p);

  public:
    void init(vecD x_in, vecD y_in);
    double calc(double t);  // complete
    double calcd(double t);  // complete
    double calcdd(double t);  // complete
};

class Spline2D
{
  private:
  vecD x, y, s, ds;
  Spline sx, sy;

  public:
    Spline2D(vecD x_in, vecD y_in)
    {
      x = x_in;
      y = y_in;
      s = calc_s(x, y);
      sx.init(s, x);
      sy.init(s, y);
    }


    vecD calc_s(vecD x, vecD y);
    void calc_position(double &x, double &y, double t);
    double calc_curvature(double t);
    double calc_yaw(double t);
    double get_s_last();
};  // end of class

Spline2D calc_spline_course(vecD x, vecD y, vecD &rx, vecD &ry, vecD &ryaw, vecD &rk, double ds);
void printVecD(vecD A);

void printVecD(vecD A)
{
  cout << "\n------------Printing the vector----------\n";
  for(unsigned int i = 0; i < A.size(); i++)
    cout << A[i] << "\n";
  cout << "\n-----------------------------------------\n" << endl;
}

MatrixXd Spline::calc_A(vecD h)  // get matrix A
{
  MatrixXd A = MatrixXd::Zero(nx, nx);
  A(0, 0) = 1.0;
  for(int i = 0; i < nx - 1; i++)
  {
    if(i != nx - 2)
      A(i + 1, i + 1) = 2.0*(h[i] + h[i + 1]);
    A(i + 1, i) = h[i];
    A(i, i + 1) = h[i];
  }

  A(0, 1) = 0.0;
  A(nx - 1, nx - 2) = 0.0;
  A(nx - 1, nx - 1) = 1.0;

  return A;
}

MatrixXd Spline::calc_B(vecD h)  // get matrix B
{
  MatrixXd B = MatrixXd::Zero(nx, 1);

  for(int i = 0; i < nx - 2; i++)
    B(i + 1, 0) = 3.0*(a[i + 2] - a[i + 1]) / h[i + 1] - 3.0*(a[i + 1] - a[i]) / h[i];

  return B;
}

int Spline::search_index(double p)  // returns the index of the just greater element
{
  auto it  = upper_bound(x.begin(), x.end(), p);
  if(it == x.end())
  {
    return x.size()-2;
  } else {
    return it-x.begin()-1;
  }
}

void Spline::init(vecD x_in, vecD y_in)  // calculates the coeffs for splines
{
  x = x_in;
  y = y_in;
  nx = x.size();
  vecD h;
  for(int i = 1; i < nx; i++)
    h.push_back(x[i] - x[i - 1]);
  a = y;

  MatrixXd A = calc_A(h);

  MatrixXd B = calc_B(h);

  MatrixXd C = A.inverse()*B;  // C = A^(-1)*B

  for(int i = 0; i < C.rows(); ++i)
    c.push_back(C(i, 0));
  for(int i = 0; i < nx -1; i++)
  {
    d.push_back((c[i + 1] - c[i]) / (3.0*h[i]));
    double tb = (a[i + 1] - a[i]) / h[i] - h[i]*(c[i + 1] + 2.0*c[i]) / 3.0;
    b.push_back(tb);
  }
}

double Spline::calc(double t)  // find y at given x
{
  if(x.size() == 0)
  {
    cerr<<136<<endl;
    return NONE;
  } else if(t < x[0]) {
    cerr<<139<<endl;
    return NONE;
  } else if(t > x[nx - 1]) {
    cerr<<"sixe "<<nx-1<<"  "<<x.size()<<endl;
    cerr<<"last element "<<x.back()<<endl;
    cerr<<t<<"  "<<x[nx-1]<<endl;
    cerr<<142<<endl;
    return NONE;
  }
  int i = search_index(t);
  double dx = t - x[i];
  if(i > nx-1)
  {
    cerr<<149<<endl;
    return NONE;
  }
  // assert(i<b.size());
  double result = a[i] + b[i]*dx + c[i]*dx*dx + d[i]*dx*dx*dx;
  return result;
}

double Spline::calcd(double t)  // find y_dot at given x
{
  if(t < x[0])
  {
    return NONE;
  }else if(t > x[nx - 1]){
    return NONE;
  }
  int i = search_index(t);
  double dx = t - x[i];
  double result = b[i] + 2*c[i]*dx + 3*d[i]*dx*dx;

  return result;
}


double Spline::calcdd(double t)  // y_doubleDot at given x
{
  if(t < x[0])
    return NONE;
  else if(t > x[nx - 1])
    return NONE;

  int i = search_index(t);

  double dx = t - x[i];
  double result = 2*c[i] + 6*d[i]*dx;

  return result;
}


/****************************
Spline 2D generates the parametric cubic spline 
of x and y as a function of s:

x = f(s)
y = g(s)
****************************/


vecD Spline2D::calc_s(vecD x, vecD y)  // approximately calculates s along the spline
{
  vecD dx;
  for(unsigned int i = 1; i < x.size(); i++)
  {
    dx.push_back(x[i] - x[i - 1]);
  }
  vecD dy;
  for(unsigned int i = 1; i < x.size(); i++)
  {
    dy.push_back(y[i] - y[i - 1]);
  }
  ds.clear();
  for(unsigned int i = 0; i < dx.size(); i++)
  {
    double temp;
    temp = sqrt(dx[i]*dx[i] + dy[i]*dy[i]);
    ds.push_back(temp);
  }

  vecD t;
  t.push_back(0);
  t.push_back(ds[0]);
  for(unsigned int i = 1; i < ds.size(); i++)
  {
    cerr<<t.back()+ds[i]<<endl;
    t.push_back(t.back() + ds[i]);
  }

  return t;
}

void Spline2D::calc_position(double &x, double &y, double t)
{
  x = sx.calc(t);
  y = sy.calc(t);
}


double Spline2D::calc_curvature(double t)
{
  double dx = sx.calcd(t);
  double ddx = sx.calcdd(t);

  double dy = sy.calcd(t);
  double ddy = sy.calcdd(t);

// DOUBT : denominator should have power of 3/2.
  // https://www.math24.net/curvature-radius/
// ==============================================================================
  double k = (ddy*dx - ddx*dy) / (dx*dx  + dy *dy);
// ==============================================================================

  return k;
}


double Spline2D::calc_yaw(double t)
{
  double dx = sx.calcd(t);
  double dy = sy.calcd(t);

  if(dx == 0)
  {
    return 1.57*(dy > 0);
  }
  double yaw = atan2(dy, dx);

  return yaw;
}


double Spline2D::get_s_last()
{
  return s.back();
}


// generates the Spline2D with points along the spline at distance = ds, also returns
// yaw and curvature
Spline2D calc_spline_course(vecD x, vecD y, vecD &rx, vecD &ry, vecD &ryaw, vecD &rk, double ds)
{
  Spline2D sp(x, y);
  vecD s;
  double sRange = sp.get_s_last();
  double sInc = 0;
  cerr<<"Sincccccccccc\n"<<sRange<<endl;
  while(1)
  {
    if(sInc >= sRange)
    {
      break;
    }
    s.push_back(sInc);
    //cerr<<sInc<<endl;
    sInc = sInc + ds;
  }
  // for(i =0 ; i*ds<sRange;i++)  i*ds = sInc
  //   s[i]=i*ds;
  rx.resize(s.size());
  ry.resize(s.size());
  ryaw.resize(s.size());
  rk.resize(s.size());
  for(int i = 0; i < s.size(); i++)
  {
    double ix, iy;
    sp.calc_position(ix, iy, s[i]);
    rx[i] = ix;
    ry[i] = iy;
    ryaw[i] = sp.calc_yaw(s[i]);
    rk[i] = sp.calc_curvature(s[i]);
  }
  return sp;
}
inline double calc_dis(double x1, double y1, double x2, double y2)
{
  return sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2));
}

int main(){
  vector<double> W_X = {38, 38, 38, 38, 38, 38, 38 , 38.21, 39.25, 42.56, 48.97, 61} ;
vector<double> W_Y = {-57, -45, -32.0, -18.5, -12.0, 0.0, 12, 35, 42.89, 80.41, 131.48, 139.47};
vecD rx, ry, ryaw, rk;
double ds1 = 0.1; 
Spline2D csp = calc_spline_course(W_X, W_Y, rx, ry, ryaw, rk, ds1);
  
  vector<double> global_s(rx.size());
  double s2 = 0;
  global_s[0] = 0;
  for(unsigned int i = 1; i < rx.size(); i++)
  {
	double dis = calc_dis(rx[i], ry[i], rx[i - 1], ry[i - 1]);
	s2 = s2 + dis;
	global_s[i] = s2;
  }
  
  vecD s1;
  double sRange = csp.get_s_last();
  double sInc = 0;
  while(1)
  {
    if(sInc >= sRange)
    {
      break;
    }
    s1.push_back(sInc);
    sInc = sInc + ds1;
  }
  
  
  /*double sRange = csp.get_s_last();
  double sInc = 0;
  vector<double> s2;
  //double ds1 = 0.1;
  while(1)
  {
    if(sInc >= sRange)
    {
      break;
    }
    s2.push_back(sInc);
    sInc = sInc + ds1;
  }*/
  cerr<<"hiiiiiiiiii\n";
  cerr<<endl<<s1.back()<<"  "<<s1.size();
  cerr<<endl<<global_s.back()<<"  "<<global_s.size();







 vecD x, y, yaw, ds, c;

 vecD d={-0.523238,  -0.525306,  -0.526709,  -0.526887,  -0.525388,  -0.521859,  -0.516044,  -0.507773,  -0.496957,  -0.483583,  -0.467704,  -0.449434,  -0.428941,  -0.406441,  -0.382191,  -0.356481,  -0.329629,  -0.301975,  -0.27387,  -0.245674,  -0.217749,  -0.190448,  -0.164112,  -0.139064,  -0.115599,  -0.0939799,  -0.0744295,  -0.0571245,  -0.0421886,  -0.0296855,  -0.0196126,  -0.0118942,  -0.00637459,  -0.0028115,  -0.000869461,  -0.000113007 };
 
 vecD s={189.343,  190.646,  191.933,  193.19,  194.406,  195.57,  196.674,  197.712,  198.679,  199.57,  200.383,  201.118,  201.774,  202.353,  202.856,  203.287,  203.65,  203.947,  204.186,  204.37, 204.505,  204.597,  204.652,  204.676,  204.676,  204.656,  204.622,  204.58,  204.534,  204.488,  204.445,  204.409,  204.38,  204.36,  204.349,  204.344  };

/*int n = s.size();
  //cerr<<"n is "<<n<<endl;
  /*if(STOP_CAR)
  {
    cerr<<"s_dest ="<<s_dest<<" size is "<<n<<" is"<<endl;
    for(auto i : s)
      cerr<<i<<" ";
    cerr<<endl;
  }
  x.resize(n);
  y.resize(n);
  for(unsigned int i = 0; i < n; i++)
  {
    double ix, iy;
    // trace(i);
    csp.calc_position(ix, iy, s[i]);
    // trace("done calc_position");
    if(ix == NONE)
    {
      // if(STOP_CAR)
      cerr<<"hii"<<endl;
      cerr<<"ix "<<ix<<" iy  "<<iy<<" s[i] "<<s[i]<<" i is "<<i;
      return 0;
      //break;
    }
    double iyaw = csp.calc_yaw(s[i]);
    
    double fx = ix - d[i]*sin(iyaw);
    double fy = iy + d[i]*cos(iyaw);
    x[i] = (fx);
    y[i] = (fy);
  }
  cerr<<"\n\nHOlaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa\n\n";
  
  yaw.resize(n-1);
  ds.resize(n-1);
  
  for(unsigned int i = 0; i < n - 1; i++)
  {
    double dx = x[i + 1] - x[i];
    double dy = y[i + 1] - y[i];
    if(abs(dx)>0.0001){
        yaw[i] = (atan2(dy, dx));
    }
    else{
      yaw[i]=0;
    }
    
    ds[i] = (sqrt(dx*dx + dy*dy));
  }
  // TO remove paths whose predicted s goes out of bounds of global path.
  if(s.size() == x.size())
  {
    //if(STOP_CAR)
   cerr<<"hello"<<endl;
    return 0;
  }
  c.resize((n-1) - 1);
  for(unsigned int i = 0; i < (n-1) - 1; i++){
    if(ds[i]!=0)
     c[i]=((yaw[i + 1] - yaw[i]) / ds[i]);
    else
    {
      c[i]=0;
    }
    
  }

  cerr<<endl<<" s size="<<s.size()<<endl;
      for(auto i : s)
        cerr<<i<<"  ";
      cerr<<endl<<" x size="<<x.size()<<endl;
      for(auto i : x)
        cerr<<i<<"  ";
      cerr<<endl<<" y"<<endl;
      for(auto i : y)
        cerr<<i<<"  ";
      cerr<<endl<<" yaw"<<endl;
      for(auto i : yaw)
        cerr<<i<<"  ";
      cerr<<endl<<" ds"<<endl;
      for(auto i : ds)
        cerr<<i<<"  ";
      cerr<<endl<<" c"<<endl;
      for(auto i : c)
        cerr<<i<<"  ";*/

  
  return 0;
}