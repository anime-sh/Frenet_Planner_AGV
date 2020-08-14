#include "../include/cubic_spline_planner.hpp"
#include <vector>
int main()
{

  vecD dx,ds;
  int size;
  cin>>size;
  dx.resize(size);
  unsigned int sizedx=dx.size();
  // ds.resize(sizedx);
  for(int i=0;i<sizedx;i++)
  {
    dx[i]=i;
  }
  double temp;
  for(unsigned int i = 0; i < dx.size(); i++)
  {
    
    temp = sqrt(dx[i]*dx[i] + dx[i]*dx[i]);
    ds[i]=temp;
  }

  // vecD x;
  // x.resize(sizedx-1);
  // //unsigned int sizex=dx.size();
  // for(unsigned int i = 1; i < sizedx; i++)
  // {
  //   x[i-1]=(dx[i] - dx[i - 1]);
  // }

  return 0;
}
