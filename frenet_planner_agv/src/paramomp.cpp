// generating a random sequence of distinct elements
#include <bits/stdc++.h>
// #include <mkl.h>
#include <omp.h>
using namespace std;

int rand(int a, int b) {
    return a + rand() % (b - a + 1);
}

int main(int argc, char* argv[]) {

  for( int i=0;i<10;i++)
  {

      vector<double> a, b;
      /*
  clock_t start1, end1; 

  // Recording the end clock tick. 
    start1= clock(); 
  
    // Calculating total time taken by the program. 
    


  for(double di = 0; di <= 10; di += 1)  // sampling for lateral
                                                                       // offset
  {
    for(double Ti = 2; Ti <= 5 + 0.2; Ti += 0.2)  // Sampling for prediction time
    {
      for(double di_d = -1.0; di_d <= 1.0 + 0.5; di_d+=0.5)
      {
        // FrenetPath fp;
        // FrenetPath tfp;
        // fp.calc_lat_paths(c_d, c_d_d, c_d_dd, Ti, di, di_d);
        // vecD d_ddd_vec = fp.get_d_ddd();
        // fp.set_Jp( inner_product(d_ddd_vec.begin(), d_ddd_vec.end(), d_ddd_vec.begin(), 0));
        double minV = 8.33 - 1.389 * 1.5;
        double maxV = 8.33 + 1.389 * 1.5;

        // sampling for longitudnal velocity
        for(double tv = minV; tv <= maxV + 1.389; tv += 1.389)
        {
          double hh = sqrt(di*Ti*di_d*tv);
          a.push_back(hh);
        }
      }
    }
  }

  double sum =0;

  for(int i=0;i<a.size();i++){
    sum+=a[i];
  }

  end1 = clock(); 


cout<<"sum 1 ="<<sum<<endl;
double time_taken1 = double(end1 - start1) / double(CLOCKS_PER_SEC); 
    cout << "Time taken 1 by program is : " << fixed  
         << time_taken1 << setprecision(5); 
    cout << " sec " << endl; 
*/
clock_t start2, end2; 
  
    // Recording the starting clock tick.

      
   omp_set_dynamic(1);
      int count=0;
 omp_set_num_threads(10);
    start2 = clock(); 
  
    

 //omp_set_nested(1);
    


  #pragma omp parallel
  
  {

    #pragma omp for collapse(4) schedule(static,100)
    for(int di = 0; di <= 100; di += 1)  // sampling for lateral
                                                                       // offset
  {
     //#pragma omp parallel for
    // {

        
      for(int Ti = 2; Ti <= 50; Ti += 1)  // Sampling for prediction time
      {
        //#pragma omp parallel for
        for(int di_d = -10; di_d <= 10; di_d+=1)
        {
          // FrenetPath fp;
          // FrenetPath tfp;
          // fp.calc_lat_paths(c_d, c_d_d, c_d_dd, Ti, di, di_d);
          // vecD d_ddd_vec = fp.get_d_ddd();
          // fp.set_Jp( inner_product(d_ddd_vec.begin(), d_ddd_vec.end(), d_ddd_vec.begin(), 0));
          // double minV = 8.33 - 1.389 * 1.5;
          // double maxV = 8.33 + 1.389 * 1.5;

          // sampling for longitudnal velocity
          //#pragma omp parallel for

          //#pragma omp parallel for
          for(int tv = 5; tv <= 20; tv += 1)
          {
            //count++;
            double hh = sqrt(di*Ti*tv*di_d*1034);
             //#pragma omp single
        
      //         printf("Num threads in dynamic region is = %d\n", 
      // omp_get_thread_num());
            
            //cout<<10<<endl;

            //#pragma omp critical
            //b.push_back(hh);
          }
        }
     //}
      
    }

  }

  }

      
      // #pragma omp for nowait
      // for(double Ti = 2; Ti <= 5 + 0.2; Ti += 0.2)  // Sampling for prediction time
      // {
      //   //#pragma omp parallel for
      //   for(double di_d = -1.0; di_d <= 1.0 + 0.5; di_d+=0.5)
      //   {
      //     // FrenetPath fp;
      //     // FrenetPath tfp;
      //     // fp.calc_lat_paths(c_d, c_d_d, c_d_dd, Ti, di, di_d);
      //     // vecD d_ddd_vec = fp.get_d_ddd();
      //     // fp.set_Jp( inner_product(d_ddd_vec.begin(), d_ddd_vec.end(), d_ddd_vec.begin(), 0));
      //     double minV = 8.33 - 1.389 * 1.5;
      //     double maxV = 8.33 + 1.389 * 1.5;

      //     // sampling for longitudnal velocity
      //     //#pragma omp parallel for
      //     for(double tv = minV; tv <= maxV + 1.389; tv += 1.389)
      //     {
      //       count++;
      //       double hh = sqrt(10);
      //       //cout<<10<<endl;

      //       //#pragma omp critical
      //       //b.push_back(hh);
      //     }
      //   }
      // }
  

              // double sum1 =0;

              // for(int i=0;i<b.size();i++){
              //   sum1+=b[i];
              // }

            // Recording the end clock tick. 
              end2 = clock(); 

              // Calculating total time taken by the program. 
              double time_taken2 = double(end2 - start2) / double(CLOCKS_PER_SEC); 

            //cout<<"sum 2 ="<<sum1<<endl;
              cout << "count = "<<count<<endl<<"Time taken 2 by program is : " << fixed  
                    << time_taken2 << setprecision(5); 
              cout << " sec " << endl << endl << endl; 






            clock_t start1, end1; 

            // Recording the end clock tick. 
              start1= clock(); 

              // Calculating total time taken by the program. 
              


              //#pragma omp parallel for

      for(int di = 0; di <= 100; di += 1)  // sampling for lateral
                                                                          // offset
      {
        //#pragma omp parallel for
        // {

            
          for(int Ti = 2; Ti <= 50; Ti += 1)  // Sampling for prediction time
          {
            //#pragma omp parallel for
            for(int di_d = -10; di_d <= 10; di_d+=1)
            {
              // FrenetPath fp;
              // FrenetPath tfp;
              // fp.calc_lat_paths(c_d, c_d_d, c_d_dd, Ti, di, di_d);
              // vecD d_ddd_vec = fp.get_d_ddd();
              // fp.set_Jp( inner_product(d_ddd_vec.begin(), d_ddd_vec.end(), d_ddd_vec.begin(), 0));
              // double minV = 8.33 - 1.389 * 1.5;
              // double maxV = 8.33 + 1.389 * 1.5;

              // sampling for longitudnal velocity
              //#pragma omp parallel for

              //#pragma omp parallel for
              for(int tv = 5; tv <= 20; tv += 1)
              {
                //count++;
                double hh = sqrt(di*Ti*tv*di_d*1034);
                // #pragma omp single
            
                 // printf("Num threads in dynamic region is = %d\n", omp_get_num_threads());
                
                //cout<<10<<endl;

                //#pragma omp critical
                //b.push_back(hh);
              }
            }
        //}
          
        }
      }
            // double sum =0;

            // for(int i=0;i<a.size();i++){
            //   sum+=a[i];
            // }

            end1 = clock(); 


            //cout<<"sum 1 ="<<sum<<endl;
            double time_taken1 = double(end1 - start1) / double(CLOCKS_PER_SEC); 
              cout << "Time taken 1 by program is : " << fixed  
                    << time_taken1 << setprecision(5); 
              cout << " sec " << endl; 



  }
  


  }



