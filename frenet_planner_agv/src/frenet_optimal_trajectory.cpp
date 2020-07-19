#include "../include/frenet_optimal_trajectory.hpp"
#include <ros/console.h>
#include <bits/stdc++.h> 
namespace plt = matplotlibcpp;


// vecD FrenetPath::get_d_ddd()
// {
// 	return d_ddd;
// }
// double FrenetPath::get_cf()
// {
// 	return cf;
// }
// vecD FrenetPath::get_d()
// {
// 	return d;
// }
// vecD FrenetPath::get_yaw()
// {
// 	return yaw;
// }

//calculates lateral paths using the sampling parameters passed
void FrenetPath::calc_lat_paths(double c_d, double c_d_d, double c_d_dd, double Ti, double di, double di_d)
{
	// trace(c_d,c_d_d,c_d_dd,Ti,di,di_d);
	quintic lat_qp(c_d, c_d_d, c_d_dd, di, di_d, 0.0, Ti);
	for(double te = 0.0; te <= Ti + DT; te += DT)
	{
		t.emplace_back(te);
		d.emplace_back(lat_qp.calc_point(te));
		d_d.emplace_back(lat_qp.calc_first_derivative(te));
		d_dd.emplace_back(lat_qp.calc_second_derivative(te));
		d_ddd.emplace_back(lat_qp.calc_third_derivative(te));
	}
	// trace(d);
}

//calculates longitudnal paths  using quartic polynomial  
// void FrenetPath::calc_lon_paths(double c_speed, double s0, double Ti, FrenetPath &fp, double tv)
void FrenetPath::calc_lon_paths(double c_speed, double s0, double Ti, double tv)
{

	// Trying to give an initial acceleration in the sampling
	/*if (c_speed< 0.01)
	{
		ROS_INFO("LOW SPEED");
		quartic lon_qp(s0, c_speed, 10.0, tv, 0.0, Ti);	//s_dd (longitudnal accln.)is set constant for low speed
		for(auto const& te : t) 
		{
			s.push_back(lon_qp.calc_point(te));
			s_d.push_back(lon_qp.calc_first_derivative(te));
			s_dd.push_back(lon_qp.calc_second_derivative(te));
			s_ddd.push_back(lon_qp.calc_third_derivative(te));
		}
	}
	else
	{
		quartic lon_qp(s0, c_speed, 0.0, tv, 0.0, Ti);	//s_dd is set to const. 0 (i.e. not being sampled) 
		for(auto const& te : t) 
		{
			s.push_back(lon_qp.calc_point(te));
			s_d.push_back(lon_qp.calc_first_derivative(te));
			s_dd.push_back(lon_qp.calc_second_derivative(te));
			s_ddd.push_back(lon_qp.calc_third_derivative(te));
		}
	}*/

	// Normal Implementation
	quartic lon_qp(s0, c_speed, 0.0, tv, 0.0, Ti);	//s_dd is set to const. 0 (i.e. not being sampled) 
	int size = t.size();
	s.resize(size);
	s_d.resize(size);
	s_dd.resize(size);
	s_ddd.resize(size);
	int i = 0;
	for(auto const te : t) 
	{
		s[i] = (lon_qp.calc_point(te));
		s_d[i] = (lon_qp.calc_first_derivative(te));
		s_dd[i] = (lon_qp.calc_second_derivative(te));
		s_ddd[i] = (lon_qp.calc_third_derivative(te));
		++i;
	}

	//https://www.geeksforgeeks.org/std-inner_product-in-cpp/
	Js = inner_product(s_ddd.begin(), s_ddd.end(), s_ddd.begin(), 0);
	double ds = pow((TARGET_SPEED - s_d.back()), 2);

	cd = (KJ*Jp + KT*Ti + KD*d.back()*d.back());
	cv = (KJ*Js + KT*Ti + KD_V*ds);
	cf = (KLAT*cd + KLON*cv);

}

//calculates longitudnal paths  using quintic polynomial  
// void FrenetPath::calc_lon_paths_quintic_poly(double c_speed, double s0, double Ti, FrenetPath &fp, double ts, double tv)
void FrenetPath::calc_lon_paths_quintic_poly(double c_speed, double s0, double Ti, double ts, double tv)
{
	quintic lon_qp(s0, c_speed, 0.0, s0 + ts, tv, 0.0, Ti);	// s_dd is not being sampled
	int size = t.size();
	s.resize(size);
	s_d.resize(size);
	s_dd.resize(size);
	s_ddd.resize(size);
	int i = 0;
	for(auto te : t) 
	{
		s[i] = (lon_qp.calc_point(te));
		s_d[i] = (lon_qp.calc_first_derivative(te));
		s_dd[i] = (lon_qp.calc_second_derivative(te));
		s_ddd[i] = (lon_qp.calc_third_derivative(te));
		++i;
	}
	//https://www.geeksforgeeks.org/std-inner_product-in-cpp/
	Js = inner_product(s_ddd.begin(), s_ddd.end(), s_ddd.begin(), 0);
	double ds = pow((TARGET_SPEED - s_d.back()), 2);

	//calculation of lateral, longitudnal and overall cost of the trajectories
	cd = (KJ*Jp + KT*Ti + KD*d.back()*d.back());
	cv = (KJ*Js + KT*Ti + KD_V*ds);
	cf = (KLAT*cd + KLON*cv);
}

//get sampling limits of d using the previously calculated paths (if present)
void get_limits_d(FrenetPath lp, double *lower_limit_d, double *upper_limit_d)  
{
	vecD d_sampling = lp.get_d();
	if(d_sampling.size() != 0)
	{
		*lower_limit_d = d_sampling.back() - MAX_SHIFT_D;
		*upper_limit_d = d_sampling.back() + MAX_SHIFT_D + D_ROAD_W;
	}

}

// generates frenet path parameters
vector<FrenetPath> calc_frenet_paths(double c_speed, double c_d, double c_d_d, double c_d_dd, double s0, FrenetPath lp)
{
	// trace(c_d_d);
	vector<FrenetPath> frenet_paths;
	double lower_limit_d, upper_limit_d;
	lower_limit_d = -MAX_ROAD_WIDTH;
	upper_limit_d = MAX_ROAD_WIDTH + D_ROAD_W;
	get_limits_d(lp, &lower_limit_d, &upper_limit_d);//IF not required to sample around previous sampled d(th) then comment this line.
	for(double di = lower_limit_d; di <= upper_limit_d; di += D_ROAD_W)  // sampling for lateral offset
	{
		for(double Ti = MINT; Ti <= MAXT + DT; Ti += DT)   //Sampling for prediction time
		{
			for(double di_d = -MAX_LAT_VEL; di_d <= MAX_LAT_VEL + D_D_NS; di_d+=D_D_NS)//Sampling for lateral velocity
			{
				FrenetPath fp;
				FrenetPath tfp;
				fp.calc_lat_paths(c_d, c_d_d, c_d_dd, Ti, di, di_d);
				vecD d_ddd_vec = fp.get_d_ddd();
				fp.set_Jp( inner_product(d_ddd_vec.begin(), d_ddd_vec.end(), d_ddd_vec.begin(), 0));  //
				if(STOP_CAR)
				{
					// double minV=0;
					// double maxV=0;
					tfp = fp;
						// tfp.calc_lon_paths(c_speed, s0, Ti, fp, tv, Jp);
						
						// Fixed S
						// tfp.calc_lon_paths_quintic_poly(c_speed, s0, Ti, fp, 15, tv); 
						tfp.calc_lon_paths_quintic_poly(c_speed, s0, Ti, 15, 0); 
						frenet_paths.push_back(tfp);	
				}
				else
				{
					double minV = TARGET_SPEED - D_T_S*N_S_SAMPLE;
					double maxV = TARGET_SPEED + D_T_S*N_S_SAMPLE;
					for(double tv = minV; tv <= maxV + D_T_S; tv += D_T_S)  //sampling for longitudnal velocity
					{
						tfp = fp;
						// tfp.calc_lon_paths(c_speed, s0, Ti, fp, tv, Jp);
						
						// Fixed S
						// tfp.calc_lon_paths_quintic_poly(c_speed, s0, Ti, fp, 15, tv); 
						tfp.calc_lon_paths_quintic_poly(c_speed, s0, Ti, 15, tv); 
						frenet_paths.push_back(tfp);		
						// cerr<<tfp<<endl;

					}	
				}
				
				
			}
		}
	}
	return frenet_paths;
}

void FrenetPath::adding_global_path(Spline2D csp)
{
	for(unsigned int i = 0; i < s.size(); i++)
	{
		double ix, iy;
		// trace(i);
		csp.calc_position(ix, iy, s[i]);
		// trace("done calc_position");
		if(ix == NONE)
		{
			break;
		}
		double iyaw = csp.calc_yaw(s[i]);
		double di = d[i];
		double fx = ix - di*sin(iyaw);
		double fy = iy + di*cos(iyaw);
		x.push_back(fx);
		y.push_back(fy);
	}
	for(unsigned int i = 0; i < x.size() - 1; i++)
	{
		double dx = x[i + 1] - x[i];
		double dy = y[i + 1] - y[i];
		yaw.push_back(atan2(dy, dx));
		ds.push_back(sqrt(dx*dx + dy*dy));
	}
	if(s.size() - x.size() != 0)//TO remove paths whose predicted s goes out of bounds of global path.
	{
		return;
	}
	for(unsigned int i = 0; i < yaw.size() - 1; i++){
		c.push_back((yaw[i + 1] - yaw[i]) / ds[i]);
	}
}

// convert the frenet paths to global frame 
vector<FrenetPath> calc_global_paths(vector<FrenetPath> fplist, Spline2D csp)
{
	for(auto& fp : fplist)
	{
		// cerr<<fp<<endl;
		// trace("adding global path");
		fp.adding_global_path(csp);	
	}
	return fplist;
}	

//transforms robot's footprint 
vector<geometry_msgs::Point32> transformation(vector<geometry_msgs::Point32> fp, geometry_msgs::Pose cp, double px, double py, double pyaw)
{
	vector<geometry_msgs::Point32> new_fp(fp.size());
	tf::Quaternion qb(cp.orientation.x, cp.orientation.y, cp.orientation.z, cp.orientation.w);
	tf::Matrix3x3 mb(qb);

	double broll, bpitch, byaw;
	mb.getRPY(broll, bpitch, byaw);

	double bx, by;
	bx = cp.position.x;
	by = cp.position.y;

	double x, y, theta;
	theta = pyaw - byaw;
	x = px - bx;
	y = py - by;
	
	for(unsigned int i = 0; i < new_fp.size(); i++)
	{
		new_fp[i].x = (fp[i].x - bx)* cos(theta) + (fp[i].y - by) * sin(theta) + x + bx;
		new_fp[i].y = -(fp[i].x - bx) * sin(theta) + (fp[i].y - by) * cos(theta) + y + by;
	}
	return new_fp;
}

//returns distance between two points 
double dist(double x1, double y1, double x2, double y2)
{
	return sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2));
}


bool point_obcheck(geometry_msgs::Point32 p, double obst_r)
{
	int xlower, ylower, xupper, yupper;
	auto it = lower_bound(ob_x.begin(), ob_x.end(), p.x);
	if (ob_x.size() == 0)
	{
		return 0;
	}
	if (it == ob_x.begin())
	{ 
		xlower = xupper = it - ob_x.begin(); // no smaller value  than val in vector
	}
	else if (it == ob_x.end()) 
	{
		xupper = xlower = (it-1)- ob_x.begin(); // no bigger value than val in vector
	}
	else
	{
    	xlower = (it-1) - ob_x.begin();
    	xupper = it - ob_x.begin();
	}
	double dist1 = dist(p.x,p.y, ob_x[xlower], ob_y[xlower]);
	double dist2 = dist(p.x, p.y, ob_x[xupper], ob_y[xupper]);
	if(min(dist1, dist2) < obst_r)
	{
		return 1;
	}
	it = lower_bound(ob_y.begin(), ob_y.end(), p.y);
	if (it == ob_y.begin()) 
	{
		ylower = yupper = it - ob_y.begin(); // no smaller value  than val in vector
	}
	else if (it == ob_y.end()) {
		yupper = ylower = (it-1)- ob_y.begin(); // no bigger value than val in vector
	}
	else{
    	ylower = (it-1) - ob_y.begin();
    	yupper = it - ob_y.begin();
	}
	dist1 = dist(p.x,p.y, ob_x[ylower], ob_y[ylower]);
	dist2 = dist(p.x, p.y, ob_x[yupper], ob_y[yupper]);
	if(min(dist1, dist2) < obst_r){
		return 1;		
	}
	return 0;	 
}
 
//checks for collision of the bot
bool FrenetPath::check_collision(double obst_r)
{
	// trace("in");
	if(s.size() != x.size())
	{
		// trace("307");
		return 1;
	}
	for(unsigned int i = 0; i < min(x.size(),yaw.size()); i++)
	{
		// trace("poss");
		vector<geometry_msgs::Point32> trans_footprint = transformation(footprint.polygon.points, odom.pose.pose, x[i], y[i], yaw[i]);
		// trace("tno");
		for(unsigned int j = 0; j < trans_footprint.size(); j++)
		{
			// trace(i,j);
			if(point_obcheck(trans_footprint[j],obst_r) ==1 )
			{
				// trace("out1");
				return 1;
			}
		}
	}
	// trace("out2");
	return 0;
}

inline bool sortByCost(FrenetPath a, FrenetPath b){

	if(a.get_cf() != b.get_cf()){
		return a.get_cf() < b.get_cf();
	}else{
		double jerkCost1,jerkCost2;
		jerkCost1 = KLAT * a.get_Jp() + KLON * a.get_Js();
		jerkCost2 = KLAT * b.get_Jp() + KLON * b.get_Js();
		return jerkCost1 < jerkCost2;
	}
	
}

// check for specified velocity, acceleration, curvature constraints and collisions 
FrenetPath check_path(vector<FrenetPath>& fplist,Spline2D csp, double bot_yaw, double yaw_error, double obst_r)
{
	vector<FrenetPath> fplist_final;
	FrenetPath fp;
	// double leastCost;
	// bool first =true;
	//int size=fplist.size();
	sort(fplist.begin(),fplist.end(),sortByCost);
	for(unsigned int i = 0; i < fplist.size(); i++)
	{
		// trace(i);
		fp = fplist[i];
		fp.adding_global_path(csp);
		int flag = 0;
		vecD path_yaw = fp.get_yaw();
		if (path_yaw.size()==0)
			continue;
		// trace(path_yaw);
		if ((path_yaw[0] - bot_yaw)> yaw_error || (path_yaw[0] - bot_yaw)< -yaw_error) //20 deg
		{
			flag=1;
		}
		
		if(flag == 1){continue;}
		else if(fp.check_collision(obst_r)==0)
		{
			//fplist_final.push_back(fplist[i]);
			return fp;
		}			
	}
	//return fplist_final;
}


void FrenetPath::plot_path()
{
	plt::plot(x,y);
	plt::pause(0.001);
}
void FrenetPath::plot_velocity_profile()
{
	plt::plot(t, s_d);
	plt::pause(0.001);
}
static int flag_for_display_paths = 0;
void display_paths(vector<FrenetPath> fplist)
{
	plt::ion();
	plt::show();
	int count=0;
	for(auto &fp : fplist)
	{
		if(count%50 == 0 && flag_for_display_paths){
			fp.plot_path();
		}	
		count++;
	}
	flag_for_display_paths = 1;
}

// generates the path and returns the bestpath
FrenetPath frenet_optimal_planning(Spline2D csp, double s0, double c_speed, double c_d, double c_d_d, double c_d_dd, FrenetPath lp, double bot_yaw)
{
	trace("start");
	vector<FrenetPath> fplist = calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0, lp);
	trace("calc_global_paths");
	//fplist = calc_global_paths(fplist, csp);
	trace("check_path");
	FrenetPath bestpath = check_path(fplist,csp, bot_yaw, 0.523599, 2.0); // for now maximum possilble paths are taken into list
	trace("done checking ");
	// For displaying all paths
	if(false)
	{
		display_paths(fplist);
	}

	// double min_cost = FLT_MAX;
	// double cf;
	// for(auto & fp : fplist)
	// {
	// 	cf = fp.get_cf();
	// 	if(min_cost >= cf)
	// 	{
	// 		min_cost = cf;
	// 		bestpath = fp;
	// 	}
	// }
	// For showing the bestpath
	if(true)
	{
		plt::ion();
		plt::show();
		bestpath.plot_path();
	}
	// For plotting velocity profile (x,y) = (t,s_d)
	if(false)
	{
		// trace("436");
		plt::ion();
		// trace("438");
		plt::show();
		// trace("440");
		bestpath.plot_velocity_profile();
		// trace("442");
	}
	trace("DONE");
	return bestpath;
}