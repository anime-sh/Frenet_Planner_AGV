#include "../include/frenet_optimal_trajectory.hpp"
#include <ros/console.h>
#include <bits/stdc++.h>
#include <algorithm>
#include <vector>

namespace plt = matplotlibcpp;

// convert the frenet path to global frame
FrenetPath calc_global_path(FrenetPath fp, Spline2D csp)
{
	fp.adding_global_path(csp);
	return fp;
}

void FrenetPath::adding_global_path(Spline2D csp)
{
	int n = s.size();
	x.resize(n);
	y.resize(n);
	for (int i = 0; i < n; i++)
	{
		double ix, iy;
		csp.calc_position(ix, iy, s[i]);
		if (ix == NONE)
		{
			return;
		}
		double iyaw = csp.calc_yaw(s[i]);
		double fx = ix - d[i] * sin(iyaw);
		double fy = iy + d[i] * cos(iyaw);
		x[i] = (fx);
		y[i] = (fy);
	}
	yaw.resize(n - 1);
	ds.resize(n - 1);

	for (int i = 0; i < n - 1; i++)
	{
		double dx = x[i + 1] - x[i];
		double dy = y[i + 1] - y[i];
		if (abs(dx) > 0.0001)
		{
			yaw[i] = (atan2(dy, dx));
		}
		else
		{
			yaw[i] = 0;
		}

		ds[i] = (sqrt(dx * dx + dy * dy));
	}
	// To remove paths whose predicted s goes out of bounds of global path.
	if (s.size() == x.size())
	{
		return;
	}
	c.resize((n - 1) - 1);
	for (int i = 0; i < (n - 1) - 1; i++)
	{
		if (ds[i] != 0)
			c[i] = ((yaw[i + 1] - yaw[i]) / ds[i]);
		else
		{
			c[i] = FLT_MAX;
		}
	}
}

// check for specified velocity, acceleration, curvature constraints and collisions
bool check_path(FrenetPath fp, double bot_yaw, double yaw_error,
				double obst_r)
{
	int flag = 0;
	vecD path_yaw = fp.get_yaw();
	if (path_yaw.size() == 0)
		return 0;
	if ((path_yaw[0] - bot_yaw) > yaw_error || (path_yaw[0] - bot_yaw) < -yaw_error) // 20 deg
	{

		flag = 1;
	}
	if (flag == 1)
	{
		return 0;
	}
	else if (fp.check_collision(obst_r) == 0)
	{
		return 1;
	}
	return 0;
}

// transforms robot's footprint
vector<geometry_msgs::Point32> transformation(vector<geometry_msgs::Point32> fp,
											  geometry_msgs::Pose cp, double px, double py, double pyaw)
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
	int n = new_fp.size();
	for (int i = 0; i < n; i++)
	{
		new_fp[i].x = (fp[i].x - bx) * cos(theta) + (fp[i].y - by) * sin(theta) + x + bx;
		new_fp[i].y = -(fp[i].x - bx) * sin(theta) + (fp[i].y - by) * cos(theta) + y + by;
	}
	return new_fp;
}

// returns distance between two points
#pragma omp declare simd
double dist(double x1, double y1, double x2, double y2)
{
	return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

//check collision with the two bounding nearest obstacle on x and y axes 
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
		xlower = xupper = it - ob_x.begin(); // no smaller value than val in vector
	}
	else if (it == ob_x.end())
	{
		xupper = xlower = (it - 1) - ob_x.begin(); // no bigger value than val in vector
	}
	else
	{
		xlower = (it - 1) - ob_x.begin();
		xupper = it - ob_x.begin();
	}
	double dist1 = dist(p.x, p.y, ob_x[xlower], ob_y[xlower]);
	double dist2 = dist(p.x, p.y, ob_x[xupper], ob_y[xupper]);
	if (min(dist1, dist2) < obst_r)
	{
		return 1;
	}
	it = lower_bound(ob_y.begin(), ob_y.end(), p.y);
	if (it == ob_y.begin())
	{
		ylower = yupper = it - ob_y.begin(); // no smaller value  than val in vector
	}
	else if (it == ob_y.end())
	{
		yupper = ylower = (it - 1) - ob_y.begin(); // no bigger value than val in vector
	}
	else
	{
		ylower = (it - 1) - ob_y.begin();
		yupper = it - ob_y.begin();
	}
	dist1 = dist(p.x, p.y, ob_x[ylower], ob_y[ylower]);
	dist2 = dist(p.x, p.y, ob_x[yupper], ob_y[yupper]);
	if (min(dist1, dist2) < obst_r)
	{
		return 1;
	}
	return 0;
}

// checks for collision of the bot
bool FrenetPath::check_collision(double obst_r)
{
	if (s.size() != x.size())
	{
		return 1;
	}
	for (unsigned int i = 0; i < min(x.size(), yaw.size()); i++)
	{
		vector<geometry_msgs::Point32> trans_footprint = transformation(footprint.polygon.points,
																		odom.pose.pose, x[i], y[i], yaw[i]);
		for (unsigned int j = 0; j < trans_footprint.size(); j++)
		{
			if (point_obcheck(trans_footprint[j], obst_r) == 1)
			{
				return 1;
			}
		}
	}
	return 0;
}

//comparator function for sorting FrenetPaths on cost
inline bool sortByCost(FrenetPath &a, FrenetPath &b)
{
	if (a.get_cf() != b.get_cf())
	{
		return a.get_cf() < b.get_cf();
	}
	else
	{
		double jerkCost1, jerkCost2;
		jerkCost1 = KLAT * a.get_Jp() + KLON * a.get_Js();
		jerkCost2 = KLAT * b.get_Jp() + KLON * b.get_Js();
		return jerkCost1 < jerkCost2;
	}
}

//helper function to plot path
void FrenetPath::plot_path()
{
	plt::plot(x, y);
	plt::pause(0.001);
}

//helper function to plot velocity profile
void FrenetPath::plot_velocity_profile()
{
	plt::plot(t, s_d);
	plt::pause(0.001);
}

static int flag_for_display_paths = 0;

//helper function to display paths
void display_paths(vector<FrenetPath> fplist)
{
	plt::ion();
	plt::show();
	int count = 0;
	for (auto &fp : fplist)
	{
		if (count % 50 == 0 && flag_for_display_paths)
		{
			fp.plot_path();
		}
		count++;
	}
	flag_for_display_paths = 1;
}

// samples paths and returns the bestpath
FrenetPath frenet_optimal_planning(Spline2D csp, double s0, double c_speed, double c_d,
								   double c_d_d, double c_d_dd, FrenetPath lp, double bot_yaw)
{
	//static bestpath to ensure the vehicle keeps moving on previous path if no new path is found except when STOP_CAR
	static FrenetPath bestpath;
	if (STOP_CAR)
	{
		FrenetPath empty;
		bestpath = empty;
	}

	//initialise object of class Fplist
	//object member fplist_lat stores sampled frenet paths
	vector<FrenetPath> fplist = Fplist(c_speed, c_d, c_d_d, c_d_dd, s0).fplist_lat;

	//sort sampled paths and check for collision in order of path cost
	std::sort(fplist.begin(), fplist.end(), sortByCost);
	for (int i = 0; i < fplist.size(); i++)
	{
		//path converted to global frame first
		fplist[i] = calc_global_path(fplist[i], csp);
		if (check_path(fplist[i], bot_yaw, 0.523599, 2.0) == 1)
		{
			bestpath = fplist[i];
			break;
		}
	}

	return bestpath;
}

//class constructor which generates paths
Fplist::Fplist(double c_speedc, double c_dc, double c_d_dc, double c_d_ddc, double s00)
{
	c_speed = c_speedc;
	c_d = c_dc;
	c_d_d = c_d_dc;
	c_d_dd = c_d_ddc;
	s0 = s00;

	//latitudinal part of the FrenetPath object is sampled and stored in fplist_lat
	if (STOP_CAR)
	{

		for (double di = -MAX_ROAD_WIDTH; di < MAX_ROAD_WIDTH; di += D_ROAD_W)
		{
			for (double Ti = MINT; Ti < MAXT; Ti += DT)
			{
				FrenetPath temp = calc_lat(di, Ti, 0.0);
				for (int p = 0; p < samples_tv; p++)
				{
					fplist_lat.push_back(temp);
				}
			}
		}
	}
	else
	{

		for (double di = -MAX_ROAD_WIDTH; di < MAX_ROAD_WIDTH; di += D_ROAD_W)
		{
			for (double Ti = MINT; Ti < MAXT; Ti += DT)
			{
				for (double Di_d = -MAX_LAT_VEL; Di_d < MAX_LAT_VEL + 0.001; Di_d += D_D_NS)
				{
					FrenetPath temp = calc_lat(di, Ti, Di_d);
					for (int p = 0; p < samples_tv; p++)
					{
						fplist_lat.push_back(temp);
					}
				}
			}
		}
	}

	//longitudinal part of the FrenetPath object is sampled and stored in fplist_lon
	if (STOP_CAR)
	{

		for (double Ti = MINT; Ti < MAXT; Ti += DT)
		{
			fplist_lon.push_back(calc_lon(TARGET_SPEED, Ti));
		}
	}
	else
	{

		for (double Ti = MINT; Ti < MAXT; Ti += DT)
		{
			for (double tv = TARGET_SPEED - D_T_S * N_S_SAMPLE; tv < TARGET_SPEED + D_T_S * N_S_SAMPLE; tv += D_T_S)
			{
				fplist_lon.push_back(calc_lon(tv, Ti));
			}
		}
	}

	//copy longitudinal part of the FrenetPath object to corresponding paths in fplist_lat
	for (int i = 0; i < fplist_lat.size(); i += samples_tv)
	{
		copy(i);
	}

	//cost calculated as a weighted sum of longitudinal and latitudinal path costs
	for (int i = 0; i < fplist_lat.size(); i++)
	{
		calc_cost(i);
	}
}

// calculates latitudinal part of FrenetPath object using the sampling parameters passed
FrenetPath Fplist::calc_lat(double di, double Ti, double Di_d)
{
	FrenetPath fp;

	int n = 1 + Ti / DT;
	fp.t.resize(n);
	fp.d.resize(n);
	fp.d_d.resize(n);
	fp.d_dd.resize(n);
	fp.d_ddd.resize(n);

	quintic lat_qp(c_d, c_d_d, c_d_dd, di, Di_d, 0.0, Ti);

	for (int te = 0; te < n; te++)
	{
		fp.t[te] = te * DT;
		fp.d[te] = lat_qp.calc_point(te * DT);
		fp.d_d[te] = lat_qp.calc_first_derivative(te * DT);
		fp.d_dd[te] = lat_qp.calc_second_derivative(te * DT);
		fp.d_ddd[te] = lat_qp.calc_third_derivative(te * DT);
	}

	vecD d_ddd_vec = fp.d_ddd;
	fp.Jp = (inner_product(d_ddd_vec.begin(), d_ddd_vec.end(), d_ddd_vec.begin(), 0));
	fp.Ti = (Ti);
	fp.cd = (KJ * fp.Jp + KT * Ti + KD * std::pow((fp.d).back(), 2));
	return fp;
}

// calculates longitudinal part of FrenetPath object using the sampling parameters passed
FrenetPath Fplist::calc_lon(double tv, double Ti)
{
	FrenetPath fp;
	quintic lon_qp(s0, c_speed, 0.0, min(s0 + 15, s_dest), tv, 0.0, Ti);

	int n = 1 + Ti / DT;
	fp.t.resize(n);
	fp.s.resize(n);
	fp.s_d.resize(n);
	fp.s_dd.resize(n);
	fp.s_ddd.resize(n);

	for (int te = 0; te < n; te++)
	{
		fp.t[te] = te * DT;
		fp.s[te] = lon_qp.calc_point(te * DT);
		fp.s_d[te] = lon_qp.calc_first_derivative(te * DT);
		fp.s_dd[te] = lon_qp.calc_second_derivative(te * DT);
		fp.s_ddd[te] = lon_qp.calc_third_derivative(te * DT);
	}
	fp.Ti = (Ti);
	vecD s_ddd_vec = fp.s_ddd;
	fp.Js = (inner_product(s_ddd_vec.begin(), s_ddd_vec.end(), s_ddd_vec.begin(), 0));
	fp.dss = std::pow((TARGET_SPEED - fp.s_d[-1]), 2);
	fp.cv = (KJ * fp.Js + KT * fp.Ti + KD * fp.dss);
	return fp;
}

//crossproduct of sampled latitudinal and longitudinal parts
void Fplist::copy(int i)
{
	int index_start = ((fplist_lat[i]).Ti - MINT) * samples_tv;

	for (int j = 0; j < samples_tv; j++)
	{
		fplist_lat[i + j].s = (fplist_lon[index_start + j].s);
		fplist_lat[i + j].s_d = (fplist_lon[index_start + j].s_d);
		fplist_lat[i + j].s_dd = (fplist_lon[index_start + j].s_dd);
		fplist_lat[i + j].s_ddd = (fplist_lon[index_start + j].s_ddd);
		fplist_lat[i + j].Js = (fplist_lon[index_start + j].Js);
		fplist_lat[i + j].dss = (fplist_lon[index_start + j].dss);
		fplist_lat[i + j].cv = (fplist_lon[index_start + j].cv);
	}
}

//calculate final cost of path as weighted sum of latitudinal and longitudinal path costs
void Fplist::calc_cost(int i)
{
	fplist_lat[i].cf = (KLAT * fplist_lat[i].cd + KLON * fplist_lat[i].cv);
}
