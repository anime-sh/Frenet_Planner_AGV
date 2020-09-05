#include "../include/frenet_optimal_trajectory.hpp"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <time.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <utility>
#include <ros/console.h>
#include <vector>
#include<fstream>
//using namespace tbb;
namespace plt = matplotlibcpp;

int cost_count = 0, footprint_count = 0, odom_count = 0;

// accesses the costmap and updates the obstacle coordinates
void costmap_callback(const nav_msgs::OccupancyGrid::ConstPtr &occupancy_grid)
{
	cost_count++;
	//double startTime = omp_get_wtime();
	unsigned int height, width;
	::cmap = *occupancy_grid;
	ob_x.clear();
	ob_y.clear();
	geometry_msgs::Pose origin = occupancy_grid->info.origin;

	double startTime1 = omp_get_wtime();
	vector<pair<double, double>> ob1;
#pragma omp parallel for collapse(2)
	for (width = 0; width < occupancy_grid->info.width; ++width)
	{
		for (height = 0; height < occupancy_grid->info.height; ++height)
		{
			if (occupancy_grid->data[height * occupancy_grid->info.width + width] > 0)
			{
#pragma omp critical
				{
					ob1.emplace_back(width * occupancy_grid->info.resolution + occupancy_grid->info.resolution / 2 + origin.position.x, height * occupancy_grid->info.resolution + occupancy_grid->info.resolution / 2 + origin.position.y);
					//     ob_x.emplace_back(width * occupancy_grid->info.resolution + occupancy_grid->info.
					// resolution / 2 + origin.position.x);
					// ob_y.emplace_back(height * occupancy_grid->info.resolution + occupancy_grid->info.resolution
					// / 2 + origin.position.y);
				}
			}
		}
	}

	sort(ob1.begin(), ob1.end());
	ob_x.resize(ob1.size());
	ob_y.resize(ob1.size());
	for (long i = 0; i < ob1.size(); i++)
	{
		ob_x[i] = ob1[i].first;
		ob_y[i] = ob1[i].second;
	}

	double endTime1 = omp_get_wtime();

	/*cerr<<"parallel X is"<<endl;
  for(auto i :ob_x){
	cerr<<i<<"  ";
  }

  cerr<<"paralle Y is"<<endl;
  for(auto i :ob_y){
	cerr<<i<<"  ";
  }*/

	//   ob_x.clear();
	//   ob_y.clear();
	//   double startTime2 = omp_get_wtime();
	//geometry_msgs::Pose origin = occupancy_grid->info.origin;
	// vector<pair<int,int>> ob;
	// #pragma omp parallel for collapse(2)
	//   for (width=0; width < occupancy_grid->info.width; ++width)
	//   {
	// 	for (height=0; height < occupancy_grid->info.height; ++height)
	// 	{
	// 	  if(occupancy_grid->data[height*occupancy_grid->info.width + width] > 0)
	// 	  {
	//       //#pragma omp critical
	//       {
	//     //     ob.emplace_back({width * occupancy_grid->info.resolution + occupancy_grid->info.
	// 		// resolution / 2 + origin.position.x,height * occupancy_grid->info.resolution + occupancy_grid->info.resolution
	// 		// / 2 + origin.position.y});
	//         ob_x.emplace_back(width * occupancy_grid->info.resolution + occupancy_grid->info.
	// 		resolution / 2 + origin.position.x);
	// 		ob_y.emplace_back(height * occupancy_grid->info.resolution + occupancy_grid->info.resolution
	// 		/ 2 + origin.position.y);

	//       }

	// 	  }
	// 	}
	//   }
	//   double endTime2 = omp_get_wtime();

	/*cerr<<"simple X is"<<endl;
  for(auto i :ob_x){
	cerr<<i<<"  ";
  }

  cerr<<"simple Y is"<<endl;
  for(auto i :ob_y){
	cerr<<i<<"  ";
  }*/
	//ob.resize(ob_x.)

	// cerr<<"costmap parallel time = "<< endTime1 -startTime1<<endl;
	// cerr<<"costmap simple time = "<< endTime2 -startTime2<<endl;
}

// accesses the robot footprint
void footprint_callback(const geometry_msgs::PolygonStampedConstPtr &p)
{
	::footprint = *p;
}

// accesses the odometry data
void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
	::odom = *msg;
}

// calculates the distance between two points (x1,y1) and (x2,y2)
inline double calc_dis(double x1, double y1, double x2, double y2)
{
	return sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2));
}

// finds the point in the global path which is nearest to the bot
void find_nearest_in_global_path(vecD &global_x, vecD &global_y, double &min_x, double &min_y,
								 double &min_dis, int &min_id, int flag, FrenetPath &path)
{
	double bot_x, bot_y;
	if (flag == 0)
	{
		bot_x = odom.pose.pose.position.x;
		bot_y = odom.pose.pose.position.y;
	}
	else
	{
		bot_x = path.get_x()[1];
		bot_y = path.get_y()[1];
	}
	min_dis = FLT_MAX;
	for (unsigned int i = 0; i < global_x.size(); i++)
	{
		double dis = calc_dis(global_x[i], global_y[i], bot_x, bot_y);
		if (dis < min_dis)
		{
			min_dis = dis;
			min_x = global_x[i];
			min_y = global_y[i];
			min_id = i;
		}
	}
}

inline double get_bot_yaw()
{
	geometry_msgs::Pose p = odom.pose.pose;
	tf::Quaternion q(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	trace(yaw, pitch, roll); // is -nan
	return yaw;
}

void initial_conditions_path(Spline2D &csp, double &s0, double &c_speed, double &c_d, double &c_d_d,
							 double &c_d_dd, double &bot_yaw, FrenetPath &path)
{
	// Approach 1
	vecD d = path.get_d();
	vecD s_d = path.get_s_d();
	vecD d_d = path.get_d_d();
	vecD d_dd = path.get_d_dd();
	vecD s = path.get_s();
	s0 = s[1];
	c_speed = s_d[1];
	c_d = d[1];
	c_d_d = d_d[1];
	c_d_dd = 0;
	bot_yaw = get_bot_yaw();
}

int initial_conditions_new(Spline2D &csp, vecD &global_s, vecD &global_x, vecD &global_y,
						   vecD &global_R, vecD &global_yaw, double &s0, double &c_speed, double &c_d, double &c_d_d,
						   double &c_d_dd, double &bot_yaw, FrenetPath &path)
{
	double vx = odom.twist.twist.linear.x;
	double vy = odom.twist.twist.linear.y;
	double v = sqrt(vx * vx + vy * vy);
	double min_x, min_y;
	int min_id;

	// getting d
	find_nearest_in_global_path(global_x, global_y, min_x, min_y, c_d, min_id, 0, path);

	// deciding the sign for d
	pair<double, double> vec1, vec2;
	vec1.first = odom.pose.pose.position.x - global_x[min_id];
	vec1.second = odom.pose.pose.position.y - global_y[min_id];
	vec2.first = global_x[min_id] - global_x[min_id + 1];
	vec2.second = global_y[min_id] - global_y[min_id + 1];
	double curl2D = vec1.first * vec2.second - vec2.first * vec1.second;
	if (curl2D < 0)
		c_d *= -1;
	s0 = global_s[min_id];
	bot_yaw = get_bot_yaw();
	double g_path_yaw = global_yaw[min_id];
	trace(bot_yaw, g_path_yaw);
	double delta_theta = bot_yaw - g_path_yaw;
	trace(delta_theta);
	c_d_d = v * sin(delta_theta); // Equation 5
	double k_r = global_R[min_id];
	c_speed = v * cos(delta_theta) / (1 - k_r * c_d); // s_dot (Equation 7)
	c_d_dd = 0;										  // For the time being. Need to be updated
	return min_id;
}

// publishes path as ros messages
void publishPath(nav_msgs::Path &path_msg, FrenetPath &path, vecD &rk, vecD &ryaw, double &c_speed,
				 double &c_d, double &c_d_d)
{
	geometry_msgs::PoseStamped loc;
	double delta_theta, yaw;
	vecD x_vec = path.get_x();
	vecD y_vec = path.get_y();
	for (unsigned int i = 0; i < path.get_x().size(); i++)
	{
		loc.pose.position.x = x_vec[i];
		loc.pose.position.y = y_vec[i];
		delta_theta = atan(c_d_d / ((1 - rk[i] * c_d) * c_speed));
		yaw = delta_theta + ryaw[i];
		tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, yaw); // roll , pitch = 0
		q.normalize();
		quaternionTFToMsg(q, loc.pose.orientation);
		path_msg.poses.push_back(loc);
	}
}

int main(int argc, char **argv)
{
	bool gotOdom = false;
	ros::init(argc, argv, "frenet_planner");
	ros::NodeHandle n;
	ros::Publisher frenet_path = n.advertise<nav_msgs::Path>("/frenet_path", 1);   // Publish frenet
																				   // path
	ros::Publisher global_path = n.advertise<nav_msgs::Path>("/global_path", 1);   // Publish global
																				   // path
	ros::Publisher target_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10); // Publish
																				   // velocity
	ros::Subscriber odom_sub = n.subscribe("/base_pose_ground_truth", 10, odom_callback);

	ros::Subscriber footprint_sub = n.subscribe<geometry_msgs::PolygonStamped>("/move_base/local_costmap/footprint", 10, footprint_callback);

	ros::Subscriber costmap_sub = n.subscribe<nav_msgs::OccupancyGrid>(
		"/move_base/local_costmap/costmap", 10000, costmap_callback);

	n.getParam("/frenet_planner/path/max_speed", MAX_SPEED);
	n.getParam("/frenet_planner/path/max_accel", MAX_ACCEL);
	n.getParam("/frenet_planner/path/max_curvature", MAX_CURVATURE);
	n.getParam("/frenet_planner/path/max_road_width", MAX_ROAD_WIDTH);
	n.getParam("/frenet_planner/path/d_road_w", D_ROAD_W);
	n.getParam("/frenet_planner/path/dt", DT);
	n.getParam("/frenet_planner/path/maxt", MAXT);
	n.getParam("/frenet_planner/path/mint", MINT);
	n.getParam("/frenet_planner/path/target_speed", TARGET_SPEED);
	n.getParam("/frenet_planner/path/d_t_s", D_T_S);
	n.getParam("/frenet_planner/path/n_s_sample", N_S_SAMPLE);
	n.getParam("/frenet_planner/path/robot_radius", ROBOT_RADIUS);
	n.getParam("/frenet_planner/path/max_lat_vel", MAX_LAT_VEL);
	n.getParam("/frenet_planner/path/min_lat_vel", MIN_LAT_VEL);
	n.getParam("/frenet_planner/path/d_d_ns", D_D_NS);
	n.getParam("/frenet_planner/path/max_shift_d", MAX_SHIFT_D);
	n.getParam("/frenet_planner/cost/kj", KJ);
	n.getParam("/frenet_planner/cost/kt", KT);
	n.getParam("/frenet_planner/cost/kd", KD);
	n.getParam("/frenet_planner/cost/kd_v", KD_V);
	n.getParam("/frenet_planner/cost/klon", KLON);
	n.getParam("/frenet_planner/cost/klat", KLAT);

	// Waypoint Params
	n.getParam("/frenet_planner/waypoints/W_X", W_X);
	n.getParam("/frenet_planner/waypoints/W_Y", W_Y);

	/*	vecD wx = {38, 38, 38, 38, 38, 38, 38, 38}; 	//38,-57 -> starting point of the bot
  vecD wy = {-57, -45,  -32.0,  -18.5,  -12.0, 0.0, 12, 35};*/
	vecD rx, ry, ryaw, rk;
	double ds = 0.1; // ds represents the step size for cubic_spline
	double bot_yaw, bot_v;

	// Global path is made using the waypoints
	Spline2D csp = calc_spline_course(W_X, W_Y, rx, ry, ryaw, rk, ds);

	FrenetPath path;
	FrenetPath lp;
	double s0, c_d, c_d_d, c_d_dd, c_speed;
	unsigned int ctr = 0, i;
	vector<double> global_s(rx.size());
	double s = 0;
	global_s[0] = 0;
	for (unsigned int i = 1; i < rx.size(); i++)
	{
		double dis = calc_dis(rx[i], ry[i], rx[i - 1], ry[i - 1]);
		s = s + dis;
		global_s[i] = s;
	}
	s_dest = global_s.back();
	bool run_frenet = true;
	plt::ion();
		plt::show();
	vector<double> plts0,pltcspeed,plttime;
	double pltbasetime = omp_get_wtime();
	while (ros::ok())
	{
		ofstream fout;
		fout.open("/home/animesh/try_ws/src/frenet_planner_agv/src/log.txt",ios::app);
		int min_id = 0;
		// Specifing initial conditions for the frenet planner using odometry
		double startTime1 = omp_get_wtime();
		if (true)
		{
			min_id = initial_conditions_new(csp, global_s, rx, ry, rk, ryaw, s0, c_speed, c_d, c_d_d,
											c_d_dd, bot_yaw, path);
		}
		// else
		// {
		// 	initial_conditions_path(csp, s0, c_speed, c_d, c_d_d, c_d_dd, bot_yaw, path);
		// }
		double endTime1 = omp_get_wtime();
		plts0.push_back(s0);
		pltcspeed.push_back(c_speed);
		plttime.push_back(abs(endTime1-pltbasetime));
		plt::plot(plttime,pltcspeed);
		plt::pause(0.001);
		fout<<"a run"<<endl;
		fout<<plts0<<endl;
		fout<<plttime<<endl;
		fout<<pltcspeed<<endl;
		fout<<"end"<<endl;
		trace("frenet optimal planning", s0, c_speed, c_d, c_d_d, c_d_dd, lp, bot_yaw);
		if (abs(203.5 - s0) <= 15) // convert 203.5 to a variable in terms of s_dest
		{
			STOP_CAR = true;
			TARGET_SPEED = 0;
			cerr << "STOP\n";
			cerr << s_dest << endl;
		} /*else{
	  STOP_CAR = false;
	}*/
		// if(abs(s0-s_dest) <= 5)
		// {
		//c_speed /= 2;
		// }
		// Getting the optimal frenet path
		/*if(run_frenet){
	  double startTime2 = omp_get_wtime();
	path = frenet_optimal_planning(csp, s0, c_speed, c_d, c_d_d, c_d_dd, lp, bot_yaw);
	double endTime2 = omp_get_wtime();
	}*/

		double startTime2 = omp_get_wtime();
		path = frenet_optimal_planning(csp, s0, c_speed, c_d, c_d_d, c_d_dd, lp, bot_yaw);
		double endTime2 = omp_get_wtime();

		if (false)
		{
			cerr << endl
				 << " s_d" << endl;
			for (auto i : path.get_s_d())
				cerr << i << "  ";
			cerr << endl
				 << " d_d" << endl;
			for (auto i : path.get_d_d())
				cerr << i << "  ";
			//run_frenet=false;
		}

		lp = path;
		nav_msgs::Path path_msg;
		nav_msgs::Path global_path_msg;

		// paths are published in map frame
		path_msg.header.frame_id = "map";
		global_path_msg.header.frame_id = "map";

		global_path_msg.poses.resize(rx.size());
		// Global path pushed into the message
		for (i = 0; i < rx.size(); i++)
		{
			geometry_msgs::PoseStamped loc;
			loc.pose.position.x = rx[i];
			loc.pose.position.y = ry[i];
			global_path_msg.poses[i] = loc;
		}
		if (false)
		{
			plt::ion();
			plt::show();
			plt::plot(lp.get_x(), lp.get_y());
			plt::pause(0.001);
			plt::plot(rx, ry);
			plt::pause(0.001);
		}
		// Required tranformations on the Frenet path are made and pushed into message
		double startTime3 = omp_get_wtime();
		publishPath(path_msg, path, rk, ryaw, c_d, c_speed, c_d_d);
		double endTime3 = omp_get_wtime();

		/********************* have to find a proper justification for taking the midpoint******/
		auto calc_bot_v = [min_id, rk](vecD d, vecD s_d, vecD d_d) {
			return sqrt(pow(1 - rk[min_id] * d[d.size() / 2], 2) * pow(s_d[s_d.size() / 2], 2) +
						pow(d_d[d_d.size() / 2], 2));
		};

		// Next velocity along the path
		if (path.get_d().size() <= 1 || path.get_s_d().size() <= 1 || path.get_d_d().size() <= 1)
		{
			bot_v = sqrt(pow(1 - rk[min_id] * c_d, 2) * pow(c_speed, 2) + pow(c_d_d, 2));
		}
		else
		{
			if (STOP_CAR)
			{
				//cerr<< "hi" << endl;
				bot_v = calc_bot_v(path.get_d(), path.get_s_d(), path.get_d_d());
			}
			else
			{
				bot_v = sqrt(pow(1 - rk[min_id] * path.get_d()[1], 2) * pow(path.get_s_d()[1], 2) +
							 pow(path.get_d_d()[1], 2));
			}
		}
		if (STOP_CAR)
		{
			cerr << bot_v << endl;
		}
		// cerr<<"Time 1 : "<<endTime1-startTime1<<endl;
		// cerr<<"Time 2 : "<<endTime2-startTime2<<endl;
		//cerr<<"Time 3 : "<<endTime3-startTime3<<endl;
		geometry_msgs::Twist vel;
		vel.linear.x = bot_v;
		vel.linear.y = 0;
		vel.linear.z = 0;
		frenet_path.publish(path_msg);
		global_path.publish(global_path_msg);
		target_vel.publish(vel);
		ctr++;
		// if(!run_frenet){
		//   cerr<<"ending frenet"<<endl;
		//   break;
		// }
		fout.close();
		ros::spinOnce();
	}
}
