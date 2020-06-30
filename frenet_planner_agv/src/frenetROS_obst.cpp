#include "../include/frenet_optimal_trajectory.hpp"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <time.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <utility>
#include <ros/console.h>

namespace plt = matplotlibcpp;

// moved to the hpp file

// inline vecD FrenetPath::get_x()
// {
// 	return x;
// }

// inline vecD FrenetPath::get_y()
// {
// 	return y;
// }

// inline vecD FrenetPath::get_d()
// {
// 	return d;
// }

// inline vecD FrenetPath::get_c()
// {
// 	return c;
// }

// inline vecD FrenetPath::get_s_d()
// {
// 	return s_d;
// }

// inline vecD FrenetPath::get_s()
// {
// 	return s;
// }

// inline vecD FrenetPath::get_d_d()
// {
// 	return d_d;
// }

// inline vecD FrenetPath::get_d_dd()
// {
// 	return d_dd;
// }

// inline vecD FrenetPath::get_yaw()
// {
// 	return yaw;
// }

//accesses the costmap and updates the obstacle coordinates
void costmap_callback(const nav_msgs::OccupancyGrid::ConstPtr& occupancy_grid)
{

	::cmap = *occupancy_grid;
	ob_x.clear();
	ob_y.clear();
	geometry_msgs::Pose origin = occupancy_grid->info.origin;
	for (int width=0; width < occupancy_grid->info.width; ++width)
    {
        for (int height=0; height < occupancy_grid->info.height; ++height)
        {
			// cout<<"Jai Hind doston"<<endl;
	        if(occupancy_grid->data[height*occupancy_grid->info.width + width] > 0)
            {
    
	           ob_x.push_back(width * occupancy_grid->info.resolution + occupancy_grid->info.resolution / 2 + origin.position.x);
               ob_y.push_back(height * occupancy_grid->info.resolution + occupancy_grid->info.resolution / 2 + origin.position.y);
				// cout<<ob_x.back()<<" "<<ob_y.back()<<endl;
            }
        }    
	}

}
//accesses the robot footprint
void footprint_callback(const geometry_msgs::PolygonStampedConstPtr& p)
{
	::footprint = *p;

}


//accesses the odometry data
void odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
	odom = *msg;

}


//calculates the distance between two points (x1,y1) and (x2,y2)
inline double calc_dis(double x1, double y1, double x2, double y2)
{
	return sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2));
}

//finds the point in the global path which is nearest to the bot 
void find_nearest_in_global_path(vecD global_x, vecD global_y, double &min_x, double &min_y, double &min_dis, int &min_id, int flag, FrenetPath path)
{
	double bot_x, bot_y;
	if(flag==0)
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
	for(int i = 0; i < global_x.size(); i++)
	{
		double dis = calc_dis(global_x[i], global_y[i], bot_x, bot_y);
		if(dis < min_dis)
		{
			min_dis = dis;
			min_x = global_x[i];
			min_y = global_y[i];
			min_id = i;
		}
	}
}

//calculates s 
double calc_s(double ptx, double pty, vecD global_x, vecD global_y)
{
	double s = 0;
	if(global_x[0] == ptx && global_y[0] == pty)
	{	
		return s;
	}
	for(int i = 1; i < global_x.size(); i++)
	{
		double dis = calc_dis(global_x[i], global_y[i], global_x[i - 1], global_y[i - 1]);
		s = s + dis;

		if(global_x[i] == ptx && global_y[i] == pty)
		{
			break;
		}
	}

	return s;
}


inline double get_bot_yaw()
{
	geometry_msgs::Pose p = odom.pose.pose;

	tf::Quaternion q(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
	tf::Matrix3x3 m(q);

	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	return yaw;
}


vecD global_path_yaw(Spline2D csp, vecD gx, vecD gy)
{
	vecD yaw;
	vecD t = csp.calc_s(gx, gy);
	for(int i = 0; i < t.size(); i++)
	{
		yaw.push_back(csp.calc_yaw(t[i]));
	}
	return yaw;
}

void initial_conditions_path(Spline2D csp, vecD global_x, vecD global_y, vecD ryaw, double &s0, double &c_speed, double &c_d, double &c_d_d, double &c_d_dd, double &bot_yaw, FrenetPath path)
{
	// Approach 1
	vecD d = path.get_d();
	vecD s_d = path.get_s_d();
	vecD d_d = path.get_d_d();
	vecD d_dd = path.get_d_dd();
	vecD s= path.get_s();

	s0= s[1];
	c_speed = s_d[1];
	c_d = d[1];
	c_d_d= d_d[1];	
	c_d_dd= 0;
	bot_yaw = get_bot_yaw();

	// Approach 2
	/*
		vecD k= path.get_c();
		double v = sqrt(pow(1 - k[1]*d[1], 2)*pow(s_d[1], 2) + pow(d_d[1], 2));
		
		double min_x, min_y;
		int min_id;

		// getting d
		find_nearest_in_global_path(global_x, global_y, min_x, min_y, c_d, min_id, 1, path);

		// deciding the sign for d 
		pair<double, double> vec1, vec2;
		vec1.first = path.get_x()[1] - global_x[min_id];
		vec1.second = path.get_y()[1] - global_y[min_id];

		vec2.first = global_x[min_id] - global_x[min_id + 1];
		vec2.second = global_y[min_id] - global_y[min_id + 1];
		double curl2D = vec1.first*vec2.second - vec2.first*vec1.second;
		if(curl2D < 0) 
			c_d *= -1;

		s0 = calc_s(min_x, min_y, global_x, global_y);
		

		
		
		vecD theta = global_path_yaw(csp, global_x, global_y);
		double g_path_yaw = theta[min_id];	

		double delta_theta = bot_yaw - g_path_yaw;

		c_d_d = v*sin(delta_theta);//Equation 5

		double k_r = csp.calc_curvature(s0);

		c_speed = v*cos(delta_theta) / (1 - k_r*c_d); //s_dot (Equation 7)

		c_d_dd = 0; // For the time being. Need to be updated
	*/
}

void initial_conditions_new(Spline2D csp, vecD global_x, vecD global_y, vecD ryaw, double &s0, double &c_speed, double &c_d, double &c_d_d, double &c_d_dd, double bot_yaw)
{
	FrenetPath path;
	double vx = odom.twist.twist.linear.x;
	double vy = odom.twist.twist.linear.y;
	double v = sqrt(vx*vx + vy*vy);
	
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
	double curl2D = vec1.first*vec2.second - vec2.first*vec1.second;
	if(curl2D < 0) 
		c_d *= -1;

	s0 = calc_s(min_x, min_y, global_x, global_y);
	

	bot_yaw = get_bot_yaw();
	
	
	vecD theta = global_path_yaw(csp, global_x, global_y);
	double g_path_yaw = theta[min_id];	

	double delta_theta = bot_yaw - g_path_yaw;

	c_d_d = v*sin(delta_theta);//Equation 5

	double k_r = csp.calc_curvature(s0);

	c_speed = v*cos(delta_theta) / (1 - k_r*c_d); //s_dot (Equation 7)

	c_d_dd = 0; // For the time being. Need to be updated
}

//publishes path as ros messages
void publishPath(nav_msgs::Path &path_msg, FrenetPath path, vecD rk, vecD ryaw, double &c_speed, double &c_d, double &c_d_d)
{
	vecD x_vec = path.get_x();
	vecD y_vec = path.get_y();
	for(int i = 0; i < path.get_x().size(); i++)
		{
			geometry_msgs::PoseStamped loc;
			loc.pose.position.x = x_vec[i];
			loc.pose.position.y = y_vec[i];

			double delta_theta = atan(c_d_d / ((1 - rk[i]*c_d)*c_speed));
			double yaw = delta_theta + ryaw[i];

			tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, yaw); // roll , pitch = 0
			q.normalize();
			quaternionTFToMsg(q, loc.pose.orientation);

			path_msg.poses.push_back(loc);
		}
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "frenet_planner");
	ros::NodeHandle n;
	
	ros::Publisher frenet_path = n.advertise<nav_msgs::Path>("/frenet_path", 1);		//Publish frenet path
	ros::Publisher global_path = n.advertise<nav_msgs::Path>("/global_path", 1);		//Publish global path
	ros::Publisher target_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);			//Publish velocity

	ros::Subscriber odom_sub = n.subscribe("/base_pose_ground_truth", 10, odom_callback);	
	ros::Subscriber footprint_sub = n.subscribe<geometry_msgs::PolygonStamped>("/move_base/local_costmap/footprint", 10, footprint_callback);
	ros::Subscriber costmap_sub = n.subscribe<nav_msgs::OccupancyGrid>("/move_base/local_costmap/costmap", 10000, costmap_callback);	//Subscribe the initial conditions
	// ros::Subscriber goal_sub = n.subscribe("/move_base_simple/goal", 10, goal_callback);		//Goal 

	
    // get params
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
	
	double ds = 0.1;	//ds represents the step size for cubic_spline
	double bot_yaw;
	//Global path is made using the waypoints
	Spline2D csp = calc_spline_course(W_X, W_Y, rx, ry, ryaw, rk, ds);
	FrenetPath path;
	FrenetPath lp;
	double s0, c_d, c_d_d, c_d_dd, c_speed ;
	int ctr=0;
	clock_t start, end;
	while(ros::ok())
	{
		//Specifing initial conditions for the frenet planner using odometry
		if(ctr%10 == 0 or path.get_c().size() == 0)	
		{
			initial_conditions_new(csp, rx, ry, ryaw, s0, c_speed, c_d, c_d_d, c_d_dd, bot_yaw);
		}
		else
		{
			initial_conditions_path(csp, rx, ry, ryaw, s0, c_speed, c_d, c_d_d, c_d_dd, bot_yaw, path);
		}		

		//Getting the optimal frenet path
		path = frenet_optimal_planning(csp, s0, c_speed, c_d, c_d_d, c_d_dd, lp, bot_yaw);
		lp = path;

		nav_msgs::Path path_msg;
		nav_msgs::Path global_path_msg;

		// paths are published in map frame
		path_msg.header.frame_id = "map";
		global_path_msg.header.frame_id = "map";

		geometry_msgs::PoseStamped current_position;
		/*current_position.pose.position.x = odom.pose.pose.position.x;
		current_position.pose.position.y = odom.pose.pose.position.y;
		current_position.pose.orientation = odom.pose.pose.orientation;
		path_msg.poses.push_back(current_position);*/

		//Global path pushed into the message
		for(int i = 0; i < rx.size(); i++)
		{
			geometry_msgs::PoseStamped loc;
			loc.pose.position.x = rx[i];
			loc.pose.position.y = ry[i];
			global_path_msg.poses.push_back(loc);
		}

		if(false)
		{
			plt::ion();
			plt::show();
			plt::plot(lp.get_x(), lp.get_y());
			plt::pause(0.001);
			plt::plot(rx,ry);
			plt::pause(0.001);
		}

		//Required tranformations on the Frenet path are made and pushed into message
		publishPath(path_msg, path, rk, ryaw, c_d, c_speed, c_d_d);	

		//Next velocity along the path
		double bot_v = sqrt(pow(1 - rk[1]*c_d, 2)*pow(c_speed, 2) + pow(c_d_d, 2));	

		geometry_msgs::Twist vel;
		vel.linear.x = bot_v;
		vel.linear.y = 0;
		vel.linear.z = 0;

		frenet_path.publish(path_msg);
		global_path.publish(global_path_msg);
		target_vel.publish(vel);

		/*frenet_path.publish(path_msg);
		global_path.publish(global_path_msg);
		target_vel.publish(vel);*/
		ctr++;
		ros::spinOnce(); 
	}
}
