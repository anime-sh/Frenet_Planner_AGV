#include "../include/frenet_class.hpp"
#include "../include/frenet_optimal_trajectory.hpp"
#include "../include/cubic_spline_planner.hpp"
#include "../include/parameters.hpp"

namespace plt = matplotlibcpp;

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
	geometry_msgs::msg::Pose p = odom.pose.pose;
	tf2::Quaternion q(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
	tf2::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	trace(yaw, pitch, roll); // is -nan
	return yaw;
}

void initial_conditions_path(Spline2D &csp, double &s0, double &c_speed, double &c_d, double &c_d_d,
							 double &c_d_dd, double &bot_yaw, FrenetPath &path)
{
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
void publishPath(nav_msgs::msg::Path &path_msg, FrenetPath &path, vecD &rk, vecD &ryaw, double &c_speed,
				 double &c_d, double &c_d_d)
{
	geometry_msgs::msg::PoseStamped loc;
	double delta_theta, yaw;
	vecD x_vec = path.get_x();
	vecD y_vec = path.get_y();
	for (unsigned int i = 0; i < path.get_x().size(); i++)
	{
		loc.pose.position.x = x_vec[i];
		loc.pose.position.y = y_vec[i];
		delta_theta = atan(c_d_d / ((1 - rk[i] * c_d) * c_speed));
		yaw = delta_theta + ryaw[i];
		tf2::Quaternion q;
		q.setRPY(0, 0, yaw);
		q = q.normalize();
		loc.pose.orientation = tf2::toMsg(q);
		path_msg.poses.push_back(loc);
	}
}

int main(int argc, char **argv)
{
	bool gotOdom = false;
	rclcpp::init(argc, argv);
	auto node = std::make_shared<FrenetClass>();
	
	W_X = node->wx;
	W_Y = node->wy;
	TARGET_SPEED = node->target_speed;
	RCLCPP_INFO(node->get_logger(), "running main");

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
	int init_flag = true;
	int iteration_count = 1;
	while (rclcpp::ok())
	{
		double mainLoop1 = omp_get_wtime();
		rclcpp::spin_some(node);
		ofstream fout;
		fout.open("/home/animesh/try_ws/src/frenet_planner_agv/src/log.txt",ios::app);
		int min_id = 0;
		double startTime1 = omp_get_wtime();
		if (init_flag)
		{
			min_id = initial_conditions_new(csp, global_s, rx, ry, rk, ryaw, s0, c_speed, c_d, c_d_d,c_d_dd, bot_yaw, path);
			init_flag = false;
											
		}
		else
		{
		 	initial_conditions_path(csp, s0, c_speed, c_d, c_d_d, c_d_dd, bot_yaw, path);
		}
		double endTime1 = omp_get_wtime();
		//plts0.push_back(s0);
		//pltcspeed.push_back(c_speed);
		//plttime.push_back(abs(endTime1-pltbasetime));
		//plt::plot(plttime,pltcspeed);
		//plt::pause(0.001);
		//fout<<"a run"<<endl;
		//fout<<plts0<<endl;
		//fout<<plttime<<endl;
		//fout<<pltcspeed<<endl;
		//fout<<"end"<<endl;
		trace("frenet optimal planning", s0, c_speed, c_d, c_d_d, c_d_dd, lp, bot_yaw);
		if (abs(s_dest - s0) <= 15)
		{
			STOP_CAR = true;
			TARGET_SPEED = 0;
			//cerr << "STOP\n";
			//cerr << s_dest << endl;
		} else{
			STOP_CAR = false;
		}
		if(abs(s0-s_dest) <= 5)
		{
			c_speed /= 2;
		}

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
		nav_msgs::msg::Path path_msg;
		nav_msgs::msg::Path global_path_msg;
		path_msg.header.frame_id = "map";
		global_path_msg.header.frame_id = "map";

		global_path_msg.poses.resize(rx.size());
		for (i = 0; i < rx.size(); i++)
		{
			geometry_msgs::msg::PoseStamped loc;
			loc.pose.position.x = rx[i];
			loc.pose.position.y = ry[i];
			global_path_msg.poses[i] = loc;
		}
		if (true)
		{
			plt::ion();
			plt::show();
			plt::plot(lp.get_x(), lp.get_y());
			plt::pause(0.001);
			plt::plot(rx, ry);
			plt::scatter(ob_x,ob_y);
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

		if (path.get_d().size() <= 1 || path.get_s_d().size() <= 1 || path.get_d_d().size() <= 1)
		{
			bot_v = sqrt(pow(1 - rk[min_id] * c_d, 2) * pow(c_speed, 2) + pow(c_d_d, 2));
		}
		else
		{
			if (STOP_CAR)
			{
				bot_v = calc_bot_v(path.get_d(), path.get_s_d(), path.get_d_d());
			}
			else
			{
				bot_v = sqrt(pow(1 - rk[min_id] * path.get_d()[1], 2) * pow(path.get_s_d()[1], 2) +
							 pow(path.get_d_d()[1], 2));
			}
		}
		//if (STOP_CAR)
		//{
		//	cerr <<"velocity:"<< bot_v << endl;
		//}

		geometry_msgs::msg::Twist vel;
		vel.linear.x = bot_v;
		vel.linear.y = 0;
		vel.linear.z = 0;
		
		node->publisher_global_path->publish(path_msg);
		node->publisher_frenet_path->publish(global_path_msg);
		node->publisher_target_vel->publish(vel);
		
		
		ctr++;
		 //if(!run_frenet){
		   //cerr<<"ending frenet"<<endl;
		   //break;
		 //}
		fout.close();
		double mainLoop2 = omp_get_wtime();
	}
	return 0;
}
