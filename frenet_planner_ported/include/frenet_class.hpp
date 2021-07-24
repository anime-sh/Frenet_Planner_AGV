#ifndef FRENET_CLASS_HPP_
#define FRENET_CLASS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <omp.h>
#include <fstream>
#include <matplotlibcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "../include/cubic_spline_planner.hpp"

using std::placeholders::_1;
using vecD = vector<double>;
int cost_count = 0;

double MAX_SPEED;  // maximum speed [m/s]
double MAX_ACCEL;  // maximum acceleration [m/ss]
double MAX_CURVATURE;  // maximum curvature [1/m]
double MAX_ROAD_WIDTH;  // maximum road width [m]
double D_ROAD_W;  // road width sampling length [m]
double DT;  // time tick [s]
double MAXT;  // max prediction time [m]
double MINT;  // min prediction time [m]
double TARGET_SPEED;  // target speed [m/s]
double D_T_S;  // target speed sampling length [m/s]
double N_S_SAMPLE;  // sampling number of target speed
double ROBOT_RADIUS;  // robot radius [m]
double MIN_LAT_VEL;  // minimum lateral speed. Sampling for d_dot
double MAX_LAT_VEL;  // maxmum lateral speed. Sampling for d_dot
double D_D_NS;  // Step size for sampling of d_dot
double MAX_SHIFT_D;  // Sampling width for sampling of d.

// cost weights
double KJ;
double KT;
double KD;
double KD_V;
double KLAT;
double KLON;
bool STOP_CAR = false;
double s_dest;
// Waypoints
vector<double> W_X;
vector<double> W_Y;

nav_msgs::msg::Odometry odom;
nav_msgs::msg::OccupancyGrid cmap;
geometry_msgs::msg::PolygonStamped footprint;
vector<double> ob_x;   // x coordinates of the obstacles
vector<double> ob_y;   // y coordinates of the obstacles
 
 
 
class FrenetClass : public rclcpp::Node {
public:  
	vecD wx, wy;
	double target_speed;
        FrenetClass()  : Node("frenet_class")
        {
            publisher_frenet_path = this->create_publisher<nav_msgs::msg::Path>("/frenet_path", 1);
            publisher_global_path = this->create_publisher<nav_msgs::msg::Path>("/global_path", 1);
            publisher_target_vel = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        		
	    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/base_pose_ground_truth", 10, std::bind(&FrenetClass::odom_callback,this, _1));
            footprint_sub = this->create_subscription<geometry_msgs::msg::PolygonStamped>("/move_base/local_costmap/footprint", 10, std::bind(&FrenetClass::footprint_callback,this, _1));
            costmap_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/move_base/local_costmap/costmap", 10, std::bind(&FrenetClass::costmap_callback, this, _1));
	    
	   this->declare_parameter<double>("KJ", 0);
	   this->get_parameter("KJ", KJ);
	   this->declare_parameter<double>("KT", 0);
	   this->get_parameter("KT", KT);
	   this->declare_parameter<double>("KD", 0);
	   this->get_parameter("KD", KD);
	   this->declare_parameter<double>("KD_V", 0);
	   this->get_parameter("KD_V", KD_V);
	   this->declare_parameter<double>("KLAT", 0);
	   this->get_parameter("KLAT", KLAT);
	   this->declare_parameter<double>("KLON", 0);
	   this->get_parameter("KLON", KLON);
	   this->declare_parameter<double>("MAX_SPEED", 0);
	   this->get_parameter("MAX_SPEED", MAX_SPEED);
	   this->declare_parameter<double>("MAX_ACCEL", 0);
	   this->get_parameter("MAX_ACCEL",MAX_ACCEL );
	   this->declare_parameter<double>("MAX_CURVATURE", 0);
	   this->get_parameter("MAX_CURVATURE", MAX_CURVATURE);
	   this->declare_parameter<double>("D_ROAD_W", 0);
	   this->get_parameter("D_ROAD_W", D_ROAD_W);
	   this->declare_parameter<double>("DT", 0);
	   this->get_parameter("DT", DT);
	   this->declare_parameter<double>("MAXT", 0);
	   this->get_parameter("MAXT", MAXT);
	   this->declare_parameter<double>("MINT", 0);
	   this->get_parameter("MINT", MINT);
	   this->declare_parameter<double>("TARGET_SPEED", 0);
	   this->get_parameter("TARGET_SPEED", TARGET_SPEED);
	   this->declare_parameter<double>("D_T_S", 0);
	   this->get_parameter("D_T_S", D_T_S);
	   this->declare_parameter<double>("N_S_SAMPLE", 0);
	   this->get_parameter("N_S_SAMPLE", N_S_SAMPLE);
	   this->declare_parameter<double>("ROBOT_RADIUS", 0);
	   this->get_parameter("ROBOT_RADIUS", ROBOT_RADIUS);
	   this->declare_parameter<double>("MIN_LAT_VEL", 0);
	   this->get_parameter("MIN_LAT_VEL", MIN_LAT_VEL);
	   this->declare_parameter<double>("D_D_NS", 0);
	   this->get_parameter("D_D_NS", D_D_NS);
	   this->declare_parameter<double>("MAX_SHIFT_D", 0);
	   this->get_parameter("MAX_SHIFT_D", MAX_SHIFT_D);

	   this->declare_parameter("W_X");
           rclcpp::Parameter double_array_param = this->get_parameter("W_X");
           std::vector<double> w_z = double_array_param.as_double_array();
           
           this->declare_parameter("W_Y");
           rclcpp::Parameter double_array_param_y = this->get_parameter("W_Y");
           std::vector<double> w_w = double_array_param_y.as_double_array();

           wx =  w_z;
           wy = w_w;      
        }

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_frenet_path;            
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_global_path;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_target_vel;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_sub;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub;

    void footprint_callback(const geometry_msgs::msg::PolygonStamped::ConstPtr p)
    {
	     ::footprint = *p;
    }

// accesses the odometry data
    void odom_callback(const nav_msgs::msg::Odometry::ConstPtr msg)
    {
	     ::odom = *msg;
    }
    void costmap_callback(const nav_msgs::msg::OccupancyGrid::ConstPtr occupancy_grid)
    {

	    cost_count++;
	    unsigned int height, width;
	    ::cmap = *occupancy_grid;
	    ob_x.clear();
	    ob_y.clear();
	    geometry_msgs::msg::Pose origin = occupancy_grid->info.origin;

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
   }
   

};
#endif
