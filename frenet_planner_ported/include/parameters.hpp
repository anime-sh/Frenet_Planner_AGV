#ifndef PARAMETERS_HPP_
#define PARAMETERS_HPP_
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>

extern double MAX_SPEED;  // maximum speed [m/s]
extern double MAX_ACCEL;  // maximum acceleration [m/ss]
extern double MAX_CURVATURE;  // maximum curvature [1/m]
extern double MAX_ROAD_WIDTH;  // maximum road width [m]
extern double D_ROAD_W;  // road width sampling length [m]
extern double DT ;  // time tick [s]
extern double MAXT;  // max prediction time [m]
extern double MINT ;  // min prediction time [m]
extern double TARGET_SPEED;  // target speed [m/s]
extern double D_T_S;  // target speed sampling length [m/s]
extern double N_S_SAMPLE;  // sampling number of target speed
extern double ROBOT_RADIUS;  // robot radius [m]
extern double MIN_LAT_VEL;  // minimum lateral speed. Sampling for d_dot
extern double MAX_LAT_VEL;  // maxmum lateral speed. Sampling for d_dot
extern double D_D_NS;  // Step size for sampling of d_dot
extern double MAX_SHIFT_D;  // Sampling width for sampling of d.

// cost weights
extern double KJ;
extern double KT;
extern double KD;
extern double KD_V;
extern double KLAT;
extern double KLON;
extern bool STOP_CAR;
extern double s_dest;
// Waypoints
extern std::vector<double> W_X;
extern std::vector<double> W_Y;

extern nav_msgs::msg::Odometry odom;
extern nav_msgs::msg::OccupancyGrid cmap;
extern geometry_msgs::msg::PolygonStamped footprint;
extern std::vector<double> ob_x;   // x coordinates of the obstacles
extern std::vector<double> ob_y;   // y coordinates of the obstacles

#endif
