#include <rclcpp/rclcpp.hpp>
#include <math.h>       /* atan */
#include <omp.h>      //Multi-threading
#include <vector>
#include <random>
#include <algorithm> // for sort(), min()

#include <chrono>
#include <iostream>
#include <fstream>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tracker_msgs/msg/track_array.hpp>
#include <tracker_msgs/msg/track.hpp>

#include <tf2_ros/buffer.h>

#include "cluster.hpp"

typedef std::pair<double, double> Point;
typedef std::vector<double> l_shape;
typedef std::vector<l_shape> l_shapes;
typedef std::vector<Point> pointList;


using namespace std;
// This node segments the point cloud based on the break-point detector algorithm.
// This algorithm is based on "L-Shape Model Switching-Based Precise Motion Tracking 
// of Moving Vehicles Using Laser Scanners.
class Datmo : public rclcpp::Node
{
public:
  Datmo();
  ~Datmo();

  void callback(const sensor_msgs::msg::LaserScan::ConstPtr &);
  void Clustering(const sensor_msgs::msg::LaserScan::ConstPtr& , vector<pointList> &);
  void visualiseGroupedPoints(const vector<pointList> &);
  void transformPointList(const pointList& , pointList& );

private:
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_marker_array;
  rclcpp::Publisher<tracker_msgs::msg::TrackArray>::SharedPtr pub_tracks_mean;
  rclcpp::Publisher<tracker_msgs::msg::TrackArray>::SharedPtr pub_tracks_mean_kf;
  rclcpp::Publisher<tracker_msgs::msg::TrackArray>::SharedPtr pub_tracks_box;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan;
  sensor_msgs::msg::LaserScan scan;
  vector<Cluster> clusters;

  ofstream whole; // file to write the program duration
  ofstream clustering; // file to write the program duration
  ofstream rect_fitting; //write rectangle fitting duration
  ofstream testing; //various testing

  std::unique_ptr<tf2_ros::Buffer> tf_buffer;

  //Tuning Parameteres
  double dt;
  rclcpp::Time time;

  unsigned long int cg       = 1;//group counter to be used as id of the clusters
  //initialised as one, because 0 index take the msgs that fail to be initialized
  unsigned long int cclusters= 0;//counter for the cluster objects to be used as id for the markers

  //Parameters
  double dth;
  double euclidean_distance;
  bool p_marker_pub;
  bool w_exec_times;
  string lidar_frame;
  string world_frame;

};
