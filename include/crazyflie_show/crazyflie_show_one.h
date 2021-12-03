#include <ros/ros.h>
#include <Eigen/Eigen>
#include <fstream>
#include "crazyflie_driver/UpdateParams.h"
#include "geometry_msgs/PoseStamped.h"
#include "crazyflie_driver/Position.h"
#include "std_msgs/Empty.h"
#include "std_srvs/Empty.h"

namespace crazyflie_trajectory_tracking{

class WaypointWithTime {
 public:
  WaypointWithTime()
      : waiting_time(0), yaw(0.0) {
  }

  WaypointWithTime(double t, float x, float y, float z, float _yaw)
      : position(x, y, z), yaw(_yaw), waiting_time(t) {
  }

  Eigen::Vector3d position;
  double yaw;
  double waiting_time;
};

class CrazyflieShow {

public:
    CrazyflieShow(ros::NodeHandle nh, ros::NodeHandle pnh_);
    ~CrazyflieShow();
    void run();
    bool is_land_;

private:
    /* data */
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    std::string tf_prefix_;
    std::string world_frame_;
    const float DEG_2_RAD = M_PI / 180.0;

    double hz_;
    double duration_;
    double height_;

  
    bool is_start_;
    std::string waypoint_file_;

    ros::Timer timer_;

    ros::ServiceClient update_params_srv_client_;

    ros::Subscriber land_sub_;
    ros::Subscriber start_sub_;
    ros::Publisher cmd_position_pub_;
    ros::Publisher cmd_stop_pub_;
    
    ros::ServiceServer sevice_;

    std::vector<WaypointWithTime> waypoints_;

    geometry_msgs::PoseStampedConstPtr initial_pose_;
    ros::Rate rate_;
    
    void initKalmanParams();
    void takeOff();
    void landCallback(const std_msgs::EmptyConstPtr & empty_msg);
    void startTrajCallback(const std_msgs::EmptyConstPtr & empty_msg);
    void trajectoryTracking();
    void hover();
    void loadFile();
    void timerCallback(const ros::TimerEvent & event);
    bool trajService(std_srvs::Empty::Request & req,
            std_srvs::Empty::Response & res);
};



}