#include "crazyflie_show/crazyflie_show_one.h"


namespace crazyflie_trajectory_tracking{

CrazyflieShow::CrazyflieShow(ros::NodeHandle nh, ros::NodeHandle pnh):
    nh_(nh),
    pnh_(pnh),
    hz_(pnh_.param<double>("hz", 10.0)),
    duration_(pnh_.param<double>("duartion", 1.0)),
    rate_(hz_),
    is_land_(false),
    is_start_(false),
    height_(pnh_.param<double>("height", 0.4)),
    tf_prefix_(pnh_.param<std::string>("tf_prefix", "cf5")),
    waypoint_file_(pnh_.param<std::string>("waypoint_file_path", "/home/anfield/catkin_threebody_ws/src/crazyflie_ros/crazyflie_show/resources/waypoint.txt")),
    world_frame_(pnh_.param<std::string>("world_frame", "world")){
    
    initKalmanParams();
    ROS_INFO_STREAM("Before land_sub_");
    land_sub_ = nh_.subscribe("/end_and_land", 1000, &crazyflie_trajectory_tracking::CrazyflieShow::landCallback, this);
    start_sub_ =  nh_.subscribe("/start_trajectory", 1000, &crazyflie_trajectory_tracking::CrazyflieShow::startTrajCallback, this);
    ROS_INFO_STREAM("After land_sub_");
    cmd_position_pub_ = nh_.advertise<crazyflie_driver::Position>("/" + tf_prefix_ + "/cmd_position", 1);
    cmd_stop_pub_ = nh_.advertise<std_msgs::Empty>("/" + tf_prefix_ + "/stop", 1);
    sevice_ = nh_.advertiseService("/start_trajectory",&CrazyflieShow::trajService, this);

    timer_ = nh_.createTimer(ros::Duration(0), &CrazyflieShow::timerCallback, this, true, true);

}

CrazyflieShow::~CrazyflieShow(){}

bool CrazyflieShow::trajService(std_srvs::Empty::Request & req,
        std_srvs::Empty::Response & res){
            ROS_INFO_STREAM("Enter the traj");
}

void CrazyflieShow::run(){
    loadFile();
    takeOff();
    trajectoryTracking();
    hover();
}

void CrazyflieShow::timerCallback(const ros::TimerEvent & event){

}

void CrazyflieShow::initKalmanParams(){
    update_params_srv_client_ = nh_.serviceClient<crazyflie_driver::UpdateParams>("/" + tf_prefix_ + "/update_params");
    update_params_srv_client_.waitForExistence();

    crazyflie_driver::UpdateParams update_params_srv;
    initial_pose_ = ros::topic::waitForMessage<geometry_msgs::PoseStamped>(
                                                                        "/" + tf_prefix_ + "/external_pose");
    
    nh_.setParam("/" + tf_prefix_ + "/kalman/resetEstimation", 1);
    nh_.setParam("/" + tf_prefix_ + "/stabilizer/estimator", 2);

    nh_.setParam("/" + tf_prefix_ + "/kalman/initialX", initial_pose_->pose.position.x);
    nh_.setParam("/" + tf_prefix_ + "/kalman/initialY", initial_pose_->pose.position.y);
    nh_.setParam("/" + tf_prefix_ + "/kalman/initialZ", initial_pose_->pose.position.z);

    std::vector<std::string> params_to_update = {"kalman/resetEstimation",
                                                                "stabilizer/estimator" ,
                                                                "kalman/initialX",
                                                                "kalman/initialY",
                                                                "kalman/initialZ"};
    update_params_srv.request.params = params_to_update;
    if(update_params_srv_client_.call(update_params_srv)){
        ROS_INFO_STREAM("update params successfully!");
    }else{
        ROS_INFO_STREAM("update params unsuccessfully....");
        exit(1);
    }
}

void CrazyflieShow::landCallback(const std_msgs::EmptyConstPtr & empty_msg){
    ROS_INFO_STREAM("Enter the landcallback");
    is_land_ = true;
}

void CrazyflieShow::startTrajCallback(const std_msgs::EmptyConstPtr & empty_msg){
    ROS_INFO_STREAM("Enter the startTrajCallback");
    is_start_ = true;
}

void CrazyflieShow::takeOff(){
    crazyflie_driver::Position msg_position;

    msg_position.header.frame_id = "world";
    msg_position.x = waypoints_[0].position.x();
    msg_position.y = waypoints_[0].position.y();
    msg_position.yaw =  waypoints_[0].yaw;

    double cmd_z_position = initial_pose_->pose.position.z;
    double step_length = height_ / (hz_ * duration_);
    
    while (ros::ok() && cmd_z_position < height_ ) {
        cmd_z_position += step_length;
        msg_position.header.stamp = ros::Time::now();
        msg_position.z = cmd_z_position;
        cmd_position_pub_.publish(msg_position);
        ROS_INFO_STREAM("cmd_z_position: " << cmd_z_position);
        rate_.sleep();
    }
    int k = 0;
    while (ros::ok() && k < 40) {
        k++;
        cmd_z_position = height_;
        msg_position.header.stamp = ros::Time::now();
        msg_position.z = cmd_z_position;
        cmd_position_pub_.publish(msg_position);
        ROS_INFO_STREAM("cmd_z_position: " << cmd_z_position);
        rate_.sleep();
    }


}

void CrazyflieShow::hover(){
    crazyflie_driver::Position msg_position;
    msg_position.header.frame_id = "world";
    msg_position.x = waypoints_.back().position.x();
    msg_position.y = waypoints_.back().position.y();
    msg_position.yaw =  waypoints_.back().yaw;
    int k = 0; 
    while (ros::ok() && k < 80) {
        k++;
        msg_position.header.stamp = ros::Time::now();
        msg_position.z = height_;
        cmd_position_pub_.publish(msg_position);
        ROS_INFO_STREAM("hover");
        rate_.sleep();
    }

    double cmd_z_position = height_;
    double step_length = height_ / (hz_ * duration_);

    while (ros::ok() && cmd_z_position > 0.0) {
        cmd_z_position -= step_length;
        msg_position.header.stamp = ros::Time::now();
        msg_position.z = cmd_z_position;
        cmd_position_pub_.publish(msg_position);
        ROS_INFO_STREAM("cmd_z_position: " << cmd_z_position);
        rate_.sleep();
    }

    std_msgs::Empty msg_stop;
    cmd_stop_pub_.publish(msg_stop);
}

void CrazyflieShow::loadFile(){
    std::ifstream wp_file(waypoint_file_.c_str());
    if (wp_file.is_open()) {
        double t, x, y, z, yaw;
        // Only read complete waypoints.
        while (wp_file >> t >> x >> y >> z >> yaw) {
            x = x / 5 * 3;
            y = y / 5 * 3;
            waypoints_.push_back(WaypointWithTime(t, x, y, z, yaw * DEG_2_RAD));
        }
        wp_file.close();
        ROS_INFO("Read %d waypoints.", (int) waypoints_.size());
    } else {
        ROS_ERROR_STREAM("Unable to open poses file: " << waypoint_file_);
        exit(-1);
    }
}

void CrazyflieShow::trajectoryTracking(){
    
    crazyflie_driver::Position msg_position;
    msg_position.header.frame_id = "world";
    msg_position.z = height_;

    for (const auto & waypoint : waypoints_) {
        msg_position.x = waypoint.position.x();
        msg_position.y = waypoint.position.y();
        msg_position.header.stamp = ros::Time::now();
        msg_position.yaw = waypoint.yaw;
        cmd_position_pub_.publish(msg_position);
        ROS_INFO_STREAM("trajectroy tracking/x: " << msg_position.x << ", y: " << msg_position.y << ", yaw: " << msg_position.yaw);
        rate_.sleep();
        rate_.sleep();
    }
    
};

}

