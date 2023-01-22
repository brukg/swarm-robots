#include <swarm_robots/swarm_robots.h>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Float32.h>
#include <visualization_msgs/Marker.h>

#include <vector>

class SwarmRobots
{
public:
    SwarmRobots(ros::NodeHandle& nh);
    ~SwarmRobots();

private:
    // Callback functions for handling subscribed topics
    void callbackCoeffSep(const std_msgs::Float32::ConstPtr& msg);
    void callbackCoeffCoh(const std_msgs::Float32::ConstPtr& msg);
    void callbackCoeffAli(const std_msgs::Float32::ConstPtr& msg);
    void callbackRoamingCoeff(const std_msgs::Float32::ConstPtr& msg);
    void callbackAvoidanceCoeff(const std_msgs::Float32::ConstPtr& msg);
    void callbackNeighborsRadius(const std_msgs::Float32::ConstPtr& msg);

    // Publishers and subscribers
    ros::Publisher publishNeighbors;
    ros::Publisher publishFlock;
    ros::Publisher publishAgent;
    ros::Publisher publishGoal;
    ros::Subscriber subCoeffSep;
    ros::Subscriber subCoeffCoh;
    ros::Subscriber subCoeffAli;
    ros::Subscriber subRoamingCoeff;
    ros::Subscriber subAvoidanceCoeff;
    ros::Subscriber subNeighborsRadius;

    // Other member variables
    std::vector<double> odom;
    std::vector<double> cmdVel;
    int numRobots;
    std::string nsPrefix;
    double coeffSep;
    double coeffCoh;
    double coeffAli;
    double roamingCoeff;
    double avoidanceCoeff;
    double neighborsRadius;
    double slowingDistance;
    double slowingSpeed;
    double maxAcc;
    double maxVel;
    double fov;
    double maxSeeAhead;
};


SwarmRobots::SwarmRobots(ros::NodeHandle& nh) :
    odom(3),
    cmdVel(3)
{
    // Get parameters
    nh.getParam("/num_of_robots", numRobots);
    nh.getParam("/robot_name", nsPrefix);
    nh.getParam("~coeff_sep", coeffSep);
    nh.getParam("~coeff_coh", coeffCoh);
    nh.getParam("~coeff_ali", coeffAli);
    nh.getParam("~roaming_coeff", roamingCoeff);
    nh.getParam("~avoidance_coeff", avoidanceCoeff);
    nh.getParam("~neighbors_radius", neighborsRadius);
    nh.getParam("~slowing_distance", slowingDistance);
    nh.getParam("~slowing_speed", slowingSpeed);
    nh.getParam("~max_acc", maxAcc);
    nh.getParam("~max_vel", maxVel);
    nh.getParam("~fov", fov);
    nh.getParam("~max_see_ahead", maxSeeAhead);

    // Initialize publishers and subscribers
    publishNeighbors = nh.advertise<visualization_msgs::Marker>("pose_neighbors", 2);
    publishFlock = nh.advertise<visualization_msgs::Marker>("pose_flock", 2);
    publishAgent = nh.advertise<visualization_msgs::Marker>("pose_agent", 2);
    publishGoal = nh.advertise<visualization_msgs::Marker>("pose_goal", 2);
    subCoeffSep = nh.subscribe("/coeff_sep", 2, &SwarmRobots::callbackCoeffSep, this);
    subCoeffCoh = nh.subscribe("/coeff_coh", 2, &SwarmRobots::callbackCoeffCoh, this);
    subCoeffAli = nh.subscribe("/coeff_ali", 2, &SwarmRobots::callbackCoeffAli, this);
    subRoamingCoeff = nh.subscribe("/roaming_coeff", 2, &SwarmRobots::callbackRoamingCoeff, this);
    subAvoidanceCoeff = nh.subscribe("/avoidance_coeff", 2, &SwarmRobots::callbackAvoidanceCoeff, this);
    subNeighborsRadius = nh.subscribe("/neighbors_radius", 2, &SwarmRobots::callbackNeighborsRadius, this);

    // Initialize other member variables
    odom.resize(numRobots * 3);
    cmdVel.resize(numRobots * 3);
}

SwarmRobots::~SwarmRobots()
{
    // Clean up resources
}

void SwarmRobots::callbackCoeffSep(const std_msgs::Float32::ConstPtr& msg)
{
    coeffSep = msg->data;
}
void SwarmRobots::callbackCoeffCoh(const std_msgs::Float32::ConstPtr& msg)
{
    coeffCoh = msg->data;
}

void SwarmRobots::callbackCoeffAli(const std_msgs::Float32::ConstPtr& msg)
{
    coeffAli = msg->data;
}

void SwarmRobots::callbackRoamingCoeff(const std_msgs::Float32::ConstPtr& msg)
{
    roamingCoeff = msg->data;
}

void SwarmRobots::callbackAvoidanceCoeff(const std_msgs::Float32::ConstPtr& msg)
{
    avoidanceCoeff = msg->data;
}

void SwarmRobots::callbackNeighborsRadius(const std_msgs::Float32::ConstPtr& msg)
{
    neighborsRadius = msg->data;
}
int main(int argc, char **argv)
{

  // Initialize ROS
  ros::init(argc, argv, "swarm_robots");
  ros::NodeHandle node;
  ros::NodeHandle private_nh("~");

//   lidar_pose_with_covariance::Interface Interface(node, private_nh);

  ros::Rate loop_rate(10); //loop frequency
  ros::spin(); //invokes callback

  return 0;
};