#include <swarm_robots/swarm_robots.h>



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