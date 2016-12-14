#include <stdlib.h>
#include "SensorPidNode.h"

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "sensor_pid_node");
  SensorPidNode node(new ros::NodeHandle());
  node.spin();
  return EXIT_SUCCESS;
}
