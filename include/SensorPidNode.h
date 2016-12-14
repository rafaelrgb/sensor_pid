/**
 *  This header file defines the SensorPidNoce class, which is also a
 *  Node class via enhancement.
 *
 *  Version: 1.0.0
 *  Created on: 25/11/2016
 *  Modified on: 25/11/2016
 *  Author: Rafael Gomes Braga (faerugb@gmail.com)
 *  Maintainer: Rafael Gomes Braga (faerugb@gmail.com)
 */

#ifndef _SENSOR_PID_NODE_H_
#define _SENSOR_PID_NODE_H_

#include <string>
#include <math.h>
#include "Node.h"
#include <sensor_msgs/Range.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_datatypes.h>

#define FACTOR  60.0

#define MINRC   1100
#define BASERC  1500
#define MAXRC   1900

#define MAXVEL   1.0
#define MINVEL   -1.0

#define PI 3.14159

#define VISION_DISTANCE 3000.0

class SensorPidNode : public Node
{
public:
  SensorPidNode(ros::NodeHandle *nh);
  virtual ~SensorPidNode();

private:
  virtual void controlLoop();

  // Distance control variables
  double altitude_;
  double pitch_;
  double distance_;
  double setpoint_;
  double control_effort_;
  double max_range_;

  // Enable control
  bool sensorPidEnable_;

  // ROS objects
  ros::Subscriber sonar_sub_;
  ros::Subscriber control_effort_sub_;
  ros::Subscriber sensor_pid_enable_sub_;
  ros::Subscriber pose_sub_;
  ros::Publisher state_pub_;
  ros::Publisher setpoint_pub_;
  ros::Publisher cmd_vel_pub_;

  // Member functions
  void sonarCb( const sensor_msgs::RangeConstPtr &msg );
  void controlEffortCb( const std_msgs::Float64ConstPtr &msg );
  void sensorPidEnableCb( const std_msgs::BoolConstPtr &msg );
  void poseCb( const geometry_msgs::PoseStampedConstPtr &msg );
  void publishState( double state );
  void publishSetpoint( double setpoint );
  void publishVelocity( double velX, double velY );

};

#endif // _SENSOR_PID_NODE_H_
