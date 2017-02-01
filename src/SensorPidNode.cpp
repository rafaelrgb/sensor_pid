/**
 *  This source file implements the SensorPidNode class, which is also a
 *  Node class via enhancement.
 *
 *  Version: 1.0.0
 *  Created on: 25/11/2016
 *  Modified on: 25/11/2016
 *  Author: Rafael Gomes Braga (faerugb@gmail.com)
 *  Maintainer: Rafael Gomes Braga (faerugb@gmail.com)
 */

#include "SensorPidNode.h"


SensorPidNode::SensorPidNode(ros::NodeHandle *nh)
  : Node(nh, 30)
{
    sensorPidEnable_ = false;
    altitude_ = 0.0;
    pitch_ = 0.0;
    distance_ = 10.0;
    setpoint_ = 1.0;
    control_effort_ = 0.0;
    max_range_ = 10.0;

    sonar_sub_ = nh->subscribe("/rpi/sonar", 1, &SensorPidNode::sonarCb, this);
    control_effort_sub_ = nh->subscribe("/control_effort", 1, &SensorPidNode::controlEffortCb, this);
    sensor_pid_enable_sub_ = nh->subscribe("sensor_pid_enable", 1, &SensorPidNode::sensorPidEnableCb, this);
    pose_sub_ = nh->subscribe("/mavros/local_position/pose", 1, &SensorPidNode::poseCb, this);
    state_pub_ = nh->advertise<std_msgs::Float64>("/state", 10);
    setpoint_pub_ = nh->advertise<std_msgs::Float64>("/setpoint", 10);
    cmd_vel_pub_ = nh->advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);
}

SensorPidNode::~SensorPidNode()
{
  sonar_sub_.shutdown();
  control_effort_sub_.shutdown();
  sensor_pid_enable_sub_.shutdown();
  pose_sub_.shutdown();
  state_pub_.shutdown();
  setpoint_pub_.shutdown();
  cmd_vel_pub_.shutdown();
}

void SensorPidNode::controlLoop()
{
    publishSetpoint(setpoint_);

    ROS_INFO("distance = %f", distance_);

    if ( sensorPidEnable_ == true )
    {
        // Calculate velocity
        geometry_msgs::Point32 velocity;
        velocity.x = 0.0;

        if ( distance_ >= max_range_ )
        {
            velocity.y = -MAXVEL;
        }
        else
        {
            velocity.y = control_effort_;
        }

        // Limit the Velocity in x
        if (velocity.x > MAXVEL)
        {
            velocity.x = MAXVEL;
        } else if (velocity.x < MINVEL)
        {
            velocity.x = MINVEL;
        }

        // Limit the Velocity in y
        if (velocity.y > MAXVEL)
        {
            velocity.y = MAXVEL;
        } else if (velocity.y < MINVEL)
        {
            velocity.y = MINVEL;
        }

        // PUBLISH VELOCITY
        publishVelocity(velocity.x, velocity.y);
    }
}


void SensorPidNode::sonarCb( const sensor_msgs::RangeConstPtr &msg )
{
    // Given the measured range, calculate the real distance to the obstacle given drone's pose
    double range = msg->range;

    if ( range >= max_range_ )
    {
        // Drone isn't detecting anything in front of it
        distance_ = max_range_;
    }
    else
    {
        if ( pitch_ >= -0.01 && pitch_ <= 0.01 )
        {
            // Drone horizontaly aligned and is detecting the obstacle correctly
            distance_ = range;
        }
        else if ( pitch_ < 0.01)
        {
            // Drone is poiting up and there is an obstacle in front of it
            distance_ = range * cos( -pitch_ );
        }
        else
        {
            // Drone is poiting down and detecting something.
            // In this case we expect the sensor to detect the ground, so we must do calculations
            // to determine if there is in fact an obstacle or just the ground
            double expectedRange = altitude_ / sin( pitch_ );

            if ( range >= (expectedRange - 0.5) ) // tolerance
            {
                // No obstacle, just the ground
                distance_ = max_range_;
            }
            else
            {
                // There is a close obstacle
                distance_ = range * cos( pitch_ );
            }
        }
    }

    publishState(distance_);
}


void SensorPidNode::controlEffortCb( const std_msgs::Float64ConstPtr &msg )
{
    control_effort_ = msg->data;
}


void SensorPidNode::sensorPidEnableCb( const std_msgs::BoolConstPtr &msg )
{
    sensorPidEnable_ = msg->data;
}

void SensorPidNode::poseCb(const geometry_msgs::PoseStampedConstPtr &msg)
{
    // Convert quaternions to RPY
    tf::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // Save altitude and pitch in data members
    pitch_ = pitch;
    altitude_ = msg->pose.position.z;
}


void SensorPidNode::publishState( double state )
{
    std_msgs::Float64 msg;
    msg.data = state;
    state_pub_.publish(msg);
}


void SensorPidNode::publishSetpoint( double setpoint )
{
    std_msgs::Float64 msg;
    msg.data = setpoint;
    setpoint_pub_.publish(msg);
}


void SensorPidNode::publishVelocity( double velX, double velY )
{
    geometry_msgs::TwistStamped msg;

    msg.twist.linear.x = velX;
    msg.twist.linear.y = velY;

    cmd_vel_pub_.publish(msg);
}
