#include <boost/bind.hpp>
#include <string>
#include <ros/ros.h>
#include <turtlesim/Pose.h>
//#include <turtlesim/Velocity.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>
#include <std_srvs/Empty.h>
#include <gateway_msgs/GatewayInfo.h>

turtlesim::PoseConstPtr g_pose;
turtlesim::Pose g_goal;
double x_vel = 1.0;
double z_vel = 0.4;
double square_scale = 1.0;

enum State
{
  FORWARD,
  STOP_FORWARD,
  TURN,
  STOP_TURN,
};

State g_state = FORWARD;
State g_last_state = FORWARD;
bool g_first_goal_set = false;

#define PI 3.141592

void poseCallback(const turtlesim::PoseConstPtr& pose)
{
  g_pose = pose;
}

bool hasReachedGoal()
{
  return fabsf(g_pose->x - g_goal.x) < 0.1 && fabsf(g_pose->y - g_goal.y) < 0.1 && fabsf(g_pose->theta - g_goal.theta) < 0.01;
}

bool hasStopped()
{
  return g_pose->angular_velocity < 0.0001 && g_pose->linear_velocity < 0.0001;
}

void printGoal()
{
  ROS_INFO("New goal [%f %f, %f]", g_goal.x, g_goal.y, g_goal.theta);
}

void commandTurtle(ros::Publisher twist_pub, float linear, float angular)
{
  //turtlesim::Velocity vel;
  geometry_msgs::Twist twist;
  twist.linear.x = linear;
  twist.angular.z = angular;
  twist_pub.publish(twist);
}

void stopForward(ros::Publisher twist_pub)
{
  if (hasStopped())
  {
    ROS_INFO("Reached goal");
    g_state = TURN;
    g_goal.x = g_pose->x;
    g_goal.y = g_pose->y;
    g_goal.theta = fmod(g_pose->theta + PI/2.0, 2*PI);
    printGoal();
  }
  else
  {
    commandTurtle(twist_pub, 0, 0);
  }
}

void stopTurn(ros::Publisher twist_pub)
{
  if (hasStopped())
  {
    ROS_INFO("Reached goal");
    g_state = FORWARD;
    g_goal.x = cos(g_pose->theta) * 2 * square_scale + g_pose->x;
    g_goal.y = sin(g_pose->theta) * 2 * square_scale + g_pose->y;
    g_goal.theta = g_pose->theta;
    printGoal();
  }
  else
  {
    commandTurtle(twist_pub, 0, 0);
  }
}


void forward(ros::Publisher twist_pub)
{
  if (hasReachedGoal())
  {
    g_state = STOP_FORWARD;
    commandTurtle(twist_pub, 0, 0);
  }
  else
  {
    commandTurtle(twist_pub, x_vel, 0.0);
  }
}

void turn(ros::Publisher twist_pub)
{
  if (hasReachedGoal())
  {
    g_state = STOP_TURN;
    commandTurtle(twist_pub, 0, 0);
  }
  else
  {
    commandTurtle(twist_pub, 0.0, z_vel);
  }
}

void timerCallback(const ros::TimerEvent&, ros::Publisher twist_pub)
{
  if (!g_pose)
  {
    return;
  }

  if (!g_first_goal_set)
  {
    g_first_goal_set = true;
    g_state = FORWARD;
    g_goal.x = cos(g_pose->theta) * 2 * square_scale + g_pose->x;
    g_goal.y = sin(g_pose->theta) * 2 * square_scale + g_pose->y;
    g_goal.theta = g_pose->theta;
    printGoal();
  }

  if (g_state == FORWARD)
  {
    forward(twist_pub);
  }
  else if (g_state == STOP_FORWARD)
  {
    stopForward(twist_pub);
  }
  else if (g_state == TURN)
  {
    turn(twist_pub);
  }
  else if (g_state == STOP_TURN)
  {
    stopTurn(twist_pub);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "turtle_stroll");
  std::string simulation_namespace;
  ros::NodeHandle pnh("~");
  if ( !pnh.getParam("simulation_namespace", simulation_namespace) ) {
    ROS_WARN_STREAM("TurtleStroll: simulation_namespace parameter not set.");
  }
  if ( !pnh.getParam("turtle_x_vel", x_vel) ) {
    ROS_WARN_STREAM("TurtleStroll: turtle_x_vel parameter not set.");
  }
  if ( !pnh.getParam("turtle_z_vel", z_vel) ) {
    ROS_WARN_STREAM("TurtleStroll: turtle_z_vel parameter not set.");
  }
  if ( !pnh.getParam("square_scale", square_scale) ) {
    ROS_WARN_STREAM("TurtleStroll: square_size parameter not set.");
  }

  ros::NodeHandle snh(simulation_namespace);
  ros::Subscriber pose_sub = snh.subscribe("pose", 1, poseCallback);
  ros::Publisher twist_pub = snh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  ros::NodeHandle nh;
  ros::Timer timer = nh.createTimer(ros::Duration(0.016), boost::bind(timerCallback, _1, twist_pub));

  ros::spin();
}
