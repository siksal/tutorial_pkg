#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <fstream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;
using namespace std;

// Files for recording odometry trajectory, odometry velocity, laser range data, and laser map
ofstream odomTrajFile;
ofstream odomVelFile;
ofstream laserFile; // file object for recording your laser data.
ofstream laserMapFile;

struct EulerAngles
{
  double roll, pitch, yaw;
};

struct Quaternion
{
  double w, x, y, z;
};

// get the robot heading, Th, by changing the orientation from quaternion to euler angles
EulerAngles ToEulerAngles(Quaternion q)
{ // for calculating Th
  EulerAngles angles;
  // roll (x-axis rotation)
  double sinr_cosp = +2.0 * (q.w * q.x + q.y * q.z);
  double cosr_cosp = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);
  angles.roll = atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  double sinp = +2.0 * (q.w * q.y - q.z * q.x);
  if (fabs(sinp) >= 1)
    angles.pitch = copysign(M_PI/2, sinp); //use 90 degrees if out of range
  else
    angles.pitch = asin(sinp);

  // yaw (z-axis rotation)
  double siny_cosp = +2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  angles.yaw = atan2(siny_cosp, cosy_cosp);
  return angles;
}


class Stopper : public rclcpp::Node
{
  public:
    // velocity control variables
    constexpr const static double FORWARD_SPEED_LOW = 0.1;
    constexpr const static double FORWARD_SPEED_MIDDLE = 0.3;
    constexpr const static double FORWARD_SPEED_HIGH = 0.5;
    constexpr const static double FORWARD_SPEED_STOP = 0;
    constexpr const static double TURN_LEFT_SPEED_LOW = 0.3;
    constexpr const static double TURN_LEFT_SPEED_MIDDLE = 0.6;
    constexpr const static double TURN_LEFT_SPEED_HIGH = 1.0;
    constexpr const static double TURN_RIGHT_SPEED_LOW = -0.3;
    constexpr const static double TURN_RIGHT_SPEED_MIDDLE = -0.6;
    constexpr const static double TURN_RIGHT_SPEED_HIGH = -1.0;

    // class constructor to set the speed publisher. odometry subscription, and subscription to laser data
    Stopper():Node("Stopper"), count_(0)
    {
      publisher_=this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
      odomSub_=this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&Stopper::odomCallback, this, std::placeholders::_1));
      laserScan_=this->create_subscription<sensor_msgs::msg:: LaserScan> ("scan", 10, std:: bind(&Stopper::scanCallback, this, std::placeholders::_1));
    };

    // functions to move robot
    void startMoving();
    void moveStop();
    void moveForward(double forwardSpeed);
    void moveRight(double turn_right_speed);
    void moveForwardRight(double forwardSpeed, double turn_right_speed);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odomMsg);
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);

    void transformMapPoint(ofstream& fp, double laserRange, double laserTh, double robotTh, double robotX, double robotY);

    double PositionX=0.3, PositionY=0.3, homeX=0.3, homeY=0.3;
    double robVelocity;
    int numberOfCycle=0;
    Quaternion robotQuat;
    EulerAngles robotAngles;
    double robotHeadAngle;
    double leftAngle=M_PI/2, mleftAngle=M_PI/4, frontAngle=0;
    double mrightAngle=-M_PI/4, rightAngle=-M_PI/2;
    double laser_landmark1 = 1.3, laser_landmark2 = 1.4, laser_landmark3 = 0.6, laser_landmark4 = 1.2, laser_landmark5 = 0.4;
    double frontRange, mleftRange, leftRange, rightRange, mrightRange;
    int laser_index = 0;
    int stage = 1;
    
  private:
    // Publisher to the robot's velocity command topic
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;
    //Subscriber to robot’s odometry topic
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserScan_;
};

void Stopper::moveStop()
{
  auto msg = geometry_msgs::msg::Twist();
  msg.linear.x = FORWARD_SPEED_STOP;
  publisher_->publish(msg);
}

void Stopper::moveForward(double forwardSpeed)
{
  //The default constructor to set all commands to 0
  auto msg=geometry_msgs::msg::Twist();
  //Drive forward at a given speed along the x-axis.
  msg.linear.x = forwardSpeed;
  publisher_->publish(msg);
}

void Stopper::moveRight(double turn_right_speed)
{
  auto msg = geometry_msgs::msg::Twist();
  msg.angular.z = turn_right_speed;
  publisher_->publish(msg);
}

void Stopper::startMoving()
{
  odomTrajFile.open("/home/olamilekan/ros_workspace/src/tutorial_pkg/odomTrajData.csv");
  odomVelFile.open("/home/olamilekan/ros_workspace/src/tutorial_pkg/odomVelData.csv");
  laserFile.open("/home/olamilekan/ros_workspace/src/tutorial_pkg/laserData.csv");
  laserMapFile.open("/home/olamilekan/ros_workspace/src/tutorial_pkg/laserMapData.csv");

  odomTrajFile << "PositionX, PositionY" << endl;
  odomVelFile << "number of cycle, robot velocity" << endl;
  laserFile << "laser_index++, leftRange, mleftRange, frontRange, mrightRange, rightRange" << endl;
  laserMapFile << "transX, transY" << endl;

  RCLCPP_INFO(this->get_logger(), "Start moving");
  rclcpp::WallRate loop_rate(10);
  while (rclcpp::ok())
  {
    auto node = std::make_shared<Stopper>();
    rclcpp::spin(node); // update
    loop_rate.sleep(); // wait delta time
  }
  odomTrajFile.close();
  odomVelFile.close();
  laserFile.close();
  laserMapFile.close();
}

void Stopper::moveForwardRight(double forwardSpeed, double turn_right_speed)
{
  auto msg = geometry_msgs::msg::Twist();
  msg.linear.x = forwardSpeed;
  msg.angular.z = turn_right_speed;
  publisher_->publish(msg);
}

void Stopper::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odomMsg)
{
  PositionX = odomMsg->pose.pose.position.x + homeX;
  PositionY = odomMsg->pose.pose.position.y + homeY;

  robotQuat.x = odomMsg->pose.pose.orientation.x;
  robotQuat.y = odomMsg->pose.pose.orientation.y;
  robotQuat.z = odomMsg->pose.pose.orientation.z;
  robotQuat.w = odomMsg->pose.pose.orientation.w;
  robotAngles = ToEulerAngles(robotQuat);
  robotHeadAngle = robotAngles.yaw;

  RCLCPP_INFO(this->get_logger(),"RobotPostion: %.2f , %.2f",PositionX, PositionY );
  RCLCPP_INFO(this->get_logger(), "Robot stage: %d ", stage );
  odomTrajFile << PositionX << "," << PositionY << endl;
  robVelocity = odomMsg->twist.twist.linear.x;
  odomVelFile << numberOfCycle++ << "," << robVelocity << endl;
}

void Stopper::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  // get laser scan range at different index
  leftRange = scan->ranges[300]; // get a range reading at the left angle
  mleftRange = scan->ranges[250]; // get a range reading at the front-left angle
  frontRange = scan->ranges[200]; // get a range reading at the front angle
  mrightRange = scan->ranges[150]; // get a range reading at the front-right angle
  rightRange = scan->ranges[100]; // get the range reading at the right angle

  // output range data to laserData.csv
  laserFile << laser_index++ << "," << leftRange << ","<< mleftRange << "," << frontRange<<"," << mrightRange << "," << rightRange << endl;

  // transfor laser range for front, mid-left, left, right, and mid-right ranges respectively and store ranges in laserMapData.csv
  transformMapPoint(laserMapFile,frontRange,frontAngle,robotHeadAngle, PositionX, PositionY);
  transformMapPoint(laserMapFile, mleftRange, mleftAngle, robotHeadAngle, PositionX, PositionY);
  transformMapPoint(laserMapFile, leftRange, leftAngle, robotHeadAngle, PositionX, PositionY);
  transformMapPoint(laserMapFile, rightRange, rightAngle, robotHeadAngle, PositionX, PositionY);
  transformMapPoint(laserMapFile, mrightRange, mrightAngle, robotHeadAngle, PositionX, PositionY);

  // move robot to goal point based on landmark data and laser range readings.
  switch(stage)
  {
    case 1:
     if (frontRange > laser_landmark1)
      moveForward(FORWARD_SPEED_MIDDLE);
     else stage = 2;
      break;
    case 2:
     if (mleftRange < laser_landmark2)
      moveForwardRight(FORWARD_SPEED_MIDDLE, TURN_RIGHT_SPEED_MIDDLE);
     else stage = 3;
      break;
    case 3:
     if (frontRange > laser_landmark3)
      moveForward(FORWARD_SPEED_HIGH);
     else stage = 4;
      break;
    case 4:
     if (frontRange < laser_landmark4)
      moveForwardRight( FORWARD_SPEED_MIDDLE, TURN_RIGHT_SPEED_MIDDLE );
     else stage = 5;
      break;
    case 5:
     if (frontRange > laser_landmark5)
      moveForward(FORWARD_SPEED_MIDDLE);
     else stage = 6;
      break;
    case 6:
     moveStop();
     break;
  }
}

void Stopper::transformMapPoint(ofstream& fp, double laserRange, double laserTh, double robotTh, double robotX, double robotY)
{
  // transform laser points from polar coordinates in sensor frame to cartesian coordinates in global frame
  double transX, transY;
  transX = laserRange * cos(robotTh + laserTh) + robotX;
  transY = laserRange * sin(robotTh + laserTh) + robotY;
  if (transX < 0) transX = 0;
  if (transY < 0) transY = 0;

  // send laser point data to laserMap.csv
  fp << transX << ", " << transY << endl;
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  Stopper stopper;
  stopper.startMoving();
  return 0;
}