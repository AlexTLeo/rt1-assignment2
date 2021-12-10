#include <string>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "std_srvs/Empty.h"
#include "std_srvs/SetBool.h"

/**
 * Author: Alex Thanaphon Leonardi
 *
 * This controller works with the stageros node to provide the robot with
 * autonomous guidance around the rt1-assignment2/world/my_world.world circuit.
 */

/// Represents average distances of obstacles around the robots
struct scanInfo {
  float distObstacleLeft;
  float distObstacleLeftFront;
  float distObstacleRightFront;
  float distObstacleRight;
};

/**
 * Calculates new robot velocities based on obstacle distance
 */
void calculateVelocities(float &driveSpeed, float &turnSpeed, float distance);

/**
 * Scales the velocity of the robot based on the distance of the obstacle, using
 * the function: kp*velocity*(1/exp(abs(velocity)))
 * @param  distance - distance to obstacle
 * @param  velocity - velocity of robot
 * @param  kp - proportional constant
 * @return float, scaled value
 */
float scaleTurnVelocity(float distance, float velocity, float kp);

/**
 * Resets robot position back to start
 */
bool resetPosition(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

/**
 * Changes robot velocity based on boolean:
 * TRUE -> increase robot velocity
 * FALSE -> decrease robot velocity
 */
bool changeVelocity(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

/**
 * Moves robot forward with given velocity
 * @param velocity float
 */
void drive(float velocity);

/**
 * Turns robot with given speed (robot does not stop, smooth turning)
 * @param turnVelocity float
 * @param driveVelocity float
 * @param left bool - if true, turns left, if false, turns right
 */
void turn(float turnVelocity, float driveVelocity, bool left);

/**
 * Get info from laser scanner on obstacles surrounding the robot
 * @return a struct of two elements which corresponds to the distances of the
 * closest obstacles on the left (0) and on the right (1) of the robot.
 */
void baseScanParser(const sensor_msgs::LaserScan::ConstPtr &scaninfo);

/**
 * Prints a prettier message to screen and also logs everything to ROS logs
 * (assuming logging through ROS_INFO is enabled)
 * @param message
 */
void printMessage(std::string message);

/// Contains the average distance of obstacles around the robot
scanInfo scanInfo = {0, 0};

/// Publisher for robot movement (topic: cmd_vel)
ros::Publisher pubVel;
// Service client for robot position reset ("/reset_positions")
ros::ServiceClient clientReset;

float robotBaseVelocity = 1;
float robotRealVelocityDrive = 1; /// after it has been scaled
float robotRealVelocityTurn = 1; /// after it has been scaled
const float kp = 10; /// used for scaleTurnVelocity

/// distance at which robot will brake to avoid collision
const float BRAKE_THRESHOLD = 0.4f;

/// previous message that has been printed by robot
std::string messagePrevious = "";

/// used to limit terminal printing to 10 times a second
ros::Time timeOld;


int main (int argc, char **argv) {
  ros::init(argc, argv, "rt1-assignment2_controller");

  ros::NodeHandle nh;
  ros::Rate loopRate(50);
  ros::ServiceServer serverReset; // Receives from ui node
  ros::ServiceServer serverVel; // Receives from ui node

  timeOld = ros::Time::now();

  // Passing obstacle info to parser
  ros::Subscriber sub = nh.subscribe("/base_scan", 1, baseScanParser);

  pubVel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  clientReset = nh.serviceClient<std_srvs::Empty>("/reset_positions");

  // Servers for UI node
  serverReset = nh.advertiseService("/controller_reset", resetPosition);
  serverVel = nh.advertiseService("/controller_vel", changeVelocity);

  // Obstacle avoidance
  while (ros::ok()) {
    float dL = scanInfo.distObstacleLeft;
    float dLF = scanInfo.distObstacleLeftFront;
    float dRF = scanInfo.distObstacleRightFront;
    float dR = scanInfo.distObstacleRight;
    float driveSpeed = robotBaseVelocity;
    float turnSpeed = robotBaseVelocity;

    // Robot turn/drive speeds are proportional to "urgency" of obstacle distance
    if (dL <= 0.7f) {
      // Sector LEFT
      calculateVelocities(driveSpeed, turnSpeed, dL);
      turn(turnSpeed, driveSpeed, true);
      printMessage("Turning RIGHT");
    } else if (dR <= 0.7f) {
      // Sector RIGHT
      calculateVelocities(driveSpeed, turnSpeed, dR);
      turn(turnSpeed, driveSpeed, false);
      printMessage("Turning LEFT");
    } else if (dLF <= 1.8f) {
      // Sector LEFT-FRONT
      calculateVelocities(driveSpeed, turnSpeed, dLF);
      turn(turnSpeed, driveSpeed, true);
      printMessage("Turning RIGHT");
    } else if (dRF <= 1.8f) {
      // Sector RIGHT-FRONT
      calculateVelocities(driveSpeed, turnSpeed, dRF);
      turn(turnSpeed, driveSpeed, false);
      printMessage("Turning LEFT");
    } else {
      // Drive STRAIGHT
      drive(driveSpeed);
      robotRealVelocityDrive = driveSpeed;

      printMessage("Driving");
    }

    ros::spinOnce();
    loopRate.sleep();
  }
  return 0;
}

// --- //

void calculateVelocities(float &driveSpeed, float &turnSpeed, float distance) {
  turnSpeed = scaleTurnVelocity(distance, robotBaseVelocity, kp);

  if (distance <= BRAKE_THRESHOLD) {
    driveSpeed = 0;
    printMessage("Too close to wall, braking!");
  } else {
    // proportional to distance
    driveSpeed = robotBaseVelocity*distance;
  }

  robotRealVelocityDrive = driveSpeed;
  robotRealVelocityTurn = turnSpeed;
}

float scaleTurnVelocity(float distance, float velocity, float kp) {
  return kp*velocity*(1/exp(abs(distance)));
}

bool resetPosition(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
  std_srvs::Empty srvReset;
  clientReset.waitForExistence();
  clientReset.call(srvReset);
  printMessage("Received command: RESET");

  return true;
}

bool changeVelocity(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
  if (req.data) {
    robotBaseVelocity *= 2;
    printMessage("Received command: velocity increase");
  } else {
    robotBaseVelocity /= 2;
    printMessage("Received command: velocity decrease");
  }

  return true;
}

void baseScanParser(const sensor_msgs::LaserScan::ConstPtr &scaninfo) {
  // Divide into 4 sectors
  int scanLength = scaninfo->ranges.size();
  float scanSectorLength = scanLength/4;
  float leftDistAvg = 0;
  float leftFrontDistAvg = 0;
  float rightDistAvg = 0;
  float rightFrontDistAvg = 0;
  int leftCount = 0;
  int leftFrontCount = 0;
  int rightCount = 0;
  int rightFrontCount = 0;

  for (int i = 0; i < scanLength; i++) {
    if (i < scanSectorLength) {
      leftDistAvg += scaninfo->ranges[i];
      leftCount++;
    } else if (i < scanSectorLength*2) {
      leftFrontDistAvg += scaninfo->ranges[i];
      leftFrontCount++;
    } else if (i < scanSectorLength*3) {
      rightFrontDistAvg += scaninfo->ranges[i];
      rightFrontCount++;
    } else {
      rightDistAvg += scaninfo->ranges[i];
      rightCount++;
    }
  }

  leftDistAvg /= leftCount;
  leftFrontDistAvg /= leftFrontCount;
  rightFrontDistAvg /= rightFrontCount;
  rightDistAvg /= rightCount;

  scanInfo.distObstacleLeft = leftDistAvg;
  scanInfo.distObstacleLeftFront = leftFrontDistAvg;
  scanInfo.distObstacleRightFront = rightFrontDistAvg;
  scanInfo.distObstacleRight = rightDistAvg;
}

void drive(float velocity) {
  geometry_msgs::Twist vel;

  vel.linear.x = velocity;
  pubVel.publish(vel);
}

void turn(float turnVelocity, float driveVelocity, bool left) {
  geometry_msgs::Twist vel;

  vel.linear.x = driveVelocity;
  if (left) {
    // Turn LEFT
    vel.angular.z = turnVelocity;
  } else {
    // Turn RIGHT
    vel.angular.z = -turnVelocity;
  }

  pubVel.publish(vel);
}

void printMessage(std::string message) {
  // Only run 10 times a second, to avoid annoying text flickering
  ros::Time timeNow = ros::Time::now();
  double timeDiffNano = timeNow.nsec - timeOld.nsec;

  if (timeDiffNano >= 100000000) {
    // 100 milliseconds have passed...
    timeOld = ros::Time::now();

    printf("\033[2J\033[1;1H\n");
    system("clear");
    printf("\033[92m\n");
    printf(" /=====================\\\n");
    printf(" | ROS ROBOT SIMULATOR |\n");
    printf(" \\=====================/\n");
    printf("\033[96m\n");
    fflush(stdout);

    printf(" Robot: ");
    ROS_INFO("%s", message.c_str());
    printf("\n Current Velocity (m/s): %.2f", robotRealVelocityDrive);
    printf("\n Current Base Velocity (m/s): %.2f", robotBaseVelocity);
    fflush(stdout);

    if (!messagePrevious.empty()) {
      printf("\033[36m\n");
      printf("\n Previous Message: %s\n", messagePrevious.c_str());
      fflush(stdout);
    }

    printf("\033[31m\n");
    printf("\n R : reset position  |  W : velocity +  |  S : velocity —");

    fflush(stdout);
    messagePrevious = message;
  }
}
