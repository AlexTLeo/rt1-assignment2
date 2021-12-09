#include <unistd.h>
#include <termios.h>

#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "std_srvs/SetBool.h"

/**
 * Author: Alex Thanaphon Leonardi
 *
 * This node accepts user input to change the robot drive speed or reset the
 * robot's position to the start of the circuit.
 */

/// detects user key presses, without waiting for newline (ENTER)
int detectKeyPress();

int main (int argc, char **argv) {
  ros::init(argc, argv, "rt1-assignment2_ui");

  ros::NodeHandle nh;
  ros::ServiceClient clientReset;
  ros::ServiceClient clientVel;

  std_srvs::Empty srvReset;
  std_srvs::SetBool srvVel; // true -> increase velocity, false -> decrease velocity

  // Even though a srv could be sent directly to the stageros node, it is
  // preferred to send it to the controller so as to have one central node for
  // communications
  clientReset = nh.serviceClient<std_srvs::Empty>("/controller_reset", 1);
  clientVel = nh.serviceClient<std_srvs::SetBool>("/controller_vel", 1);

  while (ros::ok()) {
    switch(detectKeyPress()) {
      case 119:
        // "w" key: increase velocity
        srvVel.request.data = true;
        clientVel.waitForExistence();
        if (clientVel.call(srvVel)) {
          ROS_INFO("Message sent: increase velocity");
          ROS_INFO("%s", srvVel.response.message.c_str());
        } else {
          ROS_ERROR("Velocity increase message failed to send");
        }
        break;
      case 115:
        // "s" key: lower velocity
        srvVel.request.data = false;
        clientVel.waitForExistence();
        if (clientVel.call(srvVel)) {
          ROS_INFO("Message sent: decrease velocity");
          ROS_INFO("%s", srvVel.response.message.c_str());
        } else {
          ROS_ERROR("Velocity decrease message failed to send");
        }
        break;
      case 114:
        // "r" key: RESET
        clientReset.waitForExistence();
        if (clientReset.call(srvReset)) {
          ROS_INFO("Message sent: RESET");
        } else {
          ROS_ERROR("RESET message failed to send");
        }
        break;
      default:
        break;
    }

    ros::spinOnce();
  }

  return 0;
}

int detectKeyPress () {
  int input;
  struct termios orig_term_attr;
  struct termios new_term_attr;

  /* set the terminal to raw mode rather than canonical mode */
  tcgetattr(fileno(stdin), &orig_term_attr);
  memcpy(&new_term_attr, &orig_term_attr, sizeof(struct termios));
  new_term_attr.c_lflag &= ~(ECHO|ICANON);
  new_term_attr.c_cc[VTIME] = 0;
  new_term_attr.c_cc[VMIN] = 1;
  tcsetattr(fileno(stdin), TCSANOW, &new_term_attr);

  /* read a character from the stdin stream without blocking */
  /* returns EOF (-1) if no character is available */
  input = getchar();

  /* restore the original terminal attributes */
  tcsetattr(fileno(stdin), TCSANOW, &orig_term_attr);

  return input;
}
