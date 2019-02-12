#include <vector>
#include <string>
#include <boost/asio.hpp>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>

#define MAX_BUFFER 1000

// Target frequency
double hz = 250.0;

// Configuration variables
std::string ip;
int port, numberJoints;
double workobjectX, workobjectY, workobjectZ, workobjectQ0, workobjectQX, workobjectQY, workobjectQZ;
double toolX, toolY, toolZ, toolQ0, toolQX, toolQY, toolQZ;
double toolMass, toolCGX, toolCGY, toolCGZ, toolIX, toolIY, toolIZ;

// ROS input and output
geometry_msgs::PoseStamped lastPose;
sensor_msgs::JointState lastJoints;

// Communication
char message[MAX_BUFFER], reply[MAX_BUFFER];
std::string strMsg, strReply;

// Socket
boost::asio::io_service io_service;
boost::asio::ip::tcp::socket sock(io_service);
boost::system::error_code error;

bool socketSend()
{
  strMsg = message;
  boost::asio::write(sock, boost::asio::buffer(strMsg), error);
  if (error) {
    ROS_WARN("Send failed: %s", error.message().c_str());
    return false;
  }
  return true;
}

bool socketReceive()
{
  boost::asio::streambuf receive_buffer;
  boost::asio::read_until(sock, receive_buffer, "\n");
  if (error && error != boost::asio::error::eof) {
    ROS_WARN("Received failed: %s", error.message().c_str());
    return false;
  } else {
    strReply = boost::asio::buffer_cast<const char*>(receive_buffer.data());
    strcpy(reply, strReply.c_str());
  }
  return true;
}

void loadPose(const geometry_msgs::PoseStamped& pose) {
  lastPose = pose;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "abb_ik");
  ros::NodeHandle node;

  // Read configuration
  std::string robotId = (argc == 2 ? argv[1] : "");
  std::string root = "/robot" + robotId;
  node.getParam(root + "/ip", ip);
  node.getParam(root + "/port", port);
  node.getParam(root + "/joints", numberJoints);
  node.getParam(root + "/workobjectX", workobjectX);
  node.getParam(root + "/workobjectY", workobjectY);
  node.getParam(root + "/workobjectZ", workobjectZ);
  node.getParam(root + "/workobjectQ0", workobjectQ0);
  node.getParam(root + "/workobjectQX", workobjectQX);
  node.getParam(root + "/workobjectQY", workobjectQY);
  node.getParam(root + "/workobjectQZ", workobjectQZ);
  node.getParam(root + "/toolX", toolX);
  node.getParam(root + "/toolY", toolY);
  node.getParam(root + "/toolZ", toolZ);
  node.getParam(root + "/toolQ0", toolQ0);
  node.getParam(root + "/toolQX", toolQX);
  node.getParam(root + "/toolQY", toolQY);
  node.getParam(root + "/toolQZ", toolQZ);
  node.getParam(root + "/toolMass", toolMass);
  node.getParam(root + "/toolCGX", toolCGX);
  node.getParam(root + "/toolCGY", toolCGY);
  node.getParam(root + "/toolCGZ", toolCGZ);
  node.getParam(root + "/toolIX", toolIX);
  node.getParam(root + "/toolIY", toolIY);
  node.getParam(root + "/toolIZ", toolIZ);

  if(numberJoints != 6 and numberJoints != 7) {
    ROS_ERROR("Unsupported number of joints. Exiting.");
    return -1;
  }

  // Target working frequency
  ros::Rate rate(hz);

  // ROS side
  ros::Subscriber poseHandler = node.subscribe(root + "_ik_pose", 1, loadPose);
  ros::Publisher jointsHandler = node.advertise<sensor_msgs::JointState>(root + "_ik_joints", 1);

  // Setting up TCP socket
  sock.connect(boost::asio::ip::tcp::endpoint(boost::asio::ip::address::from_string(ip), port));

  int replySize, response;
  double j1, j2, j3, j4, j5, j6, j7;

  ROS_INFO("Node started.");

  while(ros::ok()) {
    ros::spinOnce();
    try {
      // Prepare message and send
      sprintf(message, "%08.1lf %08.1lf %08.1lf %08.5lf %08.5lf %08.5lf %08.5lf #", lastPose.pose.position.x, lastPose.pose.position.y, lastPose.pose.position.z, lastPose.pose.orientation.w, lastPose.pose.orientation.x, lastPose.pose.orientation.y, lastPose.pose.orientation.z);
      socketSend();

      // Receive message
      socketReceive();

      // Parse received message
      if(numberJoints == 7)
        sscanf(reply, "%d %lf %lf %lf %lf %lf %lf %lf #", &response, &j1, &j2, &j3, &j4, &j5, &j6, &j7);
      else
        sscanf(reply, "%d %lf %lf %lf %lf %lf %lf #", &response, &j1, &j2, &j3, &j4, &j5, &j6);

      // Publish received info
      switch(response) {
        case 0:
          // Invalid or empty pose, no error shown
          break;
        case 1:
          lastJoints.header.stamp = ros::Time::now();
          lastJoints.position = {j1, j2, j3, j4, j5, j6, j7};
          jointsHandler.publish(lastJoints);
          break;
        case 2:
          ROS_WARN("Wrong number of parameters received by robot.");
          break;
        case 1075:
          ROS_WARN("Limit error. Try another pose.");
          break;
        case 1136:
          ROS_WARN("Target is outside robot working area.");
          break;
        default:
          ROS_WARN("Error number %d.", response);
      }
    }
    catch (int e) {
      ROS_INFO("Exception: %d", e);
      break;
    }
    rate.sleep();
  }
  return 0;
}
