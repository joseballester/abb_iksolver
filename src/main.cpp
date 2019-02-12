#include <vector>
#include <string>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/types.h>

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
  int sock;
  if ((sock = socket(PF_INET, SOCK_STREAM, 0)) == -1) {
    ROS_INFO("Problem creating the socket (error %d). Exiting.", errno);
    return -1;
  } else {
    // Now try to connect to the robot
    struct sockaddr_in remoteSocket;
    remoteSocket.sin_family = AF_INET;
    remoteSocket.sin_port = htons(port);
    inet_pton(AF_INET, ip.c_str(), &remoteSocket.sin_addr.s_addr);
    if(connect(sock, (sockaddr*)&remoteSocket, sizeof(remoteSocket)) == -1)
    {
      ROS_INFO("Problem during connection (error %d). Exiting.", errno);
      return -1;
    }
  }

  char message[MAX_BUFFER], reply[MAX_BUFFER];
  int replySize, response;
  double j1, j2, j3, j4, j5, j6, j7;

  ROS_INFO("Node started.");

  while(ros::ok()) {
    ros::spinOnce();
    try {
      // Prepare message and send
      sprintf(message, "%08.1lf %08.1lf %08.1lf %08.5lf %08.5lf %08.5lf %08.5lf #", lastPose.pose.position.x, lastPose.pose.position.y, lastPose.pose.position.z, lastPose.pose.orientation.w, lastPose.pose.orientation.x, lastPose.pose.orientation.y, lastPose.pose.orientation.z);
      if (send(sock, message, strlen(message), 0) == -1) {
        ROS_ERROR("Failed to send message. Exiting.");
        return -1;
      }

      // Receive message
      if((replySize = recv(sock, reply, MAX_BUFFER-1, 0)) <= 0) {
        ROS_ERROR("Failed to receive message. Exiting.");
        return -1;
      }

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
