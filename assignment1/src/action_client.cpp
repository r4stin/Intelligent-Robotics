#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <assignment1/MoveAction.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <iostream>
#include <sstream>
#include <iterator>
#include <string>
#include <vector>

using namespace std;

/////////////////////////////////////////////////////////////
//////////////////////CLIENT CLASS///////////////////////////
/////////////////////////////////////////////////////////////
class ActionClient
{
public:
  // Default, getting the server node
  ActionClient() : ac("server", true)
  {
    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();
    ROS_INFO("Action server started, sending goal.");
  }

  /**
   * Function that asks the server to move the robot to the given 
  */
  void order(int x, int y, int z, float roll, float pitch, float yaw)
  {
    // Initializations
    // Initialize the action
    assignment1::MoveGoal goal;

    // Initialaze needed variables
    geometry_msgs::Pose desired;
    geometry_msgs::Point pointToReach;
    geometry_msgs::Quaternion angleToReach;

    // Initialize the quaternion for the angular positions
    tf2::Quaternion tempQuat;

    // Inserting the requested values to the point 
    pointToReach.x = x;
    pointToReach.y = y;
    pointToReach.z = z;
    // Assigning it to the desired position
    desired.position = pointToReach;

    // Set up the quaternion using fixed axis RPY
    tempQuat.setRPY(roll, pitch, yaw);
    // Convert to quaternions
    tf2::convert(tempQuat, angleToReach);
    // Show it in terminal
    ROS_INFO("Request: %f,%f,%f -- %f,%f,%f,%f", pointToReach.x, pointToReach.y, pointToReach.z, angleToReach.x, angleToReach.y, angleToReach.z, angleToReach.w);
    // Apply the quaternion(orientation) to the desired pose
    desired.orientation = angleToReach;
    // Set the desired pose to the desired pose of the goal
    goal.desired_pose = desired;

    ac.sendGoal(goal,
                boost::bind(&ActionClient::doneCb, this, _1, _2),
                boost::bind(&ActionClient::activeCb, this),
                boost::bind(&ActionClient::feedbackCb, this, _1));
  }

  /**
   * Function that is executed when the action has been completed, that is, when the robot has reached the goal.
   * The function will then show the obstacles that are in the visual space of the robot captured by its laser.
   * Apart from that, it will also show the final state of the robot and the answer of the server.
   * @param state
   * @param result
  */
  void doneCb(const actionlib::SimpleClientGoalState &state,
              const assignment1::MoveResultConstPtr &result)
  {
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
    ROS_INFO("Answer: %d", result->completed);

        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {

            const assignment1::MoveResultConstPtr &result = ac.getResult();
            ROS_INFO("%ld obstacles found.", result->obstacle_positions.size());
            if (result) {
                for (size_t i = 0; i < result->obstacle_positions.size(); ++i) {
                    const auto &obstacle = result->obstacle_positions[i];
                    ROS_INFO("Obstacle %ld position: X = %f, Y = %f", i + 1, obstacle.x, obstacle.y);
                }

            } else {
                ROS_WARN("No result received.");
            }
        } else {
            ROS_WARN("Action did not succeed. State: %s", ac.getState().toString().c_str());
        }
    ros::shutdown();
  }

  /**
   * Function that adverts that the goal has activated
  */
  void activeCb()
  {
    ROS_INFO("Goal just went active");
  }

  /**
   * Function that shows the feedback from the server
   * @param feedback
  */
  void feedbackCb(const assignment1::MoveFeedbackConstPtr &feedback)
  {
    ROS_INFO("Got Feedback, current status of the robot is: %s", feedback->status.c_str());
  }

private:
  actionlib::SimpleActionClient<assignment1::MoveAction> ac;
};

/**
 * Function to get the input from user
 * @return A vector of the elements the user entered
*/
vector<float> getInput()
{
  // Initializing input string
  string s;
  // Getting the line of input
  getline(std::cin, s);
  // Converting it into a separeted stream of inputs
  istringstream is(s);
  // Converting it into a iterable vector
  vector<float> v((std::istream_iterator<float>(is)), std::istream_iterator<float>());
  
  return v;
};

int main(int argc, char **argv)
{
  vector<float> input;
  ROS_INFO("Insert the coordinates of the desired position for the robot to move, separated with spaces:  x y z roll pitch yaw \n");
  // Take the input from the user, until it is correctly entered
  do{
    cout << "Enter here: ";
    // Call to get the input in one line
    input = getInput();
    // Check if there are sufficient input numbers that respresent the coordinates
    if(input.size() != 6){
      ROS_INFO("Usage for client : Please use the format -> x y z roll pitch yaw \n");
      ROS_INFO("Please try again... \n");
    }
  } while(input.size() != 6);
  
  // Initialize node
  ros::init(argc, argv, "action_client");
  ActionClient client;
  //Call for the client to make a request to server for the robot to move to the desired position
  client.order(input[0], input[1], input[2], input[3], input[4], input[5]);
  ros::spin();
  return 0;
}
