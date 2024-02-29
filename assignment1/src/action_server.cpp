#include <ros/ros.h>
#include <thread>        // std::this_thread::sleep_for
#include <chrono>         // std::chrono::milliseconds
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <assignment1/MoveAction.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h" //check if correct
#include "actionlib_msgs/GoalStatus.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include "move_base_msgs/MoveBaseGoal.h"
#include <sensor_msgs/LaserScan.h>

using namespace std;

class MoveAction {
protected:
    /*** Declaration of variables ***/


    // The robot status
    int rob_status_;
    std::string current_status;

    // Node Handler
    ros::NodeHandle nh_;

    // Action, action components and server class
    actionlib::SimpleActionServer <assignment1::MoveAction> as_;
    actionlib::SimpleActionClient <move_base_msgs::MoveBaseAction> movebase_client_;
    assignment1::MoveFeedback feedback_;
    assignment1::MoveResult result_;

    // Goal to be published
    move_base_msgs::MoveBaseGoal goal_to_publish;

    // Name of the ongoing action
    std::string action_name_;

    // Boolean to signal narrow passage
    bool narrow_passage = true;
    // Boolean to show if the robot find any obstacles or not
    bool obstacle_found = false;
    //Subscribe to the laserscan message
    ros::Subscriber sub;
    //Initialize the publisher for velocity control
    ros::Publisher vel_pub;
    //Boolean to show if the robot has reached the final pose
    bool reached_final_pose = false;

public:
    // Initialization of server action (constructor)
    MoveAction(std::string name) : as_(nh_, name, boost::bind(&MoveAction::executeCB, this, _1),
                                       false),
                                   movebase_client_("move_base", true), action_name_(name) {
        as_.start();
    }

    ~MoveAction(void) {}

    /**
     * Function that updates the robots status to the parameters depending on the integer value given
     * @param val integer value of the robot status
     */
    void updateStatus(int val) {
        // Update parameters depending on the integer value
        rob_status_ = val;
        switch (val) {
            case -2:
                current_status = "Cancel request has been received.";
                break;
            case -1:
                current_status = "Goal cannot be reached, robot has aborted the operations.";
                break;
            case 1:
                current_status = "Goal has been received, transmitting to the robot.";
                break;
            case 2:
                current_status = "Moving through the narrow passage";
                break;
            case 3:
                current_status = "Robot has left the narrow passage";
                break;
            case 4:
                current_status = "Robot is moving to reach the desired pose.";
                break;
            case 5:
                current_status = "Robot has reached the desired pose.";
                break;
            case 6:
                current_status = "Robot is scanning the environment to detect the movable obstacles.";
                break;
            case 7:
                current_status = "Obstacle detection has been completed, sending the results back.";
                break;
            case 8:
                current_status = "Going through narrow passage.";
                break;
        }
        // Update the feedback for the user
        feedback_.status = current_status;
        // Send it to user
        as_.publishFeedback(feedback_);
    }

    /**
     * Function that is executed when the robot movement status is updated(published in 'move_base/status')
     * @param msg the message published of the status
     */
    void movebaseCallback(const actionlib_msgs::GoalStatusArray::ConstPtr &msg) {
        //ROS_INFO("Received feedback");
        // Initialize variable for the status list and get the value
        std::vector <actionlib_msgs::GoalStatus> status_list;
        status_list = msg->status_list;
        // If the status is updated in publish
        if (status_list.size() > 0) {
            // Display status list
            //ROS_INFO("Status list length: %ld", status_list.size());
            //ROS_INFO("I heard: [%s]", status_list[status_list.size() - 1].text.c_str());
            // If the new status is that the robot can not reach the point
            if (status_list[status_list.size() - 1].text.compare(
                    std::string("Failed to find a valid plan. Even after executing recovery behaviors.")) == 0) {
                updateStatus(-1);
            }
        }
    }

    /**
     * This function takes the feedback from the robot's movement and displays it
     * @param feed
     */
    void movebaseDisplayFeed(const move_base_msgs::MoveBaseFeedback::ConstPtr &feed) {
        // Initialize needed variables
        geometry_msgs::PoseStamped reached_pose_stamped = feed->base_position;
        geometry_msgs::Pose reached_pose = reached_pose_stamped.pose;
        geometry_msgs::Point point_reached = reached_pose.position;
        geometry_msgs::Quaternion angle_reached = reached_pose.orientation;

        // Display the variables
        //ROS_INFO("current robot position: %f,%f,%f -- %f,%f,%f,%f\n", point_reached.x, point_reached.y, point_reached.z, angle_reached.x, angle_reached.y, angle_reached.z, angle_reached.w);
        // Update the status if necessary
        if (rob_status_ < 4)
            updateStatus(4);
    }

    /**
     * Function that is executed when the robot has finished moving. Displays the result and updates the status accordingly
     * @param state
     * @param result
     */
    void doneRobot(const actionlib::SimpleClientGoalState &state,
                   const move_base_msgs::MoveBaseResultConstPtr &result) {
        // Display state
        //ROS_INFO("Finished in state [%s]", state.toString().c_str());
        // check if the robot has reached the goal
        if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
            // Update & display status
            updateStatus(5);  // Robot has reached the desired pose
            reached_final_pose = true;
        }

    }

    /**
     * Function that adverts that the goal has activated
     */
    void activeRobot() {
        //ROS_INFO("Goal sent to robot.");
    }

    /**
     * Function that shows the feedback from the server
     * @param feedback
     */
    void feedbackRobot(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback) {
        if (rob_status_ < 3) {
            updateStatus(3);
        }
        //ROS_INFO("Got Feedback, current status of the robot is moving with,");
        // Display feedback
        movebaseDisplayFeed(feedback);
    }

    /**
     * Default function executed when creating a MoveAction class
     * @param goal the goal position for the robot to move, got from client
     */
    void executeCB(const assignment1::MoveGoalConstPtr &goal) {
        /* * Initializations * */
        // Header
        std_msgs::Header header;
        header.frame_id = "map";
        header.stamp = ros::Time::now();
        // Get position values
        geometry_msgs::Pose desired = goal->desired_pose;
        geometry_msgs::Point pointToReach = desired.position;
        geometry_msgs::Quaternion angleToReach = desired.orientation;

        // Display goal information
        //ROS_INFO("Client request: %f,%f,%f -- %f,%f,%f,%f", pointToReach.x, pointToReach.y, pointToReach.z, angleToReach.x, angleToReach.y, angleToReach.z, angleToReach.w);

        // Update & display that the goal is recieved
        updateStatus(1);

        // Setting up success true for now
        bool success = true;

        // Setting up the goal to publish
        geometry_msgs::PoseStamped pose_to_publish;
        pose_to_publish.pose = desired;
        pose_to_publish.header = header;

        goal_to_publish.target_pose = pose_to_publish;

        /* * Send Tiago desired pose and handle the feedback sending it back to action_client * */

        // Wait for the robot movement server
        while (!movebase_client_.waitForServer(ros::Duration(5.0))) {
            //ROS_INFO("Waiting for the move_base action server to come up");
        }
        //Subscriber and publisher for moving the robot after receiving info from the client
        vel_pub = nh_.advertise<geometry_msgs::Twist>("/mobile_base_controller/cmd_vel", 1000);
        sub = nh_.subscribe("/scan", 1000, &MoveAction::laserscanCB, this);
        // Send the goal to the robot movement server -> now its done after leaving the narrow passage
        /* movebase_client_.sendGoal(goal_to_publish,
                                  boost::bind(&MoveAction::doneRobot, this, _1, _2),
                                  boost::bind(&MoveAction::activeRobot, this),
                                  boost::bind(&MoveAction::feedbackRobot, this, _1)); */
        // Update & display status
        updateStatus(2);

        // Subscribe topic that describes how the robot movement is going
        ros::Subscriber movebase_status_sub = nh_.subscribe("move_base/status", 1000, &MoveAction::movebaseCallback,
                                                            this);

        ros::Rate r(100);

        while (true) {
            if (as_.isPreemptRequested() || !ros::ok()) {
                //ROS_INFO("%s: Preempted", action_name_.c_str());
                updateStatus(-2);
                as_.setPreempted();
                success = false;
                break;
            }
            r.sleep();
        }

    }

    /**
     * Default function that detects if the robot is in a narrow passage, and
     * enables the motion control law until it leaves the narrow passage.
     * To detect the narrow passage, the robot compares distances on its right
     * and left side to a predetermined threshold.
     * After leaving the narrow passage, the robot moves through
     * sending the goal to move_base.
     * @param msg receives data from the robot's laserscan
     */
    void laserscanCB(const sensor_msgs::LaserScan::ConstPtr &msg) {
        if (reached_final_pose) {
            obstacleDetection(msg);
        }
        int ranges_size = msg->ranges.size();
        bool flag_left = false;
        bool flag_right = false;
        float range_threshold = 0.8;
        float dist_min_left = 100.0;
        float dist_min_right = 100.0;
        for (int i = 21; i < (ranges_size / 2); i++) {
            if (msg->ranges[i] < range_threshold && !flag_left) {
                flag_left = true;
            }
            if (msg->ranges[i] < dist_min_left) {
                dist_min_left = msg->ranges[i];
            }
        }
        for (int i = ranges_size / 2; i < ranges_size - 21; i++) {
            if (msg->ranges[i] < range_threshold && !flag_right) {
                flag_right = true;
            }
            if (msg->ranges[i] < dist_min_right) {
                dist_min_right = msg->ranges[i];
            }
        }
        if (narrow_passage) {
            if (flag_left && flag_right) {
                ROS_INFO("In the narrow passage");
                motion_control_law(dist_min_right, dist_min_left);
            } else {
                ROS_INFO("Left the narrow passage");
                movebase_client_.sendGoal(goal_to_publish,
                                          boost::bind(&MoveAction::doneRobot, this, _1, _2),
                                          boost::bind(&MoveAction::activeRobot, this),
                                          boost::bind(&MoveAction::feedbackRobot, this, _1));
                narrow_passage = false;


            }
        }


    }

    /**
    Receives the closest range on the left side and the right side of the robot.
    Control law to be applied to the robot when in a narrow passage.
    Implements constant moving forward velocity and adjusts velocities to the sides,
    in order for the robot to move in the center of the passage.
    @param dist_right closest distance to the right of the robot measured by the laserscan
    @param dist_left closest distance to the left of the robot measured by the laserscan
    */
    void motion_control_law(float dist_right, float dist_left) {
        geometry_msgs::Twist msg;
        float gain = 1.0;
        msg.linear.x = 0.2; //constant moving forward velocity, has to be slow on narrow passage
        msg.linear.y = gain * (dist_right - dist_left); //velocity to the side, in order to avoid walls
        ROS_INFO("Publishing to cmd_vel");
        vel_pub.publish(msg);

    }
/**
 * Function that detects the obstacles in the environment.
 * It uses the laserscan data to detect the obstacles by
 * comparing the gradient of the ranges with a threshold.
 * The obstacles are detected when the gradient changes
 * from negative to positive.
 * @param msg receives data from the robot's laserscan
 */

    void obstacleDetection(const sensor_msgs::LaserScan::ConstPtr &msg) {
        std::vector<float> ranges;
        std::vector<float> angles;
        std::vector<float> gradient;
        std::vector <std::vector<double>> x_y;
        std::vector<geometry_msgs::Point> obstacles;
        // Store the ranges and angles in vectors
        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            double range = msg->ranges[i];
            ranges.push_back(range);
            float angle = msg->angle_min + i * msg->angle_increment;
            angles.push_back(angle);
        }
        // Calculate the gradient of the ranges and store the x and y coordinates
        for (size_t i = 0; i < ranges.size(); ++i) {
            gradient.push_back(ranges[i + 1] - ranges[i]);
            double x = ranges[i] * cos(angles[i]);
            double y = ranges[i] * sin(angles[i]);
            x_y.push_back({x, y});
        }

        float threshld = 0.2;
        std::vector<int> changeIndices;
        // Find indices where the absolute value of the gradient exceeds the threshold
        for (size_t i = 0; i < gradient.size() - 1; ++i) {
            if (std::abs(gradient[i]) > threshld) {
                changeIndices.push_back(i);
            }
        }
        // Update & display status
        updateStatus(6);
        for (size_t i = 0; i < changeIndices.size() - 1; ++i) {
            int sidePoint1Index = changeIndices[i];
            int sidePoint2Index = changeIndices[i + 1];

            // Check if the gradients of sides points are negative and then positive
            if ((gradient[sidePoint1Index] < 0) && (gradient[sidePoint2Index] > 0)) {

                    geometry_msgs::Point obstacle;
                    obstacle.x = (x_y[sidePoint1Index + 1][0] + x_y[sidePoint2Index][0]) / 2.0;
                    obstacle.y = (x_y[sidePoint1Index + 1][1] + x_y[sidePoint2Index][1]) / 2.0;
                    obstacles.push_back(obstacle);
                    obstacle_found = true;
            }


        }
        // Update & display status
        updateStatus(7);
        if (obstacle_found) {
            result_.completed = obstacle_found;
            ROS_INFO("Obstacles detected");
            // Send the obstacles to the client
            result_.obstacle_positions = obstacles;
            for (const auto &obstacle : result_.obstacle_positions) {
//                ROS_INFO("Obstacle location (Server): x = %f, y = %f", obstacle.x, obstacle.y);
            }
            as_.setSucceeded(result_);
            ros::shutdown();

        } else {
            result_.completed = obstacle_found;
            ROS_INFO("No obstacles detected");
            ros::shutdown();
        }

    }

};


int main(int argc, char **argv) {
    // Initialize server node
    ros::init(argc, argv, "server");
    // Create an server class object
    MoveAction server("server");
    ros::spin();
    return 0;
}

////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
//  ASSIGNMENT 2
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////

//#include <ros/ros.h>
//#include <thread>        // std::this_thread::sleep_for
//#include <chrono>         // std::chrono::milliseconds
//#include <actionlib/server/simple_action_server.h>
//#include <actionlib/client/simple_action_client.h>
//#include <actionlib/client/terminal_state.h>
//#include <assignment1/MoveAction.h>
//#include <std_msgs/Header.h>
//#include <geometry_msgs/Point.h>
//#include <geometry_msgs/Quaternion.h>
//#include <geometry_msgs/Pose.h>
//#include "geometry_msgs/PoseStamped.h"
//#include "actionlib_msgs/GoalStatus.h"
//#include "move_base_msgs/MoveBaseAction.h"
//#include "move_base_msgs/MoveBaseActionGoal.h"
//#include "move_base_msgs/MoveBaseGoal.h"
//#include <sensor_msgs/LaserScan.h>
//
//using namespace std;
//
//class MoveAction
//{
//protected:
//    /*** Declaration of variables ***/
//
//    // The robot status
//    int rob_status_;
//    std::string current_status;
//
//    // Node Handler
//    ros::NodeHandle nh_;
//
//    // Action, action components and server class
//    actionlib::SimpleActionServer<assignment1::MoveAction> as_;
//    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> movebase_client_;
//    assignment1::MoveFeedback feedback_;
//    assignment1::MoveResult result_;
//
//    // Name of the ongoing action
//    std::string action_name_;
//
//public:
//    // Initialization of server action (constructor)
//    MoveAction(std::string name) : as_(nh_, name, boost::bind(&MoveAction::executeCB, this, _1),false), movebase_client_("move_base", true), action_name_(name)
//    {
//        as_.start();
//    }
//    ~MoveAction(void) {}
//
//    /**
//     * Function that updates the robots status to the parameters depending on the integer value given
//     * @param val integer value of the robot status
//     */
//    void updateStatus(int val)
//    {
//        // Update parameters depending on the integer value
//        rob_status_ = val;
//        switch (val)
//        {
//            case -2:
//                current_status = "Cancel request has been received.";
//                break;
//            case -1:
//                current_status = "Goal cannot be reached, robot has aborted the operations.";
//                break;
//            case 1:
//                current_status = "Goal has been received, transmitting to the robot.";
//                break;
//            case 2:
//                current_status = "Goal has been sent to the robot.";
//                break;
//            case 3:
//                current_status = "Robot has accepted the desired pose.";
//                break;
//            case 4:
//                current_status = "Robot is moving to reach the desired pose.";
//                break;
//            case 5:
//                current_status = "Robot has reached the desired pose.";
//                break;
//            case 6:
//                current_status = "Robot is scanning the environment to detect the movable obstacles.";
//                break;
//            case 7:
//                current_status = "Obstacle detection has been completed, sending the results back.";
//                break;
//        }
//        // Update the feedback for the user
//        feedback_.status = current_status;
//        // Send it to user
//        as_.publishFeedback(feedback_);
//    }
//
//    /**
//     * Function that is executed when the robot movement status is updated(published in 'move_base/status')
//     * @param msg the message published of the status
//     */
//    void movebaseCallback(const actionlib_msgs::GoalStatusArray::ConstPtr &msg)
//    {
//        ROS_INFO("Received feedback");
//        // Initialize variable for the status list and get the value
//        std::vector<actionlib_msgs::GoalStatus> status_list;
//        status_list = msg->status_list;
//        // If the status is updated in publish
//        if (status_list.size() > 0)
//        {
//            // Display status list
//            ROS_INFO("Status list length: %ld", status_list.size());
//            ROS_INFO("I heard: [%s]", status_list[status_list.size() - 1].text.c_str());
//            // If the new status is that the robot can not reach the point
//            if (status_list[status_list.size() - 1].text.compare(std::string("Failed to find a valid plan. Even after executing recovery behaviors.")) == 0)
//            {
//                updateStatus(-1);
//            }
//        }
//    }
//
//    /**
//     * This function takes the feedback from the robot's movement and displays it
//     * @param feed
//     */
//    void movebaseDisplayFeed(const move_base_msgs::MoveBaseFeedback::ConstPtr &feed)
//    {
//        // Initialize needed variables
//        geometry_msgs::PoseStamped reached_pose_stamped = feed->base_position;
//        geometry_msgs::Pose reached_pose = reached_pose_stamped.pose;
//        geometry_msgs::Point point_reached = reached_pose.position;
//        geometry_msgs::Quaternion angle_reached = reached_pose.orientation;
//
//        // Display the variables
//        ROS_INFO("current robot position: %f,%f,%f -- %f,%f,%f,%f\n", point_reached.x, point_reached.y, point_reached.z, angle_reached.x, angle_reached.y, angle_reached.z, angle_reached.w);
//        // Update the status if necessary
//        if (rob_status_ < 4)
//            updateStatus(4);
//    }
//
//    /**
//     * Function that is executed when the robot has finished moving. Displays the result and updates the status accordingly
//     * @param state
//     * @param result
//     */
//    void doneRobot(const actionlib::SimpleClientGoalState &state,
//                   const move_base_msgs::MoveBaseResultConstPtr &result)
//    {
//        // Display state
//        ROS_INFO("Finished in state [%s]", state.toString().c_str());
//
//        if (state == actionlib::SimpleClientGoalState::SUCCEEDED) // If the robot has reached the given position
//        {
//            // Update and display
//            result_.completed = true;
//            ROS_INFO("%s: Succeeded", action_name_.c_str());
//            // Tell the server it was a success and passing the result
//            as_.setSucceeded(result_);
//        }
//        else
//        {
//            // Update and display
//            result_.completed = false;
//            updateStatus(-1);
//            ROS_INFO("%s: Aborted", action_name_.c_str());
//            as_.setAborted(result_);
//        }
////        ros::shutdown();
//    }
//
//
//    /**
//     * Function that adverts that the goal has activated
//     */
//    void activeRobot()
//    {
//        ROS_INFO("Goal sent to robot.");
//    }
//
//    /**
//     * Function that shows the feedback from the server
//     * @param feedback
//     */
//    void feedbackRobot(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback)
//    {
//        if (rob_status_ < 3)
//        {
//            updateStatus(3);
//        }
//        ROS_INFO("Got Feedback, current status of the robot is moving with,");
//        // Display feedback
//        movebaseDisplayFeed(feedback);
//    }
//
//    /**
//     * Default function executed when creating a MoveAction class
//     * @param goal the goal position for the robot to move, got from client
//     */
//    void executeCB(const assignment1::MoveGoalConstPtr &goal)
//    {
//        /* * Initializations * */
//        // Header
//        std_msgs::Header header;
//        header.frame_id = "map";
//        header.stamp = ros::Time::now();
//        // Get position values
//        geometry_msgs::Pose desired = goal->desired_pose;
//        geometry_msgs::Point pointToReach = desired.position;
//        geometry_msgs::Quaternion angleToReach = desired.orientation;
//
//        // Display goal information
//        ROS_INFO("Client request: %f,%f,%f -- %f,%f,%f,%f", pointToReach.x, pointToReach.y, pointToReach.z, angleToReach.x, angleToReach.y, angleToReach.z, angleToReach.w);
//
//        // Update & display that the goal is recieved
//        updateStatus(1);
//
//        // Setting up success true for now
//        bool success = true;
//
//        // Setting up the goal to publish
//        geometry_msgs::PoseStamped pose_to_publish;
//        pose_to_publish.pose = desired;
//        pose_to_publish.header = header;
//
//        move_base_msgs::MoveBaseGoal goal_to_publish;
//        goal_to_publish.target_pose = pose_to_publish;
//
//        /* * Send Tiago desired pose and handle the feedback sending it back to action_client * */
//
//        // Wait for the robot movement server
//        while (!movebase_client_.waitForServer(ros::Duration(5.0)))
//        {
//            ROS_INFO("Waiting for the move_base action server to come up");
//        }
//
//        // Send the goal to the robot movement server
//        movebase_client_.sendGoal(goal_to_publish,
//                                  boost::bind(&MoveAction::doneRobot, this, _1, _2),
//                                  boost::bind(&MoveAction::activeRobot, this),
//                                  boost::bind(&MoveAction::feedbackRobot, this, _1));
//        // Update & display status
//        updateStatus(2);
//        movebase_client_.waitForResult(); // Will wait for infinite time
//
//        if(movebase_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
//        {
//            ROS_INFO("robot reahced desire poseee");
//        }
//        // Subscribe topic that describes how the robot movement is going
//        ros::Subscriber movebase_status_sub = nh_.subscribe("move_base/status", 1000, &MoveAction::movebaseCallback, this);
////        ros::Rate r(100);
////
////        while (true)
////        {
////            if (as_.isPreemptRequested() || !ros::ok())
////            {
////                ROS_INFO("%s: Preempted", action_name_.c_str());
////                updateStatus(-2);
////                as_.setPreempted();
////                success = false;
////                break;
////            }
////            r.sleep();
////
////        }
//    }
//};
//
//
//int main(int argc, char **argv)
//{
//    // Initialize server node
//    ros::init(argc, argv, "server");
//    // Create an server class object
//    MoveAction server("server");
//    ros::spin();
//    return 0;
//}