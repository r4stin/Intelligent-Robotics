#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <assignment1/MoveAction.h>
#include <tiago_iaslab_simulation/Objs.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/LaserScan.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <boost/shared_ptr.hpp>
#include <geometry_msgs/PointStamped.h>
#include <control_msgs/PointHeadAction.h>
#include "opencv4/opencv2/opencv.hpp"
#include <assignment2/TagDetectionAction.h>
#include <assignment2/PickPlaceAction.h>

class NodeA {
protected:
    /*** Declaration of variables ***/

    // Node Handler
    ros::NodeHandle nh_;
    // Action client for the move_base action
    typedef actionlib::SimpleActionClient <control_msgs::PointHeadAction> PointHeadClient;
    typedef boost::shared_ptr <PointHeadClient> PointHeadClientPtr;
    PointHeadClientPtr pointHeadClient;

public:



    ~NodeA(void) {
    }
    // Head positions.y
    float head_up = 0.65;
    float head_down = -0.65;
    // Vector that contains the IDs of the objects
    std::vector<int> ids_vec;



/**
 * Responsible for the position of the robot
 * @param x
 * @param y
 * @param z
 */
    geometry_msgs::Point point_to_reach(const float &x, const float &y, const float &z) {
        // Initialaze needed variables
        geometry_msgs::Point pointToReach;

        // Inserting the requested values to the point
        pointToReach.x = x;
        pointToReach.y = y;
        pointToReach.z = z;
        return pointToReach;
    }

/**
 * Responsible for the orientation of the robot
 * @param roll
 * @param pitch
 * @param yaw
 */
    geometry_msgs::Quaternion angle_to_reach(const float &roll, const float &pitch, const float &yaw) {
        // Initialaze needed variables
        geometry_msgs::Quaternion angleToReach;
        tf2::Quaternion tempQuat;
        // Set up the quaternion using fixed axis RPY
        tempQuat.setRPY(roll, pitch, yaw);
        // Convert to quaternions
        tf2::convert(tempQuat, angleToReach);
        return angleToReach;
    }

/**
  * Function that is responsible for the global positions of the robot
  * @param waypoint
  * @param red_cube
  * @param green_triangle
  * @param blue_hexagon
  */
    void
    global_positions(geometry_msgs::Pose &waypoint, geometry_msgs::Pose &red_cube, geometry_msgs::Pose &green_triangle,
                     geometry_msgs::Pose &blue_hexagon) {
        // Inserting the requested values to the points and angles
        waypoint.position = point_to_reach(9.0, 0.0, 0.0);
        waypoint.orientation = angle_to_reach(0.0, 0.0, -1.5708);

        blue_hexagon.position = point_to_reach(8.2, -2.1, 0.0);
        blue_hexagon.orientation = angle_to_reach(0.0, 0.0, -1.5708);

        green_triangle.position = point_to_reach(7.7, -3.9, 0.0);
        green_triangle.orientation = angle_to_reach(0.0, 0.0, 1.5708);

        red_cube.position = point_to_reach(7.5, -2.1, 0.0);
        red_cube.orientation = angle_to_reach(0.0, 0.0, -1.5708);


    }

/**
 * returns the pose of the robot
 */
    geometry_msgs::PoseWithCovarianceStampedConstPtr robot_pose() {
        geometry_msgs::PoseWithCovarianceStampedConstPtr pose_msg = NULL;
        // We will wait until we receive some pointer from the /robot_pose topic
        while (pose_msg == NULL) {
            pose_msg = ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>("/robot_pose", nh_,
                                                                                            ros::Duration(5.0));
        }
//        ROS_INFO("Robot pose is: %f, %f", pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y);
        return pose_msg;
    }

/**
  * Function that check if the robot is in the starting position or not.
  * @param pose_msg
  */
    bool checkRobotPosition(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose_msg) {
        float ths = 0.1;
        if (fabs(pose_msg->pose.pose.position.x) < ths &&
            fabs(pose_msg->pose.pose.position.y) < ths) {
            return true;
        }
        return false;
    }


/**
 * Function that recieves the sequence of pick and place from human_node
 */
    void human_requestCB() {

        ros::ServiceClient client = nh_.serviceClient<tiago_iaslab_simulation::Objs>("/human_objects_srv");

        // Create a service object
        tiago_iaslab_simulation::Objs srv;
        srv.request.ready = true;
        srv.request.all_objs = false;


        // Call the service
        if (client.call(srv)) {

            for (int i = 0; i < 3; ++i) {

//                    ROS_INFO("Len of the request is %ld", srv.response.ids.size());
                if (std::find(ids_vec.begin(), ids_vec.end(), srv.response.ids[0]) == ids_vec.end()) {
                    ids_vec.push_back(srv.response.ids[0]);

                } else {
                    human_requestCB();
                }
            }

        } else {
            ROS_ERROR("Failed to call service human_objects_srv");
        }


    }


/**
 * Function that is responsible for the motion of Tiago head up and down.
 * @param y
 */
    void movehead(float y) {



        // Create a point head action client to move the TIAGo's head
        createPointHeadClient(pointHeadClient);

        static const std::string cameraFrame = "/head_2_link";


        geometry_msgs::PointStamped pointStamped;

        pointStamped.header.frame_id = cameraFrame;
        pointStamped.header.stamp = ros::Time(0);;

        //compute normalized coordinates of the selected pixel
        pointStamped.point.x = 1.0; //define an arbitrary distance
        pointStamped.point.y = y;
//    pointStamped.point.z = 0.0;


        //build the action goal
        control_msgs::PointHeadGoal goal;
        //the goal consists in making the X axis of the cameraFrame to point towards the pointStamped
        goal.pointing_frame = cameraFrame;
        goal.pointing_axis.x = 1.0;
        goal.pointing_axis.y = 0.0;
        goal.pointing_axis.z = 0.0;

        goal.min_duration = ros::Duration(1.0);
        goal.max_velocity = 0.25;
        goal.target = pointStamped;

        pointHeadClient->sendGoal(goal);
        ros::Duration(0.5).sleep();
        ROS_INFO_STREAM("goal sent");
//    ros::shutdown();

    }

/**
 * Function that creates a ROS action client to move TIAGo's head
 * @param actionClient the action client to be created
 */
    void createPointHeadClient(PointHeadClientPtr &actionClient) {
        ROS_INFO("Creating action client to head controller ...");

        actionClient.reset(new PointHeadClient("/head_controller/point_head_action"));

        int iterations = 0, max_iterations = 3;
        // Wait for head controller action server to come up
        while (!actionClient->waitForServer(ros::Duration(2.0)) && ros::ok() && iterations < max_iterations) {
            ROS_DEBUG("Waiting for the point_head_action server to come up");
            ++iterations;
        }

        if (iterations == max_iterations)
            throw std::runtime_error("Error in createPointHeadClient: head controller action server not available");
    }

/**
 * Function that is responsible for the motion of Tiago (using assignment1::MoveAction)
 * @param aGoal the goal position for the robot to move
 */
    bool navigation(const assignment1::MoveGoal &aGoal) {

        // Create the action client
        actionlib::SimpleActionClient <assignment1::MoveAction> as("server", true);
        ROS_INFO("Waiting for action server to start");
        // Wait for the action server to come up
        as.waitForServer();
        ROS_INFO("action server started");

        // Send a goal to the action server
        as.sendGoal(aGoal);
        ROS_INFO("goal sent");

        // Wait for the action to return
        bool timeout = as.waitForResult(ros::Duration(200.0));

        // If the goal finished before the time out the goal status is reported
        if (timeout) {

            return true;
        }
            // goal did not finish in the desire time.
        else {
            ROS_INFO("server_node action server did not finish before the time out");
            return false;
        }
    }


/**
 * Function that is responsible for the pick and place of the markers
 * @param aGoal
 * @return
 */
    bool pick_place(const assignment2::PickPlaceGoal &aGoal) {
        // Create the action client.
        actionlib::SimpleActionClient <assignment2::PickPlaceAction> ac_pickplace("node_c", true);
        ROS_INFO("Waiting for node_c action server to start");
        // Wait for the action server to come up
        ac_pickplace.waitForServer();
        ROS_INFO("node_c action server started, sending goal");

        // Send a goal to the action server
        ac_pickplace.sendGoal(aGoal);

        // Wait for the action to return
        ac_pickplace.waitForResult();

        // If the goal finished before the time out the goal status is reported
        if (ac_pickplace.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            actionlib::SimpleClientGoalState state = ac_pickplace.getState();
            //ROS_INFO("node_c action server finish: %s",state.toString().c_str());

            return true;
        }
            // Otherwise the user is notified that the goal did not finish in the allotted time.
        else {
            ROS_INFO("node_c action server did not finish before the time out");
            return false;
        }
    }



/**
 * Function which is printing the result of the detection phase
 * @param state
 * @param result
 */
    void markerDoneCb(const actionlib::SimpleClientGoalState &state,
                      const assignment2::TagDetectionResultConstPtr &result) {
        // The desired marker ID was detected
        if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
            //ROS_INFO("node_b finish: %s",state.toString().c_str());
            ROS_INFO("Number of Marker detected: %i", (int) result->obstacles_ID.size() + 1);
            // Desired marker detected
            ROS_INFO("Marker ID: %i, Size: %f, Pose: [x: %f, y: %f, z: %f] [x: %f, y: %f, z: %f, w: %f]",
                     result->marker_ID, result->marker_size,
                     result->marker_pose.position.x, result->marker_pose.position.y,
                     result->marker_pose.position.z, result->marker_pose.orientation.x,
                     result->marker_pose.orientation.y, result->marker_pose.orientation.z,
                     result->marker_pose.orientation.w);

            // Other marker detected
            for (int i = 0; i < result->obstacles_ID.size(); i++) {
                // Print for each obstacle, its position
                ROS_INFO("Marker ID: %i, Size: %f, Pose: [x: %f, y: %f, z: %f] [x: %f, y: %f, z: %f, w: %f]",
                         result->obstacles_ID.at(i), result->obstacles_size.at(i),
                         result->obstacles_pose.at(i).position.x, result->obstacles_pose.at(i).position.y,
                         result->obstacles_pose.at(i).position.z, result->obstacles_pose.at(i).orientation.x,
                         result->obstacles_pose.at(i).orientation.y, result->obstacles_pose.at(i).orientation.z,
                         result->obstacles_pose.at(i).orientation.w);
            }
        }
    }

    void markerActiveCb() {}
    /** It is only declared and do nothing
     * @param feedback
     */
    void markerFeedbackCb(const assignment2::TagDetectionFeedbackConstPtr &feedback) {}

/**
 * Function that is responsible for the detection of the objects
 * @param aGoal
 * @return
 */
    assignment2::TagDetectionResultConstPtr marker_detection(const assignment2::TagDetectionGoal &aGoal) {
        // Create the action client.
        actionlib::SimpleActionClient <assignment2::TagDetectionAction> ac_detection("node_b", true);
        ROS_INFO("Waiting for node_b server to start");

        ac_detection.waitForServer();
        ROS_INFO("node_b server started, sending goal");

        // Send a goal to the node_b action server
        //ac_detection.sendGoal(aGoal);
        ac_detection.sendGoal(aGoal, boost::bind(&NodeA::markerDoneCb, this, _1, _2),
                               boost::bind(&NodeA::markerActiveCb, this),
                               boost::bind(&NodeA::markerFeedbackCb, this, _1));

        // Wait for the action to return
        bool finished_before_timeout = ac_detection.waitForResult(ros::Duration(150.0));

        // If the goal finished before the time out the goal status is reported
        if (finished_before_timeout) {
            return ac_detection.getResult();
        }
            // Otherwise the user is notified that the goal did not finish in the allotted time.
        else {
            ROS_INFO("node_b action server did not finish before the time out");
            return NULL;
        }
    }

};

int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "node_a");
    // Create an instance of NodeA
    NodeA nodeA;

    ros::Rate r(1);

    //Generate the sequence of pick and place
//    for (size_t i = 0; i < 3; ++i) {
        nodeA.human_requestCB();
        r.sleep();
        r.sleep();

//    }

    ROS_INFO("Human request received");
    ROS_INFO("Number of IDs received: %ld", nodeA.ids_vec.size());
    for (size_t i = 0; i < nodeA.ids_vec.size(); ++i) {
        ROS_INFO("ID: %d", nodeA.ids_vec.at(i));
    }
    // Create the goal for the navigation
    assignment1::MoveGoal move_goal;
    // Create the goal for the pick and place
    assignment1::MoveGoal place_goal;
    // Create poses
        geometry_msgs::Pose blue_hexagon;
        geometry_msgs::Pose green_triangle;
        geometry_msgs::Pose red_cube;
        geometry_msgs::Pose waypoint;
        geometry_msgs::Pose place_pose;

        nodeA.global_positions(waypoint, red_cube, green_triangle, blue_hexagon);
        // robot pose
        geometry_msgs::PoseWithCovarianceStampedConstPtr pose_msg = nodeA.robot_pose();

        // Recieve the sequence and manipulate pick and place
for (int i = 0; i < nodeA.ids_vec.size(); ++i) {

    // Check if the robot is in the starting position
    if (nodeA.checkRobotPosition(pose_msg)) {

        move_goal.desired_pose = waypoint;

        if (!nodeA.navigation(move_goal)) {
            ROS_INFO("Fail to reach waypoint position!");
        }
        ROS_INFO("Robot reachced waypoint!");
    }

    if (nodeA.ids_vec.at(i) == 1) {

        // Set the desired pose to the desired pose of the goal
        move_goal.desired_pose = blue_hexagon;

        if (!nodeA.navigation(move_goal)) {
            ROS_INFO("Fail to reach blue_hexagon position!");
        }
        ROS_INFO("ID: %d, Robot reached blue_hexagon ", nodeA.ids_vec.at(i));

    } else if (nodeA.ids_vec.at(i) == 2) {
        // Set the desired pose to the desired pose of the goal
        move_goal.desired_pose = green_triangle;

        if (!nodeA.navigation(move_goal)) {
            ROS_INFO("ID: , Fail to reach green_triangle position !");
        }
        ROS_INFO("ID: %d, Robot reached green_triangle ", nodeA.ids_vec.at(i));

    } else if (nodeA.ids_vec.at(i) == 3) {
        // Set the desired pose to the desired pose of the goal
        move_goal.desired_pose = red_cube;

        if (!nodeA.navigation(move_goal)) {
            ROS_INFO("Fail to reach red_cube position !");
        }
        ROS_INFO("ID: %d, Robot reahced red_cube", nodeA.ids_vec.at(i));

    } else {
        ROS_INFO("Navigation Fail !");
    }
    nodeA.movehead(nodeA.head_down);
    ROS_INFO("Head down");
    // wait for 3 seconds till the robot head move down
    r.sleep();
    r.sleep();
    r.sleep();

    assignment2::TagDetectionGoal ID_goal;

    // Detection -> node_b
    ROS_INFO("ID: %i, Start detection!", nodeA.ids_vec.at(i));
    ID_goal.marker_ID = nodeA.ids_vec.at(i);

    // Save the result of the detection inside this pointer
    assignment2::TagDetectionResultConstPtr DetectionPtr = nodeA.marker_detection(ID_goal);
    if (DetectionPtr == NULL)
    {
        ROS_INFO("ID: %i, Detection failed!", nodeA.ids_vec.at(i));
        return 1;
    }
    ROS_INFO("ID: %i, End detection!", nodeA.ids_vec.at(i));


    // Only pick when the objcet is detected
    // Use the result of the detection to pick the i-th object
    assignment2::PickPlaceGoal pick_goal;
    assignment2::PickPlaceGoal place_goal;
    pick_goal.pick = true;
    pick_goal.place = false;
    pick_goal.marker_ID = DetectionPtr->marker_ID;
    pick_goal.marker_size = DetectionPtr->marker_size;
    pick_goal.marker_pose = DetectionPtr->marker_pose;
    pick_goal.obstacles_ID = DetectionPtr->obstacles_ID;
    pick_goal.obstacles_size = DetectionPtr->obstacles_size;
    pick_goal.obstacles_pose = DetectionPtr->obstacles_pose;
    place_goal.pick = false;
    place_goal.place = true;
    place_goal.marker_ID = DetectionPtr->marker_ID;

    // Pick and place -> node_c
    ROS_INFO("ID: %i, Start Pick!", nodeA.ids_vec.at(i));
    if (! nodeA.pick_place(pick_goal))
    {  nodeA.movehead(nodeA.head_up);
        ROS_INFO("ID: %i, Pick is failed!", nodeA.ids_vec.at(i));
        return 1;
    }
    ROS_INFO("ID: %i, Pick successfully!", nodeA.ids_vec.at(i));
    r.sleep();
    r.sleep();

    if (nodeA.ids_vec.at(i) == 1) {
        place_pose.position = nodeA.point_to_reach(12.4, 0.4, 0.0);
        place_pose.orientation = nodeA.angle_to_reach(0.0, 0.0, -1.5708);

        ROS_INFO("Robot moving toward blue table");
    } else if (nodeA.ids_vec.at(i) == 2) {

        place_pose.position = nodeA.point_to_reach(11.4, 0.4, 0.0);
        place_pose.orientation = nodeA.angle_to_reach(0.0, 0.0, -1.5708);
        ROS_INFO("Robot moving toward green table");
    } else if (nodeA.ids_vec.at(i) == 3) {
        place_pose.position = nodeA.point_to_reach(10.4, 0.4, 0.0);
        place_pose.orientation = nodeA.angle_to_reach(0.0, 0.0, -1.5708);

        ROS_INFO("Robot moving toward red table");
    } else {
        ROS_INFO("Place Pose Fail !");
    }
    // navigate to place table
    move_goal.desired_pose = place_pose;

    nodeA.navigation(move_goal);
    ROS_INFO("Robot reached place pose");
    if(! nodeA.pick_place(place_goal))
    {
        ROS_INFO("ID: %i, Place is failed!", nodeA.ids_vec.at(i));
        return 1;
    }
    ROS_INFO("ID: %i, Place successfully!", nodeA.ids_vec.at(i));


}
    return 0;
}
