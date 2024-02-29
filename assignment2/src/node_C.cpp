#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <gazebo_ros_link_attacher/Attach.h>
#include <assignment2/PickPlaceAction.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <math.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "constant.h"
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>



class PickPlace {


protected:
    ros::NodeHandle nh;
    actionlib::SimpleActionServer<assignment2::PickPlaceAction> as_;
    assignment2::PickPlaceFeedback feedback_;
    assignment2::PickPlaceResult result_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface group_arm;
    bool collision_done = false;


public:

    PickPlace() : as_(nh, "node_c", boost::bind(&PickPlace::executeCB, this, _1), false), group_arm(constant::INTERFACE_ARM_TORSO){
        as_.start();

    }
    ~PickPlace(void) {}


    const geometry_msgs::Point table_position = PickPlace::point_to_reach(7.826, -2.983, (0.775)/2.0);
    const geometry_msgs::Quaternion table_orientation = PickPlace::angle_to_reach(0.0, 0.0, 0.0);
    // All object
    const float x_angle = M_PI/2.0;
    const float y_angle =  M_PI/8.0;




    // Blue and Green intermediate_pose
    const geometry_msgs::Point left_intermediate_point = PickPlace::point_to_reach(0.3, 0.6, 0.98);
    const geometry_msgs::Quaternion left_intermediate_orientation = PickPlace::angle_to_reach(x_angle, 0, M_PI/2.0);
    // Red intermediate_pose
    const geometry_msgs::Point right_intermediate_point = PickPlace::point_to_reach(0.3, -0.6, 1.2);
    const geometry_msgs::Quaternion right_intermediate_orientation = PickPlace::angle_to_reach(x_angle, 0, -M_PI/2.0);

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
     * Function used to fix pose of table
     */
    geometry_msgs::Pose fixPose(const geometry_msgs::Pose &pose) {
        tf::TransformListener listener;
        tf::StampedTransform transform;

        try {
            listener.waitForTransform(constant::BASE_FOOTPRINT, constant::MAP, ros::Time(0), ros::Duration(10.0));
            listener.lookupTransform(constant::BASE_FOOTPRINT, constant::MAP, ros::Time(0), transform);
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("Received an exception trying to transform a point : %s", ex.what());
        }
        return transformPose(pose, transform);
    }


    /**
     * Responsible for the transformation of the pose
     *
     */
    geometry_msgs::Pose transformPose(const geometry_msgs::Pose &pose, const tf::StampedTransform &transform) {
        tf::Vector3 v(pose.position.x, pose.position.y, pose.position.z);
        v = transform * v;
        tf::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
        q = transform * q;

        geometry_msgs::Pose transformed_pose;
        transformed_pose.position.x = v.x();
        transformed_pose.position.y = v.y();
        transformed_pose.position.z = v.z();
        transformed_pose.orientation.x = q.x();
        transformed_pose.orientation.y = q.y();
        transformed_pose.orientation.z = q.z();
        transformed_pose.orientation.w = q.w();
        return transformed_pose;
    }

    /**
     * Generates a simple trajectory with one waypoints to move TIAGo's gripper
     * @param gripper_joint_value
     * @param seconds
     */
    control_msgs::FollowJointTrajectoryGoal build_gripper_goal(const std::vector<float> &gripper_joint_value, const float &seconds) {
        control_msgs::FollowJointTrajectoryGoal goal;
        // The joint names, which apply to all waypoints
        goal.trajectory.joint_names.push_back(constant::GRIPPER_RIGHT_FINGER_JOINT);
        goal.trajectory.joint_names.push_back(constant::GRIPPER_LEFT_FINGER_JOINT);

        // Only one waypoints in this goal trajectory
        goal.trajectory.points.resize(1);

        // Trajectory point
        int index = 0;
        // Positions
        goal.trajectory.points.at(index).positions.resize(2);
        // Velocities
        goal.trajectory.points.at(index).velocities.resize(2);
        for (int j = 0; j < gripper_joint_value.size(); ++j) {
            goal.trajectory.points.at(index).positions.at(j) = gripper_joint_value.at(j);
            goal.trajectory.points.at(index).velocities.at(j) = 0.0;
        }
        // To be reached 5 second after starting along the trajectory
        goal.trajectory.points[index].time_from_start = ros::Duration(seconds);
        return goal;
    }

    /**
     *  Function that perform the gripper moving
     * @param gripper_joint_value
     * @param seconds
     */
    void move_gripper(const std::vector<float> &gripper_joint_value, const float &seconds) {
        // Create an head controller action client to move the TIAGo's head
        actionlib::SimpleActionClient <control_msgs::FollowJointTrajectoryAction> arm_client(
                constant::GRIPPER_FOLLOW_JOINT_TRAJECTORY_ACTION, true);
        ROS_INFO("Waiting for action server to start");
        // Wait for the action server to come up
        arm_client.waitForServer(); // Will wait for infinite time
        ROS_INFO("Action server started, sending goal");

        // Send a goal to the action server
        control_msgs::FollowJointTrajectoryGoal aGoal = build_gripper_goal(gripper_joint_value, seconds);
        arm_client.sendGoal(aGoal);

        // Wait for the action to return
        bool finished_before_timeout = arm_client.waitForResult(ros::Duration(20.0));
        //bool finished_before_timeout = arm_client.waitForResult();

        // If the goal finished before the time out the goal status is reported
        if (finished_before_timeout) {
            actionlib::SimpleClientGoalState state = arm_client.getState();
            ROS_INFO("Action Server finish: %s", state.toString().c_str());
        }
            // Otherwise the user is notified that the goal did not finish in the allotted time.
        else {
            ROS_INFO("Action Server did not finish before the time out");
        }
    }
//

    /**
     * Function to add obstacles, objects and initial table to the collision object vector
     * @param target_id
     * @param target_pose
     * @param target_size
     * @param obstacles_id
     * @param obstacle_pose
     * @param obstacle_size
     */
    std::vector<std::string> addPickCollisionObjects(const int& target_id, const geometry_msgs::Pose& target_pose, const float& target_size,
                                                     const std::vector<int>& obstacles_id, const std::vector<geometry_msgs::Pose>& obstacle_pose, const std::vector<float>& obstacle_size)
    {
        std::vector<std::string> object_ids;


        // Creating collisions
        std::vector<moveit_msgs::CollisionObject> collision_objects;
        // 1 for the table and 1 for the target object to pick
        int number_objects = obstacles_id.size() + 2;
        collision_objects.resize(number_objects);
        ROS_INFO("Number of Collision object: %i", number_objects);
        // Define the id of the object, which is used to identify it.
        collision_objects[0].id = constant::PICK_TABLE_COLLISION_OBJECT;
        object_ids.push_back("table");
        collision_objects[0].header.frame_id = group_arm.getPlanningFrame();


        // Enlarge the table along z axes to avoid contact with it
        float enlargment_table_z = 0.01;

        // Define the pose of the table (box)
        // It should be wrt to the center of the object
        // Transform given costant and predifined value from map to base_footprint
        geometry_msgs::Pose table_pose;
        table_pose.position = table_position;
        table_pose.orientation = table_orientation;

        table_pose = fixPose(table_pose);
        table_pose.position.z += enlargment_table_z/2.0;

        // Define the primitive and its dimensions
        shape_msgs::SolidPrimitive primitive_table;
        primitive_table.type = primitive_table.BOX;
        primitive_table.dimensions.resize(3);
        primitive_table.dimensions[0] = constant::PICK_TABLE_LENGTH;
        primitive_table.dimensions[1] = constant::PICK_TABLE_WIDTH;
        primitive_table.dimensions[2] = constant::PICK_TABLE_HEIGHT;
        collision_objects[0].primitives.push_back(primitive_table);
        collision_objects[0].primitive_poses.push_back(table_pose);
        collision_objects[0].operation = collision_objects[0].ADD;

        // Define the object that we will be manipulating
        collision_objects[1].header.frame_id = group_arm.getPlanningFrame();
        collision_objects[1].id = std::to_string(target_id);


        // Enlarge the the object to be picked on z to void contact with it
        float enlargment_blue_height = 0.1;
        float enlargment_red_height = 0.04;

        // Define the primitive and its dimensions
        shape_msgs::SolidPrimitive primitive_target;
        // Standard value, update for the specific object
        float target_heigth_z = target_pose.position.z - constant::PICK_TABLE_HEIGHT;
        float target_pose_z = constant::PICK_TABLE_HEIGHT + (target_heigth_z/2.0);
        // Check if the target ID correspond to a red cube (3)
        if (target_id == 3)
        {
            target_heigth_z = target_pose.position.z + enlargment_red_height - constant::PICK_TABLE_HEIGHT;
            target_pose_z = constant::PICK_TABLE_HEIGHT + (target_heigth_z/2.0);
            //ROS_INFO("ID: %i is a red cube! Collision", target_id);
            primitive_target.type = primitive_target.BOX;
            primitive_target.dimensions.resize(3);
            primitive_target.dimensions[0] = target_size*2.0;
            primitive_target.dimensions[1] = target_size*2.0;
            primitive_target.dimensions[2] = target_heigth_z;
        }
        else if (target_id == 1)
        {
            primitive_target.type = primitive_target.CYLINDER;
            target_heigth_z = target_pose.position.z + enlargment_blue_height - constant::PICK_TABLE_HEIGHT;
            target_pose_z = constant::PICK_TABLE_HEIGHT + (target_heigth_z/2.0);
            // Define the primitive and its dimensions
            primitive_target.dimensions.resize(2);
            primitive_target.dimensions[0] = target_heigth_z; // Height
            primitive_target.dimensions[1] = target_size; // Radius

        }
            // Check if the target ID correspond to a green triangle (2)
        else if (target_id == 2)
        {
            std::vector < std::string> a = {std::to_string(target_id)};
            planning_scene_interface.removeCollisionObjects(a);
            primitive_target.type = primitive_target.CONE;
            target_heigth_z = target_pose.position.z + target_size - constant::PICK_TABLE_HEIGHT;
            target_pose_z = constant::PICK_TABLE_HEIGHT;
            // Define the primitive and its dimensions
            primitive_target.dimensions.resize(2);
            primitive_target.dimensions[0] = target_heigth_z;
            primitive_target.dimensions[1] = target_size;
        } else {
            ROS_INFO("ID: %i is not a valid ID! Collision", target_id);

        }

        // Define the collision objects for the obstacles to avoid
        float enlargment_obstacle_height = 0.07;
        float enlargment_obstacle_radius = 0.02;
        for(int i = 2, j=0; i < number_objects; i++, j++)
        {
            collision_objects[i].header.frame_id = group_arm.getPlanningFrame();
            collision_objects[i].id = std::to_string(obstacles_id.at(j));
            object_ids.push_back(std::to_string(obstacles_id.at(j)));

            float obstacle_heigth_z = obstacle_pose.at(j).position.z - constant::PICK_TABLE_HEIGHT;
            float obstacle_pose_z = constant::PICK_TABLE_HEIGHT + (obstacle_heigth_z/2.0);

            // Define the primitive and its dimensions
            shape_msgs::SolidPrimitive primitive_obstacle;
            primitive_obstacle.type = primitive_obstacle.CYLINDER;
            primitive_obstacle.dimensions.resize(2);

            // Enlargment
            primitive_obstacle.dimensions[0] = obstacle_heigth_z + enlargment_obstacle_height; // Height
            primitive_obstacle.dimensions[1] = obstacle_size.at(j) + enlargment_obstacle_radius; // Radius

            // Define the pose of the obstacles
            geometry_msgs::Pose obstacle_object_pose;
            obstacle_object_pose.position.x = obstacle_pose.at(j).position.x;
            obstacle_object_pose.position.y = obstacle_pose.at(j).position.y;
            obstacle_object_pose.position.z = obstacle_pose_z;
            obstacle_object_pose.orientation = obstacle_pose.at(j).orientation;

            collision_objects[i].primitives.push_back(primitive_obstacle);
            collision_objects[i].primitive_poses.push_back(obstacle_object_pose);
            collision_objects[i].operation = collision_objects[i].ADD;
        }



        // Define the pose of the target object
        geometry_msgs::Pose target_object_pose;
        target_object_pose.position.x = target_pose.position.x;
        target_object_pose.position.y = target_pose.position.y;
        target_object_pose.position.z = target_pose_z;
        target_object_pose.orientation = target_pose.orientation;

        collision_objects[1].primitives.push_back(primitive_target);
        collision_objects[1].primitive_poses.push_back(target_object_pose);
        collision_objects[1].operation = collision_objects[1].ADD;

        // Now, add the collision objects into the world
        planning_scene_interface.addCollisionObjects(collision_objects);
        return object_ids;



    }

    /**
     * Function necessary to give correct value to robot (approximation to 0.**m so 1cm)
     * @param aPoint
     */
    void roundPoint(geometry_msgs::Point& aPoint)
    {
        aPoint.x = ceil(aPoint.x * 1000.0) / 1000.0;
        aPoint.y = ceil(aPoint.y * 1000.0) / 1000.0;
        aPoint.z = ceil(aPoint.z * 1000.0) / 1000.0;
    }


    /**
     * Function necessary to give correct value to robot (approximation to 0.**m so 1cm)
     * @param group_arm
     * @param aPose
     * @return
     */
    bool perform_motion(moveit::planning_interface::MoveGroupInterface& group_arm, const geometry_msgs::Pose& aPose)
    {
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        group_arm.setStartState(*group_arm.getCurrentState());
        group_arm.setPoseTarget(aPose);
        moveit::core::MoveItErrorCode error_code = group_arm.plan(my_plan);
        if (error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            ROS_INFO("Plan found in %f seconds", my_plan.planning_time_);
            //  Execute the plan
            error_code = group_arm.move();
            if (error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS)
            {
                ROS_INFO("Motion completed!");
                return true;
            }
            else
            {
                ROS_INFO("Motion failed! Error: %i:", error_code.val);
                return false;
            }
        }
        else
        {
            ROS_INFO("No plan found! Error: %i", error_code.val);
            return false;
        }
    }

    // Function to close the gripper
    void closedGripper(const float& gripper_link_value, const float& seconds)
    {
        // Set them as closed
        std::vector<float> gripper_joint_value = {gripper_link_value, gripper_link_value};
        move_gripper(gripper_joint_value, seconds);
    }
    // Function to open the gripper
    void openGripper(const float& gripper_link_value, const float& seconds)
    {
        // Set them as open, wide enough for the object to fit
        std::vector<float> gripper_joint_value = {gripper_link_value, gripper_link_value};
        move_gripper(gripper_joint_value, seconds);
    }

/**
 * Function responsible to pick the object
 */

    bool pick_marker(const assignment2::PickPlaceGoalConstPtr &goal) {
        ROS_INFO("Received goal from node B");
        ros::Rate r(1);
        int marker_id = goal->marker_ID;

        ROS_INFO("Marker ID: %d", marker_id);


                std::vector<std::string> collision_objects_ids;
        if(!collision_done) {
            collision_objects_ids = addPickCollisionObjects(goal->marker_ID, goal->marker_pose, goal->marker_size, goal->obstacles_ID, goal->obstacles_pose, goal->obstacles_size);
            collision_done = true;
        }
        if (collision_objects_ids.size() == 0) {
            ROS_INFO("Fail to addPickCollisionObjects");
            return false;
        }
        ROS_INFO("collision added");
        ros::WallDuration(1.0).sleep();


        geometry_msgs::Pose real_marker_pose = goal->marker_pose;
        geometry_msgs::Pose intermediate_pose;
        geometry_msgs::Pose secure_pose;
        geometry_msgs::Pose grasp_pose;
        // pick routine for blue hexagon
        if (marker_id == 1) {


            secure_pose.position = left_intermediate_point;
            secure_pose.orientation = left_intermediate_orientation;
            roundPoint(secure_pose.position);

            real_marker_pose.position.z += 0.098;
            real_marker_pose.position.y += 0.3;
            tf2::Quaternion orientation;


            orientation.setRPY(x_angle, y_angle, -x_angle);
            real_marker_pose.orientation = tf2::toMsg(orientation);
            grasp_pose = real_marker_pose;
            grasp_pose.position.y -= 0.11;

            intermediate_pose = grasp_pose;
            intermediate_pose.position.z += 0.3;
            }
        // pick routine for green triangle
        else if (marker_id == 2) {
            secure_pose.position = left_intermediate_point;
            secure_pose.orientation = left_intermediate_orientation;

            roundPoint(secure_pose.position);

            real_marker_pose.position.z += 0.198;
            real_marker_pose.position.y += 0.3;
            tf2::Quaternion orientation;


            orientation.setRPY(x_angle, y_angle, -x_angle);
            real_marker_pose.orientation = tf2::toMsg(orientation);
            grasp_pose = real_marker_pose;
            grasp_pose.position.z -= 0.1;
            grasp_pose.position.y -= 0.11;


            intermediate_pose = grasp_pose;
            intermediate_pose.position.z += 0.3;



        }
        // pick routine for red cube
        else if (marker_id == 3) {
            secure_pose.position = right_intermediate_point;
            secure_pose.orientation = right_intermediate_orientation;
            roundPoint(secure_pose.position);

            real_marker_pose.position.z += 0.198;
            real_marker_pose.position.y -= 0.3;
            tf2::Quaternion orientation;

            // Pick test 1

//        orientation.setRPY(-tau/4, - tau/4, 0);
            orientation.setRPY(-x_angle, y_angle, x_angle);
            real_marker_pose.orientation = tf2::toMsg(orientation);
            grasp_pose = real_marker_pose;
            grasp_pose.position.z -= 0.1;
            grasp_pose.position.y += 0.11;


            intermediate_pose = grasp_pose;
            intermediate_pose.position.z += 0.3;




        }
        else {
            ROS_INFO("Marker ID not valid");
        }
// Move arm to secure pose
        if (!perform_motion(group_arm, secure_pose))
        {
            ROS_INFO("Fail to reach secure_pose");
            return  false;
        }
        r.sleep();
        r.sleep();



// move arm to real marker pose (upper than target pose)
        if (!perform_motion(group_arm, real_marker_pose))
        {
            ROS_INFO("Fail to reach real_marker_pose");
            return  false;
        }
        r.sleep();
        r.sleep();

// move arm to grasp pose
        if (!perform_motion(group_arm, grasp_pose)) {
            ROS_INFO("Fail to reach grasp_pose");
            return  false;

        }
        r.sleep();
        r.sleep();



        // Remove the target object from the collision objects
        ROS_INFO("Remove object from the collision objects");
        feedback_.status = "Remove object from the collision objects";
        std::vector<std::string> object_ids;
        object_ids.push_back(std::to_string(goal->marker_ID));
        planning_scene_interface.removeCollisionObjects(object_ids);
        r.sleep();

        // Attach virtually the object to the gripper (use arm_7_link) using the appropriate Gazebo plugin: Gazebo_ros_link_attacher
        ROS_INFO("Attach virtually the object to the gripper");
        feedback_.status = "Attach virtually the object to the gripper";
        if ( ! attachGripper(goal->marker_ID))
        {
            ROS_INFO("Failed to attach virtually the object to the gripper");
            return  false;
        }
        r.sleep();

        // Close the gripper.
        ROS_INFO("Close the gripper");
        feedback_.status = "Close the gripper";
        // 0.00 for closedGripper
        closedGripper(0.00, 1.0);
        r.sleep();

        // Come back to the previous target position through a linear movement of the arm.
        ROS_INFO("Return to target position");
        feedback_.status = "Return to target position";


// move arm to intermediate pose (upper than target pose)
        if (! perform_motion(group_arm, intermediate_pose))
        {
            ROS_INFO("Fail to reach linear intermediate_pose");
            return  false;
        }
        r.sleep();
        r.sleep();



    }


/**
 * Function responsible to place the object
 * @param goal
 * @return
 */

    bool place_marker(const assignment2::PickPlaceGoalConstPtr &goal) {


        // Get current pose of the robot arm and torso
        geometry_msgs::Pose current_pose;
        current_pose = group_arm.getCurrentPose().pose;


        ros::Rate r(1);
        geometry_msgs::Pose place_pose;
        geometry_msgs::Pose intermediate_pose;


        place_pose = current_pose;
        // set the placing pose 10 cm upper than place table height
        place_pose.position.z = constant::PLACE_TABLE_HEIGHT + 0.1;
        place_pose.position.x += 0.1;
// after object placed, robot will reach this pose and ready ro pick next object 30 cm upper than place table height
        intermediate_pose = place_pose;
        intermediate_pose.position.z += 0.3;

        if (!perform_motion(group_arm, place_pose)) {
            ROS_INFO("Fail to reach place_pose");
            return false;
        }
        r.sleep();
        r.sleep();
        openGripper(0.04, 1.0);
        r.sleep();
        r.sleep();


        // Detach virtually the object to the gripper (use arm_7_link) using the appropriate Gazebo plugin: Gazebo_ros_link_attacher
        ROS_INFO("Detach virtually the object to the gripper");
        feedback_.status = "Detach virtually the object to the gripper";
        if ( ! detachGripper(goal->marker_ID))
        {
            ROS_INFO("Failed to detach virtually the object to the gripper");
            return  false;
        }
// move arm to intermediate pose (upper than place table height)
        if (!perform_motion(group_arm, intermediate_pose)) {
            ROS_INFO("Fail to reach intermediate_pose");
            return false;
        }




    }
/**
 * Function responsible to execute the action and manage pick and place
 * @param goal
 */
    void executeCB(const assignment2::PickPlaceGoalConstPtr &goal) {
    // initial configuration of the robot
        initial_tiago_configuration();

        // Check if the goal is to pick
        if (goal-> pick == true && goal-> place == false) {
            if (pick_marker(goal)) {
                result_.success = true;
                as_.setSucceeded(result_);
                ROS_INFO("Pick success!");
            } else {
                result_.success = false;
                as_.setAborted(result_);
                ROS_INFO("Pick failed!");
            }
        }
        // Check if the goal is to place
        else if (goal-> pick == false && goal-> place == true) {
            if (place_marker(goal)) {
                result_.success = true;
                as_.setSucceeded(result_);
                ROS_INFO("Place success!");
            } else {
                result_.success  = false;
                as_.setAborted(result_);
                ROS_INFO("Place failed!");
            }

        }

        else {
            ROS_INFO("Pick and place failed!");
        }



    }

    /**
     * Function to virtually attach the object to the gripper
     * @param target_id
     * @return
     */
    bool attachGripper(const int& target_id) {
        ros::ServiceClient attach_client = nh.serviceClient<gazebo_ros_link_attacher::Attach>(
                "/link_attacher_node/attach");
        gazebo_ros_link_attacher::Attach attach_srv;
        // Set the names of the link to be attached and the object to be attached
        if(target_id == 1) // If ID == 1 then detach the blue hexagon
        {
            attach_srv.request.model_name_1 = constant::HEXAGON;
            attach_srv.request.link_name_1 = constant::HEXAGON_LINK;
        }
        else if(target_id == 2) // If ID == 2 then detach the green triangle
        {
            attach_srv.request.model_name_1 = constant::TRIANGLE;
            attach_srv.request.link_name_1 = constant::TRIANGLE_LINK;
        }
        else if(target_id  == 3) // If ID == 3 then detach the red cube
        {
            attach_srv.request.model_name_1 = constant::CUBE;
            attach_srv.request.link_name_1 = constant::CUBE_LINK;
        }
        attach_srv.request.model_name_2 = constant::TIAGO;
        attach_srv.request.link_name_2 = constant::ARM_7_LINK;
        // Call the link_attacher service
        if (attach_client.call(attach_srv)) {
            ROS_INFO("Object successfully attached to the gripper");
            return true;
        } else {
            ROS_INFO("Failed to attach the object to the gripper");
            return false;
        }
    }


    /**
     * Function to detach the object from the gripper
     * @param target_id
     * @return
     */
    bool detachGripper(const int& target_id)
    {
        ros::ServiceClient detach_client = nh.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/detach");
        gazebo_ros_link_attacher::Attach detach_srv;
        // Set the names of the link to be attached and the object to be attached
        if(target_id == 1) // If ID == 1 then detach the blue hexagon
        {
            detach_srv.request.model_name_1 = constant::HEXAGON;
            detach_srv.request.link_name_1 = constant::HEXAGON_LINK;
        }
        else if(target_id == 2) // If ID == 2 then detach the green triangle
        {
            detach_srv.request.model_name_1 = constant::TRIANGLE;
            detach_srv.request.link_name_1 = constant::TRIANGLE_LINK;
        }
        else if(target_id  == 3) // If ID == 3 then detach the red cube
        {
            detach_srv.request.model_name_1 = constant::CUBE;
            detach_srv.request.link_name_1 = constant::CUBE_LINK;
        }
        detach_srv.request.model_name_2 = constant::TIAGO;
        detach_srv.request.link_name_2 = constant::ARM_7_LINK;
        // Call the link_attacher service
        if(detach_client.call(detach_srv))
        {
            ROS_INFO("Object successfully detached to the gripper");
            return true;
        }
        else
        {
            ROS_INFO("Failed to detach the object to the gripper");
            return false;
        }
    }
    /**
     * Function to set the initial configuration of the group_arm
     */
    void initial_tiago_configuration()
    {
        group_arm.setPoseReferenceFrame(constant::BASE_FOOTPRINT);
        group_arm.setMaxVelocityScalingFactor(constant::MAX_VELOCITY_SCALING_FACTOR);
        group_arm.setPlannerId(constant::PLANNER_ID);
        group_arm.setPlanningTime(constant::PLANNING_TIME);
    }

    };


int main(int argc, char **argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "node_C");
    // Create an instance of NodeB
    PickPlace pickplace;
    ros::spin();
    return 0;

}



