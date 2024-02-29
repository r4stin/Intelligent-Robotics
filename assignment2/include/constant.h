#ifndef ASSIGNMENT2_CONSTANT_H
#define ASSIGNMENT2_CONSTANT_H

#include <string>

namespace constant{

    // FRAMES
    static const std::string BASE_FOOTPRINT = "base_footprint";
    static const std::string MAP = "map";





    // JOINTS
    static const std::string GRIPPER_LEFT_FINGER_JOINT = "gripper_left_finger_joint";
    static const std::string GRIPPER_RIGHT_FINGER_JOINT = "gripper_right_finger_joint";

    // ACTION FOR FOLLOW JOINT TRAJECTORY
    static const std::string HEAD_FOLLOW_JOINT_TRAJECTORY_ACTION = "/head_controller/follow_joint_trajectory";
    static const std::string GRIPPER_FOLLOW_JOINT_TRAJECTORY_ACTION = "/gripper_controller/follow_joint_trajectory";
    // PLANNER_ID
    static const std::string PLANNER_ID = "SBLkConfigDefault";
    // INTERFACES
    static const std::string INTERFACE_ARM_TORSO = "arm_torso";

    // MAX_VELOCITY_SCALING_FACTOR
    static const double MAX_VELOCITY_SCALING_FACTOR = 1.0;

    // PLANNING TIME
    static const double PLANNING_TIME = 70.0;

    // GAZEBO NAMES
    static const std::string HEXAGON = "Hexagon";
    static const std::string TRIANGLE = "Triangle";
    static const std::string CUBE = "cube";
    static const std::string TIAGO = "tiago";


    // LINKS
    static const std::string ARM_TOOL_LINK = "arm_tool_link";
    static const std::string ARM_7_LINK = "arm_7_link";
    static const std::string HEXAGON_LINK = "Hexagon_link";
    static const std::string TRIANGLE_LINK = "Triangle_link";
    static const std::string CUBE_LINK = "cube_link";


    // ATTACH DETEACH MODALITIES
    static const std::string ATTACH_MODALITY = "attach";
    static const std::string DETACH_MODALITY = "detach";

    // COLLISION OBJECTS
    static const std::string PICK_TABLE_COLLISION_OBJECT = "pick_table";


    // COLLISION OBJECTS DIMENSIONS
    static const double PICK_TABLE_LENGTH = 0.913;
    static const double PICK_TABLE_WIDTH = 0.913;
    static const double PICK_TABLE_HEIGHT = 0.775;


    static const double PLACE_TABLE_HEIGHT = 0.69;
    static const double PLACE_TABLE_RADIUS = 0.225;



}

#endif //ASSIGNMENT2_CONSTANT_H

