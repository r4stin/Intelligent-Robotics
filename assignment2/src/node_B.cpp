#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <apriltag_ros/AprilTagDetection.h>

#include <apriltag_ros/AprilTagDetection.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <actionlib/server/simple_action_server.h>
#include <assignment2/TagDetectionAction.h>


class AprilTagDetectionServer{

protected:

    ros::NodeHandle nh;
    actionlib::SimpleActionServer <assignment2::TagDetectionAction> as_;
    bool detection_done = false;
    assignment2::TagDetectionResult result;
    assignment2::TagDetectionFeedback feedback;


public:




    // Initialization of server action (constructor)
    AprilTagDetectionServer(std::string name) : as_(nh, name, boost::bind(&AprilTagDetectionServer::aprilCallback, this, _1), false){
        as_.start();
    }

    ~AprilTagDetectionServer(void) {}




private:
/**
 * Receives the data from the topic /tag_detections
 * @param data
 * @return
 */
    bool updateArrayTags(apriltag_ros::AprilTagDetectionArrayConstPtr& data)
    {
        data = ros::topic::waitForMessage<apriltag_ros::AprilTagDetectionArray>("/tag_detections", nh, ros::Duration(5.0));
        if (data != NULL)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
/**
 * Function to transform the pose of the camera to the pose of the base_footprint
 * @param pose
 * @param transform
 * @return
 */
    geometry_msgs::Pose transformPose(const geometry_msgs::Pose &pose, const tf::StampedTransform &transform) {
        tf::Vector3 v(pose.position.x, pose.position.y, pose.position.z);
        v = transform * v;
        tf::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
        q = transform * q;

        geometry_msgs::Pose tmp_pose;
        tmp_pose.position.x = v.x();
        tmp_pose.position.y = v.y();
        tmp_pose.position.z = v.z();
        tmp_pose.orientation.x = q.x();
        tmp_pose.orientation.y = q.y();
        tmp_pose.orientation.z = q.z();
        tmp_pose.orientation.w = q.w();
        return tmp_pose;
    }

    /**
     * Function to transform the pose of the camera to the pose of the base_footprint
     * @param data
     */
    void aprilCallback(const assignment2::TagDetectionGoalConstPtr &goal) {

        apriltag_ros::AprilTagDetectionArrayConstPtr data = NULL;
        tf::TransformListener listener;
        tf::StampedTransform transform;


        ROS_INFO("Starting the detection");
        updateStatus(1);
        while (! updateArrayTags(data));

        std::string target_frame = "base_footprint";
        std::string source_frame = data->header.frame_id.c_str();
        updateStatus(2);
        try {
            listener.waitForTransform(target_frame, source_frame, ros::Time(0), ros::Duration(10.0));
            listener.lookupTransform(target_frame, source_frame, ros::Time(0), transform);
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("Received an exception trying to transform a point : %s", ex.what());

            return;
        }



        // Search for desire marker
        updateStatus(3);
        int obstacle_iterator = 0;
        for (int i = 0; i < data->detections.size(); i++) {

            if (data->detections.at(i).id.at(0) == goal->marker_ID) {


                result.marker_pose = transformPose(data->detections.at(i).pose.pose.pose, transform);
                result.marker_ID = data->detections.at(i).id.at(0);
                result.marker_size = data->detections.at(i).size.at(0);
                ROS_INFO_STREAM("tag_" << data->detections.at(i).id.at(0));
                ROS_INFO_STREAM(result.marker_pose);

                detection_done = true;
            }
            else {
                geometry_msgs::Pose tmp = transformPose(data->detections.at(i).pose.pose.pose, transform);
                result.obstacles_pose.push_back(tmp);
                result.obstacles_ID.push_back(data->detections.at(i).id.at(0));
                result.obstacles_size.push_back(data->detections.at(i).size.at(0));


            }


        }

        if(detection_done){
            as_.setSucceeded(result);
        }
    }

    /**
     * Function that updates the detection status
     * @param val integer value of the detection status
     */
    void updateStatus(int val) {
        std::string current_status;
        // Update parameters depending on the integer value
        switch (val) {
            case 1:
                current_status = "Starting Detection";
                break;
            case 2:
                current_status = "Transforming to base_footprint frame";
                break;
            case 3:
                current_status = "Setting up the results...";
                break;
        }
        // Update the feedback for the user
        feedback.status = current_status;
        // Send it to user
        as_.publishFeedback(feedback);
    }

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "node_b");

    AprilTagDetectionServer server("node_b");
    ros::spin();

    return 0;
}
