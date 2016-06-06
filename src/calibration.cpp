#include <ros/ros.h>
#include <cat_common/cloud_common.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <geometry_msgs/PoseArray.h>
#include <boost/unordered_set.hpp>
#include <boost/unordered_map.hpp>
#include <boost/make_shared.hpp>

#include <sensor_msgs/JointState.h>

#include <apriltags/AprilTagDetections.h>
#include <cat_move_to_target/ApriltagsClass.hpp>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include "tf2_ros/buffer.h"
#include <geometry_msgs/TransformStamped.h>

#include "cat_move_to_target/GetTagPose.h"

#define STOP_ROBOT 0
#define MOVE_ROBOT 1
#define CHANGE_ROBOT_POSITION 2
#define DETECT_TRANSFORM 3
#define TAG_ID 6
#define STOP_COUNTER 4

ros::ServiceClient client;

ros::Publisher pub_tag_pose_array;
ros::Publisher pub_targetTag_pose;
ros::Publisher pub_robot_joints;

ros::Subscriber sub_robot_joints ;

int move_robot_to_calibrate; //0 - don't move, 1- move, 2 - change state
int target_pose_id; // #pose
ros::Subscriber sub_apriltags_detection ;

std::vector <geometry_msgs::TransformStamped> tf_cameraToBase;

geometry_msgs::Pose previous_targetTagPose;

ApriltagsClass ctags;

// (0) subscribe to image and show it (done by 14:32, problems with visualizer in common)
// (1) find tag
// (2) find tag pose
// (3) find conversion from tag pose and robot joint
// (4) find conversion from joint to base frame
// (5) find conversion from camera-end effector


//CloudClass cloud;
//void calibration_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
//    cloud.loadCloud(cloud_msg);
//    cloud.showOriginalCloud(true);
//}


Eigen::MatrixXd rx( double s )
{
    Eigen::MatrixXd R(3,3);

    R << 1.0, 	 0.0, 	  0.0,
         0.0, cos(s), -sin(s),
         0.0, sin(s),  cos(s);

    return R;
}

Eigen::MatrixXd ry( double s )
{
    Eigen::MatrixXd R(3,3);

    R << cos(s), 0.0, sin(s),
         0.0, 	 1.0, 	 0.0,
        -sin(s), 0.0, cos(s);

    return R;
}

Eigen::MatrixXd rz( double s )
{
    Eigen::MatrixXd R(3,3);

    R << cos(s), -sin(s), 0.0,
         sin(s),  cos(s), 0.0,
            0.0,     0.0, 1.0;

    return R;
}

Eigen::MatrixXd rpy2rotation( double r, double p, double y )
{
    Eigen::MatrixXd R = rz(y)*ry(p)*rx(r);

    return R;
}

Eigen::Quaterniond rpy2quaternion( double r, double p, double y )
{
    Eigen::MatrixXd R = rpy2rotation(r, p, y);

    Eigen::Matrix3d R_plus;
    R_plus = R.block(0,0,3,3);

    Eigen::Quaterniond QR;
    QR = R_plus;

    return QR;
}

void clearPreviousTagPose(){
    previous_targetTagPose.orientation.x = 0;
    previous_targetTagPose.orientation.y = 0;
    previous_targetTagPose.orientation.z = 0;
    previous_targetTagPose.orientation.w = 0;
    previous_targetTagPose.position.x = 0;
    previous_targetTagPose.position.y = 0;
    previous_targetTagPose.position.z = 0;
}

void updateRobotAction(){

    std::cout << " target_pose_id :"<<  target_pose_id << std::endl;

    if(target_pose_id != STOP_COUNTER & move_robot_to_calibrate == CHANGE_ROBOT_POSITION)
    {
        target_pose_id += 1;
        move_robot_to_calibrate = MOVE_ROBOT;
        clearPreviousTagPose();
        std::cout << "Must go to pose " << target_pose_id << std::endl;
    }
    else if(move_robot_to_calibrate == CHANGE_ROBOT_POSITION)
    {
        target_pose_id = 0 ;
        move_robot_to_calibrate = MOVE_ROBOT;
     }
}

bool cameraRGBtoBaseFrame(){

    geometry_msgs::TransformStamped tf_geomMsg;
    ROS_INFO("In cameraRGBtoBaseFrame");
    tf::TransformListener listener;
    tf::StampedTransform transformListen;

    ros::Duration(5.0).sleep();

    bool found_tf = false;

    std::string source_frameid = "world";
    std::string child_frameid = "camera_link_aux";

    try{
        listener.waitForTransform(source_frameid, child_frameid, ros::Time(), ros::Duration(5.0) );
        listener.lookupTransform(source_frameid, child_frameid, ros::Time(), transformListen);
        found_tf = true;
    }
    catch (tf::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
        found_tf = false;
    }

    if(found_tf){

        // to make it stable, retrieve again again

        listener.waitForTransform(source_frameid, child_frameid, ros::Time(), ros::Duration(1.0) );
        listener.lookupTransform(source_frameid, child_frameid, ros::Time(), transformListen);
        tf::transformStampedTFToMsg(transformListen, tf_geomMsg);

        tf_cameraToBase.push_back(tf_geomMsg);

        std::cout << "****************************************" <<std::endl;
        std::cout << "cam -> world *****************************" <<std::endl;
        std::cout << tf_geomMsg <<std::endl;
        std::cout << "****************************************" <<std::endl;

        ROS_INFO(" CHANGE_ROBOT_POSITION ");
        move_robot_to_calibrate = CHANGE_ROBOT_POSITION;

        updateRobotAction();
        return true;
    }
    else
        return false;
}

void calibrateRobot(){

    ROS_INFO("--------------------------------------");
    ROS_INFO("Starting calibration");
    ROS_INFO("--------------------------------------");

    ROS_INFO(" tf_cameraToBaseFrame ");

    int size_tfs = tf_cameraToBase.size();

    std::cout << " Size of tf_cameraToBase : " << size_tfs << "/n";

    for(int i =1 ;  i < size_tfs; i++)
        std::cout << tf_cameraToBase[i] << std::endl;


    double roll, pitch, yaw;
    double final_roll, final_pitch, final_yaw;

    geometry_msgs::TransformStamped tf_final;

    for(int i =1 ;  i <= size_tfs; i++){
        if(i == size_tfs){
            tf_final.transform.translation.x /= i-1;
            tf_final.transform.translation.y /= i-1;
            tf_final.transform.translation.z /= i-1;

            final_roll /= i-1;
            final_pitch /= i-1;
            final_yaw /= i-1;

            std::cout << "roll : " << final_roll <<
                         " pitch : " << final_pitch <<
                         " yaw : " << final_yaw << std::endl;

            Eigen::Quaterniond final_quat;
            final_quat = rpy2quaternion( final_roll, final_pitch, final_yaw);

            tf_final.transform.rotation.x = final_quat.x();
            tf_final.transform.rotation.y = final_quat.y();
            tf_final.transform.rotation.z = final_quat.z();
            tf_final.transform.rotation.w = final_quat.w();
        }
        else
        {
            tf_final.transform.translation.x += (tf_cameraToBase[i]).transform.translation.x;
            tf_final.transform.translation.y += (tf_cameraToBase[i]).transform.translation.y;
            tf_final.transform.translation.z += (tf_cameraToBase[i]).transform.translation.z;

            // from quaternion to rpy
            Eigen::Quaterniond q( (tf_cameraToBase[i]).transform.rotation.w,
                    (tf_cameraToBase[i]).transform.rotation.x,
                    (tf_cameraToBase[i]).transform.rotation.y,
                    (tf_cameraToBase[i]).transform.rotation.z);

            Eigen::MatrixXd R = q.toRotationMatrix();

            roll = atan2( R.coeff(2,1), R.coeff(2,2) );
            pitch = atan2( -R.coeff(2,0), sqrt( pow(R.coeff(2,1),2) + pow(R.coeff(2,2),2) ) ) ;
            yaw = atan2 ( R.coeff(1,0) , R.coeff(0,0) ) ;

            final_roll += roll;
            final_pitch += pitch;
            final_yaw += yaw;

            std::cout << "roll : " << roll <<
                         " pitch : " << pitch <<
                         " yaw : " << yaw << std::endl;
        }
    }

    tf_final.header = tf_cameraToBase[0].header;
    tf_final.child_frame_id = tf_cameraToBase[0].child_frame_id;
    tf_final.header.stamp = ros::Time::now();

    ROS_INFO("--------------------------------------");
    ROS_INFO("FINAL TF");
    ROS_INFO("--------------------------------------");

    std::cout << tf_final << std::endl;
}

bool call_tagpose_srv(){
    cat_move_to_target::GetTagPose srv;
    srv.request.tag_id = TAG_ID;

    if(client.call(srv)){
         ROS_INFO("pos_x: %3.7f", srv.response.pos_x);
         ROS_INFO("pos_y: %3.7f", srv.response.pos_y);
         ROS_INFO("pos_z: %3.7f", srv.response.pos_z);
         ROS_INFO("ori_x: %3.7f", srv.response.ori_x);
         ROS_INFO("ori_y: %3.7f", srv.response.ori_y);
         ROS_INFO("ori_y: %3.7f", srv.response.ori_z);
         ROS_INFO("ori_w: %3.7f", srv.response.ori_w);

         if(!cameraRGBtoBaseFrame())
                 ROS_INFO("Not found tf");
         return true;
    }
    else
    {
        ROS_INFO("Failed to call service ");
        return false;
    }
}

void move_robot_callback(const sensor_msgs::JointState::ConstPtr& current_joint_states)    {

    if(move_robot_to_calibrate == MOVE_ROBOT)
    {
         ROS_INFO("Move robot indeed");
        //        std::cout << "Pose : " << current_joint_states->position[0] << " , "
        //                  << current_joint_states->position[1] << " , "
        //                  << current_joint_states->position[2] << " , "
        //                  << current_joint_states->position[3] << " , "
        //                  << current_joint_states->position[4] << " , "
        //                  << current_joint_states->position[5] << " , "
        //                  << current_joint_states->position[6] << std::endl;

        sensor_msgs::JointState msg;
        msg.header.stamp = ros::Time::now();
        int num_joints = 6;
        msg.name.resize(num_joints);
        msg.position.resize(num_joints);

        msg.name[0] = "joint1";
        msg.name[1] = "joint2";
        msg.name[2] = "joint3";
        msg.name[3] = "joint4";
        msg.name[4] = "joint5";
        msg.name[5] = "joint6";

        msg.position[3] = 1.7;
        msg.position[4] = -1.10;
        msg.position[5] = 0;

        if ( target_pose_id == 1 ){
            ROS_INFO("Going to Pose 1");
            msg.position[0] = -0.8;
            msg.position[1] = 0.4;
            msg.position[2] = 0.1;
        }
        else if (target_pose_id == 2){
            ROS_INFO("Going to Pose 2");
            msg.position[0] = -0.75;
            msg.position[1] = 0.4;
            msg.position[2] = 0.1;
        }
        else if (target_pose_id == 3){
            ROS_INFO("Going to Pose 3");
            msg.position[0] = -0.75;
            msg.position[1] = 0.3;
            msg.position[2] = 0.2;
        }
        else if (target_pose_id == 4){
            ROS_INFO("Going to Pose 4");
            msg.position[0] = -0.8;
            msg.position[1] = 0.3;
            msg.position[2] = 0.2;
        }
        else if (target_pose_id == 5){
            ROS_INFO("Going to Pose 5 (back to 1)");
            msg.position[0] = -0.8;
            msg.position[1] = 0.4;
            msg.position[2] = 0.1;
        }
//        else if (target_pose_id == 6){
//            ROS_INFO("Going to Pose 6");
//            msg.position[0] = -0.65;
//            msg.position[1] = 0;
//            msg.position[2] = 0.7;
//        }
//        else if ( target_pose_id == 7 ){
//            ROS_INFO("Going to Pose 1");
//            msg.position[0] = -0.65;
//            msg.position[1] = -0.1;
//            msg.position[2] = 0.8;
//        }
        else{
            ROS_INFO("Going REST");
            msg.position[0] = -0.8;
            msg.position[1] = 0.4;
            msg.position[2] = 0.1;
            sub_robot_joints.shutdown();
            sub_apriltags_detection.shutdown();
            calibrateRobot();
        }
        pub_robot_joints.publish(msg);

        ros::Duration(5.0).sleep();

        // check if robot moved
        double error_allowed = 0.1;
        if ( fabs(msg.position[0] - current_joint_states->position[0]) < error_allowed &
             fabs(msg.position[1] - current_joint_states->position[1]) < error_allowed &
             fabs(msg.position[2] - current_joint_states->position[2]) < error_allowed &
             fabs(msg.position[3] - current_joint_states->position[3]) < error_allowed &
             fabs(msg.position[4] - current_joint_states->position[4]) < error_allowed &
             fabs(msg.position[5] - current_joint_states->position[5]) < error_allowed)
        {
            ROS_INFO("CHECKUP : Robot Moved");
            ros::Duration(2.0).sleep();

            call_tagpose_srv();

        }
        else
        {
            move_robot_to_calibrate = MOVE_ROBOT;
        }

    }
}

int main(int argc, char** argv){

   move_robot_to_calibrate = MOVE_ROBOT;
   target_pose_id = 1 ;

   ros::init(argc, argv, "robot_camera_calibration");

   ros::NodeHandle nh;

   ros::AsyncSpinner spinner(1);
   spinner.start();

   sub_robot_joints = nh.subscribe("/joint_states",1, move_robot_callback);

   pub_tag_pose_array = nh.advertise<geometry_msgs::PoseArray>("tag_pose",1);
   pub_targetTag_pose = nh.advertise<geometry_msgs::PoseStamped>("tag3_pose",1);

   client = nh.serviceClient<cat_move_to_target::GetTagPose>("get_tag_pose");

   pub_robot_joints = nh.advertise<sensor_msgs::JointState>("/controller_joint_states",1);

   ros::spin();

   return 0;

}
