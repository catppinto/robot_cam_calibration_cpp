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

#include <fstream>

#include "cat_move_to_target/GetTagPose.h"

#define STOP_ROBOT 0
#define MOVE_ROBOT 1
#define CHANGE_ROBOT_POSITION 2
#define DETECT_TRANSFORM 3
#define TAG_ID 6
#define STOP_COUNTER 4

std::vector<sensor_msgs::JointState> all_calibration_positions;

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

std::ofstream storing_file;

// (0) subscribe to image and show it (done by 14:32, problems with visualizer in common)
// (1) find tag
// (2) find tag pose
// (3) find conversion from tag pose and robot joint
// (4) find conversion from joint to base frame
// (5) find conversion from camera-end effector


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

   // if(target_pose_id != STOP_COUNTER & move_robot_to_calibrate == CHANGE_ROBOT_POSITION)
    if(target_pose_id <all_calibration_positions.size() & move_robot_to_calibrate == CHANGE_ROBOT_POSITION)
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

        //storing in file
        storing_file << " TF WORLD TO CAMERA: \n ";
        storing_file << tf_geomMsg << "\n";

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
        if(i == size_tfs & i!=1){
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
        else if(i == size_tfs){
             tf_final.transform.translation.x /= i;
             tf_final.transform.translation.y /= i;
             tf_final.transform.translation.z /= i;

             final_roll /= i;
             final_pitch /= i;
             final_yaw /= i;

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
    storing_file << " ****************************** \n"
                 << " ------------------------------ \n";
    storing_file << " FINAL TF WORLD TO CAMERA: \n ";
    storing_file << tf_final << "\n";
    storing_file << " ------------------------------ \n"
                 <<  " ****************************** \n";

    //closing file
    storing_file.close();
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

         storing_file << " TAG POSITION: \n ";
         storing_file << "pos_x : " <<  srv.response.pos_x << "\n";
         storing_file << "pos_y : " <<  srv.response.pos_y << "\n";
         storing_file << "pos_z : " <<  srv.response.pos_z << "\n";
         storing_file << "ori_x : " <<  srv.response.ori_x << "\n";
         storing_file << "ori_y : " <<  srv.response.ori_y << "\n";
         storing_file << "ori_z : " <<  srv.response.ori_z << "\n";
         storing_file << "ori_w : " <<  srv.response.ori_w << "\n";

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

        sensor_msgs::JointState joint_msg;
        if ( (target_pose_id-1) < all_calibration_positions.size()){
            ROS_INFO("Going to Pose %d", target_pose_id);
            joint_msg.header.stamp = ros::Time::now();
            joint_msg.position = (all_calibration_positions[target_pose_id-1]).position;
            joint_msg.name = (all_calibration_positions[target_pose_id-1]).name;
            std::cout << joint_msg << std::endl;
        }
        else{
            ROS_INFO("Going REST");
            joint_msg.header.stamp = ros::Time::now();
            joint_msg.name.resize(6);
            joint_msg.name[0] = "joint1";
            joint_msg.name[1] = "joint2";
            joint_msg.name[2] = "joint3";
            joint_msg.name[3] = "joint4";
            joint_msg.name[4] = "joint5";
            joint_msg.name[5] = "joint6";
            joint_msg.position.resize(6);
            joint_msg.position[0] = -1.2;
            joint_msg.position[1] = 0;
            joint_msg.position[2] = 0;
            joint_msg.position[3] = 0;
            joint_msg.position[4] = 0;
            joint_msg.position[5] = 0;
            sub_robot_joints.shutdown();
            sub_apriltags_detection.shutdown();
            calibrateRobot();
        }
        pub_robot_joints.publish(joint_msg);

        ros::Duration(5.0).sleep();

        // check if robot moved
        double error_allowed = 0.1;
        if ( fabs(joint_msg.position[0] - current_joint_states->position[0]) < error_allowed &
             fabs(joint_msg.position[1] - current_joint_states->position[1]) < error_allowed &
             fabs(joint_msg.position[2] - current_joint_states->position[2]) < error_allowed &
             fabs(joint_msg.position[3] - current_joint_states->position[3]) < error_allowed &
             fabs(joint_msg.position[4] - current_joint_states->position[4]) < error_allowed &
             fabs(joint_msg.position[5] - current_joint_states->position[5]) < error_allowed)
        {
            ROS_INFO("CHECKUP : Robot Moved");

            //store in file
            storing_file << "****************************************\n" ;
            storing_file << " POSE " << target_pose_id  << "\n****************************************\n" ;
            storing_file << " JOINTS: \n ";

            for (int i=0; i< joint_msg.position.size(); i++)
                storing_file << "joint " << i <<  " : " << joint_msg.position[i] << "\n";
            ros::Duration(2.0).sleep();

            if( !call_tagpose_srv() )
                ROS_INFO("wHAT???");

        }
        else
        {
            move_robot_to_calibrate = MOVE_ROBOT;
        }

    }
}

void loadCalibrationJointPositions()  {

    sensor_msgs::JointState msg;
    int num_joints = 6;
    msg.name.resize(num_joints);
    msg.position.resize(num_joints);
    msg.name[0] = "joint1";
    msg.name[1] = "joint2";
    msg.name[2] = "joint3";
    msg.name[3] = "joint4";
    msg.name[4] = "joint5";
    msg.name[5] = "joint6";

    msg.position[0] = -1.4;        msg.position[1] = -0.3;        msg.position[2] = 1;
    msg.position[3] = 0;        msg.position[4] = 0.2;         msg.position[5] = 0;
    all_calibration_positions.push_back(msg);
//this one
    msg.position[0] = -1.4;        msg.position[1] = 0.4;        msg.position[2] = 0.7;
    msg.position[3] = 0.2;        msg.position[4] = 0.1;         msg.position[5] = 0;
    all_calibration_positions.push_back(msg);

    msg.position[0] = -1.4;        msg.position[1] = -0.1;        msg.position[2] = 0.8;
    msg.position[3] = 0;        msg.position[4] = 0.2;         msg.position[5] = 0;
    all_calibration_positions.push_back(msg);

    msg.position[0] = -1.2;        msg.position[1] = -0.6;        msg.position[2] = 1.3;
    msg.position[3] = 0;        msg.position[4] = 0.1;         msg.position[5] = 0;
    all_calibration_positions.push_back(msg);

    msg.position[0] = -1.4;        msg.position[1] = -0.4;        msg.position[2] = 1.2;
    msg.position[3] = 0;        msg.position[4] = 0;         msg.position[5] = 0;
    all_calibration_positions.push_back(msg);

    msg.position[0] = -1.5;        msg.position[1] = -1;        msg.position[2] = 1.8;
    msg.position[3] = 0.2;        msg.position[4] = 0;         msg.position[5] = 0;
    all_calibration_positions.push_back(msg);

    msg.position[0] = -1.5;        msg.position[1] = -0.5;        msg.position[2] = 1.3;
    msg.position[3] = 0.2;        msg.position[4] = 0;         msg.position[5] = 0;
    all_calibration_positions.push_back(msg);

    msg.position[0] = -1.5;        msg.position[1] = -0.5;        msg.position[2] = 1.3;
    msg.position[3] = 0.0;        msg.position[4] = 0;         msg.position[5] = 0;
    all_calibration_positions.push_back(msg);

    msg.position[0] = -1.7;        msg.position[1] = -0.5;        msg.position[2] = 1.3;
    msg.position[3] = -0.2;        msg.position[4] = 0;         msg.position[5] = 0;
    all_calibration_positions.push_back(msg);

    msg.position[0] = -1.2;        msg.position[1] = 0.55;        msg.position[2] = 0.25;
    msg.position[3] = 0.2;        msg.position[4] = 0;         msg.position[5] = 0;
    all_calibration_positions.push_back(msg);
//this one
    msg.position[0] = -1.4;        msg.position[1] = 0.55;        msg.position[2] = 0.25;
    msg.position[3] = 0.2;        msg.position[4] = 0;         msg.position[5] = 0;
    all_calibration_positions.push_back(msg);
//this one
    msg.position[0] = -1.4;        msg.position[1] = 0.5;        msg.position[2] = 0.2;
    msg.position[3] = 0.2;        msg.position[4] = 0;         msg.position[5] = 0;
    all_calibration_positions.push_back(msg);
//this one
    msg.position[0] = -1.4 ;       msg.position[1] = 0.5;        msg.position[2] = 0.3;
    msg.position[3] = 0.2;        msg.position[4] = 0;         msg.position[5] = 0;
   all_calibration_positions.push_back(msg);
//this one
    msg.position[0] = -1.4;        msg.position[1] = 0.3;        msg.position[2] = 0.45;
    msg.position[3] = 0.2;        msg.position[4] = 0;         msg.position[5] = 0;
    all_calibration_positions.push_back(msg);

    msg.position[0] = -1.6;        msg.position[1] = 0;        msg.position[2] = 1.2;
    msg.position[3] = 0.2;        msg.position[4] = 0;         msg.position[5] = 0;
   all_calibration_positions.push_back(msg);

    msg.position[0] = -1.4;        msg.position[1] = -0.3;        msg.position[2] = 1.4;
    msg.position[3] = 0.2;        msg.position[4] = 0;         msg.position[5] = 0;
    all_calibration_positions.push_back(msg);

    msg.position[0] = -1.4;        msg.position[1] = 0.1;        msg.position[2] = 1.4;
    msg.position[3] = 0.5;        msg.position[4] = 0.785;         msg.position[5] = 0;
    all_calibration_positions.push_back(msg);

    msg.position[0] = -1.4;        msg.position[1] = 0.5;        msg.position[2] = 0.8;
    msg.position[3] = 0.5;        msg.position[4] = 0.785;         msg.position[5] = 0;
    all_calibration_positions.push_back(msg);

    msg.position[0] = -1.5;        msg.position[1] = 0.5;        msg.position[2] = 0.8;
    msg.position[3] = 0.5;        msg.position[4] = 0.785;         msg.position[5] = 0;
    all_calibration_positions.push_back(msg);

    msg.position[0] = -1.2;        msg.position[1] = 0.5;        msg.position[2] = 0.8;
    msg.position[3] = 0.5;        msg.position[4] = 0.785;         msg.position[5] = 0;
    all_calibration_positions.push_back(msg);

//    for (int i=0; i<all_calibration_positions.size(); i++)
//        std::cout << all_calibration_positions[i] << std::endl;

}

int main(int argc, char** argv){

   move_robot_to_calibrate = MOVE_ROBOT;
   target_pose_id = 1 ;

   loadCalibrationJointPositions();

   ros::init(argc, argv, "robot_camera_calibration");

   ros::NodeHandle nh;

   ros::AsyncSpinner spinner(1);
   spinner.start();

   sub_robot_joints = nh.subscribe("/joint_states",1, move_robot_callback);

   pub_tag_pose_array = nh.advertise<geometry_msgs::PoseArray>("tag_pose",1);
   pub_targetTag_pose = nh.advertise<geometry_msgs::PoseStamped>("tag3_pose",1);

   client = nh.serviceClient<cat_move_to_target::GetTagPose>("get_tag_pose");

   pub_robot_joints = nh.advertise<sensor_msgs::JointState>("/controller_joint_states",1);

   storing_file.open("/home/cat/catkin_ws/src/biovision/cat_robotcam_calibration/files/logfile_calibration.txt");

   ros::spin();

   return 0;

}
