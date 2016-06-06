#include <ros/ros.h>
#include <apriltags/AprilTagDetections.h>
#include <cat_move_to_target/ApriltagsClass.hpp>
#include <Eigen/Dense>
#include <fstream>
#include "cat_move_to_target/GetTagPose.h"
#include <../include/cat_robotcam_calibration/calculateExtrinsics.hpp>

// see NOTE in botton to use w pixels instead of xyz cam point. still not working fully

Eigen::MatrixXd RotationFromAngleAxis(Eigen::VectorXd angle_axis);

// ------------------------Globals------------------------------------
ApriltagsClass ctag;

//ros::Publisher pub_tag_pose_array;
//ros::Publisher pub_targetTag_pose;

ros::Subscriber sub_apriltags_detection ;
CamExtrinsicsCalculus camExtr;

// ------------------------Globals------------------------------------

bool checkClearPoseContent(geometry_msgs::PoseStamped& p){
    if(p.pose.position.x == 0 & p.pose.position.y == 0 & p.pose.position.z == 0 &
            p.pose.orientation.w ==0 & p.pose.orientation.x ==0 & p.pose.orientation.y ==0 & p.pose.orientation.z ==0 )
        return true;
    else
        return false;
}

bool getTagPoses(){

    ctag.init_noIDinput();

    while(!ctag.getTagIds())
    {    }

    std::map<int, geometry_msgs::PoseStamped> tags_pose;
    ctag.getAllTagsPose(tags_pose);

    while(!ctag.checkAllTagsStability(tags_pose)){
        ctag.getAllTagsPose(tags_pose);
    }

    geometry_msgs::PoseStamped ps;
    for (std::map<int, geometry_msgs::PoseStamped>::iterator it = tags_pose.begin(); it != tags_pose.end(); it++)
    {
        ps = it->second;
        ROS_INFO("----------------------------------");
        camExtr.addTagId(it->first);
        camExtr.addCamToTagPose(ps);
        ROS_INFO("----------------------------------");
    }

    camExtr.CalculateExtrinsics();
}

int main(int argc, char** argv){

    //init ros, subscribers, publishers and services
    ros::init(argc, argv, "fixedTagCalibration");

    ros::NodeHandle nh;

    getTagPoses();
}


// *****************************************************************************************************
Eigen::MatrixXd RotationFromAngleAxis(Eigen::VectorXd angle_axis)
{
    double theta = angle_axis(0);
    double wx = angle_axis(1);
    double wy = angle_axis(2);
    double wz = angle_axis(3);

    double v_theta = 1-cos(theta) ; double c_theta = cos(theta); double s_theta = sin(theta);

    Eigen::MatrixXd rotation_matrix(3,3);
    rotation_matrix <<  wx*wx*v_theta+c_theta       , wx*wy*v_theta-wz*s_theta  , wx*wz*v_theta+wy*s_theta,
            wx*wy*v_theta+wz*s_theta    , wy*wy*v_theta+c_theta     , wy*wz*v_theta-wx*s_theta,
            wx*wz*v_theta-wy*s_theta    , wy*wz*v_theta+wx*s_theta  , wz*wz*v_theta+c_theta;

    return rotation_matrix;
}


/*
 *
 *     TO WORK WITH PIXELS
 *
 *
   loadXYZtags(xyz_world, xyz_cam, tagIds);

    //loadTagsFromCorners(uv_cam, tagIds);

    CamExtrinsicsCalculus cam_calib;
    //std::vector<Eigen::Vector3d> xyz_normalized =
    //       cam_calib.cameraPixelsToUndistortXYZPoints(uv_cam); //TODO NOT COMPLETE

    std::cout << "xyz_cam : " << xyz_cam.size() << std::endl;
    std::cout << "xyz_world : " << xyz_world.size() << std::endl;
    //    for (int i = 0; i< xyz_normalized.size(); i++)
    //    {
    //        Eigen::Vector3d n = xyz_normalized[i];
    //        Eigen::Vector3d w = xyz_world[i];

    //        std::cout << "xyz_normalized : " << n(0) << " , "
    //                  << n(1) << " , "
    //                  << n(2) << std::endl;
    //        std::cout << "xyz_world : " << w(0) << " , "
    //                  << w(1) << " , "
    //                  << w(2) << std::endl;
    //    }

    Eigen::MatrixXd m_world_cam = cam_calib.constructPmatrix(xyz_world, xyz_cam, tagIds);
    cam_calib.getRotAndTfromPmatrix(m_world_cam);
 *
 * */
