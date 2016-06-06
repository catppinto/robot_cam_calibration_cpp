#include <ros/ros.h>
#include <apriltags/AprilTagDetections.h>
#include <cat_move_to_target/ApriltagsClass.hpp>
#include <Eigen/Dense>
#include <fstream>
#include "cat_move_to_target/GetTagPose.h"
#include <../include/cat_robotcam_calibration/calculateExtrinsics.hpp>

CamExtrinsicsCalculus camExtr;

int main(int argc, char** argv){

    //init ros, subscribers, publishers and services
    ros::init(argc, argv, "runFixedCalibration");

    ros::NodeHandle nh;

    camExtr.LoadXYZWORLDtags();
    camExtr.LoadUVCAMtags_hardcoded();
    camExtr.CalculateExtrinsics_WldUvPoints();
}
