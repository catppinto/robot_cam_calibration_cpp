#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <boost/unordered_set.hpp>
#include <boost/unordered_map.hpp>
#include <boost/make_shared.hpp>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include "tf2_ros/buffer.h"
#include <geometry_msgs/TransformStamped.h>

#include <fstream>


// this should be converted to a service

int main(int argc, char** argv){

    std::ofstream storingfile;
    storingfile.open("/home/cat/catkin_ws/src/biovision/cat_robotcam_calibration/files/logfile_conversions.txt");

   ros::init(argc, argv, "transformConverter");
   double rate_hz;

   ros::NodeHandle nh;
   nh.param("rate", rate_hz, 1.0);
   ros::Rate rate(rate_hz);

   ros::Duration(10).sleep();

   geometry_msgs::TransformStamped tf_auxiliar, tf_toSend;

    // Wait for up to one second for the first transforms to become avaiable.
   //cat
    std::string source_frameid , child_frameid  ;
    tf::TransformListener listener;

    std::vector <geometry_msgs::TransformStamped> tf_pickPoses;
    tf2_ros::StaticTransformBroadcaster Sbr;


    while(nh.ok())
    {

        storingfile << "*************************************************\n ";
        tf_pickPoses.clear();

        // -----------------------------------------------------------------------------------------
        // camera_rgb_optical_frame -> camera_rgb_frame
        // -----------------------------------------------------------------------------------------
        source_frameid = "camera_rgb_optical_frame";
        child_frameid = "camera_rgb_frame";
        try
        {
            tf::StampedTransform transformListen;
            listener.waitForTransform(source_frameid, child_frameid, ros::Time(), ros::Duration(10.0));
            listener.lookupTransform(source_frameid, child_frameid, ros::Time(), transformListen);
            tf::transformStampedTFToMsg(transformListen, tf_auxiliar);
            //std::cout << " camera_rgb_optical_frame -> camera_rgb_frame " << std::endl << tf_auxiliar << std::endl;
        }
        catch(tf::TransformException& ex)
        {
               ROS_WARN("Exception thrown: %s" , ex.what());
        }
        rate.sleep();
        tf_toSend.header.frame_id = "camera_rgb_optical_frame_aux";
        tf_toSend.child_frame_id = "camera_rgb_frame_aux";
        tf_toSend.header.stamp = tf_auxiliar.header.stamp;
        tf_toSend.transform = tf_auxiliar.transform;
        tf_pickPoses.push_back(tf_toSend);
        storingfile << tf_toSend << "\n";

        // -----------------------------------------------------------------------------------------
        // camera_rgb_frame -> camera_link
        // -----------------------------------------------------------------------------------------
        source_frameid = "camera_rgb_frame";
        child_frameid = "camera_link";
        try
        {
            tf::StampedTransform transformListen;
            listener.waitForTransform(source_frameid, child_frameid, ros::Time(), ros::Duration(10.0));
            listener.lookupTransform(source_frameid, child_frameid, ros::Time(), transformListen);
            tf::transformStampedTFToMsg(transformListen, tf_auxiliar);
            //std::cout << " camera_rgb_frame -> camera_link " << std::endl << tf_auxiliar << std::endl;
        }
        catch(tf::TransformException& ex)
        {
               ROS_WARN("Exception thrown: %s" , ex.what());
        }
        rate.sleep();
        tf_toSend.header.frame_id = "camera_rgb_frame_aux";
        tf_toSend.child_frame_id = "camera_link_aux";
        tf_toSend.header.stamp = tf_auxiliar.header.stamp;
        tf_toSend.transform = tf_auxiliar.transform;
        tf_pickPoses.push_back(tf_toSend);
        storingfile << tf_toSend << "\n";

        // -----------------------------------------------------------------------------------------
        // TAG -> CAMERA_RGB_OPTICAL_FRAME
        // -----------------------------------------------------------------------------------------
//        source_frameid = "target_tag";
        source_frameid = "tag6";
        child_frameid = "camera_rgb_optical_frame";
        try
        {
            tf::StampedTransform transformListen;
            listener.waitForTransform(source_frameid, child_frameid, ros::Time(), ros::Duration(20.0));
            listener.lookupTransform(source_frameid, child_frameid, ros::Time(), transformListen);
            tf::transformStampedTFToMsg(transformListen, tf_auxiliar);
            //std::cout << " tag->camera_rgb_of " << std::endl << tf_auxiliar << std::endl;

        }
        catch(tf::TransformException& ex)
        {
               ROS_ERROR("Exception thrown: %s" , ex.what());
        }
        rate.sleep();
        tf_toSend.header.frame_id = "tag1";
        tf_toSend.child_frame_id = "camera_rgb_optical_frame_aux";
        tf_toSend.header.stamp = tf_auxiliar.header.stamp;
        tf_toSend.transform = tf_auxiliar.transform;
        tf_pickPoses.push_back(tf_toSend);
        storingfile << tf_toSend << "\n";


        Sbr.sendTransform(tf_pickPoses);

        storingfile << "END*************************************************\n\n ";


    }
    storingfile.close();
    return 0;

}
