#include "ros/ros.h"
#include "cat_move_to_target/GetTagPose.h"


int main(int argc, char** argv){

    ros::init(argc, argv, "get_joint_values_client");

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<cat_move_to_target::GetTagPose>("get_tag_pose");

    cat_move_to_target::GetTagPose srv;
    srv.request.tag_id = 5;

    if(client.call(srv)){
         ROS_INFO("pos_x: %3.4f", srv.response.pos_x);
         ROS_INFO("pos_y: %3.4f", srv.response.pos_y);
         ROS_INFO("pos_z: %3.4f", srv.response.pos_z);
         ROS_INFO("ori_x: %3.4f", srv.response.ori_x);
         ROS_INFO("ori_y: %3.4f", srv.response.ori_y);
         ROS_INFO("ori_y: %3.4f", srv.response.ori_z);
         ROS_INFO("ori_w: %3.4f", srv.response.ori_w);
    }
    else
    {
        ROS_ERROR("Failed to call service ");
    }

    return 0;
}
