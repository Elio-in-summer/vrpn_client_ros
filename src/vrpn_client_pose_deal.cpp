#include <iostream>
#include <fstream>
#include <sstream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TwistStamped.h>

using namespace std;

geometry_msgs::PoseStamped uav0_cur_pose, uav1_cur_pose, uav2_cur_pose, uav3_cur_pose, uav4_cur_pose;
bool uav0_first_rece = true, uav1_first_rece = true, uav2_first_rece = true, uav3_first_rece = true, uav4_first_rece = true;

double uav0_init_x = 0.0, uav0_init_y = 0.0, uav0_init_z = 0.0;
double uav1_init_x = 0.0, uav1_init_y = 0.0, uav1_init_z = 0.0;
double uav2_init_x = 0.0, uav2_init_y = 0.0, uav2_init_z = 0.0;
double uav3_init_x = 0.0, uav3_init_y = 0.0, uav3_init_z = 0.0;
double uav4_init_x = 0.0, uav4_init_y = 0.0, uav4_init_z = 0.0;

int ctrl_rate = 30;


void uav0_vision_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    if (uav0_first_rece == true){
        uav0_init_x = (*msg).pose.position.x;
        uav0_init_y = (*msg).pose.position.y;
        uav0_init_z = (*msg).pose.position.z;
        uav0_first_rece = false;
    }
    uav0_cur_pose = *msg;
    uav0_cur_pose.pose.position.x = uav0_cur_pose.pose.position.x - uav0_init_x;
    uav0_cur_pose.pose.position.y = uav0_cur_pose.pose.position.y - uav0_init_y;
    uav0_cur_pose.pose.position.z = uav0_cur_pose.pose.position.z - uav0_init_z;
}

void uav1_vision_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    if (uav1_first_rece == true){
        uav1_init_x = (*msg).pose.position.x;
        uav1_init_y = (*msg).pose.position.y;
        uav1_init_z = (*msg).pose.position.z;
        uav1_first_rece = false;
    }
    uav1_cur_pose = *msg;
    uav1_cur_pose.pose.position.x = uav1_cur_pose.pose.position.x - uav1_init_x;
    uav1_cur_pose.pose.position.y = uav1_cur_pose.pose.position.y - uav1_init_y;
    uav1_cur_pose.pose.position.z = uav1_cur_pose.pose.position.z - uav1_init_z;
}

void uav2_vision_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    if (uav2_first_rece == true){
        uav2_init_x = (*msg).pose.position.x;
        uav2_init_y = (*msg).pose.position.y;
        uav2_init_z = (*msg).pose.position.z;
        uav2_first_rece = false;
    }
    uav2_cur_pose = *msg;
    uav2_cur_pose.pose.position.x = uav2_cur_pose.pose.position.x - uav2_init_x;
    uav2_cur_pose.pose.position.y = uav2_cur_pose.pose.position.y - uav2_init_y;
    uav2_cur_pose.pose.position.z = uav2_cur_pose.pose.position.z - uav2_init_z;
}

void uav3_vision_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    if (uav3_first_rece == true){
        uav3_init_x = (*msg).pose.position.x;
        uav3_init_y = (*msg).pose.position.y;
        uav3_init_z = (*msg).pose.position.z;
        uav3_first_rece = false;
    }
    uav3_cur_pose = *msg;
    uav3_cur_pose.pose.position.x = uav3_cur_pose.pose.position.x - uav3_init_x;
    uav3_cur_pose.pose.position.y = uav3_cur_pose.pose.position.y - uav3_init_y;
    uav3_cur_pose.pose.position.z = uav3_cur_pose.pose.position.z - uav3_init_z;
}

void uav4_vision_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    if (uav4_first_rece == true){
        uav4_init_x = (*msg).pose.position.x;
        uav4_init_y = (*msg).pose.position.y;
        uav4_init_z = (*msg).pose.position.z;
        uav4_first_rece = false;
    }
    uav4_cur_pose = *msg;
    uav4_cur_pose.pose.position.x = uav4_cur_pose.pose.position.x - uav4_init_x;
    uav4_cur_pose.pose.position.y = uav4_cur_pose.pose.position.y - uav4_init_y;
    uav4_cur_pose.pose.position.z = uav4_cur_pose.pose.position.z - uav4_init_z;
}


int main(int argc, char** argv){
    
    // ros init
    ros::init(argc, argv, "leader_selection", ros::init_options::AnonymousName);
    ros::NodeHandle nh;

    ros::Subscriber uav0_vision_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/uav0/mavros/vision/pose", 1, uav0_vision_cb);
    ros::Subscriber uav1_vision_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/uav1/mavros/vision/pose", 1, uav1_vision_cb);
    ros::Subscriber uav2_vision_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/uav2/mavros/vision/pose", 1, uav2_vision_cb);
    ros::Subscriber uav3_vision_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/uav3/mavros/vision/pose", 1, uav3_vision_cb);
    ros::Subscriber uav4_vision_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/uav4/mavros/vision/pose", 1, uav4_vision_cb);

    ros::Publisher uav0_vision_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/uav0/mavros/vision_pose/pose", 1);
    ros::Publisher uav1_vision_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/uav1/mavros/vision_pose/pose", 1);
    ros::Publisher uav2_vision_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/uav2/mavros/vision_pose/pose", 1);
    ros::Publisher uav3_vision_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/uav3/mavros/vision_pose/pose", 1);
    ros::Publisher uav4_vision_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/uav4/mavros/vision_pose/pose", 1);

    ros::Rate ctrl_loop(ctrl_rate);

    while(ros::ok()){

        uav0_cur_pose.header.stamp = ros::Time::now();
        uav1_cur_pose.header.stamp = ros::Time::now();
        uav2_cur_pose.header.stamp = ros::Time::now();
        uav3_cur_pose.header.stamp = ros::Time::now();
        uav4_cur_pose.header.stamp = ros::Time::now();
        uav0_vision_pub.publish(uav0_cur_pose);
        uav1_vision_pub.publish(uav1_cur_pose);
        uav2_vision_pub.publish(uav2_cur_pose);
        uav3_vision_pub.publish(uav3_cur_pose);
        uav4_vision_pub.publish(uav4_cur_pose);

        ros::spinOnce();
        // spinner.spin();
        ctrl_loop.sleep();
    }
    return 0;
}
