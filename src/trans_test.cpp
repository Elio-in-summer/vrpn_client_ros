#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <Eigen/Dense>
#include <iostream>

// Define a Rotation from MotionCapture's frame to my frame manually
// The Rotation is rotate -90 degree around z axis
Eigen::Matrix3d R_mc2my;

// Function to rotate a Vector3d
Eigen::Vector3d rotateVector(const Eigen::Vector3d& vec) {
    return R_mc2my * vec;
}

// Function to rotate a PoseStamped message
geometry_msgs::PoseStamped rotatePoseStamped(const geometry_msgs::PoseStamped& msg) {
    // Rotate position
    Eigen::Vector3d pos(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
    std::cout << "pos_before: " << pos << std::endl;
    pos = rotateVector(pos);
    std::cout << "pos_after: " << pos << std::endl;
    // Rotate orientation
    Eigen::Quaterniond quat(msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z);
    Eigen::Quaterniond rotated_quat = Eigen::Quaterniond(R_mc2my) * quat;

    geometry_msgs::PoseStamped rotated_msg = msg;
    rotated_msg.pose.position.x = pos.x();
    rotated_msg.pose.position.y = pos.y();
    rotated_msg.pose.position.z = pos.z();
    rotated_msg.pose.orientation.w = rotated_quat.w();
    rotated_msg.pose.orientation.x = rotated_quat.x();
    rotated_msg.pose.orientation.y = rotated_quat.y();
    rotated_msg.pose.orientation.z = rotated_quat.z();
    return rotated_msg;
}

// Function to rotate a TwistStamped message
geometry_msgs::TwistStamped rotateTwistStamped(const geometry_msgs::TwistStamped& msg) {
    // Rotate linear and angular velocities
    Eigen::Vector3d linear(msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z);
    Eigen::Vector3d angular(msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z);
    linear = rotateVector(linear);
    angular = rotateVector(angular);

    geometry_msgs::TwistStamped rotated_msg = msg;
    rotated_msg.twist.linear.x = linear.x();
    rotated_msg.twist.linear.y = linear.y();
    rotated_msg.twist.linear.z = linear.z();
    rotated_msg.twist.angular.x = angular.x();
    rotated_msg.twist.angular.y = angular.y();
    rotated_msg.twist.angular.z = angular.z();
    return rotated_msg;
}

// Callback functions
void uavMocapPosCallback(const geometry_msgs::PoseStamped::ConstPtr& msg, ros::Publisher& pub) {
    pub.publish(rotatePoseStamped(*msg));
}

void uavMocapVelCallback(const geometry_msgs::TwistStamped::ConstPtr& msg, ros::Publisher& pub) {
    pub.publish(rotateTwistStamped(*msg));
}

void uavMocapAccCallback(const geometry_msgs::TwistStamped::ConstPtr& msg, ros::Publisher& pub) {
    pub.publish(rotateTwistStamped(*msg));
}

void armMocapPosCallback(const geometry_msgs::PoseStamped::ConstPtr& msg, ros::Publisher& pub) {
    pub.publish(rotatePoseStamped(*msg));
}

void armMocapVelCallback(const geometry_msgs::TwistStamped::ConstPtr& msg, ros::Publisher& pub) {
    pub.publish(rotateTwistStamped(*msg));
}

void armMocapAccCallback(const geometry_msgs::TwistStamped::ConstPtr& msg, ros::Publisher& pub) {
    pub.publish(rotateTwistStamped(*msg));
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "mocap_transform_node");
    ros::NodeHandle nh;
    // Define a Rotation from MotionCapture's frame to my frame manually
    // The Rotation is rotate -90 degree around z axis
    R_mc2my << 0, -1, 0,
               1, 0, 0,
               0, 0, 1;
    std::cout << "R_mc2my: " << R_mc2my << std::endl;

    // Publishers
    ros::Publisher uav_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/hxl_uav/mocap/pos", 10);
    ros::Publisher uav_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/hxl_uav/mocap/vel", 10);
    ros::Publisher uav_acc_pub = nh.advertise<geometry_msgs::TwistStamped>("/hxl_uav/mocap/acc", 10);
    ros::Publisher arm_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/hxl_arm/mocap/pos", 10);
    ros::Publisher arm_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/hxl_arm/mocap/vel", 10);
    ros::Publisher arm_acc_pub = nh.advertise<geometry_msgs::TwistStamped>("/hxl_arm/mocap/acc", 10);

    // Subscribers
    ros::Subscriber uav_mocap_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/hxl_uav/pos", 10,
        boost::bind(uavMocapPosCallback, _1, boost::ref(uav_pos_pub)));

    ros::Subscriber uav_mocap_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("/hxl_uav/vel", 10,
        boost::bind(uavMocapVelCallback, _1, boost::ref(uav_vel_pub)));

    ros::Subscriber uav_mocap_acc_sub = nh.subscribe<geometry_msgs::TwistStamped>("/hxl_uav/acc", 10,
        boost::bind(uavMocapAccCallback, _1, boost::ref(uav_acc_pub)));

    ros::Subscriber arm_mocap_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/hxl_arm/pos", 10,
        boost::bind(armMocapPosCallback, _1, boost::ref(arm_pos_pub)));

    ros::Subscriber arm_mocap_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("/hxl_arm/vel", 10,
        boost::bind(armMocapVelCallback, _1, boost::ref(arm_vel_pub)));

    ros::Subscriber arm_mocap_acc_sub = nh.subscribe<geometry_msgs::TwistStamped>("/hxl_arm/acc", 10,
        boost::bind(armMocapAccCallback, _1, boost::ref(arm_acc_pub)));

    ros::spin();
    return 0;
}
