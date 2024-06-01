// author:xps
// 2023-03-02
// if no msg from cam use vicon for pos

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#include <iostream>

int new_msg_flag;
geometry_msgs::PoseStamped pos;
geometry_msgs::TwistStamped vel;
geometry_msgs::PoseStamped pos_to_pub;
geometry_msgs::PoseStamped arm_pos;
geometry_msgs::PoseStamped arm_pos_to_pub;
nav_msgs::Odometry odom_to_pub;

bool have_pos = false;
bool have_vel = false;
bool have_arm_pos = false;

class QuaternionAverager
{
public:
    QuaternionAverager() : count(0) {}

    void addQuaternion(const Eigen::Quaterniond &q)
    {
        std::cout << "q: " << q.norm() << std::endl;
        if (count == 0)
        {
            average = q;
            std::cout << "average: " << average.norm() << std::endl;
        }
        else
        {
            double t = 1.0 / (count + 1);
            average = average.slerp(t, q);
            std::cout << "average: " << average.norm() << std::endl;
        }
        count++;
    }

    Eigen::Quaterniond getAverage() const
    {
        return average;
    }

private:
    Eigen::Quaterniond average;
    int count;
};

class PosAverager
{
public:
    PosAverager() : count(0) {}

    void addPos(const Eigen::Vector3d &p)
    {
        if (count == 0)
        {
            average = p;
            std::cout << "average: " << average.norm() << std::endl;
        }
        else
        {
            average = (average * count + p) / (count + 1);
            std::cout << "average: " << average.norm() << std::endl;
        }
        count++;
    }

    Eigen::Vector3d getAverage() const
    {
        return average;
    }
private:
    Eigen::Vector3d average;
    int count;
};


void posCallback(const geometry_msgs::PoseStamped &msg)
{
    pos = msg;
    have_pos = true;
}

void velCallback(const geometry_msgs::TwistStamped &msg)
{
    vel = msg;
    have_vel = true;
}

void armPosCallback(const geometry_msgs::PoseStamped &msg)
{
    arm_pos = msg;
    have_arm_pos = true;
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "trans");
    ros::NodeHandle nh;
    ros::Publisher pubPose = nh.advertise<geometry_msgs::PoseStamped>(
        "/mavros/vision_pose/pose", 10);
    ros::Publisher pubOdom = nh.advertise<nav_msgs::Odometry>(
        "/mavros/vision_pose/odom", 10);
    ros::Publisher pubArmPos = nh.advertise<geometry_msgs::PoseStamped>(
        "/arm_pose", 10);

    ros::Subscriber subPos = nh.subscribe("/vicon/uav/pose", 10, posCallback);
    ros::Subscriber subVel = nh.subscribe("/vicon/uav/vel", 10, velCallback);
    ros::Subscriber subArmPos = nh.subscribe("/vicon/arm/pose", 10, posCallback);
    ros::Rate rate(50);

    int cnt = 0;
    QuaternionAverager averager;
    PosAverager pos_averager;

    while (ros::ok() && cnt < 50)
    {
        ros::spinOnce();
        if (have_pos && have_vel && have_arm_pos)
        {
            averager.addQuaternion(Eigen::Quaterniond(pos.pose.orientation.w,
                                                      pos.pose.orientation.x,
                                                      pos.pose.orientation.y,
                                                      pos.pose.orientation.z));
            pos_averager.addPos(Eigen::Vector3d(pos.pose.position.x,
                                                pos.pose.position.y,
                                                pos.pose.position.z));
            cnt++;
            ROS_INFO("Inilizing orientation...");
        }

        rate.sleep();
    }
    Eigen::Quaterniond q_init = averager.getAverage();
    ROS_INFO("Finish inilizing orientation! Average Quaternion: %f %f %f %f",
             q_init.w(), q_init.x(), q_init.y(), q_init.z());
    Eigen::Vector3d p_init = pos_averager.getAverage();
    ROS_INFO("Finish inilizing position! Average Position: %f %f %f",
             p_init.x(), p_init.y(), p_init.z());

    while (ros::ok())
    {
        ros::spinOnce();

        pos_to_pub.header = pos.header;
        odom_to_pub.header = pos.header;
        arm_pos_to_pub.header = arm_pos.header;

        pos_to_pub.pose.position = pos.pose.position;
        Eigen::Quaterniond q(pos.pose.orientation.w,
                             pos.pose.orientation.x,
                             pos.pose.orientation.y,
                             pos.pose.orientation.z);
        q = q * q_init.inverse();
        pos_to_pub.pose.orientation.w = q.w();
        pos_to_pub.pose.orientation.x = q.x();
        pos_to_pub.pose.orientation.y = q.y();
        pos_to_pub.pose.orientation.z = q.z();
        pos_to_pub.pose.position.x -= p_init.x();
        pos_to_pub.pose.position.y -= p_init.y();
        pos_to_pub.pose.position.z -= p_init.z();


        odom_to_pub.pose.pose = pos_to_pub.pose;
        odom_to_pub.twist.twist = vel.twist;

        arm_pos_to_pub.pose.position.x = arm_pos.pose.position.x - p_init.x();
        arm_pos_to_pub.pose.position.y = arm_pos.pose.position.y - p_init.y();
        arm_pos_to_pub.pose.position.z = arm_pos.pose.position.z - p_init.z();
        arm_pos_to_pub.pose.orientation = arm_pos.pose.orientation;

        pubPose.publish(pos_to_pub);
        pubOdom.publish(odom_to_pub);
        pubArmPos.publish(arm_pos_to_pub);
        rate.sleep();
    }

    return 0;
}
