#pragma once
#include <swarm_msgs/Pose.h>
#include <nav_msgs/Odometry.h>

namespace Swarm {
class Odometry {
protected:
    Vector3d velocity;
    Vector3d angular_velocity;
    Pose pose_;
public:
    double stamp;
    Odometry():
        stamp(0.0), velocity(0., 0., 0.), angular_velocity(0., 0., 0.)
    {}

    Odometry(double t):
        stamp(t), velocity(0., 0., 0.), angular_velocity(0., 0., 0.)
    {}

    Odometry(const nav_msgs::Odometry & odom):
        stamp(odom.header.stamp.toSec()),
        pose_(odom.pose.pose),
        velocity(odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z),
        angular_velocity(odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z)
    {}

    Odometry(double t, const Swarm::Pose & _pose, const Vector3d & vel, const Vector3d & ang_vel):
        stamp(t), pose_(_pose), velocity(vel), angular_velocity(ang_vel)
    {}
    
    Odometry(double t, const Swarm::Pose & _pose, const Vector3d & vel):
        stamp(t), pose_(_pose), velocity(vel), angular_velocity(0., 0., 0.)
    {}

    Odometry(double t, const Swarm::Pose & _pose):
        stamp(t), pose_(_pose), velocity(0., 0., 0.), angular_velocity(0., 0., 0.)
    {}

    Odometry(const Swarm::Pose & _pose, const Vector3d & vel):
        stamp(0.), pose_(_pose), velocity(vel), angular_velocity(0., 0., 0.)
    {}

    std::string toStr() const {
        char buf[256] = {0};
        sprintf(buf, "Pose %s Vel %.2f %.2f %.2f", pose_.toStr().c_str(), velocity.x(), velocity.y(), velocity.z());
        return std::string(buf);
    }

    nav_msgs::Odometry toRos() const {
        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time(stamp);
        odom.header.frame_id = "world";
        odom.pose.pose = pose_.to_ros_pose();
        odom.twist.twist.linear.x = velocity.x();
        odom.twist.twist.linear.y = velocity.y();
        odom.twist.twist.linear.z = velocity.z();

        odom.twist.twist.angular.x = angular_velocity.x();
        odom.twist.twist.angular.y = angular_velocity.y();
        odom.twist.twist.angular.z = angular_velocity.z();
        return odom;
    }

    Quaterniond & att() {
        return pose_.att();
    }

    Vector3d & pos() {
        return pose_.pos();
    }

    Vector3d & vel() {
        return velocity;
    }

    Vector3d & ang_vel() {
        return angular_velocity;
    }

    Swarm::Pose & pose () {
        return pose_;
    }

    Quaterniond att() const {
        return pose_.att();
    }

    Vector3d pos() const {
        return pose_.pos();
    }

    Vector3d vel() const {
        return velocity;
    }

    Vector3d ang_vel() const {
        return angular_velocity;
    }

    Swarm::Pose pose () const {
        return pose_;
    }

    void moveByPose(const Pose & delta_pose) {
        pose_ = delta_pose * pose_;
        velocity = delta_pose.att() * velocity;
        angular_velocity = delta_pose.att() * angular_velocity;
    }

    friend Odometry operator*(Pose a, Odometry odom) {
        Odometry ret;
        ret.stamp = odom.stamp;
        ret.pose_ = a * odom.pose_;
        ret.velocity = a.att() * odom.velocity;
        ret.angular_velocity = a.att() * odom.angular_velocity;
        return ret;
    }
};
}