#pragma once
#include <opencv2/opencv.hpp>
#include <swarm_msgs/Pose.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point32.h>
#include <swarm_msgs/LoopEdge.h>
#include "lcm_gen/LoopEdge_t.hpp"
#include "lcm_gen/ImageArrayDescriptor_t.hpp"
#include "lcm_gen/ImageDescriptor_t.hpp"
#include "lcm_gen/LandmarkDescriptor_t.hpp"
#include "lcm_gen/ImageDescriptorHeader_t.hpp"
#include <swarm_msgs/ImageArrayDescriptor.h>
#include <swarm_msgs/ImageDescriptor.h>
#include "base_types.hpp"
using Swarm::TsType;

inline Pose_t fromROSPose(const geometry_msgs::Pose & pose) {
    Pose_t t;
    t.orientation[0] = pose.orientation.x;
    t.orientation[1] = pose.orientation.y;
    t.orientation[2] = pose.orientation.z;
    t.orientation[3] = pose.orientation.w;

    t.position[0] = pose.position.x;
    t.position[1] = pose.position.y;
    t.position[2] = pose.position.z;
    return t;
}

inline geometry_msgs::Pose toROSPose(const Pose_t & t) {
    geometry_msgs::Pose pose;
    pose.orientation.x = t.orientation[0];
    pose.orientation.y = t.orientation[1];
    pose.orientation.z = t.orientation[2];
    pose.orientation.w = t.orientation[3];

    pose.position.x = t.position[0];
    pose.position.y = t.position[1];
    pose.position.z = t.position[2];
    return pose;
}

inline geometry_msgs::Pose toROSPose(const Swarm::Pose & t) {
    return t.toROS();
}

inline void ROSPoints2LCM(const std::vector<geometry_msgs::Point32> & src, std::vector<Point2d_t> & dst) {
    for (auto pt : src) {
        Point2d_t pt2;
        pt2.x = pt.x;
        pt2.y = pt.y;
        dst.push_back(pt2);
    }
}

inline void CVPoints2LCM(const std::vector<cv::Point2f> & src, std::vector<Point2d_t> & dst) {
    for (auto pt : src) {
        Point2d_t pt2;
        pt2.x = pt.x;
        pt2.y = pt.y;
        dst.push_back(pt2);
    }
}

inline void LCMPoints2ROS(const std::vector<Point2d_t> & src, std::vector<geometry_msgs::Point32> & dst) {
    for (auto pt : src) {
        geometry_msgs::Point32 pt2;
        pt2.x = pt.x;
        pt2.y = pt.y;
        dst.push_back(pt2);
    }
}

inline void LCMPoints2ROS(const std::vector<Point3d_t> & src, std::vector<geometry_msgs::Point32> & dst) {
    for (auto pt : src) {
        geometry_msgs::Point32 pt2;
        pt2.x = pt.x;
        pt2.y = pt.y;
        pt2.z = pt.z;
        dst.push_back(pt2);
    }
}

inline std::vector<geometry_msgs::Point32> toROSPoints(const std::vector<Eigen::Vector3d> & src) {
    std::vector<geometry_msgs::Point32> dst;
     for (auto pt : src) {
        geometry_msgs::Point32 pt2;
        pt2.x = pt.x();
        pt2.y = pt.y();
        pt2.z = pt.z();
        dst.push_back(pt2);
    }
    return dst;
}

inline std::vector<geometry_msgs::Point32> toROSPoints(const std::vector<cv::Point2f> & src) {
    std::vector<geometry_msgs::Point32> dst;
     for (auto pt : src) {
        geometry_msgs::Point32 pt2;
        pt2.x = pt.x;
        pt2.y = pt.y;
        pt2.z = 0.0;
        dst.push_back(pt2);
    }
    return dst;
}

inline std::vector<geometry_msgs::Point32> toROSPoints(const std::vector<Eigen::Vector2d> & src) {
    std::vector<geometry_msgs::Point32> dst;
     for (auto pt : src) {
        geometry_msgs::Point32 pt2;
        pt2.x = pt.x();
        pt2.y = pt.y();
        pt2.z = 0.0;
        dst.push_back(pt2);
    }
    return dst;
}

inline std::vector<Point3d_t> toLCMPoints(const std::vector<Eigen::Vector3d> & src) {
    std::vector<Point3d_t> dst;
     for (auto pt : src) {
        Point3d_t pt2;
        pt2.x = pt.x();
        pt2.y = pt.y();
        pt2.z = pt.z();
        dst.push_back(pt2);
    }
    return dst;
}

inline std::vector<Point2d_t> toLCMPoints(const std::vector<Eigen::Vector2d> & src) {
    std::vector<Point2d_t> dst;
     for (auto pt : src) {
        Point2d_t pt2;
        pt2.x = pt.x();
        pt2.y = pt.y();
        dst.push_back(pt2);
    }
    return dst;
}

inline void ROSPoints2LCM(const std::vector<geometry_msgs::Point32> & src, std::vector<Point3d_t> & dst) {
    for (auto pt : src) {
        Point3d_t pt3;
        pt3.x = pt.x;
        pt3.y = pt.y;
        pt3.z = pt.z;
        dst.push_back(pt3);
    }
}


inline cv::Mat cvfeatureFromByte(uint8_t*data, int feature_num, int feature_len = 32) {
    cv::Mat mat(feature_num, feature_len, CV_8UC1);
    memcpy(mat.data, data, feature_num*feature_len*sizeof(uint8_t));
    return mat;
}

inline cv::Point2f toCV(Point2d_t a) {
    return cv::Point2f(a.x, a.y);
}

inline cv::Point3f toCV(Point3d_t a) {
    return cv::Point3f(a.x, a.y, a.z);
}

inline cv::Point3f toCV(Vector3d a) {
    return cv::Point3f(a.x(), a.y(), a.z());
}

inline cv::Point2f toCV(Vector2d a) {
    return cv::Point2f(a.x(), a.y());
}

inline std::vector<cv::Point2f> toCV(std::vector<Point2d_t> arr) {
    std::vector<cv::Point2f> _arr;
    for (auto a : arr) {
        _arr.push_back(toCV(a));
    }
    return _arr;
}

inline std::vector<cv::Point2f> toCV(Point2d_t * arr, int len) {
    std::vector<cv::Point2f> _arr;
    for (int i = 0; i < len; i++) {
        auto a = arr[i];
        _arr.push_back(toCV(a));
    }
    return _arr;
}

inline std::vector<cv::Point3f> toCV(Point3d_t * arr, int len) {
    std::vector<cv::Point3f> _arr;
    for (int i = 0; i < len; i++) {
        auto a = arr[i];
        _arr.push_back(toCV(a));
    }
    return _arr;
}

template<typename T>
inline std::vector<cv::Point3f> toCV(std::vector<T> arr) {
    std::vector<cv::Point3f> _arr;
    for (auto a : arr) {
        _arr.push_back(toCV(a));
    }
    return _arr;
}

inline std::vector<cv::Point2f> toCV(std::vector<Vector2d> arr) {
    std::vector<cv::Point2f> _arr;
    for (auto a : arr) {
        _arr.push_back(cv::Point2f(a.x(), a.y()));
    }
    return _arr;
}

inline std::vector<cv::Point2f> toCV(const std::vector<geometry_msgs::Point32> & src) {
    std::vector<cv::Point2f> dst;
    for (auto pt : src) {
        dst.push_back(cv::Point2f(pt.x, pt.y));
    }
    return dst;
}

template<typename T>
inline std::vector<Eigen::Vector2d> toEigen(const std::vector<T> & src) {
    std::vector<Eigen::Vector2d> dst;
    for (auto pt : src) {
        dst.push_back(Vector2d(pt.x, pt.y));
    }
    return dst;
}

template<typename T>
inline std::vector<Eigen::Vector3d> toEigen3d(const std::vector<T> & src) {
    std::vector<Eigen::Vector3d> dst;
    for (auto pt : src) {
        dst.push_back(Vector3d(pt.x, pt.y, pt.z));
    }
    return dst;
}

inline ros::Time toROSTime(Time_t _time) {
    return ros::Time(_time.sec, _time.nsec);
}

inline Time_t toLCMTime(ros::Time _time) {
    Time_t t;
    t.sec = _time.sec;
    t.nsec = _time.nsec;
    return t;
}

inline std::vector<cv::KeyPoint> to_keypoints(const std::vector<cv::Point2f> & pts) {
    std::vector<cv::KeyPoint> kps;
    for (auto pt : pts) {
        cv::KeyPoint kp;
        kp.pt = pt;
        kps.push_back(kp);
    }
    return kps;
}

inline swarm_msgs::LoopEdge toROSLoopEdge(const LoopEdge_t & loop_con) {
    swarm_msgs::LoopEdge loop_conn;
    loop_conn.ts_a =  toROSTime(loop_con.ts_a);
    loop_conn.ts_b =  toROSTime(loop_con.ts_b);

    loop_conn.drone_id_a = loop_con.drone_id_a;
    loop_conn.drone_id_b = loop_con.drone_id_b;

    loop_conn.keyframe_id_a = loop_con.keyframe_id_a;
    loop_conn.keyframe_id_b = loop_con.keyframe_id_b;

    loop_conn.relative_pose = toROSPose(loop_con.relative_pose);
    loop_conn.self_pose_a = toROSPose(loop_con.self_pose_a);
    loop_conn.self_pose_b = toROSPose(loop_con.self_pose_b);
    
    loop_conn.pnp_inlier_num = loop_con.pnp_inlier_num;

    loop_conn.pos_cov.x = loop_con.pos_cov.x;
    loop_conn.pos_cov.y = loop_con.pos_cov.y;
    loop_conn.pos_cov.z = loop_con.pos_cov.z;

    loop_conn.ang_cov.x = loop_con.ang_cov.x;
    loop_conn.ang_cov.y = loop_con.ang_cov.y;
    loop_conn.ang_cov.z = loop_con.ang_cov.z;
    loop_conn.id = loop_con.id;

    return loop_conn;
}

inline LoopEdge_t toLCMLoopEdge(const swarm_msgs::LoopEdge & loop_con) {
    LoopEdge_t loop_conn;
    loop_conn.ts_a =  toLCMTime(loop_con.ts_a);
    loop_conn.ts_b =  toLCMTime(loop_con.ts_b);

    loop_conn.drone_id_a = loop_con.drone_id_a;
    loop_conn.drone_id_b = loop_con.drone_id_b;

    loop_conn.keyframe_id_a = loop_con.keyframe_id_a;
    loop_conn.keyframe_id_b = loop_con.keyframe_id_b;
    
    loop_conn.pnp_inlier_num = loop_con.pnp_inlier_num;

    loop_conn.relative_pose = fromROSPose(loop_con.relative_pose);
    loop_conn.self_pose_a = fromROSPose(loop_con.self_pose_a);
    loop_conn.self_pose_b = fromROSPose(loop_con.self_pose_b);

    loop_conn.pos_cov.x = loop_con.pos_cov.x;
    loop_conn.pos_cov.y = loop_con.pos_cov.y;
    loop_conn.pos_cov.z = loop_con.pos_cov.z;

    loop_conn.ang_cov.x = loop_con.ang_cov.x;
    loop_conn.ang_cov.y = loop_con.ang_cov.y;
    loop_conn.ang_cov.z = loop_con.ang_cov.z;
    loop_conn.id = loop_con.id;

    return loop_conn;
}

inline TsType to_nsec(Time_t stamp) {
    return stamp.sec * 1e9 + stamp.nsec;
}

inline TsType hash_stamp_drone_id(Time_t stamp, int drone_id) {
    return to_nsec(stamp)*100 + drone_id;
}

inline ImageDescriptor_t generate_null_img_desc() {
    ImageDescriptor_t empty;
    empty.frame_id = -1;
    empty.landmark_num = 0;
    empty.landmark_descriptor_size = 0;
    empty.image_desc_size = 0;
    empty.image_size = 0;
    return empty;
}