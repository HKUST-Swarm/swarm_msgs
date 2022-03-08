#pragma once

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <map>
#include <vector>
#include <swarm_msgs/Pose.h>
#include "yaml-cpp/yaml.h"
#include <exception>
#include <set>
#include <swarm_msgs/LoopEdge.h>
#include <swarm_msgs/node_detected_xyzyaw.h>
#include <swarm_msgs/node_detected.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#define UNIDENTIFIED_MIN_ID 1000
#define NO_ANNETAPOS
#define ENABLE_DETECTION
#define ENABLE_LOOP
// pixel error/focal length

typedef std::vector<Vector3d> vec_array;
typedef std::vector<Quaterniond> quat_array;
typedef std::map<int, double> DisMap;
typedef int64_t TsType;
typedef int64_t FrameIdType;

inline int TSShort(TsType ts) {
    return (ts/1000000)%10000000;
}

inline TsType TSLong(TsType ts) {
    return (ts/1000000)%10000000000;
}

template <typename T>
long search_closest(const std::vector<T>& sorted_array, double x) {

    auto iter_geq = std::lower_bound(
        sorted_array.begin(), 
        sorted_array.end(), 
        x
    );

    if (iter_geq == sorted_array.begin()) {
        return 0;
    }

    double a = *(iter_geq - 1);
    double b = *(iter_geq);

    if (fabs(x - a) < fabs(x - b)) {
        return iter_geq - sorted_array.begin() - 1;
    }

    return iter_geq - sorted_array.begin();

}


inline Eigen::Matrix<double, 2, 3> tangent_base_for_unit_detect(const Eigen::Vector3d & pts_j) {
    Eigen::Matrix<double, 2, 3> tangent_base;
    Eigen::Vector3d b1, b2;
    Eigen::Vector3d a = pts_j.normalized();
    Eigen::Vector3d tmp(0, 0, 1);
    if(a == tmp)
        tmp << 1, 0, 0;
    b1 = (tmp - a * (a.transpose() * tmp)).normalized();
    b2 = a.cross(b1);
    tangent_base.block<1, 3>(0, 0) = b1.transpose();
    tangent_base.block<1, 3>(1, 0) = b2.transpose();
    return tangent_base;
}

using namespace Eigen;
namespace Swarm {
class Node {
    protected:
        bool _has_vo = false;
        bool _has_uwb = false;
        bool _has_global_pose = false;
        bool _has_global_velocity = false;
        bool _is_static = false;

        Pose global_pose;
        Eigen::Vector3d global_velocity = Vector3d(0, 0, 0);
        Vector3d anntena_pos = Vector3d(0, 0, 0);

    public:

        std::map<int, std::vector<double>> coeffs;

        int id = -1;

        bool has_odometry() const {
            return _has_vo;
        }

        bool has_uwb() const {
            return _has_uwb;
        }

        bool has_global_pose() const {
            return _has_global_pose;
        }

        bool is_static_node() const {
            return _is_static;
        }

        double to_real_distance(const double mea, int _id) const {
            if (coeffs.find(_id) == coeffs.end() ) {
                return mea;
            }
            auto _coeffs = coeffs.at(_id);
            return _coeffs[0] + _coeffs[1] * mea;
        }

        Node(int _id, bool has_vo=true) :
                id(_id), _has_vo(has_vo) {

        }

        Node(int _id, const YAML::Node & config):
            id(_id){
            try {
//                ROS_INFO("Is parsing node %d", _id);
                _has_uwb = config["has_uwb"].as<bool>();
                _has_vo = config["has_vo"].as<bool>();
                _has_global_pose = config["has_global_pose"].as<bool>();
                _is_static = config["is_static"].as<bool>();
                if (_has_uwb && config["anntena_pos"]) {
                    this->anntena_pos = Vector3d( config["anntena_pos"][0].as<double>(),
                            config["anntena_pos"][1].as<double>(),
                            config["anntena_pos"][2].as<double>());
                }

                if (config["bias"]) {
                    const YAML::Node & bias_node = config["bias"];
                    for(auto it=bias_node.begin();it!=bias_node.end();++it) {
                        int _node_id = it->first.as<int>();
                        std::vector<double> _coeffs = it->second.as<std::vector<double>>();
                        this->coeffs[_node_id] = _coeffs;
                    }
                }
            }
            catch (YAML::ParserException & e){
                ROS_ERROR("Error while parsing node config %d: %s, exit", _id, e.what());
                exit(-1);
            }
        }

        Pose get_global_pose() const {
            return global_pose;
        }

        Vector3d get_global_velocity() const {
            return global_velocity;
        }

        Vector3d get_anntena_pos() const {
            return anntena_pos;
        }
    };

typedef std::tuple<TsType, TsType, int, int> GeneralMeasurement2DronesKey;

class GeneralMeasurement2Drones {
public:
    TsType ts_a;
    TsType ts_b;
    FrameIdType keyframe_id_a;
    FrameIdType keyframe_id_b;
    ros::Time stamp_a;
    ros::Time stamp_b;
    int id_a; // drone_id
    int id_b; // drone_id
    Pose self_pose_a;
    Pose self_pose_b;
    int res_count = 0;
    enum { 
        Loop,
        Detection3d,
        Detection4d,
        Detection6d
    } measurement_type;
    GeneralMeasurement2DronesKey key() {
        return GeneralMeasurement2DronesKey(ts_a, ts_b, id_a, id_b);
    }
    
    FrameIdType id; //unique identifier for loop edge.

    bool is_inter_loop() const { 
        return id_a != id_b;
    }

    //Same direction, return 1 else 2
    int same_robot_pair(GeneralMeasurement2Drones edge2) const {
        if (is_inter_loop()) {
            if (id_a == edge2.id_a && id_b == edge2.id_b) {
                return 1;
            }

            if (id_a == edge2.id_b && id_b == edge2.id_a) {
                return 2;
            }
        } else {
            return 1;
        }
        return 0;
    }
};

class LoopEdge: public GeneralMeasurement2Drones {
public:
    int avg_count = 1;
    Pose relative_pose;
    bool has_information_matrix = false;
    Matrix6d inf_mat;
    Matrix6d sqrt_inf_mat;
    Matrix6d cov_mat;

    LoopEdge(swarm_msgs::LoopEdge loc, bool yaw_only = false) {
        id = loc.id;
        id_a = loc.drone_id_a;
        id_b = loc.drone_id_b;
        ts_a = loc.ts_a.toNSec();
        ts_b = loc.ts_b.toNSec();

        keyframe_id_a = loc.keyframe_id_a;
        keyframe_id_b = loc.keyframe_id_b;

        stamp_a = loc.ts_a;
        stamp_b = loc.ts_b;

        relative_pose = Pose(loc.relative_pose);

        self_pose_a = Pose(loc.self_pose_a);
        self_pose_b = Pose(loc.self_pose_b);

        if (yaw_only) {
            relative_pose.set_yaw_only();
            self_pose_a.set_yaw_only();
            self_pose_b.set_yaw_only();
            res_count = 4;
        } else {
            res_count = 6;
        }
        measurement_type = Loop;

        Matrix6d cov = Matrix6d::Zero();
        cov(0, 0) = loc.pos_cov.x;
        cov(1, 1) = loc.pos_cov.y;
        cov(2, 2) = loc.pos_cov.z;

        cov(3, 3) = loc.ang_cov.x;
        cov(4, 4) = loc.ang_cov.y;
        cov(5, 5) = loc.ang_cov.z;
        
        set_covariance(cov);
    }

    LoopEdge(const swarm_msgs::node_detected & loc)  {
        id = loc.id;
        id_a = loc.self_drone_id;
        id_b = loc.remote_drone_id;
        ts_a = loc.header.stamp.toNSec();
        ts_b = loc.header.stamp.toNSec();

        stamp_a = loc.header.stamp;
        stamp_b = loc.header.stamp;

        relative_pose = Pose(loc.relative_pose.pose);

        self_pose_a = Pose(loc.local_pose_self);
        self_pose_b = Pose(loc.local_pose_remote); //Maybe absent we create

        if (loc.dof_4) {
            relative_pose.set_yaw_only();
            self_pose_a.set_yaw_only();
            self_pose_b.set_yaw_only();
            res_count = 4;
            measurement_type = Detection4d;
        } else {
            res_count = 6;
            measurement_type = Detection6d;
        }

        const Eigen::Map<const Eigen::Matrix<double,6,6,RowMajor>> cov(loc.relative_pose.covariance.data());
        set_covariance(cov);
    }

    swarm_msgs::LoopEdge toros() const {
        swarm_msgs::LoopEdge loc;
        loc.id = id;
        loc.drone_id_a = id_a;
        loc.drone_id_b = id_b;
        loc.ts_a = stamp_a;
        loc.ts_b = stamp_b;

        loc.keyframe_id_a = keyframe_id_a;
        loc.keyframe_id_b = keyframe_id_b;

        loc.relative_pose = relative_pose.to_ros_pose();

        loc.self_pose_a = self_pose_a.to_ros_pose();
        loc.self_pose_b = self_pose_b.to_ros_pose();
        return loc;
    }

    //T, Q
    Matrix6d get_covariance() const {
        return cov_mat;
    }

    //T, Q
    Matrix4d get_sqrt_information_4d() const {
        Matrix4d _sqrt_inf = Matrix4d::Zero();
        _sqrt_inf.block<3, 3>(0, 0) = sqrt_inf_mat.block<3, 3>(0, 0);
        _sqrt_inf(3, 3) = sqrt_inf_mat(5, 5);
        return _sqrt_inf;
    }

    void set_covariance(const Matrix6d & cov) {
        cov_mat = cov;
        inf_mat = cov.inverse();
        sqrt_inf_mat = inf_mat.cwiseAbs().cwiseSqrt();
        // std::cout << "cov_mat" << cov_mat << std::endl;
        // std::cout << "inf_mat" << inf_mat << std::endl;
        // std::cout << "sqrt_inf_mat" << sqrt_inf_mat << std::endl;
    }


    LoopEdge(swarm_msgs::LoopEdge loc, Eigen::Matrix6d _inf_mat):
        inf_mat(_inf_mat)
    {
        id = loc.id;
        id_a = loc.drone_id_a;
        id_b = loc.drone_id_b;
        ts_a = loc.ts_a.toNSec();
        ts_b = loc.ts_b.toNSec();

        keyframe_id_a = loc.keyframe_id_a;
        keyframe_id_b = loc.keyframe_id_b;

        stamp_a = loc.ts_a;
        stamp_b = loc.ts_b;

        relative_pose = loc.relative_pose;
        self_pose_a = Pose(loc.self_pose_a);
        self_pose_b = Pose(loc.self_pose_b);
        measurement_type = Loop;
        res_count = 6;
        has_information_matrix = true;

        //Not accurate
        sqrt_inf_mat = _inf_mat.cwiseSqrt();
    }

    LoopEdge(int keyframe_ida, int keyframe_idb, Swarm::Pose pose, Eigen::Matrix6d _inf_mat):
        relative_pose(pose),
        inf_mat(_inf_mat)
    {
        keyframe_id_a = keyframe_ida;
        keyframe_id_b = keyframe_idb;
        
        measurement_type = Loop;
        res_count = 6;
        has_information_matrix = true;

        //Not accurate
        sqrt_inf_mat = _inf_mat.cwiseSqrt();
        cov_mat = _inf_mat.cwiseInverse();
        // std::cout << "sqrt_inf_mat" << sqrt_inf_mat << std::endl;
    }

    // LoopEdge(const LoopEdge &loc) {
    //     id = loc.id;
    //     id_a = loc.id_a;
    //     id_b = loc.id_b;
    //     ts_a = loc.ts_a;
    //     ts_b = loc.ts_b;

    //     stamp_a = loc.stamp_a;
    //     stamp_b = loc.stamp_b;

    //     keyframe_id_a = loc.keyframe_id_a;
    //     keyframe_id_b = loc.keyframe_id_b;

    //     relative_pose = loc.relative_pose;

    //     self_pose_a = loc.self_pose_a;
    //     self_pose_b = loc.self_pose_b;

    //     has_information_matrix = loc.has_information_matrix;
    //     cov_mat = loc.cov_mat;
    //     sqrt_inf_mat = loc.sqrt_inf_mat;
    //     inf_mat = loc.inf_mat;

    //     measurement_type = loc.measurement_type;
    //     res_count = loc.res_count;
    // }
    
    LoopEdge() {
        measurement_type = Loop;
        res_count = 0;
    }

    LoopEdge invert_loop() const {
        LoopEdge loop;
        //Invert loop is still treat as same id for debugging.
        loop.id = id;
        loop.id_a = id_b;
        loop.id_b = id_a;

        loop.keyframe_id_a = keyframe_id_b;
        loop.keyframe_id_b = keyframe_id_a;

        loop.stamp_a = stamp_b;
        loop.stamp_b = stamp_a;
        loop.relative_pose = relative_pose.inverse();


        loop.self_pose_a = self_pose_b;
        loop.self_pose_b = self_pose_a;
        loop.measurement_type = measurement_type;
        loop.res_count = res_count;

        loop.set_covariance(cov_mat);

        loop.ts_a = ts_b;
        loop.ts_b = ts_a;

        return loop;
    }

    Eigen::Matrix6d information_matrix() const {
        return inf_mat;
    }

    Eigen::Matrix6d sqrt_information_matrix() const {
        return sqrt_inf_mat;
    }
    
};

class DroneDetection: public GeneralMeasurement2Drones {

public:
    Eigen::Matrix<double, 2, 3, RowMajor> detect_tan_base;
    Eigen::Vector3d p = Eigen::Vector3d::Zero();
    double inv_dep = 0;
    double probaility = 0;

    bool enable_depth = false;
    bool enable_dpose = false;
    bool use_inv_dep;
    

    //If disable dpose, this will act the extrinsic
    Pose dpose_self_a;
    Pose dpose_self_b;

    Pose extrinsic; //Extrinsic from IMU to Cam
    Pose GC = Swarm::Pose(Vector3d(-0.06, 0, 0), Quaterniond::Identity()); //Extrinsic from IMU to GC(e.g. detection center.)

    DroneDetection(const swarm_msgs::node_detected_xyzyaw & nd, bool _enable_dpose, Eigen::Vector3d _CG, bool _enable_depth = true):
        enable_dpose(_enable_dpose)
    {
        id = nd.id;
        id_a = nd.self_drone_id;
        id_b = nd.remote_drone_id;
        ts_a = nd.header.stamp.toNSec();
        ts_b = nd.header.stamp.toNSec();

        stamp_a = nd.header.stamp;
        stamp_b = nd.header.stamp;

        probaility = nd.probaility;
        
        extrinsic = Pose(nd.camera_extrinsic);
        self_pose_a = Pose(nd.local_pose_self);
        self_pose_b = Pose(nd.local_pose_remote)*Pose(-_CG, Eigen::Quaterniond::Identity());
        Pose GC = Swarm::Pose(_CG, Quaterniond::Identity()); //Extrinsic from IMU to GC(e.g. detection center.)
        inv_dep = nd.inv_dep;
        //Here hacked
        p = Eigen::Vector3d(nd.dpos.x, nd.dpos.y, nd.dpos.z);
        p.normalize();
        measurement_type = Detection3d;

        if (_enable_depth && nd.enable_scale) {
            enable_depth = true;
            res_count = 3;
        } else {
            enable_depth = false;
            res_count = 2;
        }


        detect_tan_base = tangent_base_for_unit_detect(p);
    }


    DroneDetection() {
        measurement_type = Detection3d;
        res_count = 0;
    }
};
class NodeFrame {
public:
    bool frame_available = false;
    bool vo_available = false;
    bool dists_available = false;
    bool has_detect_relpose = false;
    bool is_static = false;
    Node *node = nullptr;
    int drone_id = -1;
    FrameIdType keyframe_id = -1;

    DisMap dis_map;
    Pose self_pose;
    Pose estimated_pose;

    Eigen::Vector3d self_vel = Eigen::Vector3d(0, 0, 0);
    std::map<int, bool> enabled_detection;
    std::map<int, bool> enabled_distance;
    std::map<int, bool> outlier_distance;
    std::vector<DroneDetection> detected_nodes;
    std::map<int, Eigen::Matrix<double, 2, 3>> detect_tan_base;

    std::map<int, Eigen::Vector3d> detected_nodes_posvar;
    std::map<int, Eigen::Vector3d> detected_nodes_angvar;

    ros::Time stamp;
    TsType ts;
    bool is_valid = false;

    NodeFrame(Node *_node) :
            node(_node) {
        is_static = _node->is_static_node();
    }

    double to_real_distance(const double & measure, int _id) const {
        return node->to_real_distance(measure, _id);
    }

    NodeFrame() {

    }

    bool has_odometry() const {
        return vo_available;
    }

    bool has_detection() const {
        return detected_nodes.size() > 0;
    }

    int detections() const {
        return detected_nodes.size();
    }

    bool distance_is_outlier(int idj) const {
        //If distance not exist or is outlier, return true
        if (outlier_distance.find(idj) != outlier_distance.end() && outlier_distance.at(idj)) {
            // ROS_INFO("%d<->%d distance_i?s_outlier! %d");
            return true;
        }

        if (dis_map.find(idj) == dis_map.end()) {
            return true;
        }
        
        return false;
    }

    bool has_distance_to(int idj) const {
        if (dis_map.find(idj) != dis_map.end()) {
            return true;
        }
        return false;
    }

    bool distance_available(int _idj) const {
        if (_idj == drone_id) {
            return false;
        }
        // bool enab_distance = false;
        // if (enabled_distance.find(_idj) != enabled_distance.end()) {
        //     enab_distance = enabled_distance.at(_idj);
        // }
        // ROS_INFO("%d<->%d@%d has_distance_to %d enabled_distance.find() %d enabled_distance %d !distance_is_outlier %d",
        //     id, _idj,
        //     TSShort(ts),
        //     has_distance_to(_idj),
        //     enabled_distance.find(_idj) != enabled_distance.end(),
        //     enab_distance, !distance_is_outlier(_idj)
        // );
        return has_distance_to(_idj) && 
            enabled_distance.find(_idj) != enabled_distance.end() 
            && enabled_distance.at(_idj) && !distance_is_outlier(_idj);
    }

    Pose pose() const {
        //If has vo, return vo position
        if (!is_static) {
            assert((node->has_odometry() && vo_available) && "Try get position non non-static via VO failed node");
        }
        

        if (vo_available) {
            return self_pose;
        } else if(is_static){
            if (node->has_global_pose()) {
                return node->get_global_pose();
            } else {
                //Is unknown static node, using 0, 0, 0 position
                return Pose::Identity();
            }
        }
        assert(false && "MUST STH wrong on get pose()");
        return Pose();
    }

    Eigen::Vector3d position() const {
        return pose().pos();
    }

    double yaw() const {
        return pose().yaw();
    }

    Eigen::Quaterniond attitude(bool yaw_only = false) const {
        if (yaw_only) {
            return pose().att_yaw_only();
        } else {
            return pose().att();
        }

    }

    Eigen::Vector3d get_anntena_pos() const {
        return node->get_anntena_pos();
    }

    Eigen::Vector3d velocity() const {
        assert(!(node->has_odometry() && !vo_available) && "Try get velocity on VO failed node");
        if (vo_available) {
            return self_vel;
        } else {
            if (node->has_global_pose()) {
                return node->get_global_velocity();
            } else {
                //Is unknown static node, using 0, 0, 0 position

                return Vector3d(0, 0, 0);
            }
        }
    }
};

class DroneTrajectory {

    std::vector<Swarm::NodeFrame> trajectory_frames;
    std::vector<Swarm::Pose> trajectory;
    std::vector<double> cul_length;
    std::vector<TsType> ts_trajectory;
    std::vector<ros::Time> stamp_trajectory;
    std::map<TsType, int> ts2index;
    std::map<FrameIdType, int> id2index;
    bool is_traj_ego_motion = true; //If false, the is PGO traj.
    int drone_id = -1;
    int traj_points = 0;
    nav_msgs::Path ros_path;
    std::string frame_id = "world";

    double pos_covariance_per_meter = 4e-3;
    double yaw_covariance_per_meter = 4e-5;
public:
    DroneTrajectory(int _drone_id, bool is_ego_motion, double _pos_covariance_per_meter=4e-3, double _yaw_covariance_per_meter = 4e-5, std::string _frame_id="world"):
        drone_id(_drone_id), is_traj_ego_motion(is_ego_motion), frame_id(_frame_id),pos_covariance_per_meter(_pos_covariance_per_meter), yaw_covariance_per_meter(_yaw_covariance_per_meter)
    {
        // ROS_INFO("Initialize trajectory %d for ego %d with pos_covariance_per_meter %.2e yaw_covariance_per_meter %.2e",
            // _drone_id, is_ego_motion, pos_covariance_per_meter, yaw_covariance_per_meter);
        ros_path.header.frame_id = frame_id;
    }

    DroneTrajectory() {
        ros_path.header.frame_id = frame_id;
    }

    void push(const Swarm::NodeFrame & nf) {
        if (nf.has_odometry()) {
            if (is_traj_ego_motion) {
                push(nf, nf.self_pose);
            } else {
                push(nf, nf.estimated_pose);
            }
        }
    }

    uint64_t get_ts(int index) const {
        return ts_trajectory.at(index);
    }

    Swarm::Pose get_pose(int index) const {
        return trajectory.at(index);
    }

    Swarm::Pose get_latest_pose() const {
        return trajectory.back();
    }

    Swarm::NodeFrame get_node_frame(int index) const {
        assert(index < trajectory_frames.size() && "index out of range");
        return trajectory_frames.at(index);
    }

    void push(ros::Time stamp, const Swarm::Pose & pose) {
        if (cul_length.size() == 0) {
            cul_length.push_back(0);
        } else {
            Swarm::Pose last = trajectory.back();
            auto dpose = Swarm::Pose::DeltaPose(last, pose);
            cul_length.push_back(dpose.pos().norm()+cul_length.back());
        }

        auto ts = stamp.toNSec();
        trajectory.push_back(pose);
        ts_trajectory.push_back(ts);
        stamp_trajectory.push_back(stamp);
        ts2index[ts] = ts_trajectory.size() - 1;

        geometry_msgs::PoseStamped _pose_stamped;
        _pose_stamped.header.stamp = stamp;
        _pose_stamped.header.frame_id = frame_id;
        ros_path.header.stamp = _pose_stamped.header.stamp;
        _pose_stamped.pose = pose.to_ros_pose();
        ros_path.poses.push_back(_pose_stamped);

        traj_points++;
    }

    void push(const Swarm::NodeFrame & nf, const Swarm::Pose & pose) {
        if (cul_length.size() == 0) {
            cul_length.push_back(0);
        } else {
            Swarm::Pose last = trajectory.back();
            auto dpose = Swarm::Pose::DeltaPose(last, pose);
            cul_length.push_back(dpose.pos().norm()+cul_length.back());
        }

        trajectory_frames.push_back(nf);
        trajectory.push_back(pose);
        ts_trajectory.push_back(nf.ts);
        stamp_trajectory.push_back(nf.stamp);
        ts2index[nf.ts] = trajectory_frames.size() - 1;
        id2index[nf.drone_id] = trajectory_frames.size() - 1;


        geometry_msgs::PoseStamped _pose_stamped;
        _pose_stamped.header.stamp = nf.stamp;
        _pose_stamped.header.frame_id = frame_id;
        ros_path.header.stamp = _pose_stamped.header.stamp;
        _pose_stamped.pose = pose.to_ros_pose();
        ros_path.poses.push_back(_pose_stamped);

        traj_points++;
    }

    int trajectory_size() const {
        return ts_trajectory.size();
    }

    double trajectory_length() const {
        return cul_length.back();
    }

    std::pair<Swarm::Pose, Eigen::Matrix6d> get_relative_pose_by_ts(TsType tsa, TsType tsb, bool pose_4d=false) const {
        if (ts2index.find(tsa) == ts2index.end()) {
            ROS_ERROR("get_relative_pose_by_ts %ld-%ld failed. tsa not found", tsa, tsb);
            assert(false && "tsa not found. Use get_relative_pose_by_appro_ts instead");
            return std::make_pair(Swarm::Pose(), Eigen::Matrix6d());
        }

        if (ts2index.find(tsb) == ts2index.end()) {
            ROS_ERROR("get_relative_pose_by_ts %ld-%ld failed. tsb not found", tsa, tsb);
            assert(false && "tsb not found. Use get_relative_pose_by_appro_ts instead");
            return std::make_pair(Swarm::Pose(), Eigen::Matrix6d());
        }


        auto indexa = ts2index.at(tsa);
        auto indexb = ts2index.at(tsb);

        auto rp = Swarm::Pose::DeltaPose(get_pose(indexa), get_pose(indexb), pose_4d);
        // ROS_WARN("trajectory_length_by_ts %ld-%ld index %ld<->%ld, RP %s", tsa, tsb, indexa, indexb, rp.tostr().c_str());
        return std::make_pair(rp, covariance_between_ts(tsa, tsb));
    }
    
    std::pair<Swarm::Pose, Eigen::Matrix6d> get_relative_pose_by_appro_ts(TsType tsa, TsType tsb, bool pose_4d=false) const {
        auto posea = pose_by_appro_ts(tsa);
        auto poseb = pose_by_appro_ts(tsb);
        auto rp = Swarm::Pose::DeltaPose(posea, poseb, pose_4d);
        return std::make_pair(rp, covariance_between_appro_ts(tsa, tsb));
    }

    Swarm::Pose pose_by_appro_ts(TsType tsa, double & dt) const {
        if (ts2index.find(tsa) != ts2index.end()) {
            dt = 0;
            return trajectory.at(ts2index.at(tsa));
        }

        auto indexa = search_closest(ts_trajectory, tsa);
        // ROS_INFO("search_closest index %ld/%ld %ld tsa %ld dt %ld %f", indexa, ts_trajectory.size(), ts_trajectory[indexa], tsa, ts_trajectory.at(indexa) - tsa, (ts_trajectory.at(indexa) - tsa)/1e9);
        if (indexa < 0 || indexa >= ts_trajectory.size()) {
            dt = 1000000;
            return Swarm::Pose();
        }
        dt = (ts_trajectory.at(indexa) - tsa)/1e9;
        return trajectory.at(indexa);
    }

    Swarm::Pose pose_by_appro_ts(TsType tsa) const {
        if (ts2index.find(tsa) != ts2index.end()) {
            return trajectory.at(ts2index.at(tsa));
        }

        auto indexa = search_closest(ts_trajectory, tsa);
        return trajectory.at(indexa);
    }

    std::pair<TsType, Swarm::Pose> tspose_by_appro_ts(TsType tsa) const {
        if (ts2index.find(tsa) != ts2index.end()) {
            return std::make_pair(tsa, trajectory.at(ts2index.at(tsa)));
        }

        auto indexa = search_closest(ts_trajectory, tsa);
        return std::make_pair(ts_trajectory.at(indexa), trajectory.at(indexa));
    }

    Swarm::Pose pose_by_appro_ts(TsType tsa, TsType & tsb) const {
        if (ts2index.find(tsa) != ts2index.end()) {
            return trajectory.at(ts2index.at(tsa));
        }

        auto indexa = search_closest(ts_trajectory, tsa);
        tsb = ts_trajectory.at(indexa);
        return trajectory.at(indexa);
    }

    Swarm::Pose pose_by_index(int _index) const {
        return trajectory.at(_index);
    }

    TsType ts_by_index(int _index) const {
        return ts_trajectory.at(_index);
    }


    double trajectory_length_by_appro_ts(TsType tsa, TsType tsb) const {
        if (ts2index.find(tsa) != ts2index.end() && ts2index.find(tsb) != ts2index.end()) {
            return trajectory_length_by_ts(tsa, tsb);
        }

        auto indexa = search_closest(ts_trajectory, tsa);
        auto indexb = search_closest(ts_trajectory, tsb);

        // ROS_INFO("Appro ts(ms) %d<->%d find %d<->%d", tsa/1000000, tsb/1000000, 
        //     ts_trajectory[indexa]/1000000, 
        //     ts_trajectory[indexb]/1000000);
        return fabs(cul_length[indexb] - cul_length[indexa]);
    }

    double trajectory_length_by_ts(TsType tsa, TsType tsb) const {
        if (ts2index.find(tsa) == ts2index.end()) {
            ROS_ERROR("trajectory_length_by_ts %ld-%ld failed. tsa not found", tsa, tsb);
            assert(false && "tsa not found. Use trajectory_length_by_appro_ts instead");
            return -1;
        }

        if (ts2index.find(tsb) == ts2index.end()) {
            ROS_ERROR("trajectory_length_by_ts %ld-%ld failed. tsb not found", tsa, tsb);
            assert(false && "tsb not found. Use trajectory_length_by_appro_ts instead");
            exit(-1);
            return -1;
        }

        auto indexa = ts2index.at(tsa);
        auto indexb = ts2index.at(tsb);

        return fabs(cul_length[indexb] - cul_length[indexa]);
    }

    double trajectory_length_by_id(FrameIdType ida, FrameIdType idb) const {
        if (id2index.find(ida) == id2index.end()) {
            ROS_WARN("trajectory_length_by_ts %ld-%ld failed. ida not found", ida, idb);
            return -1;
        }

        if (id2index.find(idb) == id2index.end()) {
            ROS_WARN("trajectory_length_by_ts %ld-%ld failed. idb not found", ida, idb);
            return -1;
        }

        auto indexa = id2index.at(ida);
        auto indexb = id2index.at(idb);

        return fabs(cul_length[indexb] - cul_length[indexa]);
    }

    //Ang Pos
    Eigen::Matrix6d covariance_between_ts(TsType tsa, TsType tsb) const {
        double len = trajectory_length_by_ts(tsa, tsb);
        Eigen::Matrix6d cov_mat = Eigen::Matrix6d::Zero();
        cov_mat.block<3, 3>(0, 0) = Matrix3d::Identity()*pos_covariance_per_meter*len 
            + 0.5*Matrix3d::Identity()*yaw_covariance_per_meter*len*len;;
        cov_mat.block<3, 3>(3, 3) = Matrix3d::Identity()*yaw_covariance_per_meter*len;
        // std::cout << "length " << len << "Cov\n" << cov_mat << std::endl;
        return cov_mat;
    }

    Eigen::Matrix6d covariance_between_appro_ts(TsType tsa, TsType tsb) const {
        double len = trajectory_length_by_appro_ts(tsa, tsb);
        Eigen::Matrix6d cov_mat = Eigen::Matrix6d::Zero();
        cov_mat.block<3, 3>(0, 0) = Matrix3d::Identity()*pos_covariance_per_meter*len 
            + 0.5*Matrix3d::Identity()*yaw_covariance_per_meter*len*len;
        cov_mat.block<3, 3>(3, 3) = Matrix3d::Identity()*yaw_covariance_per_meter*len;
        return cov_mat;
    }

    const nav_msgs::Path & get_ros_path() const {
        return ros_path;
    }
};

struct SwarmFrameState {
    std::map<int, Pose> node_poses;
    std::map<int, Eigen::Matrix4d> node_covs;
    std::map<int, Vector3d> node_vels;

    std::map<int, Pose> base_coor_poses;
    std::map<int, Eigen::Matrix4d> base_coor_covs;

};

class SwarmFrame {
    public:
        std::map<int, NodeFrame> id2nodeframe;
        std::map<int, DisMap> dis_mat;
        std::set<int> node_id_list;

        int self_id = -1;

        ros::Time stamp;
        TsType ts;

        int swarm_size() const {
            return id2nodeframe.size();
        }

        void print() const {
            printf("SF %d Nodes: %ld\n", TSShort(ts), id2nodeframe.size());
            for (auto it: id2nodeframe) {
                printf("Node %d Pos [%3.2f, %3.2f, %3.2f] Yaw %3.2f\n",
                    it.first,
                    it.second.position().x(),
                    it.second.position().y(),
                    it.second.position().z(),
                    it.second.yaw()
                );
            }
        }

        bool has_distance_measurement(const int id) const {
            if (id2nodeframe.find(id) == id2nodeframe.end()) {
                return 0;
            }
            return id2nodeframe.at(id).node->has_uwb();
        }

        bool has_distance_measurement(const int idj, const int idi) const {
            if (dis_mat.find(idj) == dis_mat.end()) {
                return false;
            }
            return dis_mat.at(idj).find(idi) != dis_mat.at(idj).end();
        }

        bool has_node(const int _id) const{
            return (id2nodeframe.find(_id) != id2nodeframe.end());
        }

        double distance(const int idj, const int idi) const {
            assert(has_distance_measurement(idj, idi) && "Require distance not have");

            return dis_mat.at(idj).at(idi);
        }

        bool has_odometry(const int id) const {
            if (this->has_node(id)) {
                return (id2nodeframe.at(id).vo_available);
            }
            return false;
        }

        Vector3d position(const int id) const {
            assert(id2nodeframe.find(id) != id2nodeframe.end() && "Can't find position id in frame");
            return id2nodeframe.at(id).position();
        }

        Vector3d velocity(const int id) const {
            assert(id2nodeframe.find(id) != id2nodeframe.end() && "Can't find velocity id in frame");
            return id2nodeframe.at(id).velocity();
        }

        Quaterniond attitude(const int id) const {
            assert(id2nodeframe.find(id) != id2nodeframe.end() && "Can't find attitude id in frame");
            return id2nodeframe.at(id).attitude();
        }

    };
};

#include <chrono> 

class TicToc
{
  public:
    TicToc()
    {
        tic();
    }

    void tic()
    {
        start = std::chrono::system_clock::now();
    }

    double toc()
    {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        return elapsed_seconds.count() * 1000;
    }

  private:
    std::chrono::time_point<std::chrono::system_clock> start, end;
};

