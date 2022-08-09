#pragma once
#include "Pose.h"
#include "base_types.hpp"
#include <swarm_msgs/node_detected.h>
#include <swarm_msgs/node_detected_xyzyaw.h>
#include <swarm_msgs/LoopEdge.h>
#include "utils.hpp"

namespace Swarm {
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

    //Same camera_index, return 1 else 2
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
        
        setCovariance(cov);
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
        setCovariance(cov);
    }

    swarm_msgs::LoopEdge toROS() const {
        swarm_msgs::LoopEdge loc;
        loc.id = id;
        loc.drone_id_a = id_a;
        loc.drone_id_b = id_b;
        loc.ts_a = stamp_a;
        loc.ts_b = stamp_b;

        loc.keyframe_id_a = keyframe_id_a;
        loc.keyframe_id_b = keyframe_id_b;

        loc.relative_pose = relative_pose.toROS();

        loc.self_pose_a = self_pose_a.toROS();
        loc.self_pose_b = self_pose_b.toROS();
        return loc;
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

    LoopEdge() {
        measurement_type = Loop;
        res_count = 0;
    }

    LoopEdge getInvertLoop() const {
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

        loop.setCovariance(cov_mat);

        loop.ts_a = ts_b;
        loop.ts_b = ts_a;

        return loop;
    }

    Eigen::Matrix6d getInfoMat() const {
        return inf_mat;
    }

    Eigen::Matrix6d getSqrtInfoMat() const {
        return sqrt_inf_mat;
    }

    //T, Q
    Matrix6d getCovariance() const {
        return cov_mat;
    }

    //T, Q
    Matrix4d getSqrtInfoMat4D() const {
        Matrix4d _sqrt_inf = Matrix4d::Zero();
        _sqrt_inf.block<3, 3>(0, 0) = sqrt_inf_mat.block<3, 3>(0, 0);
        _sqrt_inf(3, 3) = sqrt_inf_mat(5, 5);
        return _sqrt_inf;
    }

    void setCovariance(const Matrix6d & cov) {
        cov_mat = cov;
        inf_mat = cov.inverse();
        sqrt_inf_mat = inf_mat.cwiseAbs().cwiseSqrt();
        // std::cout << "cov_mat" << cov_mat << std::endl;
        // std::cout << "inf_mat" << inf_mat << std::endl;
        // std::cout << "sqrt_inf_mat" << sqrt_inf_mat << std::endl;
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

}