#pragma once
#include "Pose.h"
#include "utils.hpp"
#include <nav_msgs/Path.h>
#include <swarm_msgs/DroneTraj.h>
#include "base_types.hpp"
#include <ros/ros.h>

namespace Swarm {
class DroneTrajectory {
    std::vector<Swarm::Pose> trajectory;
    std::vector<FrameIdType> frame_ids;
    std::vector<double> cul_length;
    std::vector<TsType> ts_trajectory;
    std::vector<double> stamp_trajectory;
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
    DroneTrajectory(int _drone_id, bool is_ego_motion, 
            double _pos_covariance_per_meter=4e-3, 
            double _yaw_covariance_per_meter = 4e-5, std::string _frame_id="world"):
        drone_id(_drone_id), 
        is_traj_ego_motion(is_ego_motion),
        frame_id(_frame_id),
        pos_covariance_per_meter(_pos_covariance_per_meter), 
        yaw_covariance_per_meter(_yaw_covariance_per_meter)
    {
        // ROS_INFO("Initialize trajectory %d for ego %d with pos_covariance_per_meter %.2e yaw_covariance_per_meter %.2e",
            // _drone_id, is_ego_motion, pos_covariance_per_meter, yaw_covariance_per_meter);
        ros_path.header.frame_id = frame_id;
    }

    DroneTrajectory() {
        ros_path.header.frame_id = frame_id;
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

    double get_latest_stamp() const {
        return stamp_trajectory.back();
    }

    const std::vector<Swarm::Pose> & get_trajectory() const {
        return trajectory;
    }

    void push(ros::Time stamp, const Swarm::Pose & pose, FrameIdType frame_id = 0) {
        if (cul_length.size() == 0) {
            cul_length.push_back(0);
        } else {
            Swarm::Pose last = trajectory.back();
            auto dpose = Swarm::Pose::DeltaPose(last, pose);
            cul_length.push_back(dpose.pos().norm()+cul_length.back());
        }

        TsType ts = stamp.toNSec();
        trajectory.push_back(pose);
        ts_trajectory.push_back(ts);
        frame_ids.push_back(frame_id);
        stamp_trajectory.push_back(stamp.toSec());
        ts2index[ts] = ts_trajectory.size() - 1;
        id2index[frame_id] = frame_ids.size() - 1;

        geometry_msgs::PoseStamped _pose_stamped;
        _pose_stamped.header.stamp = ros::Time(stamp);
        _pose_stamped.header.frame_id = frame_id;
        ros_path.header.stamp = _pose_stamped.header.stamp;
        _pose_stamped.pose = pose.toROS();
        ros_path.poses.push_back(_pose_stamped);
        traj_points++;
    }
    
    void push(double stamp, const Swarm::Pose & pose, FrameIdType frame_id = 0) {
        push(ros::Time(stamp), pose, frame_id);
    }

    int trajectory_size() const {
        return trajectory.size();
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
        // ROS_WARN("trajectory_length_by_ts %ld-%ld index %ld<->%ld, RP %s", tsa, tsb, indexa, indexb, rp.toStr().c_str());
        return std::make_pair(rp, covariance_between_ts(tsa, tsb));
    }


    std::pair<Swarm::Pose, Eigen::Matrix6d> get_relative_pose_by_frame_id(FrameIdType frame_id_a, FrameIdType frame_id_b, bool pose_4d=false) const {
        if (id2index.find(frame_id_a) == id2index.end()) {
            ROS_ERROR("get_relative_pose_by_frame_id %ld-%ld failed. tsa not found", frame_id_a, frame_id_b);
            assert(false && "frame_id_a not found. Use get_relative_pose_by_appro_ts instead");
            return std::make_pair(Swarm::Pose(), Eigen::Matrix6d());
        }

        if (id2index.find(frame_id_a) == id2index.end()) {
            ROS_ERROR("get_relative_pose_by_frame_id %ld-%ld failed. tsb not found", frame_id_a, frame_id_b);
            assert(false && "frame_id_b not found. Use get_relative_pose_by_appro_ts instead");
            return std::make_pair(Swarm::Pose(), Eigen::Matrix6d());
        }


        auto indexa = id2index.at(frame_id_a);
        auto indexb = id2index.at(frame_id_b);

        auto rp = Swarm::Pose::DeltaPose(get_pose(indexa), get_pose(indexb), pose_4d);
        double len = fabs(cul_length[indexb] - cul_length[indexa]);
        // ROS_WARN("trajectory_length_by_ts %ld-%ld index %ld<->%ld, RP %s", tsa, tsb, indexa, indexb, rp.toStr().c_str());
        return std::make_pair(rp, covariance_with_length(len));
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

    double stamp_by_index(int _index) const {
        return stamp_trajectory.at(_index);
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
    Eigen::Matrix6d covariance_with_length(double len) const {
        Eigen::Matrix6d cov_mat = Eigen::Matrix6d::Zero();
        cov_mat.block<3, 3>(0, 0) = Matrix3d::Identity()*pos_covariance_per_meter*len 
            + 0.5*Matrix3d::Identity()*yaw_covariance_per_meter*len*len;;
        cov_mat.block<3, 3>(3, 3) = Matrix3d::Identity()*yaw_covariance_per_meter*len;
        // std::cout << "length " << len << "Cov\n" << cov_mat << std::endl;
        return cov_mat;
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

    const swarm_msgs::DroneTraj toRos() const {
        swarm_msgs::DroneTraj traj;
        traj.header.stamp = ros::Time::now();
        traj.header.frame_id = "map";
        traj.poses.resize(trajectory.size());
        traj.frame_stamps.resize(trajectory.size());
        traj.frame_ids.resize(trajectory.size());
        traj.drone_id = drone_id;
        for (int i = 0; i < trajectory.size(); i++) {
            traj.poses[i] = trajectory[i].toROS();
            traj.frame_stamps[i] = ros::Time(stamp_trajectory[i]);
            traj.frame_ids[i] = frame_ids[i];
        }
        return traj;
    }
};
}