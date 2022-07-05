#pragma once

#include <iostream>
#include <Eigen/Dense>
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

using namespace Eigen;
namespace Swarm {
typedef std::vector<Vector3d> vec_array;
typedef std::vector<Quaterniond> quat_array;
typedef std::map<int, double> DisMap;


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
