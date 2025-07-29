#include <casadi_kin_dyn/casadi_kin_dyn.h>
#include <casadi/casadi.hpp>
#include <rerun.hpp>
#include <mutex>
#include <boost/circular_buffer.hpp>

#include <ros/ros.h>
#include <xbot_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include "kyon_controller/WBTrajectory.h"

namespace leg_analyzer {

class LegAnalyzer {

  public:
    LegAnalyzer(ros::NodeHandle& node_handle, const std::vector<std::string>& frames,
                  int trail_length, bool rt_logging, bool visualize_perception, 
                  bool ground_truth);
    ~LegAnalyzer(); 

    std::map<std::string, double> jmap;  
    const rerun::RecordingStream _rec; 

    void computeForwardKinematics();
    void updateState();
    void offlineLogTrj(); 
    

  private:
    std::shared_ptr<casadi_kin_dyn::CasadiKinDyn> _kin_dyn;

    int _nq; 
    int _nframes; 
    int _trail_length; 
    bool _rt_logging; 
    bool _init;
    bool _visualize_perception;
    bool _ground_truth;  
     
    std::vector<float> _base_pose;
    std::mutex _base_pose_mutex; 

    std::vector<float> _joint_state;
    std::mutex _joint_state_mutex; 

    std::vector<float> _full_state;
    std::vector<casadi::Function> _forward_kin_fncs; 
  
    std::vector<std::vector<double>> _mpc_prediction;  
    std::vector<std::vector<rerun::Position3D>> _predicted_trj;
    std::vector<boost::circular_buffer<rerun::Position3D>> _actual_trj;
    std::vector<std::vector<rerun::Position3D>> _ref_trj;
    
    std::vector<double> _ros_timeline_trjs, _ros_timeline_boundaries, _ros_timeline_pcl;
    std::vector<double> _ros_timeline_query_points, _ros_timeline_proj_points;
    std::vector<double> _ros_timeline_ref_trj; 
    std::vector<std::vector<rerun::LineStrip3D>> _strips_actual_offline, _strips_predicted_offline; 
    std::vector<std::vector<rerun::LineStrip3D>> _boundaries_offline;
    std::vector<std::vector<rerun::Color>> _boundaries_colors;
    std::vector<std::vector<rerun::Position3D>> _pointclouds;
    std::vector<std::vector<rerun::Position3D>> _query_points, _proj_points;  
    
    ros::Subscriber _base_pose_subscriber;
    ros::Subscriber _joint_state_subscriber;
    ros::Subscriber _mpc_prediction_subscriber;
    ros::Subscriber _boundaries_subscriber;
    ros::Subscriber _pcl_subscriber; 
    ros::Subscriber _query_landing_subscriber;
    ros::Subscriber _proj_landing_subscriber;
    ros::Subscriber _ref_trj_subscriber;
    
    void basePoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);
    void jointStateCallback(const xbot_msgs::JointStateConstPtr& msg); 
    void mpcPredictionCallback(const kyon_controller::WBTrajectoryConstPtr& msg); 
    void boundariesCallback(const visualization_msgs::MarkerArrayConstPtr& msg); 
    void pclCallback(const sensor_msgs::PointCloud2ConstPtr& msg); 
    void queryPointCallback(const visualization_msgs::MarkerConstPtr& msg); 
    void projectedPointCallback(const visualization_msgs::MarkerConstPtr& msg); 
    void refTrjCallback(const visualization_msgs::MarkerConstPtr& msg); 
     
};

} // namespace leg_analyzer
