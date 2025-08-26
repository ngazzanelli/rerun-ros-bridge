#include <casadi_kin_dyn/casadi_kin_dyn.h>
#include <casadi/casadi.hpp>
#include <rerun.hpp>
#include <mutex>
#include <boost/circular_buffer.hpp>
#include <boost/function.hpp>
#include <optional>

#include <ros/ros.h>
#include <xbot_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include "kyon_controller/WBTrajectory.h"

namespace leg_analyzer {

struct MarkerData {
  std::string key; 
  std::vector<std::vector<rerun::Position3D>> points;
  std::vector<double> ros_timeline;
  std::optional<uint32_t> color;
  std::optional<double> radius;  
}; 


struct MarkerArrayData {
  std::string key; 
  std::vector<std::vector<rerun::LineStrip3D>> markers;
  std::vector<double> ros_timeline;
  std::vector<std::vector<rerun::Color>> colors; 
};


struct PointCloudData {
  std::string key; 
  std::vector<std::vector<rerun::Position3D>> pointcloud;
  std::vector<double> ros_timeline; 
};


class LegAnalyzer {

  public:
    LegAnalyzer(ros::NodeHandle& node_handle, bool visualize_perception, bool ground_truth);
    ~LegAnalyzer(); 

    std::map<std::string, double> jmap;  
    const rerun::RecordingStream _rec; 

    void computeForwardKinematics(double ros_time);
    void tryUpdateState(); 
    void offlineLogTrj(); 
    

  private:
    std::shared_ptr<casadi_kin_dyn::CasadiKinDyn> _kin_dyn;

    ros::NodeHandle _nh; 

    int _nq; 
    int _nframes; 
    int _trail_length; 
    bool _rt_logging; 
    bool _init;
    bool _odom_provided; 
    bool _visualize_perception;
    bool _ground_truth;  

    // Data synchronization
    std::mutex _data_mutex;
    geometry_msgs::PoseStampedConstPtr _latest_odom;
    xbot_msgs::JointStateConstPtr _latest_joint_states;
    ros::Time last_processed_odom_time_;
    ros::Time last_processed_joint_time_;
    
    std::vector<std::string> _frames; 

    std::vector<float> _base_pose;

    std::vector<float> _joint_state;

    std::vector<float> _full_state;
    std::vector<casadi::Function> _forward_kin_fncs; 

    // Marker data structure
    std::vector<struct MarkerData> _markers; 

    // Marker array data strucutre
    std::vector<struct MarkerArrayData> _marker_arrays; 
    
    // Pointcloud data structures
    std::vector<struct PointCloudData> _pointclouds; 

    std::vector<std::vector<double>> _mpc_prediction;  
    std::vector<std::vector<rerun::Position3D>> _predicted_trj;
    std::vector<boost::circular_buffer<rerun::Position3D>> _actual_trj;
    std::vector<std::vector<rerun::Position3D>> _ref_trj;
    
    // std::vector<std::vector<rerun::Position3D>> _pointclouds;
    std::vector<std::vector<rerun::LineStrip3D>> _strips_actual_offline, _strips_predicted_offline; 
    std::vector<std::vector<rerun::LineStrip3D>> _boundaries_offline;
    std::vector<std::vector<rerun::Color>> _boundaries_colors;
    std::vector<std::vector<rerun::Position3D>> _pointclouds;
    std::vector<std::vector<rerun::Position3D>> _query_points, _proj_points;  
    
    std::vector<ros::Subscriber> _subscribers; 

    ros::Subscriber _base_pose_subscriber;
    ros::Subscriber _joint_state_subscriber;
    ros::Subscriber _mpc_prediction_subscriber;
    ros::Subscriber _boundaries_subscriber;
    ros::Subscriber _pcl_subscriber; 
    ros::Subscriber _query_landing_subscriber;
    ros::Subscriber _proj_landing_subscriber;
    ros::Subscriber _ref_trj_subscriber;
    
    bool loadParameters();
    bool initSubscribers(const XmlRpc::XmlRpcValue& subscribers);  

    void basePoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);
    void jointStateCallback(const xbot_msgs::JointStateConstPtr& msg); 
    void mpcPredictionCallback(const kyon_controller::WBTrajectoryConstPtr& msg); 
    void markerArrayCallback(const visualization_msgs::MarkerArrayConstPtr& msg, int i);
    void pclCallback(const sensor_msgs::PointCloud2ConstPtr& msg, int i); 
    void markerCallback(const visualization_msgs::MarkerConstPtr& msg, int i); 
     
};

} // namespace leg_analyzer
