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
  std::optional<float> prune_factor; // TBD   
};


struct ScalarData {
  std::string key; 
  std::vector<double> data; 
  std::vector<double> ros_timeline; 
  // TBD float data_size;          
  // TBD std::string topic_field;
};


class LegAnalyzer {

  public:
    LegAnalyzer(ros::NodeHandle& node_handle, bool ground_truth);
    ~LegAnalyzer(); 

    std::map<std::string, double> jmap;  
    const rerun::RecordingStream _rec; 

    void computeForwardKinematics(double ros_time);
    void tryUpdateState(); 
    void offlineLogTrj(); 
    

  private:
    ros::NodeHandle _nh; 

    int _nq; 
    int _nframes; 
    int _trail_length; 
    bool _init;
    bool _odom_provided; 
    bool _ground_truth;  

    // Data synchronization
    std::mutex _data_mutex;
    geometry_msgs::PoseStampedConstPtr _latest_odom;
    xbot_msgs::JointStateConstPtr _latest_joint_states;
    ros::Time last_processed_odom_time_;
    ros::Time last_processed_joint_time_;
    
    // Kinematics related data structures
    std::shared_ptr<casadi_kin_dyn::CasadiKinDyn> _kin_dyn;
    std::vector<std::string> _frames; 
    std::vector<casadi::Function> _forward_kin_fncs; 

    // State related data structures
    std::vector<float> _base_pose;
    std::vector<float> _joint_state;
    std::vector<float> _full_state;
    
    // Marker data structure
    std::vector<struct MarkerData> _markers; 

    // Marker array data strucutre
    std::vector<struct MarkerArrayData> _marker_arrays; 
    
    // Pointcloud data structures
    std::vector<struct PointCloudData> _pointclouds; 

    // Scalar data structures
    std::vector<struct ScalarData> _scalars; 

    // WORK IN PROGRESS ON PREDICTED TRAJECTORY
    std::vector<std::vector<double>> _mpc_prediction;  
    std::vector<std::vector<rerun::Position3D>> _predicted_trj;
    std::vector<boost::circular_buffer<rerun::Position3D>> _actual_trj;
    std::vector<std::vector<rerun::Position3D>> _ref_trj;
    ros::Subscriber _mpc_prediction_subscriber;
    
    std::vector<double> _ros_timeline_trjs;
    std::vector<std::vector<rerun::LineStrip3D>> _strips_actual_offline, _strips_predicted_offline; 
    
    std::vector<ros::Subscriber> _subscribers; 

    
    
    bool loadParameters();
    bool initSubscribers(const XmlRpc::XmlRpcValue& subscribers);  

    void basePoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);
    void jointStateCallback(const xbot_msgs::JointStateConstPtr& msg); 
    void mpcPredictionCallback(const kyon_controller::WBTrajectoryConstPtr& msg); 
    void markerArrayCallback(const visualization_msgs::MarkerArrayConstPtr& msg, int i);
    void pclCallback(const sensor_msgs::PointCloud2ConstPtr& msg, int i); 
    void markerCallback(const visualization_msgs::MarkerConstPtr& msg, int i); 
    void scalarCallback(const xbot_msgs::JointStateConstPtr& msg, int i);
};

} // namespace leg_analyzer
