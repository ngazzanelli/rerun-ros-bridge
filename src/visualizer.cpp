#include "visualizer.h"
#include <cmath>
#include <algorithm>
#include <sensor_msgs/PointField.h>

const double pi = M_PI;

namespace leg_analyzer {

LegAnalyzer::LegAnalyzer(ros::NodeHandle& node_handle, bool rt_logging, bool visualize_perception, bool ground_truth) : 
_nh(node_handle),
_rt_logging(rt_logging),
_visualize_perception(visualize_perception),
_rec(rerun::RecordingStream("Leg Kinematics")),
_init(true),
_ground_truth(ground_truth)
{
  
  bool parameters_loaded = loadParameters(); 

  jmap = {}; // we could read it from config.yaml

  // Initialize CasadiKinDyn obj to compute forward kinematics 
  std::string urdf;
  if (!_nh.getParam("/robot_description", urdf))
      throw std::runtime_error("Invalid URDF string");
  _kin_dyn = std::make_shared<casadi_kin_dyn::CasadiKinDyn>(urdf, false, jmap);
  _nq = _kin_dyn->nq(); // Total number of joints

  _joint_state = std::vector<float>(_nq - _base_pose.size(), 0);

  _mpc_prediction.resize(_nq);
  _nframes = _frames.size(); 

  // Retrieve forward kinematics functions 
  for(int i = 0; i < _nframes; ++i)
    _forward_kin_fncs.push_back(_kin_dyn->fk(_frames[i])); 

  // Initialize buffers for keeping trail_length points 
  _actual_trj.resize(_nframes); 
  for(int i =0; i < _nframes; ++i)
    _actual_trj[i].resize(_trail_length); 
  _predicted_trj.resize(_nframes);
  
  
  // Initialize full state vector 
  _full_state.reserve(_base_pose.size() + _joint_state.size());
  _full_state.insert(_full_state.end(), _base_pose.begin(), _base_pose.end());
  _full_state.insert(_full_state.end(), _joint_state.begin(), _joint_state.end());

  // Try to spawn a new viewer instance.
  _rec.spawn().exit_on_failure();
} 


LegAnalyzer::~LegAnalyzer() = default;


bool LegAnalyzer::loadParameters()
{
  // Load (mandatory) frames parameter
  if(!_nh.getParam("frames/name", _frames) || _frames.empty()) {
    ROS_ERROR("Frames not specified, the visualizer cannot be configured. Exit");
    return false; 
  }
  _nh.param("frames/trail_length", _trail_length, 2);

  // Load subcribers parameter
  XmlRpc::XmlRpcValue subscribers;
  _nh.getParam("subscribers", subscribers);
  if(!subscribers.valid()) {
    ROS_ERROR("Subscribers not specified; at least joint state must be present. Exit");
    return false;
  }

  bool validate_subscribers = initSubscribers(subscribers); 

  return validate_subscribers; 

}


bool LegAnalyzer::initSubscribers(const XmlRpc::XmlRpcValue& subscribers)
{
  // Check for (mandatory) joint state topic 
  bool joint_state_found = false; 
  std::string type; 
   
  for(auto& subscriber : subscribers) {
    std::string key = subscriber.first;
    std::string topic_name = subscriber.second["topic_name"];
    auto type = static_cast<std::string>(subscriber.second["data_type"]);

    if(type == "joint_states") {
      // boost::function<void(const xbot_msgs::JointStateConstPtr&)> f = boost::bind(&LegAnalyzer::jointStateCallback, this, _1, key);
      ros::Subscriber sub = _nh.subscribe(topic_name, 1, &LegAnalyzer::jointStateCallback, this);
      joint_state_found = true;
      _subscribers.push_back(sub); 
    }
    
    else if(key == "odom" and type == "pose") {
      ros::Subscriber sub = _nh.subscribe(topic_name, 1, &LegAnalyzer::basePoseCallback, this);
      int size = subscriber.second["size"];

      // Initialize base pose with a default value
      _base_pose = std::vector<float>(size, 0);
      if(size == 7)
        _base_pose[6] = 1.0; // Set the quaternion w component to 1 for identity rotation
      _subscribers.push_back(sub);
    }

    else if(type == "marker") {
      return false; 
    }

    else if(type == "marker_array")
      return false;

    else if(type == "pointcloud")
      return false;

    else {
      ROS_WARN_STREAM("Data type \"" << type << "\" for subscriber \"" << key << "\" not supported."); 
    }

  }
  // Manage Odometry 

  return joint_state_found; 
} 


void LegAnalyzer::basePoseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{ 
  std::lock_guard<std::mutex> lock(_base_pose_mutex);
  _base_pose[0] = msg->pose.position.x;
  _base_pose[1] = msg->pose.position.y;
  _base_pose[2] = msg->pose.position.z;
  _base_pose[3] = msg->pose.orientation.x;
  _base_pose[4] = msg->pose.orientation.y;
  _base_pose[5] = msg->pose.orientation.z;
  _base_pose[6] = msg->pose.orientation.w;

}


void LegAnalyzer::jointStateCallback(const xbot_msgs::JointStateConstPtr& msg)
{
  if (msg->name.size() != _nq - _base_pose.size()) {
    throw std::runtime_error("Joint state size does not match the expected size.");
    return;
  }

  std::lock_guard<std::mutex> lock(_joint_state_mutex);
  for (size_t i = 0; i < msg->name.size(); ++i) {
    _joint_state[i] = msg->motor_position[i];
  }

}


void LegAnalyzer::mpcPredictionCallback(const kyon_controller::WBTrajectoryConstPtr& msg)
{
  int prediction_steps = msg->q.size()/_nq;

  if(_init) { 
    for (int i = 0; i < _nq; ++i)
      _mpc_prediction[i].resize(prediction_steps);  

    for (int i = 0; i < _nframes; ++i) 
      _predicted_trj[i].resize(prediction_steps);
    
    _init = false; 
  }

  for(int i = 0; i < _nq; ++i)
    for(int j = 0; j < prediction_steps; ++j) 
      _mpc_prediction[i][j] = msg->q[i*prediction_steps + j];
  
  if(!_ground_truth)
  {
    for(int i = 0; i < _base_pose.size(); ++i)
      _base_pose[i] = msg->q[i*prediction_steps]; 
  }
}


void LegAnalyzer::boundariesCallback(const visualization_msgs::MarkerArrayConstPtr& msg)
{
  std::vector<rerun::Position3D> boundary;
  std::vector<rerun::LineStrip3D> boundaries; 
  std::vector<rerun::Color> colors;
  rerun::Position3D tmp;
  rerun::Color tmp_color;
  uint8_t r, g, b, a;

  for(int i = 0; i < msg->markers.size(); ++i) {
    if(msg->markers[i].points.size() > 0) {
      for(int j = 0; j < msg->markers[i].points.size(); ++j) {
        tmp = rerun::Position3D(msg->markers[i].points[j].x, msg->markers[i].points[j].y, msg->markers[i].points[j].z);
        boundary.push_back(tmp); 
      }

      r = static_cast<uint8_t>(msg->markers[i].color.r * 255.0f);
      g = static_cast<uint8_t>(msg->markers[i].color.g * 255.0f);
      b = static_cast<uint8_t>(msg->markers[i].color.b * 255.0f);
      a = static_cast<uint8_t>(msg->markers[i].color.a * 255.0f);
      tmp_color = rerun::Color(r, g, b, a);

      boundaries.push_back(rerun::LineStrip3D(boundary)); 
      colors.push_back(tmp_color);
      boundary.clear(); 
    }
  }
  
  double ros_time = ros::Time::now().toSec(); 
  if(_rt_logging) {
    _rec.set_time_duration_secs("ros_time", ros_time);
    _rec.log("boundaries", rerun::LineStrips3D(boundaries).with_colors(colors).with_radii(0.005f)); 
  }
  else {
    _ros_timeline_boundaries.push_back(ros_time); 
    _boundaries_offline.push_back(boundaries);
    _boundaries_colors.push_back(colors); 
  }

}


void LegAnalyzer::pclCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  size_t x_offset, y_offset, z_offset, rgb_offset;
  bool has_x{false}, has_y{false}, has_z{false}, has_rgb{false};

  for (const auto& field : msg->fields) {
    if (field.name == "x") {
        x_offset = field.offset;
        if (field.datatype != sensor_msgs::PointField::FLOAT32) {
            _rec.log("pointcloud", rerun::TextLog("Only FLOAT32 x field supported"));
            return;
        }
        has_x = true;
    } else if (field.name == "y") {
        y_offset = field.offset;
        if (field.datatype != sensor_msgs::PointField::FLOAT32) {
            _rec.log("pointcloud", rerun::TextLog("Only FLOAT32 y field supported"));
            return;
        }
        has_y = true;
    } else if (field.name == "z") {
        z_offset = field.offset;
        if (field.datatype != sensor_msgs::PointField::FLOAT32) {
            _rec.log("pointcloud", rerun::TextLog("Only FLOAT32 z field supported"));
            return;
        }
        has_z = true;
    }
  }

  if (!has_x || !has_y || !has_z) {
    _rec.log(
        "pointcloud",
        rerun::TextLog("Currently only PointCloud2 messages with x, y, z fields are supported")
    );
    return;
  }

  std::vector<rerun::Position3D> points(msg->width * msg->height);
  std::vector<rerun::Color> colors;

  if (has_rgb) {
      colors.reserve(msg->width * msg->height);
  }

  for (size_t i = 0; i < msg->height; ++i) {
      for (size_t j = 0; j < msg->width; ++j) {
          auto point_offset = i * msg->row_step + j * msg->point_step;
          rerun::Position3D position;
          // TODO(leo) if xyz are consecutive fields we can do this in a single memcpy
          std::memcpy(&position.xyz.xyz[0], &msg->data[point_offset + x_offset], sizeof(float));
          std::memcpy(&position.xyz.xyz[1], &msg->data[point_offset + y_offset], sizeof(float));
          std::memcpy(&position.xyz.xyz[2], &msg->data[point_offset + z_offset], sizeof(float));
          if (has_rgb) {
              uint8_t bgra[4];
              std::memcpy(&bgra, &msg->data[point_offset + rgb_offset], sizeof(uint32_t));
              colors.emplace_back(rerun::Color(bgra[2], bgra[1], bgra[0]));
          }
          points[i * msg->width + j] = position;
      }
  }
   
  double ros_time = ros::Time::now().toSec();

  if(_rt_logging) {
    _rec.set_time_duration_secs("ros_time", ros_time);
    _rec.log("pointcloud", rerun::Points3D(points).with_colors(colors));  
  }
  else {
    _pointclouds.push_back(points);
    _ros_timeline_pcl.push_back(ros_time);
  } 
   
}


void LegAnalyzer::queryPointCallback(const visualization_msgs::MarkerConstPtr& msg)
{
  double ros_time = msg->header.stamp.toSec(); 
  std::vector<rerun::Position3D> points; 

  for(int i = 0; i < msg->points.size(); ++i) {
    rerun::Position3D tmp(msg->points[i].x, msg->points[i].y, msg->points[i].z);
    points.push_back(tmp); 
  }

  if(_rt_logging) {
  _rec.set_time_duration_secs("ros_time", ros_time);
  _rec.log("query_landing_points", rerun::Points3D(points).with_colors(rerun::Color({0xb3000088})).with_radii(0.03f)); 
  }
  else {
    _query_points.push_back(points);
    _ros_timeline_query_points.push_back(ros_time);
  }

}


void LegAnalyzer::projectedPointCallback(const visualization_msgs::MarkerConstPtr& msg)
{
  double ros_time = msg->header.stamp.toSec(); 
  std::vector<rerun::Position3D> points; 

  for(int i = 0; i < msg->points.size(); ++i) {
    rerun::Position3D tmp(msg->points[i].x, msg->points[i].y, msg->points[i].z);
    points.push_back(tmp); 
  }

  if(_rt_logging) {
  _rec.set_time_duration_secs("ros_time", ros_time);
  _rec.log("projected_landing_points", rerun::Points3D(points).with_colors(rerun::Color({0x00993388})).with_radii(0.03f)); 
  }
  else {
    _proj_points.push_back(points);
    _ros_timeline_proj_points.push_back(ros_time);
  }
}


void LegAnalyzer::refTrjCallback(const visualization_msgs::MarkerConstPtr& msg)
{
  double ros_time = msg->header.stamp.toSec(); 
  std::vector<rerun::Position3D> points; 

  for(int i = 0; i < msg->points.size(); ++i) {
    rerun::Position3D tmp(msg->points[i].x, msg->points[i].y, msg->points[i].z);
    points.push_back(tmp); 
  }

  if(_rt_logging) {
  _rec.set_time_duration_secs("ros_time", ros_time);
  _rec.log("reference_trj", rerun::Points3D(points).with_colors(rerun::Color({0xfcec03AA})).with_radii(0.03f)); 
  }
  else {
    _ref_trj.push_back(points);
    _ros_timeline_ref_trj.push_back(ros_time);
  }
}


void LegAnalyzer::updateState()
{
 
  {
  std::lock_guard<std::mutex> lock(_base_pose_mutex); 
  for (int i = 0; i < _base_pose.size(); ++i)
    _full_state[i] = _base_pose[i];
  }
  
  {
  std::lock_guard<std::mutex> lock(_joint_state_mutex);
  for (int j = 0; j < _joint_state.size(); ++j)
    _full_state[j + 7] = _joint_state[j];
  }

  computeForwardKinematics(); 

}


void LegAnalyzer::computeForwardKinematics()
{
  // Prepare input and output structures 
  casadi::DMDict fk_input = {{"q", casadi::DM(_full_state)}};
  casadi::DMDict fk_output;
  std::vector<rerun::LineStrip3D> strips_actual, strips_predicted;
  std::vector<float> _predicted_full_state(_full_state);
  int prediction_steps = _mpc_prediction[0].size(); 

  // Compute actual feet trajectories
  for(int i = 0; i < _nframes; ++i) {
    fk_output = _forward_kin_fncs[i](fk_input); 

    double x = fk_output["ee_pos"](0).scalar();
    double y = fk_output["ee_pos"](1).scalar();
    double z = fk_output["ee_pos"](2).scalar();

    rerun::Position3D tmp(x, y, z); 
    _actual_trj[i].push_back(tmp);
    strips_actual.push_back(rerun::LineStrip3D(_actual_trj[i])); 
  }
  

  // Compute predicted feet trajectories 
  for(int k = 0; k < _nframes; ++k) {
    for(int j = 0; j < prediction_steps; ++j) {
      for(int i = 0; i < _nq; ++i)
        _predicted_full_state[i] = _mpc_prediction[i][j];

      fk_input["q"] = _predicted_full_state;
      fk_output = _forward_kin_fncs[k](fk_input);
      double x = fk_output["ee_pos"](0).scalar();
      double y = fk_output["ee_pos"](1).scalar();
      double z = fk_output["ee_pos"](2).scalar();

      rerun::Position3D tmp(x, y, z); 
      _predicted_trj[k].push_back(tmp);
    }
    strips_predicted.push_back(rerun::LineStrip3D(_predicted_trj[k]));
    _predicted_trj[k].clear(); 
    
  }

  double ros_time = ros::Time::now().toSec(); 
  // Rerun Visualization 
  if(_rt_logging) {
    _rec.set_time_duration_secs("ros_time", ros_time);
    std::vector<rerun::Color> trj_colors = {0x0099ffFF, 0xff8000FF, 0x00b33cFF, 0xcc0099FF};
    std::vector<rerun::Color> trj_colors_p = {0x0099ff55, 0xff800055, 0x00b33c55, 0xcc009955};
    _rec.log("actual_position", rerun::LineStrips3D().with_strips(strips_actual).with_colors(trj_colors).with_radii(0.002f));
    _rec.log("predicted_position", rerun::LineStrips3D().with_strips(strips_predicted).with_colors(trj_colors_p).with_radii(0.003f));
  }
  else {
    _ros_timeline_trjs.push_back(ros_time);
    _strips_actual_offline.push_back(strips_actual);
    _strips_predicted_offline.push_back(strips_predicted);
  }

}


void LegAnalyzer::offlineLogTrj()
{

  std::vector<rerun::Color> trj_colors = {0x0099ffFF, 0xff8000FF, 0x00b33cFF, 0xcc0099FF};
  std::vector<rerun::Color> trj_colors_p = {0x0099ff55, 0xff800055, 0x00b33c55, 0xcc009955};

  for(int i = 0; i < _ros_timeline_trjs.size(); ++i) {
    _rec.set_time_duration_secs("ros_time", _ros_timeline_trjs[i]);
    _rec.log("actual_position", rerun::LineStrips3D().with_strips(_strips_actual_offline[i]).with_colors(trj_colors).with_radii(0.002f));
    _rec.log("predicted_position", rerun::LineStrips3D().with_strips(_strips_predicted_offline[i]).with_colors(trj_colors_p).with_radii(0.003f));  
  } 

  if(_visualize_perception) { 
    for(int i = 0; i < _ros_timeline_boundaries.size(); ++i) {
      _rec.set_time_duration_secs("ros_time", _ros_timeline_boundaries[i]);
      _rec.log("boundaries", rerun::LineStrips3D(_boundaries_offline[i]).with_colors(_boundaries_colors[i]).with_radii(0.005f));
    }
    for(int i = 0; i < _ros_timeline_pcl.size(); ++i) {
      _rec.set_time_duration_secs("ros_time", _ros_timeline_pcl[i]);
      _rec.log("pointclouds", rerun::Points3D(_pointclouds[i]).with_colors(rerun::Color({0x73737388})));
    }
    for(int i = 0; i < _ros_timeline_query_points.size(); ++i) {
      _rec.set_time_duration_secs("ros_time", _ros_timeline_query_points[i]);
      _rec.log("query_landing_points", rerun::Points3D(_query_points[i]).with_colors(rerun::Color({0xb3000088})).with_radii(0.02f));
    }
    for(int i = 0; i < _ros_timeline_proj_points.size(); ++i) {
      _rec.set_time_duration_secs("ros_time", _ros_timeline_proj_points[i]);
      _rec.log("projected_landing_points", rerun::Points3D(_proj_points[i]).with_colors(rerun::Color({0x00993388})).with_radii(0.02f));
    }
    for(int i = 0; i < _ros_timeline_ref_trj.size(); ++i) {
      _rec.set_time_duration_secs("ros_time", _ros_timeline_ref_trj[i]);
      _rec.log("reference_trj", rerun::Points3D(_ref_trj[i]).with_colors(rerun::Color({0xfcec03AA})).with_radii(0.003f));
    }
  } 

}


} // namespace leg_analyzer