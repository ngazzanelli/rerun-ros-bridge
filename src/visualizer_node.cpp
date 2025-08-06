#include "visualizer.h"
#include <iostream>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "leg_analyzer_node");
  ros::NodeHandle nodeHandle("~");
  
  // std::vector<std::string> frames; 
  // if (!nodeHandle.getParam("frames", frames)) {
  //   ROS_ERROR("Frames parameter not set: exiting.");
  //   return 0; 
  // }
 
  std::vector<std::string> frames = {"contact_1", "contact_2", "contact_3", "contact_4"};
  int trail_length = 10; 
  bool rt_logging = false;
  bool visualize_perception = true;
  bool ground_truth = false;  

  leg_analyzer::LegAnalyzer legs_visualizer(nodeHandle, frames, trail_length,
                                              rt_logging, visualize_perception, ground_truth);
  
  double frequency;
  if (!nodeHandle.getParam("frequency", frequency)) {
    frequency = 30; 
  }

  ros::Timer timer = nodeHandle.createTimer(ros::Duration(1/frequency), [&](const ros::TimerEvent&) { legs_visualizer.updateState(); } );
  ros::MultiThreadedSpinner spinner(8); // Use 8 threads
  spinner.spin();
  ros::waitForShutdown();

  if(!rt_logging) {
    legs_visualizer.offlineLogTrj(); 
  }
  
  return 0; 
}
