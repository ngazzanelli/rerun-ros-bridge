#include "visualizer.h"
#include <iostream>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "leg_analyzer_node");
  ros::NodeHandle nodeHandle("~");
   
  bool rt_logging = false;
  bool visualize_perception = true;
  bool ground_truth = false;  

  leg_analyzer::LegAnalyzer legs_visualizer(nodeHandle, rt_logging, visualize_perception, ground_truth);

  ros::MultiThreadedSpinner spinner(8); // Use 8 threads
  spinner.spin();
  ros::waitForShutdown();

  if(!rt_logging) {
    legs_visualizer.offlineLogTrj(); 
  }
  
  return 0; 
}
