#include "visualizer.h"
#include <iostream>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "leg_analyzer_node");
  ros::NodeHandle nodeHandle("~");
   
  bool ground_truth = false;  

  leg_analyzer::LegAnalyzer legs_visualizer(nodeHandle, ground_truth);

  ros::MultiThreadedSpinner spinner(8); // Use 8 threads
  spinner.spin();
  ros::waitForShutdown();

  legs_visualizer.offlineLogTrj(); 
  
  return 0; 
}
