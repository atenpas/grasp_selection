#include <ros/ros.h>
#include <urdf/model.h>

#include <vector>

#include <grasp_selection/reaching.h>
#include <grasp_selection/scoring.h>
#include <grasp_selection/selection.h>


int main(int argc, char** argv)
{
	// initialize ROS
  ros::init(argc, argv, "select_grasps");
  ros::NodeHandle node("~");
  
  // read ROS launch file parameters for reaching class
  Reaching::Parameters params;
  node.getParam("workspace", params.workspace_);
  node.getParam("min_aperture", params.min_aperture_);
  node.getParam("max_aperture", params.max_aperture_);
  node.getParam("num_additional_grasps", params.num_additional_grasps_);
  node.getParam("axis_order", params.axis_order_);
  node.getParam("planning_frame", params.planning_frame_);
  node.getParam("hand_offset", params.hand_offset_);
  node.getParam("arm_link", params.arm_link_);
  node.getParam("move_group", params.move_group_);
  node.getParam("max_colliding_points", params.max_colliding_points_);
  node.getParam("JS_first_joint_index", params.js_first_joint_index_);
  node.getParam("JS_last_joint_index", params.js_last_joint_index_);
  node.getParam("IK_first_joint_index", params.ik_first_joint_index_);
  node.getParam("IK_last_joint_index", params.ik_last_joint_index_);
  node.getParam("prints", params.is_printing_);
  
  // read ROS launch file parameters for scoring class
  std::string urdf_filename;  
  int num_selected;
  std::vector<double> initial_pose;
  node.getParam("urdf", urdf_filename);  
  node.getParam("num_selected", num_selected); 
    
  // read ROS launch file parameters for selection class
  std::string grasps_topic;
  std::string cloud_topic;
  std::string joint_states_topic;
  bool plots;
  double marker_lifetime;
  bool uses_scoring;
  int scoring_mode;
  node.getParam("grasps_topic", grasps_topic);
  node.getParam("cloud_topic", cloud_topic);
  node.getParam("joint_states_topic", joint_states_topic);
  node.getParam("marker_lifetime", marker_lifetime);
  node.getParam("scoring_mode", scoring_mode);
    
  // get robot joints information from URDF file
  urdf::Model urdf;
  if (!urdf.initFile(urdf_filename))
  {
    ROS_ERROR("Failed to parse urdf file");
    return -1;
  }
  ROS_INFO("Successfully parsed urdf file");
  
  // create selection object and select grasps
  Selection selection(node, grasps_topic, cloud_topic, params, urdf, joint_states_topic, num_selected, marker_lifetime, 
    scoring_mode);
  selection.runNode();
  	
	return 0;
}
