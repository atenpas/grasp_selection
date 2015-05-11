#include <grasp_selection/selection.h>


Selection::Selection(ros::NodeHandle& node, const std::string& grasps_topic, const std::string& cloud_topic,
	const Reaching::Parameters& reaching_params, const urdf::Model& urdf, const std::string& joint_states_topic,
	int num_selected, double marker_lifetime, int scoring_mode)
	: planning_frame_(reaching_params.planning_frame_), marker_lifetime_(marker_lifetime), has_grasps_(false), 
    has_cloud_(false), cloud_(new PointCloud), hand_offset_(reaching_params.hand_offset_), scoring_mode_(scoring_mode)
{
	// create subscriber to ROS topic <grasps_topic> from antigrasp package
	grasps_sub_ = node.subscribe(grasps_topic, 10, &Selection::graspsCallback, this);
	cloud_sub_ = node.subscribe(cloud_topic, 10, &Selection::cloudCallback, this);
  joint_states_sub_ = node.subscribe(joint_states_topic, 10, &Selection::jointStatesCallback, this);
	
	// create ROS service for the selected grasps
  service_ = node.advertiseService("select_grasps", &Selection::serviceCallback, this);
  
  // create publisher for visualizing the selected grasps in Rviz
  visuals_pub_ = node.advertise<visualization_msgs::MarkerArray>("grasps_selected", 10);
	
	reaching_ = new Reaching(reaching_params, node);
  
  // wait for joint names to appear on ROS topic
  joint_states_start_index_ = reaching_params.js_first_joint_index_;
  num_joints_ = reaching_params.js_last_joint_index_ - reaching_params.js_first_joint_index_ + 1;
  joint_names_.resize(num_joints_);
  ros::Rate rate(1);
  while (joint_names_[0].compare("") == 0 && ros::ok())
  { 
    ros::spinOnce();
    rate.sleep();
  }
  std::cout << "Knows joint names:\n";
  for (int i=0; i < joint_names_.size(); i++)
    std::cout << " " << i << ". " << joint_names_[i] << std::endl;
  std::cout << std::endl;
    
	scoring_ = new Scoring(urdf, joint_names_, reaching_params.min_aperture_, reaching_params.max_aperture_, 
    num_selected, scoring_mode_);
}


void Selection::runNode()
{
	ros::Rate rate(1);
  std::cout << "Waiting for grasps topic input ...\n";
  
  while (ros::ok())
  {
		ros::spinOnce();
    rate.sleep();
	}
}


void Selection::graspsCallback(const agile_grasp::Grasps& msg)
{
	if (has_grasps_)
		return;
	
	grasps_ = msg;	
	has_grasps_ = true;
  
  std::cout << "Received " << msg.grasps.size() << " grasps\n";
}

void Selection::cloudCallback(const sensor_msgs::PointCloud2& msg)
{
	if (!has_cloud_)
  {
    // convert ROS sensor message to PCL point cloud
    pcl::fromROSMsg(msg, *cloud_);
    std::cout << "Received point cloud for collision checking\n";

    // downsample the point cloud
    std::cout << "Voxelizing point cloud ... ";
    pcl::VoxelGrid<pcl::PointXYZ> vox;
    vox.setInputCloud(cloud_);
    vox.setLeafSize(0.006f, 0.006f, 0.006f);
    vox.filter(*cloud_);
    std::cout << cloud_->size() << " voxels left\n";

    has_cloud_ = true;
  }
}


void Selection::jointStatesCallback(const sensor_msgs::JointState& msg)
{
  if (joint_names_[0].compare("") == 0)
  {
    joint_names_.assign(&msg.name[joint_states_start_index_], &msg.name[joint_states_start_index_] + num_joints_);
  }
}


grasp_selection::GraspList Selection::createGraspListMsg(const std::vector<GraspScored>& list)
{
  grasp_selection::GraspList msg;
  msg.grasps.resize(list.size());
  
  for (int i=0; i < list.size(); i++)
  {
    grasp_selection::Grasp grasp;
    grasp.pose = list[i].pose_st_.pose;
    grasp.approach = list[i].approach_;    
    msg.grasps[i] = grasp;    
  }
  
  msg.header.frame_id = planning_frame_;
  msg.header.stamp = ros::Time::now();
  return msg;
}


bool Selection::serviceCallback(grasp_selection::SelectGrasps::Request& request, 
  grasp_selection::SelectGrasps::Response& response)
{
  if (grasps_.grasps.size() == 0)
  {
    ROS_ERROR("No grasps available!");
    has_grasps_ = false;
    std::cout << "Waiting for new grasps ...\n";
    return false;
  }
         
  // create feasible grasps
  std::cout << "Finding reachable grasps ...\n";
  std::vector<GraspScored> grasp_list = reaching_->selectFeasibleGrasps(grasps_);
  if (grasp_list.size() == 0)
  {
    ROS_ERROR("No reachable grasps found!");
    has_grasps_ = false;
    std::cout << "Waiting for new grasps ...\n";
    return false;
  }
  			
  // score those grasps
  std::vector<GraspScored> scored_list;
  if (scoring_mode_ == scoring_->SCORING_MODE_NONE)
  {
    std::cout << "No scoring used, returning " << grasp_list.size() << " reachable grasps\n";
    scored_list = grasp_list;    
  }
  else
  {
    std::cout << "Scoring %i reachable grasps " << grasp_list.size() << " ...\n";
    scored_list = scoring_->scoreGrasps(grasp_list, request.hand_pose);
  }
  
  // visualize grasps and create ROS message
  drawGrasps(scored_list);
  response.grasps = createGraspListMsg(scored_list);
  std::cout << "Created response with " << (int) response.grasps.grasps.size() << " grasps\n";
  
  // reset so that grasps msg and point cloud msg can be received again
  has_grasps_ = false;
  has_cloud_ = false;
  
  return true;
}


void Selection::drawGrasps(const std::vector<GraspScored>& list)
{
  double cyan[3] = {0, 1, 1};
  visualization_msgs::MarkerArray marker_array;
  marker_array.markers.resize(list.size());  
  
  for (int i=0; i < list.size(); i++)
  {
    geometry_msgs::Point position = list[i].pose_st_.pose.position;
    position.x += hand_offset_ * list[i].approach_.x;
    position.y += hand_offset_ * list[i].approach_.y;
    position.z += hand_offset_ * list[i].approach_.z;
    marker_array.markers[i] = createApproachMarker(planning_frame_, position, list[i].approach_, i, cyan, 0.4, 0.008);
  }
  
  visuals_pub_.publish(marker_array);
  std::cout << "Visualizing grasps ...\n";
  ros::Duration(1.0).sleep();
}


visualization_msgs::Marker Selection::createApproachMarker(const std::string& frame, 
  const geometry_msgs::Point& center, const geometry_msgs::Vector3& approach, int id, const double* color, 
  double alpha, double diam) const
{
  visualization_msgs::Marker marker = createMarker(frame);
  marker.type = visualization_msgs::Marker::ARROW;
  marker.id = id;
  marker.scale.x = diam; // shaft diameter
  marker.scale.y = diam; // head diameter
  marker.scale.z = 0.01; // head length
  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.color.a = alpha;
  geometry_msgs::Point p, q;
  p.x = center.x;
  p.y = center.y;
  p.z = center.z;
  q.x = p.x - 0.15 * approach.x;
  q.y = p.y - 0.15 * approach.y;
  q.z = p.z - 0.15 * approach.z;
  marker.points.push_back(p);
  marker.points.push_back(q);
  return marker;
}


visualization_msgs::Marker Selection::createMarker(const std::string& frame) const
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame;
  marker.header.stamp = ros::Time::now();
  marker.lifetime = ros::Duration(marker_lifetime_);
  marker.action = visualization_msgs::Marker::ADD;
  return marker;
}
