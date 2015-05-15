#include <grasp_selection/reaching.h>


Reaching::Reaching(const Parameters& params, ros::NodeHandle& node) : params_(params), cloud_(new PointCloud)
{
	// wait for Inverse Kinematics service
  ik_solver_ = node.serviceClient<moveit_msgs::GetPositionIK>("/compute_ik");
  while (!ik_solver_.exists())
  {
    ROS_INFO("Waiting for compute_ik service ...");
    ros::Duration(1.0).sleep();
  }
  ROS_INFO("compute_ik service is available");
}


std::vector<GraspScored> Reaching::selectFeasibleGrasps(const agile_grasp::Grasps& grasps_in)
{
	std::vector<GraspScored> grasps_selected;
		
	// evaluate the reachability of each grasp
	for (int i = 0; i < grasps_in.grasps.size(); i++)
  {
    const agile_grasp::Grasp& grasp = grasps_in.grasps[i];
    
    // check whether grasp lies within the workspace of the robot arm
    if (params_.is_printing_)
      printf("Checking if grasp %i, position (%1.2f, %1.2f, %1.2f), can be reached: ", i, grasp.center.x, 
        grasp.center.y, grasp.center.z);
		if (!isInWorkspace(grasp.center.x, grasp.center.y, grasp.center.z))
		{
			if (params_.is_printing_)
        printf(" NOT OK!\n");
			continue;
		}
		if (params_.is_printing_)
      printf(" OK\n");

    // avoid objects that are smaller/larger than the minimum/maximum robot hand aperture
    if (params_.is_printing_)
      printf("Checking aperture: ");
    if (grasp.width.data < params_.min_aperture_ || grasp.width.data > params_.max_aperture_)
    {
      if (params_.is_printing_)
        printf("too small/large for the hand (min, max): %.4f (%.4f, %.4f)!\n", grasp.width.data, params_.min_aperture_, 
          params_.max_aperture_);
      continue;
    }
    if (params_.is_printing_)
      printf(" OK\n");
    
    GraspEigen grasp_eigen(grasp);
    
    // generate additional grasps
    Eigen::VectorXd theta;
    if (params_.num_additional_grasps_ > 0)
    {
      theta = Eigen::VectorXd::LinSpaced(1 + params_.num_additional_grasps_, -15.0, 15.0);
    }
    else
    {
      theta.resize(1);
      theta << 0.0;
    }
    
    // check all grasps for reachability
    for (int j = 0; j < theta.size(); j++)
    {
			if (params_.is_printing_)
        printf("j: %i \n", j);
      
      // calculate approach vector and hand axis for the new grasp
			GraspEigen grasp_eigen_rot = rotateGrasp(grasp_eigen, theta[j]);
			
			// filter out side grasps
			//if (isSideGrasp(grasp_eigen_rot.approach_))
			//{
				//printf("Grasp %i is a side grasp (restricted to top grasps)!\n", i);
				//continue;
			//}
			
			// create a grasp for each hand orientation and check whether they are reachable by the IK and collision-free
      std::vector<tf::Quaternion> quats = calculateHandOrientations(grasp_eigen_rot);
      bool is_collision_free = false;      
      for (int k = 0; k < quats.size(); k++)
      {
        if (params_.is_printing_)
          printf("k: %i \n", k);
        
        // create grasp pose
        geometry_msgs::PoseStamped grasp_pose = createGraspPose(grasp_eigen_rot, quats[k], theta[j]);
        
        // try to solve IK
        if (params_.is_printing_)
          printf(" Solving IK: ");
        double tik0 = omp_get_wtime();
				moveit_msgs::GetPositionIK::Response response = solveIK(grasp_pose);
        std::cout << " IK runtime: " << omp_get_wtime() - tik0 << std::endl;
				if (response.error_code.val == response.error_code.NO_IK_SOLUTION) // IK fails
				{
					if (params_.is_printing_)
            printf("IK failed for grasp %i, approach %i, orientation %i!\n", i, j, k);
					continue;
				}
        if (params_.is_printing_)
          printf("OK\n");
        
        // check collisions (only required for one orientation/quaternion)
        if (params_.is_printing_)
          printf(" Checking collisions: ");
        if (!is_collision_free)
        {
          double tcoll0 = omp_get_wtime();
					is_collision_free = isCollisionFree(grasp_pose, grasp_eigen_rot.approach_);
          std::cout << " Collision checker runtime: " << omp_get_wtime() - tcoll0 << std::endl;
					if (!is_collision_free)
					{
						if (params_.is_printing_)
              printf("Grasp %i, approach %i, orientation %i collides with point cloud!\n", i, j, k);
						continue;
					}
				}
        if (params_.is_printing_)
          printf("OK\n");
				
        if (params_.is_printing_)
          printf("before extracting joint positions\n");
				std::vector<double> joint_positions = extractJointPositions(response);
        if (params_.is_printing_)
          printf("extracted joint positions\n");
				GraspScored grasp_scored(i, grasp_pose, grasp_eigen_rot.approach_, grasp.width.data, joint_positions, 0.0, 
          grasps_in.grasps[i].surface_center);
				grasps_selected.push_back(grasp_scored);
      }
		}
	}
	
	return grasps_selected;
}


bool Reaching::isInWorkspace(double x, double y, double z)
{
	if (x >= params_.workspace_[0] && x <= params_.workspace_[1] && y >= params_.workspace_[2] 
		&& y <= params_.workspace_[3]	&& z >= params_.workspace_[4] && z <= params_.workspace_[5])
	{
		return true;
	}
	
	return false;
}


Reaching::GraspEigen Reaching::rotateGrasp(const GraspEigen& grasp_in, double theta)
{
	GraspEigen grasp_out;
	
	Eigen::Transform<double, 3, Eigen::Affine> R(Eigen::AngleAxis<double>(theta * (M_PI / 180.0), grasp_in.binormal_));
	grasp_out.axis_ = R * grasp_in.axis_;
	grasp_out.approach_ = R * (-1.0 * grasp_in.approach_);	
	
	grasp_out.binormal_ = grasp_out.axis_.cross(grasp_out.approach_);
	grasp_out.center_ = grasp_in.center_;
	
	return grasp_out;
}


std::vector<tf::Quaternion> Reaching::calculateHandOrientations(const GraspEigen& grasp)
{
  // calculate first hand orientation
  Eigen::Matrix3d R = Eigen::MatrixXd::Zero(3, 3);
  R.col(0) = -1.0 * grasp.approach_;
  R.col(1) = grasp.axis_;
  R.col(2) << R.col(0).cross(R.col(1));
	
	// rotate by 180deg around the grasp approach vector to get the "opposite" hand orientation
	Eigen::Transform<double, 3, Eigen::Affine> T(Eigen::AngleAxis<double>(M_PI, grasp.approach_));
	
	// calculate second hand orientation
  Eigen::Matrix3d Q = Eigen::MatrixXd::Zero(3, 3);
  Q.col(0) = T * grasp.approach_;
  Q.col(1) = T * grasp.axis_;
  Q.col(2) << Q.col(0).cross(Q.col(1));
  
  // reorder rotation matrix columns according to axes ordering of the robot hand
  Eigen::Matrix3d R1 = reorderHandAxes(R);
  Eigen::Matrix3d R2 = reorderHandAxes(Q);
	
	// convert Eigen rotation matrices to TF quaternions and normalize them
	tf::Matrix3x3 TF1, TF2;
  tf::matrixEigenToTF(R1, TF1);
  tf::matrixEigenToTF(R2, TF2);
  tf::Quaternion quat1, quat2;
  TF1.getRotation(quat1);
  TF2.getRotation(quat2);
  quat1.normalize();
  quat2.normalize();
		
	std::vector<tf::Quaternion> quats;
	quats.push_back(quat1);
	quats.push_back(quat2);
  
  // for debugging
  //tf::TransformBroadcaster br;
  //tf::Transform transform;
  //transform.setOrigin( tf::Vector3(grasp.center_(0), grasp.center_(1), grasp.center_(2)) );
  //transform.setRotation( quat1 );
  //br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base", "carrot1"));
  //ros::Duration(1.0).sleep();
  //transform.setRotation( quat2 );
  //br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base", "carrot2"));
  //ros::Duration(1.0).sleep();
  //std::cout << "transform:\n" << transform.getOrigin() << std::endl;
  
	return quats;
}


Eigen::Matrix3d Reaching::reorderHandAxes(const Eigen::Matrix3d& Q)
{
  Eigen::Matrix3d R = Eigen::MatrixXd::Zero(3, 3);
  R.col(params_.axis_order_[0]) = Q.col(0); // grasp approach vector
  R.col(params_.axis_order_[1]) = Q.col(1); // hand axis
  R.col(params_.axis_order_[2]) = Q.col(2); // hand binormal
  return R;
}


geometry_msgs::PoseStamped Reaching::createGraspPose(const GraspEigen& grasp, const tf::Quaternion& quat, double theta)
{  
  // calculate grasp position
  Eigen::Vector3d position;
  Eigen::Vector3d approach = -1.0 * grasp.approach_;
  if (theta != 0)
  {
    // project grasp bottom position onto the line defined by grasp surface position and approach vector
    Eigen::Vector3d s, b, a;
    position = (grasp.center_ - grasp.surface_center_).dot(approach) * approach;
    position += grasp.surface_center_;
  }
  else
    position = grasp.center_;
    
	// translate grasp position by <hand_offset_> along the grasp approach vector
	position = position + params_.hand_offset_ * approach;
  		
	geometry_msgs::PoseStamped pose_st;
	pose_st.header.stamp = ros::Time(0);
  pose_st.header.frame_id = params_.planning_frame_;
  tf::pointEigenToMsg(position, pose_st.pose.position);
  tf::quaternionTFToMsg(quat, pose_st.pose.orientation);
    
	return pose_st;
}


moveit_msgs::GetPositionIK::Response Reaching::solveIK(const geometry_msgs::PoseStamped& pose, int attempts, 
	double timeout)
{
  // create IK request
  moveit_msgs::GetPositionIK::Request request;
  request.ik_request.group_name = params_.move_group_;
  request.ik_request.attempts = attempts;
  request.ik_request.timeout = ros::Duration(timeout);
  request.ik_request.pose_stamped = pose;
  request.ik_request.pose_stamped.header.stamp = ros::Time::now();
  request.ik_request.ik_link_name = params_.arm_link_;
  request.ik_request.avoid_collisions = true;
  
  //std::cout << "IK Request:\n" << request << std::endl;

  // solve IK
  moveit_msgs::GetPositionIK::Response response;
  ik_solver_.call(request, response);
  return response;
}


bool Reaching::isCollisionFree(const geometry_msgs::PoseStamped& pose_st, const Eigen::Vector3d& approach)
{
	const double R = 0.06; // radius of cylinder
  const double L = 0.1; // height of cylinder
  const double OFFSET = 0.005; // used to compensate invalid sensor measurements on object sides
  double r2 = R * R;

  // calculate lower and upper cylinder caps
  Eigen::Vector3d c0;
  tf::pointMsgToEigen(pose_st.pose.position, c0);
  Eigen::Vector3d c1 = c0 - L * approach;
  Eigen::Vector3d c = c0 + 0.5 * (c1 - c0);

  // plane defined by handle approach and hands centroid
  Eigen::Vector3d n = -1.0 * approach;
  Eigen::Vector3d s = c - OFFSET * approach;
  
  // check each point in point cloud against collision cylinder
  int k = 0;
  for (int j = 0; j < cloud_->size(); j++)
  {
    Eigen::Vector3d p = (cloud_->points[j].getVector3fMap()).cast<double>();

    // check whether point lies on side of plane (s,n) that points toward upper cylinder cap and between lower and 
    // upper cylinder cap, and compare distance(point,cylinder_axis)^2 with radius^2
    if (n.dot(p - s) < 0 && approach.dot(p - c0) < 0 && approach.dot(p - c1) > 0
        && (((p - c) - (p - c).dot(approach) * approach).squaredNorm() <= r2))
    {
      k++;
      if (k > params_.max_colliding_points_)
        return false;
    }
  }
  
	return true;
}


std::vector<double> Reaching::extractJointPositions(const moveit_msgs::GetPositionIK::Response& ik_response)
{
	int num_joints = params_.ik_last_joint_index_ - params_.ik_first_joint_index_ + 1;  
  std::vector<double> joint_positions(ik_response.solution.joint_state.position.begin() + params_.ik_first_joint_index_, 
		ik_response.solution.joint_state.position.begin() + params_.ik_first_joint_index_ + num_joints);
	return joint_positions;
}
