#include <grasp_selection/scoring.h>


Scoring::Scoring(const urdf::Model& urdf, const std::vector<std::string>& joint_names, double min_aperture, 
	double max_aperture, int num_selected, int scoring_mode)
	: min_aperture_(min_aperture), max_aperture_(max_aperture), num_selected_(num_selected)
{
	// get joint limits from URDF	
	joint_limits_.resize(2, joint_names.size());
	std::cout << "Joint Limits (given by URDF)\n";
  for (int i = 0; i < joint_limits_.cols(); i++)
  {
    joint_limits_(0, i) = urdf.getJoint(joint_names[i])->limits->lower;
    joint_limits_(1, i) = urdf.getJoint(joint_names[i])->limits->upper;
    std::cout << joint_names[i].c_str() << ": [" << joint_limits_(0, i) << ", " << joint_limits_(1, i) << "]\n";
  }
}


std::vector<GraspScored> Scoring::scoreGrasps(const std::vector<GraspScored>& grasps_in, const geometry_msgs::Pose& current_pose)
{
	std::vector<GraspScored> grasps = grasps_in;
	
	// calculate joint limits score
	for (int i = 0; i < grasps.size(); i++)
	{
		grasps[i].score_ = calculateJointScore(grasps[i].joint_positions_);
	}
	
	// sort grasps by joint limits score (ascending)
  std::sort(grasps.begin(), grasps.end(), Scoring::compareJointScore);
  
  std::cout << "-- Grasps sorted by joint limits score --\n";
  for (int i = 0; i < grasps.size(); i++)
    std::cout << "Grasp: " << i << ", id: " << grasps[i].id_ << ", joint limits score: " << grasps[i].score_ << std::endl;
  std::cout << "----------------------------------\n";
  
	// check that there is a zero joint limits score
  if (scoring_mode_ == SCORING_MODE_APERTURE && grasps[0].score_ == 0)
	{
		// calculate hand aperture score
		std::vector<std::vector<double> > width_scores = calculateApertureScore(grasps);
		if (width_scores.size() > 1)
		{
			// sort grasps by aperture
			std::sort(width_scores.begin(), width_scores.end(), Scoring::compareSecondElement);
			
			std::cout << "-- Grasps sorted by aperture score --\n";
			for (int i = 0; i < width_scores.size(); i++)
			{
				std::cout << "Grasp: " << grasps[width_scores[i][0]].id_ << ", aperture score: " << width_scores[i][1] << "\n";
			}
			std::cout << "----------------------------------\n";
            			
			// calculate distance to current hand pose
			if (scoring_mode_ == SCORING_MODE_WORKSPACE)
      {
        std::vector<std::vector<double> > distances = calculateWorkspaceDistance(current_pose, grasps, width_scores);
						
        // select grasp based on distance to current hand pose
        if (distances.size() > 1)
        {
          std::cout << "Using workspace distance to select grasps\n";
          
          // sort grasps by distance to current hand pose
          std::sort(distances.begin(), distances.end(), Scoring::compareSecondElement);
          
          std::vector<GraspScored> grasps_out(std::min((int) distances.size(), num_selected_));
          for (int i=0; i < grasps_out.size(); i++)
          {
            grasps_out[i] = grasps[distances[i][0]];
            grasps_out[i].score_ = distances[i][1];
          }
          return grasps_out;
        }
			
        // select grasp based on distance to aperture limits
        std::cout << "Using aperture score to select grasps\n";        
        std::vector<GraspScored> grasps_out(std::min((int) width_scores.size(), num_selected_));
				for (int i=0; i < grasps_out.size(); i++)
				{
					grasps_out[i] = grasps[width_scores[i][0]];
					grasps_out[i].score_ = width_scores[i][1];
				}
				return grasps_out;
			}
		}		
	}
  
  std::cout << "Using joint limits score to select grasps\n";        
  std::vector<GraspScored> grasps_out(std::min((int) grasps.size(), num_selected_));
  for (int i=0; i < grasps_out.size(); i++)
  {
    grasps_out[i] = grasps[i];
  }

  return grasps_out;
}


double Scoring::calculateJointScore(const std::vector<double>& joint_positions)
{
	double score = 0.0;
	for (int i = 0; i < joint_limits_.cols(); i++)
	{
		const double q = joint_positions[i];
		double min = std::min(fabs(q - joint_limits_(0, i)), fabs(q - joint_limits_(1, i)));
		score += std::max(0.0, (ARM_JOINT_LIMITS_DISTANCE - min)) * std::max(0.0, (ARM_JOINT_LIMITS_DISTANCE - min));
	}
	return score;
}


std::vector<std::vector<double> > Scoring::calculateApertureScore(const std::vector<GraspScored>& grasps)
{
	std::vector<std::vector<double> > scores;
	for (int i = 0; i == 0 || (i < grasps.size() && grasps[i].score_ == grasps[i - 1].score_); i++)
	{
		std::vector<double> score(2);
		score[0] = i;
		double min = std::min(fabs(grasps[i].width_ - min_aperture_), fabs(grasps[i].width_ - max_aperture_));
		score[1] = std::max(0.0, HAND_APERTURE_LIMITS_DISTANCE - min);
		scores.push_back(score);
	}	
	return scores;
}


std::vector<std::vector<double> > Scoring::calculateWorkspaceDistance(const geometry_msgs::Pose& current_pose, 
	const std::vector<GraspScored>& grasps, const std::vector<std::vector<double> >& widths)
{
	std::vector<std::vector<double> > distances;
  distances.resize(0);
  
  // check that there is a zero aperture score
  if (widths[0][1] > 0)
  {    
    return distances;
  }
  
	for (int i = 0; i == 0 || (i < widths.size() && widths[i][1] == widths[i - 1][1]); i++)
	{
		Eigen::Vector3d x, y;
		tf::pointMsgToEigen(current_pose.position, x);
		tf::pointMsgToEigen(grasps[widths[i][0]].pose_st_.pose.position, y);
		std::vector<double> score(2);
		score[0] = widths[i][0];
		score[1] = (y - x).squaredNorm();
		distances.push_back(score);
	}
    
  std::cout << "done w/ distance scores, created " << distances.size() << " scores\n";
  
	return distances;
}


bool Scoring::compareJointScore(const GraspScored& g1, const GraspScored& g2)
{
	return (g1.score_ < g2.score_);
}


bool Scoring::compareSecondElement(const std::vector<double>& v1, const std::vector<double>& v2)
{
  return (v1[1] < v2[1]);
}
