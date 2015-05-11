#ifndef GRASP_SCORED_H
#define GRASP_SCORED_H

#include <Eigen/Dense>

#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>

/** GraspScored class
 *
 * \brief Grasp data structure
 * 
 * This class stores a single grasp. The grasp is described by the grasp pose, the grasp approach direction, the 
 * aperture that the robot hand needs to have to execute the grasp, the joint positions given by the Inverse 
 * Kinematics, and the score that represents how likely the grasp is to succeed.
 * 
*/
struct GraspScored
{
	int id_; ///< the grasp's index in the agile_grasp message
	geometry_msgs::PoseStamped pose_st_; ///< the grasp pose
	geometry_msgs::Vector3 approach_; ///< the grasp approach direction
	double width_; ///< the aperture required by the robot hand to execute the grasp
	std::vector<double> joint_positions_; ///< the Inverse Kinematics solution for the grasp pose
	double score_; ///< the score, represents how likely the grasp is to succeed (the lower, the likelier)

	/**
	 * \brief Default constructor.
	*/ 
	GraspScored()	{	}

	/**
	 * \brief Constructor.
	 * \param id the grasp's index in the agile_grasp message
	 * \param pose_st the grasp pose
	 * \param approach_eigen the grasp approach direction
	 * \param width the aperture required by the robot hand to execute the grasp
	 * \param joint_positions the Inverse Kinematics solution for the grasp pose
	 * \param score the score: how likely the grasp is to succeed
	*/
	GraspScored(int id, const geometry_msgs::PoseStamped& pose_st, const Eigen::Vector3d& approach_eigen, double width,
		std::vector<double> joint_positions, double score)	
		:	id_(id), pose_st_(pose_st), width_(width), joint_positions_(joint_positions), score_(score)
	{ 
		tf::vectorEigenToMsg(approach_eigen, approach_);
	}
};

#endif /* GRASP_SCORED_H */
