#ifndef SCORING_H
#define SCORING_H

#include <Eigen/Dense>

#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Pose.h>
#include <urdf/model.h>

#include <string>
#include <vector>

#include <grasp_selection/grasp_scored.h>


/** Scoring class
 *
 * \brief Score grasps based on three scoring functions
 * 
 * This class contains methods to score grasps based on three scoring functions: 
 * 1. joint limits distance,
 * 2. aperture limits distance, and
 * 3. workspace distance.
 * 
*/
class Scoring
{
	public:
		
		/**
		* \brief Constructor.
		* \param urdf the URDF model
		* \param joint_names the joint names of the robot arm
		* \param min_aperture the minimum aperture of the robot hand
		* \param max_aperture the maximum aperture of the robot hand
		* \param num_selected the number of selected grasps
		* \param scoring_mode the scoring mode
		*/
		Scoring(const urdf::Model& urdf, const std::vector<std::string>& joint_names, double min_aperture, 
			double max_aperture, int num_selected, int scoring_mode);
		
		/**
		 * \brief Assign scores to a given set of grasps.
		 * \param grasps_in the grasps to which scores are assigned
		 * \param current_pose the current pose of the robot hand
		 * \return the set of grasps with scores assigned
		*/
		std::vector<GraspScored> scoreGrasps(const std::vector<GraspScored>& grasps_in, 
			const geometry_msgs::Pose& current_pose);
		
    /** Constants for which scoring functions are used. */ 
    static const int SCORING_MODE_NONE = 0; ///< no scoring
    static const int SCORING_MODE_JOINTS = 1; ///< only use joint limits distance
    static const int SCORING_MODE_APERTURE = 2; ///< use joint limits distance and aperture limits distance
    static const int SCORING_MODE_WORKSPACE = 3; ///< use all three scoring functions
			
	private:
		
		/**
		 * \brief Calculate the joint limits distance.
		 * \param joint_positions the set of joint angles for which the limits distance is calculated
		 * \return the joint limits distance
		*/
		double calculateJointScore(const std::vector<double>& joint_positions);
		
		/**
		 * \brief Calculate the aperture limits distance.
		 * \param grasps the set of grasps for which the limits distance is calculated
		 * \return the aperture limits distance
		*/
		std::vector<std::vector<double> > calculateApertureScore(const std::vector<GraspScored>& grasps);
		
		/**
		 * \brief Calculate the workspace distance
		 * \param current_pose the current pose of the robot hand
		 * \param grasps the set of grasps for which the distance is calculated
		 * \param widths the scores assigned by the calculateApertureScore function
		 * \return the distance between the grasp pose and the current hand pose of the robot
		*/
		std::vector<std::vector<double> > calculateWorkspaceDistance(const geometry_msgs::Pose& current_pose, 
			const std::vector<GraspScored>& grasps, const std::vector<std::vector<double> >& widths);
		
		/**
		 * \brief Compare the joint scores of two grasps.
		 * \param g1 the first grasp
		 * \param g2 the second grasp
		 * \return true if the first grasp's score is lower than the second, false otherwise
		*/
		static bool compareJointScore(const GraspScored& g1, const GraspScored& g2);
		
		/**
		 * \brief Compare the second element of a two-element list for two lists of numbers.
		 * \param v1 the first list
		 * \param v2 the second list
		 * \return true if the first list's second element is lower than the second list's second element, false otherwise
		*/
		static bool compareSecondElement(const std::vector<double>& v1, const std::vector<double>& v2);
	
		Eigen::MatrixXd joint_limits_; ///< the joint limits of the robot arm (read out from the URDF)
		double min_aperture_; ///< the minimum aperture of the robot hand
		double max_aperture_; ///< the maximum aperture of the robot hand
		int num_selected_; ///< the number of selected grasps (= the top K grasps)
    int scoring_mode_; ///< the scoring mode: which scoring functions are used
		
		static const double ARM_JOINT_LIMITS_DISTANCE = 20.0 * (M_PI / 180.0); // distance from joint limits
		static const double HAND_APERTURE_LIMITS_DISTANCE = 0.015; // preferred distance from min and max gripper width   
};

#endif /* SCORING_H */ 
