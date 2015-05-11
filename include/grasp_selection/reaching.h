#ifndef REACHING_H
#define REACHING_H

#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <eigen_conversions/eigen_msg.h>
#include <moveit_msgs/GetPositionIK.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>

#include <omp.h>
#include <string>
#include <vector>

#include <agile_grasp/Grasp.h>
#include <agile_grasp/Grasps.h>

#include <grasp_selection/grasp_scored.h>


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;


/** Reaching class
 *
 * \brief Filter grasp based on reachability
 * 
 * This class contains methods to filter grasp based on their reachability. The reachability depends on the robot's 
 * workspace, the Inverse Kinematics (solver), and the minimum/maximum aperture of the robot hand.
 * 
*/
class Reaching
{
	public:
	
		/**
		* \brief Data structure containing reaching parameters.
		*/
		struct Parameters
		{
			std::vector<double> workspace_; ///< the robot's workspace
			double min_aperture_; ///< the minimum aperture of the robot hand
			double max_aperture_; ///< the maximum aperture of the robot hand
			int num_additional_grasps_; ///< the number of additionally generated grasps (with a different approach direction)
			std::vector<int> axis_order_; ///< the ordering of the axes in the robot hand frame
			std::string planning_frame_; ///< the planning frame
			double hand_offset_; ///< distance between grasp position (fingertips of robot hand) and origin of hand frame
			std::string arm_link_; ///< the name of the link for motion planning, e.g., right_gripper
			std::string move_group_; ///< the name of the move group for MoveIt, e.g., right_arm
			int max_colliding_points_; ///< the maximum number of points that are allowed to be in collision
			int ik_first_joint_index_; ///< the first index of the arm joints when using the Inverse Kinematics solver
			int ik_last_joint_index_; ///< the last index of the arm joints when using the Inverse Kinematics solver
      int js_first_joint_index_; ///< the first index of the arm joints on the joint_states ROS topic
			int js_last_joint_index_; ///< the last index of the arm joints on the joint_states ROS topic
      bool is_printing_; ///< whether additional information is printed while evaluating grasps for reachability
		};
		
		/**
		* \brief Constructor.
		* \param params the parameters
		* \param node a reference to the ROS node
		*/
		Reaching(const Parameters& params, ros::NodeHandle& node);
		
		/**
		* \brief Select all reachable grasps from the set of available grasps.
		* \param grasp_in the set of available grasps
		* \return the set of reachable grasps
		*/
		std::vector<GraspScored> selectFeasibleGrasps(const agile_grasp::Grasps& grasps_in);
		
		/**
		* \brief Set the point cloud.
		* \param cloud the new point cloud
		*/
    void setPointCloud(const PointCloud::Ptr& cloud)
    {
      cloud_ = cloud;
    }
    
		
	private:
		
		/**
		* \brief Grasp data structure.
		*/
		struct GraspEigen
		{
			/**
			* \brief Default constructor.
			*/
			GraspEigen() { }
			
			/**
			* \brief Constructor. Convert an agile_grasp::Grasp message to a GraspEigen struct.
			* \param the agile_grasp message
			*/
			GraspEigen(const agile_grasp::Grasp& grasp) 
			{
				tf::vectorMsgToEigen(grasp.axis, axis_);
				tf::vectorMsgToEigen(grasp.approach, approach_);
				tf::vectorMsgToEigen(grasp.center, center_);
				tf::vectorMsgToEigen(grasp.surface_center, surface_center_);				
				
				approach_ = -1.0 * approach_; // make approach vector point away from handle centroid
				binormal_ = axis_.cross(approach_); // binormal (used as rotation axis to generate additional approach vectors)
			}
			
			Eigen::Vector3d center_; ///< the grasp position
			Eigen::Vector3d surface_center_; ///< the grasp position projected back onto the surface of the object
			Eigen::Vector3d axis_; ///< the hand axis
			Eigen::Vector3d approach_; ///< the grasp approach direction
			Eigen::Vector3d binormal_; ///< the vector orthogonal to the hand axis and the grasp approach direction
		};
	
		/**
			* \brief Check whether a given position lies within the robot's workspace.
			* \param x the x-coordinate of the position
			* \param y the y-coordinate of the position
			* \param z the z-coordinate of the position
		*/
		bool isInWorkspace(double x, double y, double z);
		
		/**
			* \brief Generate an additional grasp with a different approach direction from a given grasp.
			* \param grasp_in the original grasp
			* \param theta the angle by which the new grasp deviates from the original grasp
			* \return the new grasp
		*/
		GraspEigen rotateGrasp(const GraspEigen& grasp_in, double theta);
		
		/**
			* \brief Calculate the robot hand orientations for a given grasp.
			* \param grasp the grasp for which the robot hand orientation is calculated
			* \return the two orientations, the second one is the first one rotated by 180deg
		*/
		std::vector<tf::Quaternion> calculateHandOrientations(const GraspEigen& grasp);
		
		/**
			* \brief Reorder the columns of a given matrix.
			* \param Q the matrix whose columns are to be reordered
			* \return the new matrix with the column ordering changed
		*/
		Eigen::Matrix3d reorderHandAxes(const Eigen::Matrix3d& Q);
		
		/**
			* \brief Create a grasp pose ROS message from a given grasp, a robot hand orientation, and an approach angle.
			* \param grasp the given grasp
			* \param quat the robot hand orientation in quaternion form
			* \param theta the approach angle
			* \return the grasp pose ROS message
		*/
		geometry_msgs::PoseStamped createGraspPose(const GraspEigen& grasp, const tf::Quaternion& quat, double theta);
		
		/**
			* \brief Solve the Inverse Kinematics problem for a given pose.
			* \param pose the pose for which the Inverse Kinematics problem is solved
			* \param attempts the maximum number of attempts that the Inverse Kinematics solver can use to find a solution
			* \param timeout the maximum time that the Inverse Kinematics can spend to find a solution
			* \return the joint angles that the Inverse Kinematics solver found
		*/
		moveit_msgs::GetPositionIK::Response solveIK(const geometry_msgs::PoseStamped& pose, int attempts = 1, 
			double timeout = 0.01);
		
		/**
			* \brief Check whether a given grasp pose is collision-free.
			* \param pose_st the grasp pose that is checked for collisions
			* \param approach the grasp approach direction
			* \return true if the grasp pose is not in collision, false otherwise
		*/
		bool isCollisionFree(const geometry_msgs::PoseStamped& pose_st, const Eigen::Vector3d& approach);
		
		/**
			* \brief Extract the joint angles of the robot arm from the response of the Inverse Kinematics solver.
			* \param ik_response the response of the Inverse Kinematics solver
			* \return the set of joint angles of the robot arm
		*/
		std::vector<double> extractJointPositions(const moveit_msgs::GetPositionIK::Response& ik_response);
		
		ros::ServiceClient ik_solver_;
		PointCloud::Ptr cloud_;
	
		///< Parameters
		Parameters params_;
};

#endif /* REACHING_H */ 
