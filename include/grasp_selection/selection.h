#ifndef SELECTION_H
#define SELECTION_H

#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <eigen_conversions/eigen_msg.h>
#include <moveit_msgs/GetPositionIK.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <string>
#include <vector>

#include <agile_grasp/Grasp.h>
#include <agile_grasp/Grasps.h>

#include <grasp_selection/grasp_scored.h>
#include <grasp_selection/reaching.h>
#include <grasp_selection/scoring.h>

#include <grasp_selection/Grasp.h>
#include <grasp_selection/GraspList.h>
#include <grasp_selection/SelectGrasps.h>


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;


/** Selection class
 *
 * \brief Select grasps by first filtering out unreachable grasps and then scoring each remaining grasp
 * 
 * This class selects grasps by first filtering out all grasps that cannot be reached by the robot arm/hand. The 
 * remaining grasps are then scored based on three scoring functions. From those grasps, the grasps with the K highest 
 * scores are chosen. The grasp selection can be accessed with a ROS service. Also visualizes the selected grasps 
 * so that they can be viewed in Rviz.
 * 
*/
class Selection
{
	public:
		
		/**
		 * \brief Constructor.
		 * \param node the ROS node
		 * \param grasps_topic the ROS topic where the agile_grasp package publishes the detected grasps
		 * \param cloud_topic the ROS topic where the point cloud is published
		 * \param reaching_params the parameters for the reaching class
		 * \param urdf the URDF model
		 * \param joint_states_topic the ROS topic where the joint states of the robot are published
		 * \param num_selected the maximum number of selected grasps
		 * \param marker_lifetime the lifetime of visual markers in the Rviz visualization
     * \param draws_from_surface whether grasp positions are drawn on the object surface in Rviz
		*/
		Selection(ros::NodeHandle& node, const std::string& grasps_topic, const std::string& cloud_topic, 
      const Reaching::Parameters& reaching_params, const urdf::Model& urdf, const std::string& joint_states_topic, 
      int num_selected, double marker_lifetime, int scoring_mode, bool draws_from_surface);
			
		/**
		 * \brief Destructor.
		*/
		~Selection()
		{
			delete reaching_;
			delete scoring_;
		}
		
		/**
		 * \brief Run the ROS node. The ROS node waits for requests to the ROS service.
		*/ 
		void runNode();

			
	private:
		
		/**
		 * \brief The callback function for the ROS topic that contains the detected grasps.
		 * \param msg the ROS message containing the detected grasps
		*/	
		void graspsCallback(const agile_grasp::Grasps& msg);
		
		/**
		 * \brief The callback function for the ROS topic that contains the point cloud.
		 * \param msg the ROS message containing the point cloud
		*/	
		void cloudCallback(const sensor_msgs::PointCloud2& msg);
    
    /**
		 * \brief The callback function for the ROS topic that contains the joint states of the robot.
		 * \param msg the ROS message containing the robot's joint states
		*/	
    void jointStatesCallback(const sensor_msgs::JointState& msg);
    
    /**
		 * \brief Create a ROS message that contains the selected grasps.
		 * \param list the list of scored grasps
		 * \return the ROS message containing the selected grasps
		*/	
    grasp_selection::GraspList createGraspListMsg(const std::vector<GraspScored>& list);
    
    /**
     * \brief Callback for the ROS service.
     * \param request the request send to the service
     * \param response the response send back by the service
     * return true if there are no errors, false otherwise
    */ 
    bool serviceCallback(grasp_selection::SelectGrasps::Request& request, 
			grasp_selection::SelectGrasps::Response& response);
		
		/**
		 * \brief Visualize the selected grasps so that they can be viewed in Rviz.
		 * \param the list of grasps to be visualized
		*/
    void drawGrasps(const std::vector<GraspScored>& list);
    
    /**
		 * \brief Create a list of visual grasp approach direction markers.
		 * \param frame the frame in which the grasps were calculated
		 * \param center the grasp position
		 * \param approach the grasp approach direction
		 * \param id the index of the grasp on the ROS topic
		 * \param color the color of the marker (RGB)
		 * \param alpha the transparency level of the marker
		 * \param diam the diameter of the marker
		 * \return the set of visual markers
		*/  
    visualization_msgs::Marker createApproachMarker(const std::string& frame, const geometry_msgs::Point& center, 
      const geometry_msgs::Vector3& approach, int id, const double* color, double alpha, double diam) const;
    
    /**
     * \brief Create a visual marker.
     * \param frame the frame associated with the marker's pose
     * \return the visual marker
    */
    visualization_msgs::Marker createMarker(const std::string& frame) const;
    
		ros::Subscriber grasps_sub_;
		ros::Subscriber cloud_sub_;
    ros::Subscriber joint_states_sub_;
    ros::Publisher visuals_pub_;
    ros::ServiceServer service_;
		agile_grasp::Grasps grasps_;
		PointCloud::Ptr cloud_;
    std::vector<std::string> joint_names_;
    int num_joints_;
    int joint_states_start_index_;
    std::string planning_frame_;
		bool has_grasps_;
		bool has_cloud_;    
		Reaching* reaching_;
		Scoring* scoring_;    
    double marker_lifetime_;
    double hand_offset_;
    int scoring_mode_;
    bool draws_from_surface_;
};

#endif /* SELECTION_H */ 
