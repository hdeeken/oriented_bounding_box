#ifndef ORIENTED_BOUNDING_BOX_H_
#define ORIENTED_BOUNDING_BOX_H_
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include <lvr_tools/Mesh.h>
#include <lvr_tools/BoundingBox.h>
#include <oriented_bounding_box/GetOBB.h>

#include <OBB.h>
#include <Vec3.h>

using namespace std;

namespace oriented_bounding_box
{
	/**
	 * @brief  The MapServer2D interfaces between the semantic map and the robot navigation methods by turning the environments geometry and semantics into 2d grid maps used by the
	 * standard components of the navigation stack.
	 */

class OrientedBoundingBoxNode
{
private:
	ros::NodeHandle nh;

	/// internal parameters

	/// service servers
	ros::ServiceServer get_obb_srv_;

public:
	/**
	 * @brief Constructor
	 */
	OrientedBoundingBoxNode(ros::NodeHandle n);

	/**
	 * @brief Destructor
	 */
	~OrientedBoundingBoxNode() { }

	/**
	 * @brief method to extract all parameters from the parameter server
	 * @return status if setup was successfull
	 */
	bool setupParameters();

	/**
	 * @brief method to connect all services required to implement the node's functionality
	 * @return status if setup was successfull
	 */
	bool setupServices();

	/**
	 * @brief TODO service method to setup colors and scales for the markers
	 *
	 * @param request of the map_server_2d::SetMarkerConfig ROS Service, see definition for details 
	 * @param response of the map_server_2d::SetMarkerConfig ROS Service, see definition for details
	 */
	bool srvGetOBB(oriented_bounding_box::GetOBB::Request& request, oriented_bounding_box::GetOBB::Response& response);

visualization_msgs::Marker createBoxMarker(std::vector<geometry_msgs::Point> points, std::string name, std::string frame, geometry_msgs::Vector3 scale,std_msgs::ColorRGBA color);
	};

}
#endif /* ORIENTED_BOUNDING_BOX_H_ */
