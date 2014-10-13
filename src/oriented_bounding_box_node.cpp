/*
 * Studygroup MUFFIN Packages - Robot Operating System
 *
 * Copyright (C) 2013 University of Osnabrück, Germany
 * 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 * oriented_bounding_box.cpp
 *
 *  Created on: 15.12.2013
 *      Author: Henning Deeken <hdeeken@uos.de>
 */
 
#include <oriented_bounding_box_node.h>

namespace oriented_bounding_box
{
    OrientedBoundingBoxNode::OrientedBoundingBoxNode(ros::NodeHandle n)
    {
        nh = n;
        // read parameters
        if(setupParameters())
        {
            ROS_INFO("Parameter Setup succeded.");
        }
        else
        {
            ROS_ERROR("Check the yaml file!");
            nh.shutdown();
            return;
        }

        // connect required services
        if(setupServices())
        {
            ROS_INFO("Services Setup succeded.");
        }
        else
        {
            ROS_ERROR("Check the services setups");
            nh.shutdown();
            return;
        }

        // advertise services
        get_obb_srv_ = nh.advertiseService("get_obb", &OrientedBoundingBoxNode::srvGetOBB, this);

        ROS_INFO("OBB Node is running.");
    }

    //
    // SETUP
    //

    bool OrientedBoundingBoxNode::setupServices()
    {
        return true;
    }

    bool OrientedBoundingBoxNode::setupParameters()
    {
        return true;
    }

    //
    // SERVICES
    //

    bool OrientedBoundingBoxNode::srvGetOBB(oriented_bounding_box::GetOBB::Request& request,  oriented_bounding_box::GetOBB::Response& response)
    {
        Vec3 p;
        std::vector<Vec3> pnts;

        for(int i = 0; i < request.points.size(); i++)
        {
            p.x = request.points[i].x;
            p.y = request.points[i].y;
            p.z = request.points[i].z;
            pnts.push_back( p );
        }

    // define 3 OBB objects, and build one using just
    // the model points, one using the model triangles
    // and one using the convex hull of the model
    OBB obb_pnts, obb_tris, obb_hull;
    //std::cout << "got " << pnts.size() << " points" << endl;
    //obb_pnts.build_from_points( pnts );
    //obb_tris.build_from_triangles( pnts, tris );
    obb_pnts.build_from_convex_hull( pnts );

    Vec3 box[8];
    obb_pnts.get_bounding_box(box);
    vector<geometry_msgs::Point> obb_points;
    for(int i = 0; i < 8; i++)
    {
        geometry_msgs::Point point;
        point.x = box[i].x;
        point.y = box[i].y;
        point.z = box[i].z;
        std::cout << "# " << i << ": " << point.x << " " << point.y << " " << point.z << std::endl;
        obb_points.push_back( point );
    }

    std_msgs::ColorRGBA color;
    color.r =  (float) 1.0;
    color.g =  (float) 0.0;
    color.b =  (float) 0.0;
    color.a =  (float) 1.0;
    geometry_msgs::Vector3 scale;
    scale.x = 0.05;
    scale.y = 0.05;
    scale.z = 0.05;

    response.corners = obb_points;
    response.marker = createBoxMarker(obb_points, string("obb"), string("map"), scale,color);

    return true;
    }

visualization_msgs::Marker OrientedBoundingBoxNode::createBoxMarker(vector<geometry_msgs::Point> points, std::string name, std::string frame, geometry_msgs::Vector3 scale,std_msgs::ColorRGBA color)
{
    visualization_msgs::Marker marker;

    marker.header.frame_id = frame;
    marker.header.stamp = ros::Time::now();
    marker.ns = name;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_LIST;

    geometry_msgs::Pose pose;
    marker.pose =  pose;
    marker.scale = scale;
    marker.color = color;
    // all edges
    //zero
    marker.points.push_back(points[0]);
    marker.points.push_back(points[1]);
    //one
    marker.points.push_back(points[1]);
    marker.points.push_back(points[2]);
    //two
    marker.points.push_back(points[2]);
    marker.points.push_back(points[3]);
    //three
    marker.points.push_back(points[0]);
    marker.points.push_back(points[3]);
    //four
    marker.points.push_back(points[4]);
    marker.points.push_back(points[5]);
    //five
    marker.points.push_back(points[5]);
    marker.points.push_back(points[6]);
    //six
    marker.points.push_back(points[6]);
    marker.points.push_back(points[7]);
    //seven
    marker.points.push_back(points[4]);
    marker.points.push_back(points[7]);
    //eight
    marker.points.push_back(points[0]);
    marker.points.push_back(points[4]);
    //nine
    marker.points.push_back(points[1]);
    marker.points.push_back(points[5]);
    //ten
    marker.points.push_back(points[2]);
    marker.points.push_back(points[6]);
    //eleven
    marker.points.push_back(points[3]);
    marker.points.push_back(points[7]);

    return marker;
}
}

// Let the magic happen...

int main(int argc, char **argv)
{
    ros::init(argc, argv, "oriented_bounding_box");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);
    oriented_bounding_box::OrientedBoundingBoxNode node(n);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
