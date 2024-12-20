#ifndef ROS_VISUALIZATION_H
#define ROS_VISUALIZATION_H

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include <string>
#include <cmath>
#include "navigation/navigation_types.h"
#include "shared/math/gps_util.h"

// json support for geojson
#include <foxglove_msgs/GeoJSON.h>
#include "navigation/json.hpp"

using gps_util::GPSPoint;
using json = nlohmann::json;
using navigation::PathOption;

namespace ros_visualization {

void PathOptionToMarkerArray(
    ros::Publisher& marker_array_pub,
    const std::string& frame_id,
    const std::vector<PathOption>& path_options,
    const std::vector<std::vector<float> >& colors,
    bool show_clearance = false) {

    visualization_msgs::MarkerArray marker_array;

    for (size_t i = 0; i < path_options.size(); ++i) {
        const auto& path_option = path_options[i];

        // Create primary path marker
        visualization_msgs::Marker path_marker;
        path_marker.header.frame_id = frame_id;
        path_marker.header.stamp = ros::Time::now();
        path_marker.ns = "path_options";
        path_marker.id = static_cast<int>(i * 3); // Unique ID
        path_marker.type = visualization_msgs::Marker::LINE_STRIP;
        path_marker.action = visualization_msgs::Marker::ADD;
        path_marker.scale.x = 0.05; // Line thickness
        path_marker.color.r = colors[i][0];
        path_marker.color.g = colors[i][1];
        path_marker.color.b = colors[i][2];
        path_marker.color.a = colors[i][3];

        const int num_points = 100;
        float radius = (fabs(path_option.curvature) > 1e-6) ? (1.0f / path_option.curvature) : std::numeric_limits<float>::infinity();
        float arc_angle = path_option.curvature * path_option.free_path_length;

        for (int j = 0; j <= num_points; ++j) {
            float theta = (arc_angle / num_points) * j;
            geometry_msgs::Point p;
            if (fabs(path_option.curvature) < 1e-6) {
                p.x = (path_option.free_path_length / num_points) * j;
                p.y = 0.0;
            } else {
                p.x = radius * sin(theta);
                p.y = radius * (1 - cos(theta));
            }
            p.z = 0.0;
            path_marker.points.push_back(p);
        }

        marker_array.markers.push_back(path_marker);

        if (show_clearance) {
            // Draw clearance arcs
            visualization_msgs::Marker clearance_inner_marker = path_marker;
            visualization_msgs::Marker clearance_outer_marker = path_marker;

            clearance_inner_marker.id = static_cast<int>(i * 3 + 1);
            clearance_outer_marker.id = static_cast<int>(i * 3 + 2);

            clearance_inner_marker.color.r = 0.5;
            clearance_inner_marker.color.g = 0.5;
            clearance_inner_marker.color.b = 0.5;
            clearance_inner_marker.color.a = 0.5;

            clearance_outer_marker.color.r = 0.5;
            clearance_outer_marker.color.g = 0.5;
            clearance_outer_marker.color.b = 0.5;
            clearance_outer_marker.color.a = 0.5;

            clearance_inner_marker.points.clear();
            clearance_outer_marker.points.clear();

            for (int j = 0; j <= num_points; ++j) {
                float theta = (arc_angle / num_points) * j;
                geometry_msgs::Point p_inner, p_outer;
                if (fabs(path_option.curvature) < 1e-6) {
                    p_inner.x = (path_option.free_path_length / num_points) * j;
                    p_inner.y = -path_option.clearance;
                    p_outer.x = (path_option.free_path_length / num_points) * j;
                    p_outer.y = path_option.clearance;
                } else {
                    p_inner.x = (radius - path_option.clearance) * sin(theta);
                    p_inner.y = (radius - path_option.clearance) * (1 - cos(theta));
                    p_outer.x = (radius + path_option.clearance) * sin(theta);
                    p_outer.y = (radius + path_option.clearance) * (1 - cos(theta));
                }
                p_inner.z = 0.0;
                p_outer.z = 0.0;
                clearance_inner_marker.points.push_back(p_inner);
                clearance_outer_marker.points.push_back(p_outer);
            }

            marker_array.markers.push_back(clearance_inner_marker);
            marker_array.markers.push_back(clearance_outer_marker);
        }
    }

    marker_array_pub.publish(marker_array);
}

void GPSRouteToGeoJSON(ros::Publisher& geojson_pub, const std::vector<GPSPoint>& route) {
    // Create GeoJSON message
    foxglove_msgs::GeoJSON geojson_msg;

    // Create the GeoJSON object
    json geojson;
    geojson["type"] = "FeatureCollection";
    geojson["features"] = json::array();

    // Create a single LineString feature
    json feature;
    feature["type"] = "Feature";

    // Add properties (customizable as needed)
    feature["properties"] = {
        {"name", "Route"},
        {"style", {
            {"color", "#FF0000"},  // Green color
            {"weight", 5},         // Line thickness
            {"opacity", 1.0}       // Line opacity
        }}
    };

    // Add geometry for LineString
    feature["geometry"] = {
        {"type", "LineString"},
        {"coordinates", json::array()}
    };

    // Populate coordinates array with route points
    for (const auto& point : route) {
        feature["geometry"]["coordinates"].push_back({point.lon, point.lat});
    }

    // Add the LineString feature to the GeoJSON
    geojson["features"].push_back(feature);

    // Serialize GeoJSON to string
    geojson_msg.geojson = geojson.dump();

    // Publish the GeoJSON message
    geojson_pub.publish(geojson_msg);
}

}  // namespace ros_visualization

#endif  // ROS_VISUALIZATION_H
