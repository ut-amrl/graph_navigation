#ifndef ROS_VISUALIZATION_H
#define ROS_VISUALIZATION_H

#ifdef ROS1
  #include <ros/ros.h>
  #include <visualization_msgs/Marker.h>
  #include <visualization_msgs/MarkerArray.h>
  #include <geometry_msgs/Point.h>
  #include "amrl_msgs/VisualizationMsg.h"
  #include <foxglove_msgs/GeoJSON.h>
  using Marker = visualization_msgs::Marker;
  using MarkerArray = visualization_msgs::MarkerArray;
  using Point = geometry_msgs::Point;
  using VisualizationMsg = amrl_msgs::VisualizationMsg;
  using FoxgloveGeoJSON = foxglove_msgs::GeoJSON;
#else
  #include <rclcpp/rclcpp.hpp>
  #include <visualization_msgs/msg/marker.hpp>
  #include <visualization_msgs/msg/marker_array.hpp>
  #include <geometry_msgs/msg/point.hpp>
  #include "amrl_msgs/msg/visualization_msg.hpp"
  #include "foxglove_msgs/msg/geo_json.hpp"
  using Marker = visualization_msgs::msg::Marker;
  using MarkerArray = visualization_msgs::msg::MarkerArray;
  using Point = geometry_msgs::msg::Point;
  using VisualizationMsg = amrl_msgs::msg::VisualizationMsg;
  using FoxgloveGeoJSON = foxglove_msgs::msg::GeoJSON;
#endif

#include <vector>
#include <string>
#include <cmath>
#include "navigation/navigation_types.h"
#include "shared/math/gps_util.h"
#include <type_traits>

// JSON support for GeoJSON (using nlohmann::json)
#include "navigation/json.hpp"

using gps_util::GPSPoint;
using json = nlohmann::json;
using navigation::PathOption;
using navigation::CarrotPlan;

namespace ros_visualization {

template <typename PublisherT, typename MessageT>
inline void PublishMessage(const PublisherT& pub, const MessageT& message) {
    #ifdef ROS1
    pub.publish(message);
    #else
    pub->publish(message);
    #endif
}

// This function converts a vector of PathOption to a MarkerArray and publishes it.
// The publisher parameter is of type MarkerArrayPublisher (which is defined differently for ROS1 and ROS2).
template <typename PublisherT>
void PathOptionToMarkerArray(
    PublisherT marker_array_pub,
    const std::string& frame_id,
    const std::vector<PathOption>& path_options,
    const std::vector<std::vector<float>>& colors,
    bool show_clearance = false)
{
    MarkerArray marker_array;

    for (size_t i = 0; i < path_options.size(); ++i) {
        const auto& path_option = path_options[i];

        // Create primary path marker.
        Marker path_marker;
        path_marker.header.frame_id = frame_id;
        path_marker.header.stamp = GET_TIME();
        path_marker.ns = "path_options";
        path_marker.id = static_cast<int>(i * 3); // unique ID
        path_marker.type = Marker::LINE_STRIP;
        path_marker.action = Marker::ADD;
        path_marker.scale.x = 0.05; // line thickness
        path_marker.color.r = colors[i][0];
        path_marker.color.g = colors[i][1];
        path_marker.color.b = colors[i][2];
        path_marker.color.a = colors[i][3];

        const int num_points = 100;
        float radius = (std::fabs(path_option.curvature) > 1e-6f)
                           ? (1.0f / path_option.curvature)
                           : std::numeric_limits<float>::infinity();
        float arc_angle = path_option.curvature * path_option.free_path_length;

        for (int j = 0; j <= num_points; ++j) {
            float theta = (arc_angle / num_points) * j;
            Point p;
            if (std::fabs(path_option.curvature) < 1e-6f) {
                p.x = (path_option.free_path_length / num_points) * j;
                p.y = 0.0;
            } else {
                p.x = radius * std::sin(theta);
                p.y = radius * (1 - std::cos(theta));
            }
            p.z = 0.0;
            path_marker.points.push_back(p);
        }
        marker_array.markers.push_back(path_marker);

        if (show_clearance) {
            // Draw clearance arcs.
            Marker clearance_inner_marker = path_marker;
            Marker clearance_outer_marker = path_marker;
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
                Point p_inner, p_outer;
                if (std::fabs(path_option.curvature) < 1e-6f) {
                    p_inner.x = (path_option.free_path_length / num_points) * j;
                    p_inner.y = -path_option.clearance;
                    p_outer.x = (path_option.free_path_length / num_points) * j;
                    p_outer.y = path_option.clearance;
                } else {
                    p_inner.x = (radius - path_option.clearance) * std::sin(theta);
                    p_inner.y = (radius - path_option.clearance) * (1 - std::cos(theta));
                    p_outer.x = (radius + path_option.clearance) * std::sin(theta);
                    p_outer.y = (radius + path_option.clearance) * (1 - std::cos(theta));
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
    PublishMessage(marker_array_pub, marker_array);
}

template <typename PublisherT>
void GPSRouteToGeoJSON(PublisherT geojson_pub, const std::vector<GPSPoint>& route) {
    FoxgloveGeoJSON geojson_msg;
    json geojson;
    geojson["type"] = "FeatureCollection";
    geojson["features"] = json::array();
    json feature;
    feature["type"] = "Feature";
    feature["properties"] = {
        {"name", "Route"},
        {"style", {
            {"color", "#FF0000"},
            {"weight", 5},
            {"opacity", 1.0}
        }}
    };
    feature["geometry"] = {
        {"type", "LineString"},
        {"coordinates", json::array()}
    };
    for (const auto& point : route) {
        feature["geometry"]["coordinates"].push_back({point.lon, point.lat});
    }
    geojson["features"].push_back(feature);
    geojson_msg.geojson = geojson.dump();
    PublishMessage(geojson_pub, geojson_msg);
}

template <typename PublisherT>
void CarrotPlanToMarkerArray(PublisherT marker_array_pub,
                             const std::string& frame_id,
                             const CarrotPlan& carrot_plan) {
  MarkerArray marker_array;
  Marker line_strip;
  line_strip.header.frame_id = frame_id;
  line_strip.header.stamp = GET_TIME();
  line_strip.ns = "carrot_path";
  line_strip.id = 0;
  line_strip.type = Marker::LINE_STRIP;
  line_strip.action = Marker::ADD;
  line_strip.lifetime = DURATION(0.0);
  line_strip.scale.x = 0.05;
  // Set line color (cyan)
  line_strip.color.r = 0.0f;
  line_strip.color.g = 0.80f;
  line_strip.color.b = 0.83f;
  line_strip.color.a = 1.0f;
  for (const auto& path_point : carrot_plan.path) {
    Point p;
    p.x = static_cast<double>(path_point.x());
    p.y = static_cast<double>(path_point.y());
    p.z = 0.0;
    line_strip.points.push_back(p);
  }
  marker_array.markers.push_back(line_strip);
  PublishMessage(marker_array_pub, marker_array);
}

} // namespace ros_visualization

#endif // ROS_VISUALIZATION_H
