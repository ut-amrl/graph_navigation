#pragma once
#include <cmath>
#include <cstdlib>
#include <exception>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "eigen3/Eigen/Geometry"
#include "osrm/coordinate.hpp"
#include "osrm/engine_config.hpp"
#include "osrm/json_container.hpp"
#include "osrm/osrm.hpp"
#include "osrm/route_parameters.hpp"
#include "osrm/status.hpp"
#include "shared/math/gps_util.h"

using Eigen::Vector2d;
using std::pow;
using std::string;
using std::vector;
using namespace gps_util;

class OSMPlanner {
 public:
  OSMPlanner() = default;

  OSMPlanner(const string &osrm_file, const double osrm_path_resolution) {
    osrm::EngineConfig config;
    config.storage_config = {osrm_file};
    config.use_shared_memory = false;
    config.algorithm = osrm::EngineConfig::Algorithm::MLD;
    osrm = std::make_unique<osrm::OSRM>(config);

    osrm_path_resolution_ = osrm_path_resolution;
  }

  // Utility function to decode polyline
  vector<GPSPoint> decodePolyline(const string &polyline, int precision = 5) {
    vector<GPSPoint> coordinates;
    int index = 0, lat = 0, lon = 0;
    const int factor = pow(10, precision);
    while (index < int(polyline.size())) {
      int b, shift = 0, result = 0;
      do {
        b = polyline[index++] - 63;
        result |= (b & 0x1f) << shift;
        shift += 5;
      } while (b >= 0x20);
      int dlat = ((result & 1) ? ~(result >> 1) : (result >> 1));
      lat += dlat;

      shift = 0;
      result = 0;
      do {
        b = polyline[index++] - 63;
        result |= (b & 0x1f) << shift;
        shift += 5;
      } while (b >= 0x20);
      int dlon = ((result & 1) ? ~(result >> 1) : (result >> 1));
      lon += dlon;

      coordinates.emplace_back(GPSPoint{lat / static_cast<double>(factor),
                                        lon / static_cast<double>(factor)});
    }
    return coordinates;
  }

  vector<GPSPoint> plan(GPSPoint start, GPSPoint end) {
    osrm::RouteParameters params;
    params.coordinates.push_back({osrm::util::FloatLongitude{start.lon},
                                  osrm::util::FloatLatitude{start.lat}});
    params.coordinates.push_back({osrm::util::FloatLongitude{end.lon},
                                  osrm::util::FloatLatitude{end.lat}});
    params.geometries = osrm::RouteParameters::GeometriesType::Polyline;

    osrm::engine::api::ResultT result = osrm::util::json::Object();
    const auto status = osrm->Route(params, result);
    vector<GPSPoint> path_coordinates;

    if (status == osrm::Status::Ok) {
      auto &json_result = result.get<osrm::util::json::Object>();
      auto &routes =
          json_result.values["routes"].get<osrm::util::json::Array>();
      auto &route = routes.values.at(0).get<osrm::util::json::Object>();
      const auto &geometry_encoded =
          route.values["geometry"].get<osrm::util::json::String>().value;
      path_coordinates = this->decodePolyline(geometry_encoded);

      // Interpolate additional points to achieve 20-meter spacing
      vector<GPSPoint> dense_path;

      // Ensure the first point is exactly the start location
      dense_path.push_back(start);

      double accumulated_distance = 0.0;
      for (size_t i = 1; i < path_coordinates.size(); ++i) {
        GPSPoint prev_point = path_coordinates[i - 1];
        GPSPoint curr_point = path_coordinates[i];

        // Compute the distance between previous and current points
        auto distance = gpsDistance(prev_point.lat, prev_point.lon,
                                    curr_point.lat, curr_point.lon);

        accumulated_distance += distance;

        // If accumulated distance is 20 meters or more, interpolate a point
        while (accumulated_distance >= osrm_path_resolution_) {
          double ratio =
              (osrm_path_resolution_ - (accumulated_distance - distance)) /
              distance;

          // Interpolate latitude and longitude
          double interp_lat =
              prev_point.lat + ratio * (curr_point.lat - prev_point.lat);
          double interp_lon =
              prev_point.lon + ratio * (curr_point.lon - prev_point.lon);

          dense_path.emplace_back(GPSPoint{interp_lat, interp_lon});

          accumulated_distance -= osrm_path_resolution_;
        }
      }

      // Ensure the last point is exactly the end location
      if (dense_path.back() != path_coordinates.back()) {
        dense_path.push_back(path_coordinates.back());
      }

      return dense_path;
    } else {
      std::cerr << "Error: Failed to retrieve route.\n";
    }
    return {};
  }

  // vector<GPSPoint> plan(GPSPoint start, GPSPoint end) {
  //     osrm::RouteParameters params;
  //     params.coordinates.push_back({osrm::util::FloatLongitude{start.lon},
  //                                   osrm::util::FloatLatitude{start.lat}});
  //     params.coordinates.push_back({osrm::util::FloatLongitude{end.lon},
  //                                   osrm::util::FloatLatitude{end.lat}});
  //     params.geometries = osrm::RouteParameters::GeometriesType::Polyline;

  //     osrm::engine::api::ResultT result = osrm::util::json::Object();
  //     const auto status = osrm->Route(params, result);
  //     vector<GPSPoint> path_coordinates;

  //     if (status == osrm::Status::Ok) {
  //         auto &json_result = result.get<osrm::util::json::Object>();
  //         auto &routes =
  //         json_result.values["routes"].get<osrm::util::json::Array>(); auto
  //         &route = routes.values.at(0).get<osrm::util::json::Object>(); const
  //         auto &geometry_encoded =
  //         route.values["geometry"].get<osrm::util::json::String>().value;
  //         path_coordinates = this->decodePolyline(geometry_encoded);

  //         // Interpolate additional points to achieve 20-meter spacing
  //         vector<GPSPoint> dense_path;
  //         dense_path.push_back(path_coordinates.front());

  //         double accumulated_distance = 0.0;
  //         for (size_t i = 1; i < path_coordinates.size(); ++i) {
  //             GPSPoint prev_point = path_coordinates[i - 1];
  //             GPSPoint curr_point = path_coordinates[i];

  //             // Compute the distance between previous and current points
  //             auto distance = gpsDistance(prev_point.lat, prev_point.lon,
  //             curr_point.lat, curr_point.lon);

  //             accumulated_distance += distance;

  //             // If accumulated distance is 20 meters or more, interpolate a
  //             point while (accumulated_distance >= 20.0) {
  //                 double ratio = (20.0 - (accumulated_distance - distance)) /
  //                 distance;

  //                 // Interpolate latitude and longitude
  //                 double interp_lat = prev_point.lat + ratio *
  //                 (curr_point.lat - prev_point.lat); double interp_lon =
  //                 prev_point.lon + ratio * (curr_point.lon - prev_point.lon);

  //                 dense_path.emplace_back(GPSPoint{interp_lat, interp_lon});

  //                 accumulated_distance -= 20.0;
  //             }
  //         }
  //         // Add the last point
  //         if (dense_path.back() != path_coordinates.back()) {
  //             dense_path.push_back(path_coordinates.back());
  //         }

  //         return dense_path;
  //     } else {
  //         std::cerr << "Error: Failed to retrieve route.\n";
  //     }
  //     return {};
  // }

  bool isGoalReached(const GPSPoint &current, const GPSPoint &goal,
                     double threshold = 3.0) {
    const auto &global_coord =
        gpsToGlobalCoord(current.lat, current.lon, goal.lat, goal.lon);
    // Check if the current location is within the threshold of the goal
    double distance = sqrt(pow(std::get<0>(global_coord), 2) +
                           pow(std::get<1>(global_coord), 2));
    return distance < threshold;
  }

 private:
  std::unique_ptr<osrm::OSRM> osrm;
  double osrm_path_resolution_;
};
