//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    streetmap_planner.h
\brief   Interface for reference StreetMapPlanner class.
\author  Arthur K. Zhang (C) 2024
*/
//========================================================================
#include <cstdlib>
#include <exception>
#include <iomanip>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include "osrm/coordinate.hpp"
#include "osrm/engine_config.hpp"
#include "osrm/json_container.hpp"
#include "osrm/match_parameters.hpp"
#include "osrm/nearest_parameters.hpp"
#include "osrm/osrm.hpp"
#include "osrm/route_parameters.hpp"
#include "osrm/status.hpp"
#include "osrm/table_parameters.hpp"
#include "osrm/trip_parameters.hpp"

// Polyline decoding function
std::vector<std::pair<double, double>> decodePolyline(
    const std::string &polyline, int precision = 5) {
  std::vector<std::pair<double, double>> coordinates;
  int index = 0, lat = 0, lon = 0;
  const int factor = std::pow(10, precision);  while (index < int(polyline.size())) {
    int b, shift = 0, result = 0;
    do {
      b = polyline[index++] - 63;
      result |= (b & 0x1f) << shift;
      shift += 5;
    } while (b >= 0x20);
    int dlat = ((result & 1) ? ~(result >> 1) : (result >> 1));
    lat += dlat;    shift = 0;
    result = 0;
    do {
      b = polyline[index++] - 63;
      result |= (b & 0x1f) << shift;
      shift += 5;
    } while (b >= 0x20);
    int dlon = ((result & 1) ? ~(result >> 1) : (result >> 1));
    lon += dlon;    coordinates.emplace_back(lat / static_cast<double>(factor),
                             lon / static_cast<double>(factor));
  }  return coordinates;
}

/**
 * Examples of how to use the OSRM C++ binding.
 *  ./bin/streetmap_test osrm-profile/texas-latest.osrm 30.287189
 * -97.737154 30.288778 -97.7411476
 */
int main(int argc, const char *argv[]) {
  if (argc < 6) {
    std::cerr << "Usage: " << argv[0]
              << " <data.osrm> <start_lat> <start_lon> <end_lat> <end_lon>\n";
    return EXIT_FAILURE;
  }

  using namespace osrm;

  // Configure OSRM with the .osrm base path and without shared memory
  EngineConfig config;
  config.storage_config = {argv[1]};
  config.use_shared_memory = false;
  config.algorithm = EngineConfig::Algorithm::MLD;  // Multi-Level Dijkstra

  const OSRM osrm{config};

  // Convert input coordinates
  double start_lat = std::stod(argv[2]);
  double start_lon = std::stod(argv[3]);
  double end_lat = std::stod(argv[4]);
  double end_lon = std::stod(argv[5]);
  std::cout << "Start: " << std::fixed << std::setprecision(8) << start_lat
            << ", " << start_lon << "\n";
  std::cout << "End: " << std::fixed << std::setprecision(8) << end_lat << ", "
            << end_lon << "\n";

  // Set up RouteParameters for the shortest path request
  RouteParameters params;
  params.coordinates.push_back(
      {util::FloatLongitude{start_lon}, util::FloatLatitude{start_lat}});
  params.coordinates.push_back(
      {util::FloatLongitude{end_lon}, util::FloatLatitude{end_lat}});
  // params.steps = true;  // Get steps for the route
  // params.overview = RouteParameters::OverviewType::Full;  // Get the full route
  params.geometries = RouteParameters::GeometriesType::Polyline;
  // Prepare a container for the response in JSON format
  engine::api::ResultT result = json::Object();

  // Execute routing request
  const auto status = osrm.Route(params, result);

  // Process the result
  auto &json_result = result.get<json::Object>();
  if (status == Status::Ok) {
    auto &routes = json_result.values["routes"].get<json::Array>();

    // print number of routes
    std::cout << "Number of routes: " << routes.values.size() << "\n";
    // Use the first route
    auto &route = routes.values.at(0).get<json::Object>();

    // Print the route nodes
    std::cout << "Route: ";

    const auto distance = route.values["distance"].get<json::Number>().value;
    const auto duration = route.values["duration"].get<json::Number>().value;
    const auto &geometry_encoded =
        route.values["geometry"].get<json::String>().value;

    // Warn users if route distance or duration is zero
    if (distance == 0 || duration == 0) {
      std::cout << "Note: distance or duration is zero. ";
      std::cout
          << "You are probably doing a query outside of the OSM extract.\n\n";
    }

    // Decode the polyline geometry to get coordinates
    auto coordinates = decodePolyline(geometry_encoded);
    // Print each coordinate with high precision
    for (const auto &coord : coordinates) {
      std::cout << "Lat: " << coord.first << ", Lon: " << coord.second << "\n";
    }

    std::cout << "Distance: " << distance << " meters\n";
    std::cout << "Duration: " << duration << " seconds\n";
    return EXIT_SUCCESS;
  } else if (status == Status::Error) {
    const auto code = json_result.values["code"].get<json::String>().value;
    const auto message =
        json_result.values["message"].get<json::String>().value;

    std::cerr << "Error Code: " << code << "\n";
    std::cerr << "Error Message: " << message << "\n";
    return EXIT_FAILURE;
  }
}