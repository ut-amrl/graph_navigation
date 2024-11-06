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
#include "osrm/match_parameters.hpp"
#include "osrm/nearest_parameters.hpp"
#include "osrm/route_parameters.hpp"
#include "osrm/table_parameters.hpp"
#include "osrm/trip_parameters.hpp"

#include "osrm/coordinate.hpp"
#include "osrm/engine_config.hpp"
#include "osrm/json_container.hpp"

#include "osrm/osrm.hpp"
#include "osrm/status.hpp"

#include <exception>
#include <iostream>
#include <string>
#include <utility>

#include <cstdlib>

/**
 * Examples of how to use the OSRM C++ binding.
 *  ./bin/streetmap_test osrm/texas-latest.osrm 30.287335 -97.736967 30.288778 -97.7411476
 */
int main(int argc, const char *argv[])
{
    if (argc < 6)
    {
        std::cerr << "Usage: " << argv[0] << " <data.osrm> <start_lat> <start_lon> <end_lat> <end_lon>\n";
        return EXIT_FAILURE;
    }

    using namespace osrm;

    // Configure OSRM with the .osrm base path and without shared memory
    EngineConfig config;
    config.storage_config = {argv[1]};
    config.use_shared_memory = false;
    config.algorithm = EngineConfig::Algorithm::MLD; // Multi-Level Dijkstra

    const OSRM osrm{config};

    // Convert input coordinates
    double start_lat = std::stod(argv[2]);
    double start_lon = std::stod(argv[3]);
    double end_lat = std::stod(argv[4]);
    double end_lon = std::stod(argv[5]);
    std::cout << "Start: " << start_lon << ", " << start_lat << "\n";
    std::cout << "End: " << end_lon << ", " << end_lat << "\n";

    // Set up RouteParameters for the shortest path request
    RouteParameters params;
    params.coordinates.push_back({util::FloatLongitude{start_lon}, util::FloatLatitude{start_lat}});
    params.coordinates.push_back({util::FloatLongitude{end_lon}, util::FloatLatitude{end_lat}});

    // Prepare a container for the response in JSON format
    engine::api::ResultT result = json::Object();

    // Execute routing request
    const auto status = osrm.Route(params, result);

    // Process the result
    auto &json_result = result.get<json::Object>();
    if (status == Status::Ok)
    {
        auto &routes = json_result.values["routes"].get<json::Array>();

        // Use the first route
        auto &route = routes.values.at(0).get<json::Object>();
        const auto distance = route.values["distance"].get<json::Number>().value;
        const auto duration = route.values["duration"].get<json::Number>().value;

        // Warn users if route distance or duration is zero
        if (distance == 0 || duration == 0)
        {
            std::cout << "Note: distance or duration is zero. ";
            std::cout << "You are probably doing a query outside of the OSM extract.\n\n";
        }

        std::cout << "Distance: " << distance << " meters\n";
        std::cout << "Duration: " << duration << " seconds\n";
        return EXIT_SUCCESS;
    }
    else if (status == Status::Error)
    {
        const auto code = json_result.values["code"].get<json::String>().value;
        const auto message = json_result.values["message"].get<json::String>().value;

        std::cerr << "Error Code: " << code << "\n";
        std::cerr << "Error Message: " << message << "\n";
        return EXIT_FAILURE;
    }
}