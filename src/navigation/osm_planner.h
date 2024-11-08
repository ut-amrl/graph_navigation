#pragma once
#include <cstdlib>
#include <exception>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>
#include <cmath>

#include "eigen3/Eigen/Geometry"

#include "osrm/osrm.hpp"
#include "osrm/coordinate.hpp"
#include "osrm/route_parameters.hpp"
#include "osrm/engine_config.hpp"
#include "osrm/status.hpp"
#include "osrm/json_container.hpp"

#include "shared/math/conversion.h"

using Eigen::Vector2d;
using std::pow;
using std::string;
using std::vector;

struct GPSPoint {
    GPSPoint() : time(0), lat(0), lon(0) {}
    GPSPoint(double time, double lat, double lon) : time(time), lat(lat), lon(lon) {}
    GPSPoint(double time, double lat, double lon, double heading) : 
        time(time), lat(lat), lon(lon), heading(heading) {}
    GPSPoint(double lat, double lon) : time(0), lat(lat), lon(lon) {}
    
    bool operator==(const GPSPoint& other) const {
        return lat == other.lat && lon == other.lon;
    }

    double time;
    double lat;
    double lon;
    double heading; // True North
};

class OSMPlanner {
public:
    OSMPlanner() = default;

    OSMPlanner(const string &osrm_file) {
        osrm::EngineConfig config;
        config.storage_config = {osrm_file};
        config.use_shared_memory = false;
        config.algorithm = osrm::EngineConfig::Algorithm::MLD;
        osrm = std::make_unique<osrm::OSRM>(config);
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

            coordinates.emplace_back(GPSPoint{lat / static_cast<double>(factor), lon / static_cast<double>(factor)});
        }
        return coordinates;
    }

    vector<GPSPoint> plan(GPSPoint start, GPSPoint end) {
        osrm::RouteParameters params;
        params.coordinates.push_back({osrm::util::FloatLongitude{start.lon}, osrm::util::FloatLatitude{start.lat}});
        params.coordinates.push_back({osrm::util::FloatLongitude{end.lon}, osrm::util::FloatLatitude{end.lat}});
        params.geometries = osrm::RouteParameters::GeometriesType::Polyline;

        osrm::engine::api::ResultT result = osrm::util::json::Object();
        const auto status = osrm->Route(params, result);
        vector<GPSPoint> path_coordinates;

        if (status == osrm::Status::Ok) {
            auto &json_result = result.get<osrm::util::json::Object>();
            auto &routes = json_result.values["routes"].get<osrm::util::json::Array>();
            auto &route = routes.values.at(0).get<osrm::util::json::Object>();
            const auto &geometry_encoded = route.values["geometry"].get<osrm::util::json::String>().value;
            path_coordinates = this->decodePolyline(geometry_encoded);
        } else {
            std::cerr << "Error: Failed to retrieve route.\n";
        }
        return path_coordinates;
    }

    bool checkGPSGoalReached(const GPSPoint& current, const GPSPoint& goal, double threshold = 3.0) {
        global_coord = gpsToGlobalCoord(current.lat, current.lon, goal.lat, goal.lon);
        // Check if the current location is within the threshold of the goal
        double distance = sqrt(pow(global_coord.first, 2) + pow(global_coord.second, 2));
        return distance < threshold;
    }

private:
    std::unique_ptr<osrm::OSRM> osrm;
};
