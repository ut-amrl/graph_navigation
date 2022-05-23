#include "eigen3/Eigen/Dense"

#include "math/geometry.h"
#include "math/line2d.h"

#include "graph_domain.h"

#ifndef RACING_LINE_H
#define RACING_LINE_H

namespace navigation {

struct RacingLine {
  struct Edge {
    Eigen::Vector2f start;
    Eigen::Vector2f end;
    float max_speed;
    float ang_vel_factor;
    float accel_factor;
    float decel_factor;

    Eigen::Vector2f GetIntermediateLocation(const float dist_along) const {
      if (dist_along > (end - start).norm())
        return end;
      
      return start + (end - start).normalized() * dist_along;
    }

    Eigen::Vector2f GetProjectedLocation(const Eigen::Vector2f loc) const {
      auto dir = (end - start);
      auto cur = (loc - start);

      // project cur onto dir
      auto projected = cur.dot(dir) / dir.dot(dir) * dir;
      return start + projected;
    }
  };

  std::vector<Edge> edges;
  bool loaded = false;

  // RacingLine(navigation::GraphDomain domain) {
  //     for (auto e : domain.edges) {
  //         Edge edge = {domain.KeyToState(e.s0_id).loc, domain.KeyToState(e.s1_id).loc, e.max_speed};
  //         edges.push_back(edge);
  //     }
  // }
  bool Load(const std::string& file) {
    printf("Loading racing line from %s...\n", file.c_str());
    std::ifstream f(file);
    edges.clear();

    std::vector<Eigen::Vector2f> nodes;
    std::vector<float> max_speeds, ang_vel_factors, accel_factors, decel_factors;
    while (f) {
      float x, y, max_speed, ang_vel_factor, accel_factor, decel_factor;
      f >> x >> y >> max_speed >> ang_vel_factor >> accel_factor >> decel_factor;
      nodes.push_back({x, y});
      max_speeds.push_back(max_speed);
      ang_vel_factors.push_back(ang_vel_factor);
      accel_factors.push_back(accel_factor);
      decel_factors.push_back(decel_factor);
    }

    if (nodes.size() < 2) {
      printf("Failed to read enough nodes from racing line file!\n");
      return false;
    }

    for (size_t i = 0; i < nodes.size(); i++) {
      edges.push_back({nodes[i], nodes[i+1], max_speeds[i], ang_vel_factors[i], accel_factors[i], decel_factors[i]});
    }
    edges.push_back({nodes[nodes.size()-1], nodes[0], max_speeds[nodes.size()-1], ang_vel_factors[nodes.size()-1], accel_factors[nodes.size()-1], decel_factors[nodes.size()-1]});

    loaded = true;
    return true;
  }

  size_t GetClosestEdgeIndex(const Eigen::Vector2f loc) const {
    size_t closest = 0;
    float closest_dist = FLT_MAX;
    for (size_t i = 0; i < edges.size(); i++) {
      auto e = edges[i];
      geometry::Line2f line {e.start, e.end};
      float dist = line.Distance(loc);
      if (dist < closest_dist) {
        closest_dist = dist;
        closest = i;
      }
    }

    return closest;
  }

  Edge GetClosestEdge(const Eigen::Vector2f loc) const {
    return edges[GetClosestEdgeIndex(loc)];
  }

  Eigen::Vector2f GetLookahead(const Eigen::Vector2f loc,
                               const float lookahead_dist,
                               float& max_speed,
                               float& ang_vel_factor,
                               float& accel_factor,
                               float& decel_factor) const {
    if (edges.size() == 0)
      return loc;
    
    auto i = GetClosestEdgeIndex(loc);
    auto e = edges[i];
    auto proj = e.GetProjectedLocation(loc);

    Eigen::Vector2f target_point;
    float dist_along;
    if ((e.end - proj).norm() < lookahead_dist) {
      dist_along = lookahead_dist - (e.end - proj).norm();
      e = edges[(i+1) % edges.size()];
    } else {
      dist_along = (proj - e.start).norm() + lookahead_dist;
    }
    
    // TODO: figure out bug with larger lookaheads sometimes returning 
    // endpoint of edge even when they should return point on next edge
    auto target = e.GetIntermediateLocation(dist_along);
    max_speed = e.max_speed;
    ang_vel_factor = e.ang_vel_factor;
    accel_factor = e.accel_factor;
    decel_factor = e.decel_factor;
    return target;
  }
};

}

#endif