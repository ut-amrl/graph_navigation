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

        Eigen::Vector2f GetIntermediateLocation(float dist_along) {
            if (dist_along > (end - start).norm())
                return end;
            
            return start + (end - start).normalized() * dist_along;
        }

        Eigen::Vector2f GetProjectedLocation(Eigen::Vector2f loc) {
            auto dir = (end - start);
            auto cur = (loc - start);

            // project cur onto dir
            auto projected = cur.dot(dir) / dir.dot(dir) * dir;
            return start + projected;
        }
    };

    std::vector<Edge> edges;

    RacingLine(navigation::GraphDomain domain) {
        for (auto e : domain.edges) {
            Edge edge = {domain.KeyToState(e.s0_id).loc, domain.KeyToState(e.s1_id).loc, e.max_speed};
            edges.push_back(edge);
        }
    }

    size_t GetClosestEdgeIndex(Eigen::Vector2f loc) {
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

    Edge GetClosestEdge(Eigen::Vector2f loc) {
        return edges[GetClosestEdgeIndex(loc)];
    }

    Eigen::Vector2f GetLookahead(Eigen::Vector2f loc, float lookahead_dist) {
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
        return target;
    }
};

}

#endif