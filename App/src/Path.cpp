#include "Path.hpp"

#include <Eigen/Core>
#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <queue>
#include <set>
#include <vector>

namespace sm {

std::vector<std::vector<int>> compute_adjacency_list(
    const Eigen::MatrixXd& vertices, const Eigen::MatrixXi& faces) {
  // create adjacency list
  std::vector<std::vector<int>> result;
  result.resize(vertices.rows());

  // parse all faces
  for (int i = 0; i < faces.rows(); ++i) {
    // parse all edges
    for (long j = 0; j < faces.cols(); ++j) {
      int start = faces(i, j);
      assert(start < vertices.rows());
      int end = faces(i, (j + 1) % faces.cols());
      assert(end < vertices.rows());
      result[start].push_back(end);
      result[end].push_back(start);
    }
  }
  // remove duplicates
  for (auto& it : result) {
    std::sort(it.begin(), it.end());
    it.erase(std::unique(it.begin(), it.end()), it.end());
  }
  return result;
}

Eigen::VectorXd get_point(const int index, const Eigen::MatrixXd& vertices) {
  assert(index < vertices.rows());
  Eigen::VectorXd vector(vertices.cols());
  for (int i = 0; i < vertices.cols(); ++i) {
    vector(i) = vertices(index, i);
  }
  return vector;
}

double edge_length(const int start, const int end,
                   const Eigen::MatrixXd& vertices, const CostCallback& cost) {
  assert(start < vertices.rows());
  assert(end < vertices.rows());
  if (start == end) return double{0};

  const Eigen::VectorXd s = get_point(start, vertices);
  const Eigen::VectorXd e = get_point(end, vertices);

  return cost(s, e);
}

std::tuple<Distances, Parents> shortest_path(const std::vector<int>& sources,
                                             const int target,
                                             const Eigen::MatrixXd& vertices,
                                             const Eigen::MatrixXi& faces,
                                             const CostCallback& cost,
                                             bool compute_all_distances) {
  std::vector<std::tuple<int, double>> src_dst;
  src_dst.reserve(sources.size());

  for (const auto it : sources) {
    assert(it < vertices.rows());
    src_dst.emplace_back(it, 0.);
  }
  assert(target < vertices.rows());

  return shortest_path(src_dst, target, vertices, faces, cost,
                       compute_all_distances);
}

struct ClosestNode {
  int index;
  double distance;
};

// Find the closest vertex of M for each point in the list and return its index
// and distance.
std::vector<std::tuple<int, double>> closest_vertices(
    const std::vector<Eigen::VectorXd>& pts, const Eigen::MatrixXd& vertices) {
  auto square_distance = [&](const Eigen::VectorXd& s,
                             const Eigen::VectorXd& e) -> double {
    assert(s.rows() == e.rows());
    assert(s.cols() == e.cols());
    double sum = double{0};
    for (int i = 0; i < s.rows(); ++i) {
      sum += std::pow(e(i) - s(i), double{2.0});
    }
    // sqrt is not needed to find the closest point
    return sum;
  };

  std::vector<std::tuple<int, double>> closest_nodes(
      pts.size(), {-1, std::numeric_limits<double>::max()});
  for (int vertex_id = 0; vertex_id < vertices.rows(); ++vertex_id) {
    auto vertex = get_point(vertex_id, vertices);
    for (std::size_t i = 0; i < pts.size(); ++i) {
      double distance = square_distance(pts[i], vertex);
      if (distance < std::get<1>(closest_nodes[i])) {
        std::get<0>(closest_nodes[i]) = vertex_id;
        std::get<1>(closest_nodes[i]) = distance;
      }
    }
  }

  // Only compute std::sqrt for final points
  for (auto& it : closest_nodes) {
    std::get<1>(it) = std::sqrt(std::get<1>(it));
  }
  return closest_nodes;
}

std::tuple<Distances, Parents> shortest_path(
    const std::vector<Eigen::VectorXd>& sources, const Eigen::VectorXd& target,
    const Eigen::MatrixXd& vertices, const Eigen::MatrixXi& faces,
    const CostCallback& cost, bool compute_all_distances) {
  const auto source_vertices = closest_vertices(sources, vertices);
  const auto target_vertex = closest_vertices({target}, vertices).front();

  return shortest_path(source_vertices, std::get<0>(target_vertex), vertices,
                       faces, cost, compute_all_distances);
}

double geodesic_distance(const std::vector<Eigen::VectorXd>& sources,
                         const Eigen::VectorXd& target,
                         const Eigen::MatrixXd& vertices,
                         const Eigen::MatrixXi& faces,
                         const CostCallback& cost) {
  // Get distance from source point to the target closest vertex of M.
  const auto distances = shortest_path(sources, target, vertices, faces, cost,
                                       /*compute_all_distances=*/false);

  // Get distance from target to closest vertex of M using euclidean distance.
  const auto target_vertex = closest_vertices({target}, vertices).front();

  return std::get<0>(distances)[std::get<0>(target_vertex)] +
         std::get<1>(target_vertex);
}

std::tuple<Distances, Parents> shortest_path(
    const std::vector<std::tuple<int, double>>& sources, const int target,
    const Eigen::MatrixXd& vertices, const Eigen::MatrixXi& faces,
    const CostCallback& cost, bool compute_all_distances) {
  assert(target < vertices.rows());

  // Dijkstra using a priority queue.
  // ref: https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm
  auto num_nodes = vertices.rows();
  std::vector<double> distances(num_nodes, std::numeric_limits<double>::max());
  std::vector<int> parents(num_nodes, std::numeric_limits<int>::quiet_NaN());

  for (const auto& [index, dst] : sources) {
    assert(index < vertices.rows());
    // we may have duplicate sources index, i.e. same vertex but with different
    // distance.
    distances[index] = std::min(distances[index], dst);
    parents[index] = index;
  }

  // Check if Target is a source point and we don't want to compute all
  // distances.
  if (!compute_all_distances) {
    auto comp = [=](const std::tuple<int, double>& a) -> bool {
      return std::get<0>(a) == target;
    };
    if (std::find_if(sources.begin(), sources.end(), comp) != sources.end())
      return {distances, parents};
  }

  // Create adjacency list
  auto adjacency_list = compute_adjacency_list(vertices, faces);
  // Create the priority queue
  struct QueueNode {
    int index;
    double distance;
  };
  auto comp = [](const QueueNode& a, const QueueNode& b) {
    return a.distance > b.distance;
  };
  std::priority_queue<QueueNode, std::vector<QueueNode>, decltype(comp)> queue(
      comp);

  for (const auto& [index, dst] : sources) {
    queue.push({index, dst});
  }
  while (!queue.empty()) {
    const auto [index, distance] = queue.top();
    queue.pop();
    // check if we need to update neighbours.
    if (distance <= distances[index]) {
      for (const auto& adj_index : adjacency_list[index]) {
        auto adj_distance =
            distances[index] + edge_length(index, adj_index, vertices, cost);
        if (adj_distance < distances[adj_index]) {
          distances[adj_index] = adj_distance;
          parents[adj_index] = index;
          queue.push({adj_index, adj_distance});
        }
      }
    }
    // Stop algorithm once target point is fetch
    // but you won't have all [distance, parent] for all vertices of M.
    if ((!compute_all_distances) && (index == target)) {
      return {distances, parents};
    }
  }
  return {distances, parents};
}

}  // namespace sm
