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
  Eigen::VectorXd result(vertices.cols());
  for (int i = 0; i < vertices.cols(); ++i) {
    result(i) = vertices(index, i);
  }
  return result;
}

double length(const int start, const int end, const Eigen::MatrixXd& vertices,
              const std::function<double(const Eigen::VectorXd&,
                                         const Eigen::VectorXd)>& cost) {
  assert(start < vertices.rows());
  assert(end < vertices.rows());
  if (start == end) return double{0};

  const Eigen::VectorXd s = get_point(start, vertices);
  const Eigen::VectorXd e = get_point(end, vertices);

  return cost(s, e);
}

std::tuple<Distances, Parents> shortest_path(
    const std::vector<int>& sources, const int target,
    const Eigen::MatrixXd& vertices, const Eigen::MatrixXi& faces,
    const std::function<double(const Eigen::VectorXd&, const Eigen::VectorXd)>&
        cost) {
  assert(source < vertices.rows());
  assert(target < vertices.rows());

  // Dijkstra using a priority queue.
  // ref: https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm
  auto num_nodes = vertices.rows();
  std::vector<double> distances(num_nodes, std::numeric_limits<double>::max());
  std::vector<int> parents(num_nodes, std::numeric_limits<int>::quiet_NaN());

  for (const auto src : sources) {
    distances[src] = double{0};
    parents[src] = src;
  }
  // Target is a source point
  // must be disable for question 2.5
  if (std::find(sources.begin(), sources.end(), target) != sources.end())
    return {distances, parents};

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

  for (const auto src : sources) {
    queue.push({src, double{0}});
  }
  while (!queue.empty()) {
    QueueNode node = queue.top();
    queue.pop();
    // check if we need to update neighbours.
    if (node.distance <= distances[node.index]) {
      for (const auto& i : adjacency_list[node.index]) {
        auto distance =
            distances[node.index] + length(node.index, i, vertices, cost);
        if (distance < distances[i]) {
          distances[i] = distance;
          parents[i] = node.index;
          queue.push({i, distance});
        }
      }
    }

    // uncomment to stop algorithm once target point is reach
    // but you won't have distance, path for all vertices of M.
    // if (node.index == target) {
    //  return {distances, parents};
    //}
  }
  return {distances, parents};
}

struct ClosestNode {
  int index;
  double distance;
};

// Find the closest vertex of M from list of point pts.
std::vector<ClosestNode> closest_nodes(const std::vector<Eigen::VectorXd>& pts,
                                       const Eigen::MatrixXd& vertices) {
  auto square_distance = [&](const Eigen::VectorXd& s,
                             const Eigen::VectorXd& e) -> double {
    assert(s.rows() == e.rows());
    assert(s.cols() == e.cols());
    double sum = double{0};
    for (int i = 0; i < s.rows(); ++i) {
      sum += std::pow(e(i) - s(i), double{2.0});
    }
    // don't compute the std::sqrt(sum)
    return sum;
  };

  std::vector<ClosestNode> closest_nodes(
      pts.size(), {-1, std::numeric_limits<double>::max()});
  for (int vertex_id = 0; vertex_id < vertices.rows(); ++vertex_id) {
    auto vertex = get_point(vertex_id, vertices);
    for (std::size_t i = 0; i < pts.size(); ++i) {
      double distance = square_distance(pts[i], vertex);
      if (distance < closest_nodes[i].distance) {
        closest_nodes[i].index = vertex_id;
        closest_nodes[i].distance = distance;
      }
    }
  }
  return closest_nodes;
}

std::tuple<Distances, Parents> shortest_path(
    const std::vector<Eigen::VectorXd>& sources, const Eigen::VectorXd& target,
    const Eigen::MatrixXd& vertices, const Eigen::MatrixXi& faces,
    const std::function<double(const Eigen::VectorXd&, const Eigen::VectorXd)>&
        cost) {
  auto source_vertices = closest_nodes(sources, vertices);
  ClosestNode target_vertex = closest_nodes({target}, vertices).front();

  // Dijkstra using a priority queue.
  // ref: https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm
  auto num_nodes = vertices.rows();
  std::vector<double> distances(num_nodes, std::numeric_limits<double>::max());
  std::vector<int> parents(num_nodes, std::numeric_limits<int>::quiet_NaN());

  // This time, initialize using the closest vertices
  std::set<int> start_indices;
  for (const auto& it : source_vertices) {
    // if two source points map to the same vertex only keep the closest one.
    distances[it.index] = std::min(distances[it.index], it.distance);
    parents[it.index] = it.index;
    start_indices.insert(it.index);
  }

  // Create the vertices adjacency list
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

  // Init the priority queue by adding the closest vertices.
  for (const auto index : start_indices) {
    queue.push({index, distances[index]});
  }

  while (!queue.empty()) {
    QueueNode node = queue.top();
    queue.pop();
    // check if we need to update neighbours.
    if (node.distance <= distances[node.index]) {
      for (const auto& i : adjacency_list[node.index]) {
        auto distance =
            distances[node.index] + length(node.index, i, vertices, cost);
        if (distance < distances[i]) {
          distances[i] = distance;
          parents[i] = node.index;
          queue.push({i, distance});
        }
      }
    }

    // to stop once we reach the closest vertex from the target.
    // if (node.index == target_vertex.index) {
    //  return {distances, parents};
    //}
  }

  return {distances, parents};
  // geodesic shortest distance is: "distances[target_vertex.index] +
  // target_vertex.distance"
}

}  // namespace sm
