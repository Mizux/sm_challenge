#pragma once

#include <Eigen/Core>
#include <functional>
#include <tuple>
#include <vector>

namespace sm {

using Distances = std::vector<double>;
using Parents = std::vector<int>;
using CostCallback =
    std::function<double(const Eigen::VectorXd&, const Eigen::VectorXd&)>;

std::vector<std::vector<int>> compute_adjacency_list(
    const Eigen::MatrixXd& vertices, const Eigen::MatrixXi& faces);

Eigen::VectorXd get_point(const int index, const Eigen::MatrixXd& vertices);

// Sources and Target are vertices of the Mesh.
std::tuple<Distances, Parents> shortest_path(const std::vector<int>& sources,
                                             const int target,
                                             const Eigen::MatrixXd& vertices,
                                             const Eigen::MatrixXi& faces,
                                             const CostCallback& cost,
                                             bool compute_all_distances = true);

// Sources and Target are 3D point on the Mesh surface.
std::tuple<Distances, Parents> shortest_path(
    const std::vector<Eigen::VectorXd>& sources, const Eigen::VectorXd& target,
    const Eigen::MatrixXd& vertices, const Eigen::MatrixXi& faces,
    const CostCallback& cost, bool compute_all_distances = true);

double geodesic_distance(const std::vector<Eigen::VectorXd>& sources,
                         const Eigen::VectorXd& target,
                         const Eigen::MatrixXd& vertices,
                         const Eigen::MatrixXi& faces,
                         const CostCallback& cost);

// Associate a default distance to Sources vertices.
std::tuple<Distances, Parents> shortest_path(
    const std::vector<std::tuple<int, double>>& sources, const int target,
    const Eigen::MatrixXd& vertices, const Eigen::MatrixXi& faces,
    const CostCallback& cost, bool compute_all_distances = true);

}  // namespace sm
