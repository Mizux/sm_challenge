#pragma once

#include <Eigen/Core>
#include <functional>
#include <tuple>
#include <vector>

namespace sm {

using Distances = std::vector<double>;
using Parents = std::vector<int>;

std::vector<std::vector<int>> compute_adjacency_list(
    const Eigen::MatrixXd& vertices, const Eigen::MatrixXi& faces);

Eigen::VectorXd get_point(const int index, const Eigen::MatrixXd& vertices);

double distance(const int start, const int end,
                const Eigen::MatrixXd& vertices);

std::tuple<Distances, Parents> shortest_path(
    const std::vector<int>& sources, const int target,
    const Eigen::MatrixXd& vertices, const Eigen::MatrixXi& faces,
    const std::function<double(const Eigen::VectorXd&, const Eigen::VectorXd)>&
        cost);

std::tuple<Distances, Parents> shortest_path(
    const std::vector<Eigen::VectorXd>& sources, const int target,
    const Eigen::MatrixXd& vertices, const Eigen::MatrixXi& faces,
    const std::function<double(const Eigen::VectorXd&, const Eigen::VectorXd)>&
        cost);

}  // namespace sm
