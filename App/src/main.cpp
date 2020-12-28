#include <igl/opengl/glfw/Viewer.h>

#include <iostream>

#include "Menu.hpp"
#include "Path.hpp"

int main(int /*argc*/, char* /*argv*/[]) {
  igl::opengl::glfw::Viewer viewer;

  // igl::opengl::glfw::imgui::ImGuiMenu menu;
  sm::Menu menu;
  viewer.plugins.push_back(&menu);

  viewer.launch();

  // Eigen::MatrixXd line_c = Eigen::MatrixXd::Constant(vertices.rows(), 3, 1);
  // std::cout << "Path: ";
  // int current = target;
  // line_c(current, 1) = 0;
  // line_c(current, 2) = 0;
  // std::cout << std::to_string(current);
  // while (current != source) {
  //   std::cout << " <- " << std::to_string(parents[current]);
  //   current = parents[current];
  //   line_c(current, 1) = 0;
  //   line_c(current, 2) = 0;
  // }
  // std::cout << std::endl;

  // auto id = viewer.append_mesh(/*visible=*/true);
  // const Eigen::MatrixXi edges =
  //     (Eigen::MatrixXi(2, 2) << 7, 1, 1, 0).finished();
  // viewer.data(id).set_edges(vertices, edges, line_c);
  // viewer.data(id).line_width = 2.0f;
  // std::cout << std::to_string(viewer.data(id).line_width) << "\n";
}
