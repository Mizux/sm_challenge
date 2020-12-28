#pragma once

#include <igl/opengl/glfw/imgui/ImGuiMenu.h>

#include "Path.hpp"

namespace sm {

namespace iggui = igl::opengl::glfw::imgui;

class Menu : public iggui::ImGuiMenu {
 public:
  IGL_INLINE virtual void init(igl::opengl::glfw::Viewer* _viewer) override;

  IGL_INLINE virtual bool load(std::string filename) override;
  IGL_INLINE virtual bool save(std::string filename) override;

 private:
  IGL_INLINE void draw_custom_menu();
  IGL_INLINE bool update_mesh();
  IGL_INLINE bool update_path();
  IGL_INLINE void update_mesh_lines();
  IGL_INLINE void update_mesh_colors();

  // Mesh
  int mesh_id_ = -1;
  Eigen::MatrixXd mesh_vertices_;
  Eigen::MatrixXi mesh_faces_;
  Eigen::MatrixXd mesh_colors_;
  bool show_mesh_lines_ = false;
  bool use_color_distance_ = false;
  igl::ColorMapType mesh_colormap_ = igl::ColorMapType::COLOR_MAP_TYPE_JET;

  // Shortest path
  int path_source_1_ = 0;
  int path_source_2_ = 1;
  int path_target_ = 7;
  bool use_euclidean_distance_ = true;
  Distances distances_ = {};
  Parents parents_ = {};
  int path_id_ = -1;
  Eigen::MatrixXd path_vertices_;
  Eigen::MatrixXi path_edges_;
  Eigen::MatrixXd path_colors_;
  bool path_show_ = false;
};
}  // namespace sm
