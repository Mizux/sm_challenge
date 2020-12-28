#include "Menu.hpp"

#include <igl/adjacency_list.h>
#include <igl/dijkstra.h>
#include <igl/file_dialog_open.h>
#include <igl/file_dialog_save.h>
#include <igl/opengl/MeshGL.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <igl/readPLY.h>
#include <igl/writePLY.h>
#include <imgui/imgui.h>

#include <functional>

namespace sm {

IGL_INLINE void Menu::init(igl::opengl::glfw::Viewer* _viewer) {
  ImGuiMenu::init(_viewer);
  callback_draw_viewer_menu = std::bind(&Menu::draw_custom_menu, this);

  // Inline mesh of a cube
  // clang-format off
  mesh_vertices_ = (Eigen::MatrixXd(8, 3) <<
    0.0, 0.0, 0.0,
    0.0, 0.0, 1.0,
    0.0, 1.0, 0.0,
    0.0, 1.0, 1.0,
    1.0, 0.0, 0.0,
    1.0, 0.0, 1.0,
    1.0, 1.0, 0.0,
    1.0, 1.0, 1.0).finished();
  mesh_faces_ = (Eigen::MatrixXi(12, 3) <<
    1, 7, 5,
    1, 3, 7,
    1, 4, 3,
    1, 2, 4,
    3, 8, 7,
    3, 4, 8,
    5, 7, 8,
    5, 8, 6,
    1, 5, 6,
    1, 6, 2,
    2, 6, 8,
    2, 8, 4).finished().array()-1;
  mesh_colors_ = Eigen::MatrixXd::Constant(mesh_vertices_.rows(), 3, 1.0);
  mesh_id_ = viewer->append_mesh(/*visible=*/true);
  // clang-format on

  path_id_ = viewer->append_mesh(/*visible=*/true);

  update_mesh();
}

IGL_INLINE bool Menu::update_mesh() {
  viewer->data(mesh_id_).clear();
  viewer->data(mesh_id_).set_mesh(mesh_vertices_, mesh_faces_);
  update_mesh_lines();
  update_mesh_colors();
  viewer->data(mesh_id_).dirty = igl::opengl::MeshGL::DIRTY_ALL;

  return update_path();
}

IGL_INLINE void Menu::update_mesh_lines() {
  viewer->data(mesh_id_).show_lines = show_mesh_lines_;
  viewer->data(mesh_id_).dirty = igl::opengl::MeshGL::DIRTY_ALL;
}

IGL_INLINE void Menu::update_mesh_colors() {
  if ((!use_color_distance_) || distances_.size() != mesh_vertices_.rows()) {
    mesh_colors_ = Eigen::MatrixXd::Constant(1, 3, 1.0);
  } else {
    // find the longest distance
    double max_dst = 0.0;
    for (const auto it : distances_) {
      max_dst = std::max(max_dst, it);
    }
    std::cout << "Max distance: " << std::to_string(max_dst) << std::endl;
    // colorize each vertices
    mesh_colors_ = Eigen::MatrixXd(mesh_vertices_.rows(), 3);
    for (long i = 0; i < mesh_vertices_.rows(); ++i) {
      igl::colormap(mesh_colormap_, distances_[i] / std::max(1.0, max_dst),
                    mesh_colors_(i, 0), mesh_colors_(i, 1), mesh_colors_(i, 2));
    }
  }
  viewer->data(mesh_id_).set_colors(mesh_colors_);
  viewer->data(mesh_id_).dirty = igl::opengl::MeshGL::DIRTY_ALL;
}

IGL_INLINE bool Menu::update_path() {
  viewer->data(path_id_).clear();
  // reset source/target if outside the mesh vertices
  if (path_source_1_ >= mesh_vertices_.rows()) path_source_1_ = 0;
  if (path_source_2_ >= mesh_vertices_.rows()) path_source_2_ = 0;
  if (path_target_ >= mesh_vertices_.rows()) path_target_ = 0;

  // compute the distance field
  if (use_euclidean_distance_) {
    auto euclidean_distance = [&](const Eigen::VectorXd& s,
                                  const Eigen::VectorXd& e) -> double {
      assert(s.rows() == e.rows());
      assert(s.cols() == e.cols());
      double sum = double{0};
      for (int i = 0; i < s.rows(); ++i) {
        sum += std::pow(e(i) - s(i), double{2.0});
      }
      return std::sqrt(sum);
    };
    std::tie(distances_, parents_) =
        shortest_path({path_source_1_, path_source_2_}, path_target_,
                      mesh_vertices_, mesh_faces_, euclidean_distance);
  } else {
    auto edge_distance = [&](const Eigen::VectorXd&,
                             const Eigen::VectorXd&) -> double { return 1.; };
    std::tie(distances_, parents_) =
        shortest_path({path_source_1_, path_source_2_}, path_target_,
                      mesh_vertices_, mesh_faces_, edge_distance);
  }

  update_mesh_colors();
  std::cout << "Distance: " << std::to_string(distances_[path_target_])
            << std::endl;

  // Get the line vertices
  std::vector<int> vertices = {path_target_};
  std::cout << "Path: " << std::to_string(path_target_);
  while (vertices.back() != path_source_1_ &&
         vertices.back() != path_source_2_) {
    vertices.push_back(parents_[vertices.back()]);
    std::cout << " <- " << std::to_string(vertices.back());
  }
  std::cout << std::endl;
  std::cerr << "vertices: " << vertices.size() << std::endl;

  // draw line from source to target;
  std::reverse(vertices.begin(), vertices.end());
  path_vertices_ = Eigen::MatrixXd(vertices.size(), 3);
  for (std::size_t i = 0; i < vertices.size(); ++i) {
    path_vertices_(i, 0) = mesh_vertices_(vertices[i], 0);
    path_vertices_(i, 1) = mesh_vertices_(vertices[i], 1);
    path_vertices_(i, 2) = mesh_vertices_(vertices[i], 2);
  }
  // create the line "edges"
  if (vertices.size() > 1) {
    path_edges_ = Eigen::MatrixXi(vertices.size() - 1, 2);
    for (std::size_t i = 0; i + 1 < vertices.size(); ++i) {
      path_edges_(i, 0) = i;
      path_edges_(i, 1) = i + 1;
    }
    path_colors_ = (Eigen::MatrixXd(1, 3) << 1.0, 0.0, 0.0).finished();
    viewer->data(path_id_).set_edges(path_vertices_, path_edges_, path_colors_);
  }
  viewer->data(path_id_).line_width = 2.0f;
  viewer->data(path_id_).dirty = igl::opengl::MeshGL::DIRTY_ALL;
  return true;
}

IGL_INLINE bool Menu::load(std::string filename) {
  viewer->data(mesh_id_).clear();
  std::cerr << "loading model: " << filename << std::endl;
  igl::readPLY(filename, mesh_vertices_, mesh_faces_);
  std::cerr << "loading model DONE" << std::endl;

  return update_mesh();
}

IGL_INLINE bool Menu::save(std::string filename) {
  return igl::writePLY(filename, mesh_vertices_, mesh_faces_);
}

IGL_INLINE void Menu::draw_custom_menu() {
  // Helper for setting viewport specific mesh options
  auto make_checkbox = [&](const char* label, unsigned int& option) {
    return ImGui::Checkbox(
        label, [&]() { return viewer->core().is_set(option); },
        [&](bool value) { return viewer->core().set(option, value); });
  };

  // Draw option
  if (ImGui::CollapsingHeader("Draw Options", ImGuiTreeNodeFlags_DefaultOpen)) {
    make_checkbox("Show overlay", viewer->data().show_overlay);
    make_checkbox("Show overlay depth", viewer->data().show_overlay_depth);
    ImGui::ColorEdit4(
        "Background", viewer->core().background_color.data(),
        ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_PickerHueWheel);
  }

  // Mesh
  if (ImGui::CollapsingHeader("Mesh", ImGuiTreeNodeFlags_DefaultOpen)) {
    float w = ImGui::GetContentRegionAvailWidth();
    float p = ImGui::GetStyle().FramePadding.x;
    if (ImGui::Button("Load##Mesh", ImVec2((w - p) / 2.f, 0))) {
      viewer->open_dialog_load_mesh();
    }
    ImGui::SameLine(0, p);
    if (ImGui::Button("Save##Mesh", ImVec2((w - p) / 2.f, 0))) {
      viewer->open_dialog_save_mesh();
    }

    // Draw option
    if (ImGui::Button("Center object", ImVec2(-1, 0))) {
      viewer->core().align_camera_center(viewer->data(mesh_id_).V,
                                         viewer->data(mesh_id_).F);
    }
    if (ImGui::Checkbox("Show mesh lines", &show_mesh_lines_)) {
      update_mesh_lines();
    }
    if (ImGui::Checkbox("Use distance color", &use_color_distance_)) {
      update_mesh_colors();
    }
  }

  // Path
  if (ImGui::CollapsingHeader("Path", ImGuiTreeNodeFlags_DefaultOpen)) {
    // float w = ImGui::GetContentRegionAvailWidth();
    // float p = ImGui::GetStyle().FramePadding.x;
    if (ImGui::SliderInt("Source 1", &path_source_1_, 0,
                         mesh_vertices_.rows() - 1)) {
      update_path();
    }
    if (ImGui::SliderInt("Source 2", &path_source_2_, 0,
                         mesh_vertices_.rows() - 1)) {
      update_path();
    }
    if (ImGui::SliderInt("Target", &path_target_, 0,
                         mesh_vertices_.rows() - 1)) {
      update_path();
    }
    if (ImGui::Checkbox("Use Euclidean distance", &use_euclidean_distance_)) {
      update_path();
    }
    if (ImGui::Checkbox("Show path", &path_show_)) {
      viewer->data(path_id_).is_visible = path_show_;
      viewer->data(path_id_).dirty = igl::opengl::MeshGL::DIRTY_ALL;
    }
  }
}

}  // namespace sm
