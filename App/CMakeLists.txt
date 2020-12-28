cmake_minimum_required(VERSION 3.14)
project(App VERSION 1.0.0 LANGUAGES CXX)

file(GLOB _SRCS "src/*.[hc]pp")

add_executable(${PROJECT_NAME} ${_SRCS})
target_include_directories(${PROJECT_NAME} PRIVATE
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
  $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
  )
target_link_libraries(${PROJECT_NAME} PRIVATE
  Eigen3::Eigen
  igl::core
  igl::opengl_glfw
  igl::opengl_glfw_imgui
  )
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)
if(UNIX)
  if(NOT APPLE)
    set_target_properties(${PROJECT_NAME} PROPERTIES
      INSTALL_RPATH "$ORIGIN/../${CMAKE_INSTALL_LIBDIR}")
  else()
    set_target_properties(${PROJECT_NAME} PROPERTIES
      INSTALL_RPATH
      "@loader_path/../${CMAKE_INSTALL_LIBDIR}")
  endif()
endif()
add_executable(${PROJECT_NAMESPACE}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

# Install
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAMESPACE}Targets
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  )
