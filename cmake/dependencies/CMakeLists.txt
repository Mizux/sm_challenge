include(FetchContent)
set(FETCHCONTENT_QUIET OFF)
set(FETCHCONTENT_UPDATES_DISCONNECTED ON)
set(BUILD_SHARED_LIBS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(BUILD_TESTING OFF)

set(CMAKE_FIND_PACKAGE_PREFER_CONFIG TRUE)
# ##############################################################################
# LIBIGL
# ##############################################################################
if(NOT TARGET igl::core)
  if(BUILD_LIBIGL)
    message(CHECK_START "Fetching LIBIGL")
    list(APPEND CMAKE_MESSAGE_INDENT "  ")
    set(LIBIGL_USE_STATIC_LIBRARY ON)
    set(LIBIGL_WITH_COMISO OFF)
    set(LIBIGL_WITH_EMBREE OFF)
    set(LIBIGL_WITH_MATLAB OFF)
    set(LIBIGL_WITH_MOSEK  OFF)
    set(LIBIGL_WITH_OPENGL ON)
    set(LIBIGL_WITH_OPENGL_GLFW ON)
    set(LIBIGL_WITH_OPENGL_GLFW_IMGUI ON)
    set(LIBIGL_WITH_PNG    OFF)
    set(LIBIGL_WITH_TETGEN OFF)
    set(LIBIGL_WITH_TRIANGLE OFF)
    set(LIBIGL_WITH_PREDICATES OFF)
    set(LIBIGL_WITH_XML OFF)
    FetchContent_Declare(
      libigl
      GIT_REPOSITORY "https://github.com/libigl/libigl.git"
      GIT_TAG "v2.2.0"
      PATCH_COMMAND git apply "${CMAKE_CURRENT_LIST_DIR}/../../patches/libigl.patch"
      )
    FetchContent_MakeAvailable(libigl)
    list(POP_BACK CMAKE_MESSAGE_INDENT)
    message(CHECK_PASS "fetched")
  else()
    set(LIBIGL_WITH_OPENGL ON)
    set(LIBIGL_WITH_OPENGL_GLFW ON)
    set(LIBIGL_WITH_OPENGL_GLFW_IMGUI ON)
    find_package(libigl REQUIRED)
  endif()
endif()

# XXXX does not support C++17 (use of 'register' storage class specifier)
#set(CMAKE_CXX_STANDARD 11)
# ...
#set(CMAKE_CXX_STANDARD 17)
