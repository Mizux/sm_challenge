[![Build Status][docker_status]][docker_link]

[docker_status]: https://github.com/Mizux/sm_challenge/workflows/Docker/badge.svg?branch=main
[docker_link]: https://github.com/Mizux/sm_challenge/actions?query=workflow%3A"Docker"

# Introduction
Simple [Modern CMake](https://cmake.org/)/C++ App using libigl to draw shortest path.<br>

## Build
This project should run on Linux, Mac and Windows.

### CMake Dependencies Tree
This CMake project is composed of few executables with the following dependencies:
```sh
libigl: eigen3 opengl glfw imgui
app: libigl
```
note: All dependencies are built in static to have one standalone executable `App`.  

### Project directory layout
Thus the project layout is as follow:
```
.
├── CMakeLists.txt
├── patches
│   └── libigl.patch
└── App
    ├── CMakeLists.txt
    └── src
        ├── Menu.hpp
        ├── Menu.cpp
        └── main.cpp
```

### C++ Project Build
To build the C++ project, as usual:
```sh
cmake -S. -Bbuild -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
CTEST_OUTPUT_ON_FAILURE=1 cmake --build build --config Release --target test
```

### Build directory layout
Since we want to use the [CMAKE_BINARY_DIR](https://cmake.org/cmake/help/latest/variable/CMAKE_BINARY_DIR.html) to generate the binary package.  
We want this layout (tree build --prune -P "*.a|FooApp"):
```
build
├── lib
│   ├── libigl.a
│   ├── libigl_opengl.a
│   ├── libigl_opengl_glfw.a
│   ├── libigl_opengl_glfw_imgui.a
│   ├── libglfw3.a
│   └── libimgui.a
└── bin
   └── App
```

## License
See the [LICENSE](LICENSE) file for details.
